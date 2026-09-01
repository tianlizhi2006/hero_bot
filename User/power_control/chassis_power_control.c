#include "chassis_power_control.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "power_board.h"

// 可行性测试阶段使用固定60W，不读取报文中的power_limit
#define CHASSIS_POWER_FIXED_LIMIT_W                60.0f

// 功率安全裕量和反馈超时时间
#define CHASSIS_POWER_SAFETY_MARGIN_W               3.0f
#define CHASSIS_POWER_EMERGENCY_MARGIN_W            2.0f
#define CHASSIS_POWER_FEEDBACK_TIMEOUT_MS           20U

// 功率反馈失联时的保守限制
#define CHASSIS_POWER_FAILSAFE_LIMIT_W              50.0f
#define CHASSIS_POWER_FAILSAFE_MARGIN_W              5.0f
#define CHASSIS_POWER_FAILSAFE_BUDGET_FACTOR         0.65f

// 功率模型参数
#define CHASSIS_POWER_MODEL_TORQUE_SQ                0.55f
#define CHASSIS_POWER_MODEL_SPEED_SQ                 0.005f
#define CHASSIS_POWER_MODEL_CONSTANT_W               1.6f

// 根据功率板Pout修正模型预算
#define CHASSIS_POWER_BUDGET_FACTOR_INITIAL          0.80f
#define CHASSIS_POWER_BUDGET_FACTOR_MIN              0.25f
#define CHASSIS_POWER_BUDGET_FACTOR_MAX              1.50f
#define CHASSIS_POWER_ADAPT_UP_GAIN                   0.08f
#define CHASSIS_POWER_ADAPT_DOWN_GAIN                 0.40f
#define CHASSIS_POWER_ADAPT_UP_MAX_STEP               0.02f
#define CHASSIS_POWER_ADAPT_DOWN_MAX_STEP             0.25f
#define CHASSIS_POWER_EMERGENCY_HOLD_CYCLES           5U
#define CHASSIS_POWER_BISECTION_STEPS                12U

typedef struct
{
    float control_target_w;
    float model_budget_factor;
    bool limiting;
} chassis_power_control_state_t;

typedef struct
{
    uint32_t last_feedback_count;
    uint8_t emergency_cycles;
} power_control_runtime_t;

static chassis_power_control_state_t power_control_state;
static power_control_runtime_t power_control_runtime;

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

// 估算四个电机的总输入功率：P=max(T*w,0)+K1*T^2+K2*w^2+C
static float estimate_power(const float torque[CHASSIS_POWER_MOTOR_COUNT],
                            const float speed[CHASSIS_POWER_MOTOR_COUNT],
                            float torque_scale)
{
    float total_power = 0.0f;
    uint8_t i;

    for (i = 0U; i < CHASSIS_POWER_MOTOR_COUNT; ++i)
    {
        float scaled_torque = torque[i] * torque_scale;
        float mechanical_power = scaled_torque * speed[i];

        // 不使用负机械功率抵扣功耗，避免高估制动回馈能力
        if (mechanical_power < 0.0f) mechanical_power = 0.0f;

        total_power += mechanical_power
                     + CHASSIS_POWER_MODEL_TORQUE_SQ * scaled_torque * scaled_torque
                     + CHASSIS_POWER_MODEL_SPEED_SQ * speed[i] * speed[i]
                     + CHASSIS_POWER_MODEL_CONSTANT_W;
    }

    return total_power;
}

// 测试阶段仅检查反馈时效和功率板状态
static bool feedback_is_valid(const power_board_data_t *power_data, uint32_t now)
{
    return (power_data->message_count != 0U) &&
           ((now - power_data->last_rx_tick) <= CHASSIS_POWER_FEEDBACK_TIMEOUT_MS) &&
           (power_data->situation != POWER_BOARD_SITUATION_ERROR);
}

// 功率反馈失联时按照最低功率限制降额运行
static void enter_failsafe(void)
{
    power_control_state.control_target_w =
        CHASSIS_POWER_FAILSAFE_LIMIT_W - CHASSIS_POWER_FAILSAFE_MARGIN_W;

    if (power_control_state.model_budget_factor > CHASSIS_POWER_FAILSAFE_BUDGET_FACTOR)
    {
        power_control_state.model_budget_factor = CHASSIS_POWER_FAILSAFE_BUDGET_FACTOR;
    }
}

// 收到新反馈后修正模型预算，低于目标慢慢放开，高于目标快速收紧
static void adapt_budget_from_feedback(const power_board_data_t *power_data)
{
    float limit_w = CHASSIS_POWER_FIXED_LIMIT_W;
    float target_w = limit_w - CHASSIS_POWER_SAFETY_MARGIN_W;
    float measured_w = power_data->robot_power;
    float normalized_error;
    float factor_step;

    power_control_state.control_target_w = target_w;

    if (power_data->message_count == power_control_runtime.last_feedback_count) return;
    power_control_runtime.last_feedback_count = power_data->message_count;

    normalized_error = (target_w - measured_w) / limit_w;
    if (measured_w > target_w)
    {
        factor_step = clamp_float(CHASSIS_POWER_ADAPT_DOWN_GAIN * normalized_error,
                                  -CHASSIS_POWER_ADAPT_DOWN_MAX_STEP,
                                  0.0f);
        power_control_state.model_budget_factor += factor_step;
    }
    else if (power_control_state.limiting)
    {
        factor_step = clamp_float(CHASSIS_POWER_ADAPT_UP_GAIN * normalized_error,
                                  0.0f,
                                  CHASSIS_POWER_ADAPT_UP_MAX_STEP);
        power_control_state.model_budget_factor += factor_step;
    }

    // 超过功率上限时再次收紧，超出2W时保持10ms零力矩
    if (measured_w > limit_w)
    {
        power_control_state.model_budget_factor *= target_w / measured_w;

        if (measured_w > (limit_w + CHASSIS_POWER_EMERGENCY_MARGIN_W))
        {
            power_control_state.model_budget_factor *= 0.5f;
            power_control_runtime.emergency_cycles = CHASSIS_POWER_EMERGENCY_HOLD_CYCLES;
        }
    }

    power_control_state.model_budget_factor =
        clamp_float(power_control_state.model_budget_factor,
                    CHASSIS_POWER_BUDGET_FACTOR_MIN,
                    CHASSIS_POWER_BUDGET_FACTOR_MAX);
}

static void update_feedback(void)
{
    power_board_data_t power_data;

    if (!power_board_get_data(&power_data) || !feedback_is_valid(&power_data, HAL_GetTick()))
    {
        enter_failsafe();
        return;
    }

    adapt_budget_from_feedback(&power_data);
}

// 在0~1之间查找满足功率限制的最大力矩缩放系数
static float solve_torque_scale(const float torque[CHASSIS_POWER_MOTOR_COUNT],
                                const float speed[CHASSIS_POWER_MOTOR_COUNT],
                                float model_budget_w)
{
    float lower = 0.0f;
    float upper = 1.0f;
    float middle;
    uint8_t step;

    if (power_control_runtime.emergency_cycles > 0U)
    {
        --power_control_runtime.emergency_cycles;
        return 0.0f;
    }

    if (estimate_power(torque, speed, 1.0f) <= model_budget_w) return 1.0f;
    if (estimate_power(torque, speed, 0.0f) >= model_budget_w) return 0.0f;

    for (step = 0U; step < CHASSIS_POWER_BISECTION_STEPS; ++step)
    {
        middle = 0.5f * (lower + upper);
        if (estimate_power(torque, speed, middle) > model_budget_w)
        {
            upper = middle;
        }
        else
        {
            lower = middle;
        }
    }

    return lower;
}

void chassis_power_control_init(void)
{
    power_control_state.control_target_w =
        CHASSIS_POWER_FAILSAFE_LIMIT_W - CHASSIS_POWER_FAILSAFE_MARGIN_W;
    power_control_state.model_budget_factor = CHASSIS_POWER_BUDGET_FACTOR_INITIAL;
    power_control_state.limiting = true;
    power_control_runtime.last_feedback_count = 0U;
    power_control_runtime.emergency_cycles = 0U;
}

// 停车时复位功率控制
void chassis_power_control_reset(void)
{
    power_control_state.model_budget_factor = CHASSIS_POWER_BUDGET_FACTOR_INITIAL;
    power_control_state.limiting = false;
    power_control_runtime.emergency_cycles = 0U;
}

void chassis_power_control_apply(const float requested_torque[CHASSIS_POWER_MOTOR_COUNT],
                                 const float motor_speed[CHASSIS_POWER_MOTOR_COUNT],
                                 float limited_torque[CHASSIS_POWER_MOTOR_COUNT])
{
    float model_budget_w;
    float scale;
    uint8_t i;

    if ((requested_torque == NULL) || (motor_speed == NULL) || (limited_torque == NULL)) return;

    update_feedback();
    model_budget_w = power_control_state.control_target_w * power_control_state.model_budget_factor;
    scale = solve_torque_scale(requested_torque, motor_speed, model_budget_w);

    // 四轮使用同一缩放系数，保持原始力矩比例
    for (i = 0U; i < CHASSIS_POWER_MOTOR_COUNT; ++i)
    {
        limited_torque[i] = requested_torque[i] * scale;
    }

    power_control_state.limiting = scale < 0.999f;
}
