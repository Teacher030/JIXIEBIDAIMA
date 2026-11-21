/**
  ******************************************************************************
  * @file    robot_arm.c
  * @brief   四自由度机械臂控制业务逻辑
  *          NRF24L01 接收 → 数据包校验 → 姿态解析 → 角度换算/限幅 → PWM 输出
  ******************************************************************************
  */
#include "robot_arm.h"

/* ============================ 参数配置 ============================ */

/* 舵机脉宽范围(微秒), MG996R 典型 500~2500 对应 0~180° */
#define SERVO_ANGLE_MIN       0.0f
#define SERVO_ANGLE_MAX       180.0f
#define SERVO_PULSE_MIN_US    500
#define SERVO_PULSE_MAX_US    2500

/* 关节中性角(度) */
#define JOINT_NEUTRAL_DEG     90.0f

/* 数据包协议(与手持端保持一致) */
#define PKT_SYNC1             0xAA
#define PKT_SYNC2             0x55
#define PKT_DATA_START        2
#define PKT_CHECKSUM_OFFSET   9
#define PKT_ANGLE_SCALE       0.1f   /* 姿态数据单位: 0.1° */

/* 丢包容错与失联保护 */
#define LINK_LOST_TIMEOUT_MS  1000   /* 连续 1000ms 无有效数据判定失联 */
#define LINK_LOST_GO_HOME     1      /* 1: 失联自动回中; 0: 保持最后姿态 */

/* 舵机抖动抑制(低通滤波 + 死区) */
#define ANGLE_SMOOTH_ALPHA    0.15f  /* 低通滤波系数(0~1): 越小越平滑, 响应越慢(0.3~0.5 响应更快) */
#define ANGLE_DEADBAND_DEG    0.5f   /* 死区(度): 目标变化小于此值不更新输出, 抑制抖动 */

/* 舵机自检: 置 1 后上电会逐路扫动每路舵机(0→180→90°), 用于确认接线/PWM 是否正常, 排查完置 0 */
#define SERVO_SELFTEST_ENABLE   0

/* ============================ 内部变量 ============================ */
 
static TIM_HandleTypeDef *s_servoTimer[SERVO_COUNT];
static uint32_t          s_servoChannel[SERVO_COUNT];
static uint32_t          s_lastRxTick = 0;
static float             s_servoOut[SERVO_COUNT];  /* 滤波后的输出角度 */
static uint8_t           s_servoOutInit = 0;       /* 滤波状态是否已初始化 */

/* ============================ 内部函数 ============================ */

static void RobotArm_SelfTest(void);   /* 舵机自检(由 SERVO_SELFTEST_ENABLE 开关) */

/* 初始化 5 路 PWM 通道并启动 */
static void RobotArm_InitPwm(void)
{
    uint8_t i;

    /* 4 路关节舵机使用 TIM2 CH1~CH4, 夹爪使用 TIM3 CH4 */
    s_servoTimer[SERVO_BASE]       = &htim2;
    s_servoChannel[SERVO_BASE]     = TIM_CHANNEL_1;
    s_servoTimer[SERVO_SHOULDER]   = &htim2;
    s_servoChannel[SERVO_SHOULDER] = TIM_CHANNEL_2;
    s_servoTimer[SERVO_ELBOW]      = &htim2;
    s_servoChannel[SERVO_ELBOW]    = TIM_CHANNEL_3;
    s_servoTimer[SERVO_WRIST]      = &htim2;
    s_servoChannel[SERVO_WRIST]    = TIM_CHANNEL_4;
    s_servoTimer[SERVO_GRIPPER]    = &htim3;
    s_servoChannel[SERVO_GRIPPER]  = TIM_CHANNEL_4;

    for (i = 0; i < SERVO_COUNT; i++)
    {
        HAL_TIM_PWM_Start(s_servoTimer[i], s_servoChannel[i]);
    }
}

/* ============================ 接口函数 ============================ */

/* 系统初始化: 启动 PWM、舵机回中、初始化无线接收 */
void RobotArm_Init(void)
{
    RobotArm_InitPwm();       /* 启动 5 路 PWM */
    RobotArm_GoHome();        /* 舵机回到中性位置 */

#if SERVO_SELFTEST_ENABLE
    RobotArm_SelfTest();      /* 自检: 逐路扫动舵机, 确认接线/PWM */
#endif

    NRF24_Init();             /* 初始化 NRF24L01 接收 */
    /* NRF24_Check() 返回 0 表示模块通信异常, 可外接 LED 提示 */
    s_lastRxTick = HAL_GetTick();
}

/* 设置单路舵机角度(含角度限幅保护) */
void RobotArm_SetServoAngle(ServoIndex_t index, float angle)
{
    uint32_t pulse;

    if (index >= SERVO_COUNT)
    {
        return;
    }

    /* 角度限幅保护 */
    if (angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;
    if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;

    /* 角度 → 脉宽(微秒): pulse = 500 + angle/180 * 2000 */
    pulse = SERVO_PULSE_MIN_US +
            (uint32_t)((angle - SERVO_ANGLE_MIN) *
                       (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) /
                       (SERVO_ANGLE_MAX - SERVO_ANGLE_MIN) + 0.5f);

    /* 定时器计数频率 1MHz(1 计数 = 1us), 比较值直接等于脉宽微秒数 */
    __HAL_TIM_SET_COMPARE(s_servoTimer[index], s_servoChannel[index], pulse);
}

/* 一次设置 5 路舵机角度 */
void RobotArm_SetAllAngles(const float *pAngles)
{
    uint8_t i;

    for (i = 0; i < SERVO_COUNT; i++)
    {
        RobotArm_SetServoAngle((ServoIndex_t)i, pAngles[i]);
    }
}

/* 所有舵机回到中性位置 */
void RobotArm_GoHome(void)
{
    RobotArm_SetServoAngle(SERVO_BASE,     JOINT_NEUTRAL_DEG);
    RobotArm_SetServoAngle(SERVO_SHOULDER, JOINT_NEUTRAL_DEG);
    RobotArm_SetServoAngle(SERVO_ELBOW,    JOINT_NEUTRAL_DEG);
    RobotArm_SetServoAngle(SERVO_WRIST,    JOINT_NEUTRAL_DEG);
    RobotArm_SetServoAngle(SERVO_GRIPPER,  90.0f);   /* 夹爪半开 */
}

/* 舵机自检: 逐路 0°→180°→90° 扫动, 用于确认每路舵机接线与 PWM 是否正常 */
static void RobotArm_SelfTest(void)
{
    uint8_t i;

    for (i = 0; i < SERVO_COUNT; i++)
    {
        RobotArm_SetServoAngle((ServoIndex_t)i, 0.0f);
        HAL_Delay(800);
        RobotArm_SetServoAngle((ServoIndex_t)i, 180.0f);
        HAL_Delay(800);
        RobotArm_SetServoAngle((ServoIndex_t)i, 90.0f);
        HAL_Delay(800);
    }
}

/* 主循环处理: 接收 → 校验 → 解析 → 映射 → 输出 */
void RobotArm_Process(void)
{
    uint8_t  buf[NRF24_PAYLOAD_WIDTH];
    uint8_t  checksum;
    uint8_t  i;
    int16_t  roll, pitch, yaw;
    uint8_t  grip;
    float    yawDeg, pitchDeg, rollDeg;
    float    j[SERVO_COUNT];
    uint32_t now;

    now = HAL_GetTick();

    /* 1. 尝试接收一个数据包 */
    if (NRF24_RxPacket(buf, NRF24_PAYLOAD_WIDTH))
    {
        /* 2. 帧头校验 */
        if (buf[0] == PKT_SYNC1 && buf[1] == PKT_SYNC2)
        {
            /* 3. 校验和校验(对数据区字节累加) */
            checksum = 0;
            for (i = PKT_DATA_START; i < PKT_CHECKSUM_OFFSET; i++)
            {
                checksum += buf[i];
            }

            if (checksum == buf[PKT_CHECKSUM_OFFSET])
            {
                /* 4. 解析姿态数据(小端: 低字节在前) */
                roll  = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
                pitch = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
                yaw   = (int16_t)((uint16_t)buf[6] | ((uint16_t)buf[7] << 8));
                grip  = buf[8];

                /* 5. 姿态(0.1°) → 角度(°) */
                yawDeg   = (float)yaw   * PKT_ANGLE_SCALE;
                pitchDeg = (float)pitch * PKT_ANGLE_SCALE;
                rollDeg  = (float)roll  * PKT_ANGLE_SCALE;

                /* 6. 姿态 → 关节角度映射(默认 1:1, 中性 90°):
                 *    底座 = 90 + 航向角(yaw)
                 *    肩   = 90 + 俯仰角(pitch)
                 *    肘   = 90 + 横滚角(roll)
                 *    腕   = 90 + (俯仰+横滚)/2
                 *    夹爪 = 直接使用夹爪指令
                 * 若关节运动方向/幅度与机械结构不符,
                 * 可调整各项系数符号、增益或中性角。 */
                j[SERVO_BASE]     = JOINT_NEUTRAL_DEG + yawDeg;
                j[SERVO_SHOULDER] = JOINT_NEUTRAL_DEG + pitchDeg;
                j[SERVO_ELBOW]    = JOINT_NEUTRAL_DEG + rollDeg;
                j[SERVO_WRIST]    = JOINT_NEUTRAL_DEG + (pitchDeg + rollDeg) * 0.5f;
                j[SERVO_GRIPPER]  = (float)grip;

                /* 6.5 低通滤波 + 死区, 抑制舵机抖动 */
                if (!s_servoOutInit)
                {
                    for (i = 0; i < SERVO_COUNT; i++)
                    {
                        s_servoOut[i] = j[i];
                    }
                    s_servoOutInit = 1;
                }

                for (i = 0; i < SERVO_COUNT; i++)
                {
                    float diff = j[i] - s_servoOut[i];
                    float mag  = (diff < 0.0f) ? -diff : diff;

                    if (mag < ANGLE_DEADBAND_DEG)
                    {
                        j[i] = s_servoOut[i];                       /* 死区: 变化太小, 保持 */
                    }
                    else
                    {
                        s_servoOut[i] += diff * ANGLE_SMOOTH_ALPHA; /* 一阶低通滤波 */
                        j[i] = s_servoOut[i];
                    }
                }

                /* 7. 输出 PWM(内部含限幅保护) */
                RobotArm_SetAllAngles(j);

                /* 8. 更新有效数据时间戳 */
                s_lastRxTick = now;
            }
        }
    }

    /* 9. 失联保护: 长时间无有效数据自动回中 */
    if (LINK_LOST_GO_HOME && ((now - s_lastRxTick) > LINK_LOST_TIMEOUT_MS))
    {
        RobotArm_GoHome();
        s_lastRxTick = now;   /* 避免每个循环重复回中 */
    }
}
