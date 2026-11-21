/**
  ******************************************************************************
  * @file    robot_arm.h
  * @brief   四自由度机械臂控制(机械臂端业务逻辑)头文件
  ******************************************************************************
  */
#ifndef __ROBOT_ARM_H
#define __ROBOT_ARM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "nrf24l01.h"

/* 舵机编号 */
typedef enum
{
    SERVO_BASE = 0,       /* 关节1: 底座旋转   TIM2_CH1 (PA0) */
    SERVO_SHOULDER,       /* 关节2: 肩关节     TIM2_CH2 (PA1) */
    SERVO_ELBOW,          /* 关节3: 肘关节     TIM2_CH3 (PA2) */
    SERVO_WRIST,          /* 关节4: 腕关节     TIM2_CH4 (PA3) */
    SERVO_GRIPPER,        /* 夹爪:            TIM3_CH4 (PB1) */
    SERVO_COUNT           /* 共 5 路舵机 */
} ServoIndex_t;

/* 接口函数 -------------------------------------------------------------------*/
void RobotArm_Init(void);
void RobotArm_SetServoAngle(ServoIndex_t index, float angle);
void RobotArm_SetAllAngles(const float *pAngles);
void RobotArm_GoHome(void);
void RobotArm_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_ARM_H */
