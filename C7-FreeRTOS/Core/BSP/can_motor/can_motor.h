/**
  ******************************************************************************
  * @file    can_motor.h
  * @brief   DJI 电机 CAN 驱动（GM6020 / M3508）—— 对外接口
  * @note    1. 上层只调用本文件的函数，不直接碰 HAL_CAN_xxx 和 0x1FF 这类魔数
  *          2. 使用前必须在 CubeMX 里配好 CAN（1Mbps）并打开 RX0 中断
  ******************************************************************************
  */

#ifndef __CAN_MOTOR_H
#define __CAN_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "can.h"          /* Core/Inc 已在头文件搜索路径里，写相对路径反而一挪就断 */

/* ============================ 可调参数 ============================ */

#define MOTOR_MAX_NUM        8      /* 支持 ID 1~8（GM6020 只到 7） */
#define MOTOR_OFFLINE_MS     100    /* 超过这么久没收到反馈就判离线（ms） */

#define GM6020_OUT_MAX       30000  /* GM6020：电压控制量，±30000 ↔ ±24V */
#define M3508_OUT_MAX        16384  /* M3508 ：电流控制量，±16384 ↔ ±20A */

/* ============================ 类型定义 ============================ */

typedef enum {
    MOTOR_TYPE_GM6020 = 0,          /* 云台电机，电压控制 */
    MOTOR_TYPE_M3508  = 1           /* 底盘电机，电流控制 */
} Motor_Type_t;

/**
 * @brief 单个电机的反馈数据（原始值 + 解算值都保留，方便调试和上层使用）
 */
typedef struct {
    uint16_t angle;                 /* 机械角度编码值 0~8191 */
    int16_t  speed;                 /* 转速，rpm，可为负 */
    int16_t  current;               /* 转矩电流（6020）/ 实际电流（3508） */
    uint8_t  temp;                  /* 温度，℃ */
    float    degree;                /* 换算后的角度 0~360° */
    uint32_t last_tick;             /* 最近一次收到反馈的 HAL_GetTick() */
    uint8_t  online;                /* 1=在线 0=离线，由 Motor_Update 刷新 */
} Motor_Data_t;

/* ============================ 接口函数 ============================ */

/**
 * @brief  初始化 CAN 外设并挂载接收回调
 * @param  hcan 已由 CubeMX 初始化好的 CAN 句柄，例如 &hcan1
 * @note   内部会配置过滤器（全收）、启动 CAN、打开 FIFO0 接收中断
 */
void Motor_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief  注册一个电机（告诉驱动：总线上有这台电机，它是什么型号）
 * @param  id   电机 ID，1~8（GM6020 为 1~7）
 * @param  type 电机型号
 * @retval 1=注册成功  0=参数非法（ID 超范围或重复注册失败）
 * @note   必须在 Motor_Init 之后、进入主循环之前调用
 */
uint8_t Motor_Attach(uint8_t id, Motor_Type_t type);

/**
 * @brief  设置某个电机的输出量（只写缓存，不真正发送）
 * @param  id    电机 ID
 * @param  value 控制量：GM6020 为 -30000~30000，M3508 为 -16384~16384
 * @note   内部自动限幅，超范围不会打穿协议
 */
void Motor_SetOutput(uint8_t id, int16_t value);

/**
 * @brief  把缓存里的控制量按帧打包并发送（真正的 CAN 发送发生在这里）
 * @retval HAL_OK 全部发出；HAL_ERROR 至少有一帧没发出去（通常是邮箱满/无 ACK）
 * @note   建议 1kHz 调用一次，不要每个电机单独发一帧
 */
HAL_StatusTypeDef Motor_Flush(void);

/**
 * @brief  解析一帧反馈数据（可被驱动的内部回调调用，也可由用户自己调用）
 * @param  std_id 收到帧的标准帧 ID
 * @param  data   8 字节数据
 * @note   内部按注册表匹配，匹配不到直接忽略
 */
void Motor_Decode(uint32_t std_id, const uint8_t *data);

/**
 * @brief  刷新所有电机的在线状态（软件看门狗）
 * @retval 1=全部在线  0=至少有一台离线
 * @note   放在主循环里，每 1~10ms 调用一次
 */
uint8_t Motor_Update(void);

/**
 * @brief  读取某台电机的反馈数据
 * @retval 指向内部数据的指针；未注册返回 NULL。数据只读，不要在外面改
 */
const Motor_Data_t *Motor_Get(uint8_t id);

/**
 * @brief  查询某台电机是否在线
 * @retval 1=在线 0=离线（未注册也返回 0）
 */
uint8_t Motor_IsOnline(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_MOTOR_H */
