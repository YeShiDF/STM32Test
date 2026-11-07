/**
  ******************************************************************************
  * @file    can_motor.c
  * @brief   DJI 电机 CAN 驱动实现（GM6020 / M3508）
  * @note    协议要点：
  *          GM6020  控制帧 0x1FF(ID1~4) / 0x2FF(ID5~7)   反馈帧 0x205~0x20B
  *          M3508   控制帧 0x200(ID1~4) / 0x1FF(ID5~8)   反馈帧 0x201~0x204
  *          每帧 8 字节 = 4 个 int16，大端（高字节在前）
  ******************************************************************************
  */

#include "can_motor.h"
#include <string.h>

/* ============================ 内部类型 ============================ */

typedef struct {
    uint8_t      used;              /* 1=已注册 */
    uint8_t      id;                /* 电机 ID */
    Motor_Type_t type;              /* 型号 */
    uint32_t     tx_id;             /* 该电机所属的控制帧 ID */
    uint8_t      slot;              /* 在 8 字节里的槽位 0~3 */
    uint32_t     rx_id;             /* 该电机的反馈帧 ID */
    int16_t      out;               /* 待发送的控制量 */
    Motor_Data_t data;              /* 反馈数据 */
} Motor_Obj_t;

/* ============================ 内部变量 ============================ */

static CAN_HandleTypeDef *s_hcan = 0;
static Motor_Obj_t        s_motor[MOTOR_MAX_NUM];

/* ============================ 内部函数 ============================ */

/**
 * @brief 根据型号和 ID 计算控制帧 ID 与槽位
 */
static uint32_t motor_tx_id(Motor_Type_t type, uint8_t id, uint8_t *slot)
{
    if (type == MOTOR_TYPE_GM6020) {
        if (id >= 1 && id <= 4) {
            *slot = (uint8_t)(id - 1);
            return 0x1FF;
        }
        if (id >= 5 && id <= 7) {
            *slot = (uint8_t)(id - 5);
            return 0x2FF;
        }
    } else {
        if (id >= 1 && id <= 4) {
            *slot = (uint8_t)(id - 1);
            return 0x200;
        }
        if (id >= 5 && id <= 8) {
            *slot = (uint8_t)(id - 5);
            return 0x1FF;
        }
    }
    return 0;
}

/**
 * @brief 根据型号和 ID 计算反馈帧 ID
 */
static uint32_t motor_rx_id(Motor_Type_t type, uint8_t id)
{
    if (type == MOTOR_TYPE_GM6020) {
        return (uint32_t)(0x205u + id - 1u);
    }
    return (uint32_t)(0x201u + id - 1u);
}

/* ============================ 对外接口 ============================ */

void Motor_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter;

    s_hcan = hcan;
    (void)memset(s_motor, 0, sizeof(s_motor));

    /* 过滤器：掩码全 0 = 全部放行，ID 判断交给软件，初学阶段最不容易错
       注意：CAN1 只能用 0~13 组，CAN2 只能用 14~27 组，这里按实例自动选 */
    filter.FilterBank           = (hcan->Instance == CAN2) ? 14 : 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;      /* CAN2 从第 14 组开始，CAN1 用不到 */

    (void)HAL_CAN_ConfigFilter(hcan, &filter);
    (void)HAL_CAN_Start(hcan);
    (void)HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

uint8_t Motor_Attach(uint8_t id, Motor_Type_t type)
{
    Motor_Obj_t *m;
    uint32_t tx_id;
    uint8_t  slot = 0;
    uint8_t  i;

    if (id < 1 || id > MOTOR_MAX_NUM) {
        return 0;
    }

    tx_id = motor_tx_id(type, id, &slot);
    if (tx_id == 0) {                  /* ID 与型号不匹配，例如 6020 给了 ID 8 */
        return 0;
    }

    /* 槽位冲突检查：0x1FF 在 6020(ID1~4) 和 3508(ID5~8) 之间是共用的，
       两种电机混挂时同一个槽位会被后注册的覆盖，这里直接拒绝，避免"莫名少一台电机" */
    for (i = 0; i < MOTOR_MAX_NUM; i++) {
        if (s_motor[i].used && s_motor[i].tx_id == tx_id && s_motor[i].slot == slot) {
            return 0;
        }
    }

    m = &s_motor[id - 1];
    m->used    = 1;
    m->id      = id;
    m->type    = type;
    m->tx_id   = tx_id;
    m->slot    = slot;
    m->rx_id   = motor_rx_id(type, id);
    m->out     = 0;
    m->data.last_tick = HAL_GetTick(); /* 先给个初值，避免刚上电就判离线 */
    m->data.online    = 1;
    return 1;
}

void Motor_SetOutput(uint8_t id, int16_t value)
{
    Motor_Obj_t *m;
    int32_t limit;
    int32_t v;

    if (id < 1 || id > MOTOR_MAX_NUM) {
        return;
    }
    m = &s_motor[id - 1];
    if (!m->used) {
        return;
    }

    limit = (m->type == MOTOR_TYPE_GM6020) ? GM6020_OUT_MAX : M3508_OUT_MAX;
    v = (int32_t)value;
    if (v >  limit) { v =  limit; }    /* 限幅，防止把协议打穿 */
    if (v < -limit) { v = -limit; }
    m->out = (int16_t)v;
}

HAL_StatusTypeDef Motor_Flush(void)
{
    CAN_TxHeaderTypeDef tx;
    uint8_t  data[8];
    uint32_t mailbox;
    uint32_t ids[3];                   /* 最多三种控制帧：0x1FF / 0x2FF / 0x200 */
    uint8_t  n = 0;
    uint8_t  i;
    uint8_t  j;
    uint8_t  k;
    uint16_t raw;
    HAL_StatusTypeDef ret = HAL_OK;

    if (s_hcan == 0) {
        return HAL_ERROR;
    }

    /* 1. 收集本次要发送的控制帧 ID（去重） */
    for (i = 0; i < MOTOR_MAX_NUM; i++) {
        if (!s_motor[i].used) {
            continue;
        }
        for (k = 0; k < n; k++) {
            if (ids[k] == s_motor[i].tx_id) {
                break;
            }
        }
        if (k < n) {
            continue;                  /* 这一组已经在列表里 */
        }
        if (n >= 3) {
            break;
        }
        ids[n] = s_motor[i].tx_id;
        n++;
    }

    /* 2. 逐组打包并发送 */
    for (k = 0; k < n; k++) {
        for (j = 0; j < 8; j++) {
            data[j] = 0;
        }

        for (i = 0; i < MOTOR_MAX_NUM; i++) {
            if (!s_motor[i].used || s_motor[i].tx_id != ids[k]) {
                continue;
            }
            raw = (uint16_t)s_motor[i].out;
            data[2 * s_motor[i].slot]     = (uint8_t)(raw >> 8);    /* 高字节在前 */
            data[2 * s_motor[i].slot + 1] = (uint8_t)(raw & 0xFFu); /* 低字节在后 */
        }

        tx.StdId              = ids[k];
        tx.ExtId              = 0;
        tx.IDE                = CAN_ID_STD;
        tx.RTR                = CAN_RTR_DATA;
        tx.DLC                = 8;
        tx.TransmitGlobalTime = DISABLE;

        if (HAL_CAN_AddTxMessage(s_hcan, &tx, data, &mailbox) != HAL_OK) {
            ret = HAL_ERROR;           /* 邮箱满或总线无 ACK */
        }
    }

    return ret;
}

void Motor_Decode(uint32_t std_id, const uint8_t *data)
{
    uint8_t i;
    Motor_Data_t *d;

    if (data == 0) {
        return;
    }

    for (i = 0; i < MOTOR_MAX_NUM; i++) {
        if (!s_motor[i].used || s_motor[i].rx_id != std_id) {
            continue;
        }

        d = &s_motor[i].data;
        d->angle   = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);   /* 0~8191 */
        d->speed   = (int16_t)(((uint16_t)data[2] << 8) | data[3]);    /* rpm */
        d->current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);    /* 转矩电流 */
        d->temp    = data[6];                                          /* ℃ */
        d->degree  = (float)d->angle / 8192.0f * 360.0f;
        d->last_tick = HAL_GetTick();      /* 心跳打点：看门狗靠它判断在线 */
        return;
    }
}

uint8_t Motor_Update(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  i;
    uint8_t  all_online = 1;

    for (i = 0; i < MOTOR_MAX_NUM; i++) {
        if (!s_motor[i].used) {
            continue;
        }

        /* 无符号减法天然处理 HAL_GetTick 的 49.7 天溢出 */
        if ((uint32_t)(now - s_motor[i].data.last_tick) > MOTOR_OFFLINE_MS) {
            s_motor[i].data.online = 0;
            all_online = 0;
        } else {
            s_motor[i].data.online = 1;
        }
    }

    return all_online;
}

const Motor_Data_t *Motor_Get(uint8_t id)
{
    if (id < 1 || id > MOTOR_MAX_NUM || !s_motor[id - 1].used) {
        return 0;
    }
    return &s_motor[id - 1].data;
}

uint8_t Motor_IsOnline(uint8_t id)
{
    if (id < 1 || id > MOTOR_MAX_NUM || !s_motor[id - 1].used) {
        return 0;
    }
    return s_motor[id - 1].data.online;
}

/* ============================ HAL 回调 ============================ */

/**
 * @brief CAN 接收回调（FIFO0）
 * @note  CubeMX 里必须打开 CAN 的 RX0 中断，否则这个函数永远不会被调用
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t data[8];

    if (hcan != s_hcan) {
        return;
    }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK) {
        return;
    }

    Motor_Decode(rx.StdId, data);
}
