#include "pid.h"

#include <math.h>
#include <string.h>

#define  PID_SPEED_DEADZONE 1//rpm（2 会带来 2rpm 静差，正好卡在验收线上）
#define PID_ANGLE_DEADZONE 2//0-8191

#define PID_SPEED_MAX_OUTPUT 20000//+30000-- -30000
#define PID_ANGLE_MAX_OUTPUT 200//rpm
#define PID_MODE_NO_DERIVATIVE 1



void PID_Init(PID_t *pid, float kp, float ki, float kd,PID_type_t type) {
   memset(pid,0,sizeof(PID_t));

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = (PID_MODE_NO_DERIVATIVE)? 0: kd;
    pid->type = type;

    pid->max_output = (pid->type == speed) ? PID_SPEED_MAX_OUTPUT : PID_ANGLE_MAX_OUTPUT;
    pid->deadzone = (pid->type == speed) ? PID_SPEED_DEADZONE : PID_ANGLE_DEADZONE;

    pid->max_integral = (ki == 0) ?  0: pid->max_output/pid->ki;
    pid->dt = 0.001f;
    pid->k_anti_windup_integral = 0.6f;

}

void PID_Reset(PID_t *pid) {
    pid->output = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->setpoint = 0;
    pid->error = 0;//需要补充防微分冲击机制
    pid->last_error = 0;
}

float PID_AngelErrCorrect(float target,float current) {//加入环形折算，如果位移超过半圈则反转
    float e = target - current;
    if (e > 4096.0f) e -= 8192.0f;
    if (e < -4096.0f) e += 8192.0f;
    return e;
}

float PID_Calc(PID_t *pid, float target, float actual) {
    int ifSaturated;

    pid->setpoint = target;

    /* ① 算误差：角度环做环形折算（0~8191 是首尾相接的），速度环直接相减 */
    if (pid->type == angle) {
        pid->error = PID_AngelErrCorrect(target, actual);
    } else {
        pid->error = target - actual;
    }

    /* ② 死区：误差很小就当作 0，避免转速整数反馈在目标附近来回抖 */
    if (fabsf(pid->error) <= pid->deadzone) pid->error = 0.0f;

    /* ③ 抗饱和：输出已经顶到阈值、误差还在同方向推，这一拍不记账 */
    ifSaturated = 0;
    if (pid->k_anti_windup_integral > 0.0f) {
        float lim = pid->max_output * pid->k_anti_windup_integral;
        ifSaturated = ((pid->output >=  lim && pid->error > 0.0f) ||
                       (pid->output <= -lim && pid->error < 0.0f));
    }
    if (!ifSaturated) pid->integral += pid->error * pid->dt;

    /* ④ 积分限幅 */
    if (pid->integral >  pid->max_integral) pid->integral =  pid->max_integral;
    if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    /* ⑤ 微分（两个环统一：kd 里已经包含了"每拍"的量纲） */
    pid->derivative = pid->kd * (pid->error - pid->last_error);

    /* ⑥ 三项求和 + 输出限幅 */
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->derivative;
    if (pid->output >  pid->max_output) pid->output =  pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output;

    pid->last_error = pid->error;

    return pid->output;
}

void PID_SetSpeed(PID_t *pid, float target, float speed) {
    (void)PID_Calc(pid, target, speed);
}

void PID_SetAngle(PID_t *apid, PID_t *spid, float target, float angle, float speed) {
    /* 外环算出一个"目标转速"，直接喂给内环，这就是串级 */
    PID_Calc(apid, target, angle);
    PID_Calc(spid, apid->output, speed);
}