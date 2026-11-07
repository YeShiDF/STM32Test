

#ifndef C6_PID_MOTOR_PID_H
#define C6_PID_MOTOR_PID_H
typedef enum  {
    speed,
    angle,
} PID_type_t;

typedef struct {
    float max_output;//输出限制
    float max_integral;//积分限制
    float kp, ki, kd;//pid参数
    float dt;//采样周期
    PID_type_t type;
    float k_anti_windup_integral;//0-不启用//抗饱和积分屏蔽阈值系数，输出大于此值*最大输出时不记录积分
    float deadzone;//0-不启用//死区，误差不再死区内时不记录积分，防止积分跳变

    float output;
    float integral;
    float derivative;

    float setpoint;
    float error;
    float last_error;

}PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd,PID_type_t type);

void PID_Reset(PID_t *pid);

/* 统一计算入口：喂目标值和实际值，返回本拍控制量（同时也写进 pid->output）
 * 速度环/角度环由 PID_Init 里的 type 决定，调用方不用区分 */
float PID_Calc(PID_t *pid, float target, float actual);

/* 下面两个是 PID_Calc 的薄封装，保留原来的名字，老代码不用改 */
void PID_SetSpeed(PID_t *pid, float target, float speed);

void PID_SetAngle(PID_t *apid, PID_t *spid,float target, float angle,float speed);



#endif //C6_PID_MOTOR_PID_H
