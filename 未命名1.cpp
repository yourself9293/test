#include "stdio.h"
#include<stdlib.h>
 
//定义PID结构体
struct _pid{
	float SetSpeed;
	float ActualSpeed;
	float err;
	float err_last;
	float Kp,Ki,Kd;
	//位置式pid
	float voltage;
	float integral;
	//增量式pid
	float err_next;
	//抗积分饱和PID
	float umax;
	float umin;
}pid;
 
//初始化变量(基于位置式PID)
void PID_init() {
	printf("PID_init begin\n");
	pid.SetSpeed = 0.0;
	pid.ActualSpeed = 0.0;
	pid.err = 0.0;
	pid.err_last = 0.0;
	pid.voltage = 0.0;
	pid.integral = 0.0;
	pid.Kp = 0.2;
	pid.Ki = 0.015;
	pid.Kd = 0.2;
	printf("PID_init end \n");
}
 
//编写控制算法
//位置式PID
float positional_PID_realize(float speed) {
	pid.SetSpeed = speed;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
	pid.integral += pid.err;
	pid.voltage = pid.Kp*pid.err + pid.Ki*pid.integral + pid.Kd*(pid.err - pid.err_last);
	pid.err_last = pid.err;
	pid.ActualSpeed = pid.voltage*1.0;
	return pid.ActualSpeed;
}
 
//增量式PID
float incremental_PID_realize(float speed) {
	pid.err_next = 0.0;
 
	pid.SetSpeed = speed;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
	float incrementSpeed = pid.Kp*(pid.err - pid.err_next) + pid.Ki*pid.err + pid.Kd*(pid.err - 2 * pid.err_next + pid.err_last);
	pid.ActualSpeed += incrementSpeed;
	pid.err_last = pid.err_next;
	pid.err_next = pid.err;
	return pid.ActualSpeed;
}
 
//积分分离PID
float  IntegralSeparatio_PID_realize(float speed) {
	int index;
	pid.Kp = 0.2;
	pid.Ki = 0.04;
	pid.Kd = 0.2;
 
	pid.SetSpeed = speed;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
	if (abs(pid.err) > 200) {
		index = 0;
	}
	else {
		index = 1;
		pid.integral += pid.err;
	}
	pid.voltage = pid.Kp*pid.err + index*pid.Ki*pid.integral + pid.Kd*(pid.err - pid.err_last);
	pid.err_last = pid.err;
	pid.ActualSpeed = pid.voltage*1.0;
	return pid.ActualSpeed;
}
 
//抗饱和PID
float anti_windup_PID_realize(float speed) {
	pid.Kp = 0.2;
	pid.Ki = 0.1;
	pid.Kd = 0.2;
	pid.umax = 400;
	pid.umin = -200;
	int index;
	pid.SetSpeed = speed;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
	if (pid.ActualSpeed > pid.umax) {
		if (abs(pid.err) > 200)
		{
			index = 0;
		}
		else {
			index = 1;
			if (pid.err < 0)
			{
				pid.integral += pid.err;
			}
		}
	}
	else if (pid.ActualSpeed < pid.umin) {
		if (abs(pid.err) > 200)
		{
			index = 0;
		}
		else {
			index = 1;
			if (pid.err > 0)
			{
				pid.integral += pid.err;
			}
		}
	}
	else {
		if (abs(pid.err) > 200)
		{
			index = 0;
		}
		else {
			index = 1;
			pid.integral += pid.err;
		}
	}
	pid.voltage = pid.Kp*pid.err + index*pid.Ki*pid.integral + pid.Kd*(pid.err - pid.err_last);
	pid.err_last = pid.err;
	pid.ActualSpeed = pid.voltage*1.0;
	return pid.ActualSpeed;
 
}
 
//测试代码
int main() {
	printf("system begin\n");
	PID_init();
	int count = 0;
	while (count < 500)
	{
		float speed = anti_windup_PID_realize(200.0);
		printf("%f\n", speed);
		count++;
	}
	system("pause");
	return 0;
}
