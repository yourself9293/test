#include "stdio.h"
#include<stdlib.h>
 
//����PID�ṹ��
struct _pid{
	float SetSpeed;
	float ActualSpeed;
	float err;
	float err_last;
	float Kp,Ki,Kd;
	//λ��ʽpid
	float voltage;
	float integral;
	//����ʽpid
	float err_next;
	//�����ֱ���PID
	float umax;
	float umin;
}pid;
 
//��ʼ������(����λ��ʽPID)
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
 
//��д�����㷨
//λ��ʽPID
float positional_PID_realize(float speed) {
	pid.SetSpeed = speed;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
	pid.integral += pid.err;
	pid.voltage = pid.Kp*pid.err + pid.Ki*pid.integral + pid.Kd*(pid.err - pid.err_last);
	pid.err_last = pid.err;
	pid.ActualSpeed = pid.voltage*1.0;
	return pid.ActualSpeed;
}
 
//����ʽPID
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
 
//���ַ���PID
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
 
//������PID
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
 
//���Դ���
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
