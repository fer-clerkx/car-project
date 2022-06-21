#ifndef _PID_H
#define _PID_H

#define ENCA 21
#define ENCB 26
#define IN1 25
#define IN2 22
#define MOTOR_PWM 23

float set_speed = 100;
const float kp = 50;
const float ki = 25;
//const float kd = 0;

volatile int pos_i = 0;
volatile int posPrev = 0;
volatile long prevT = 0;
volatile float v1 = 0;
volatile float u = 0;

float eIntegral = 0;
//float eDerivative = 0;
float prevError = 0;

long outT = 0;

void motorInit();
void setMotor(int dir, int pwmVal);
void readEncoder (void);

#endif
