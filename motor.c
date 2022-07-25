#include <wiringPi.h>
#include <softPwm.h>
#include <errno.h>
#include <stdio.h>
#include "IPC.h"
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#define PWM_Pin 27
#define PWM_Start 13
#define PWM_Range 200

#define ENCA 21
#define ENCB 26
#define IN1 25
#define IN2 22
#define MOTOR_PWM 23

float set_speed = 10;
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

int error;
char filePath[] = "/tmp/FIFO_Sensors";
char filePathCam[] = "/tmp/FIFO_Cam";
struct Sensor_Packet packet;

void motorInit() {
	pinMode(ENCA, INPUT);
	pinMode(ENCB, INPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(MOTOR_PWM, PWM_OUTPUT);
	wiringPiISR (ENCA, INT_EDGE_RISING, &readEncoder);
}

void setMotor(int dir, int pwmVal) {
	pwmWrite(MOTOR_PWM, pwmVal);
	if(dir == 1) {
		digitalWrite(IN1,HIGH);
		digitalWrite(IN2,LOW);
	} else if (dir == -1) {
		digitalWrite(IN1,LOW);
		digitalWrite(IN2,HIGH);
	} else {
		digitalWrite(IN1,LOW);
		digitalWrite(IN2,LOW);
	}
}

void readEncoder (void) {
	long startTime = micros(); 
	static long prevTime = 0;
	int b = digitalRead(ENCB);
	int increment = 0;
	if (b > 0) {
		increment = 1;
	} else {
		increment = -1;
	}
	pos_i += increment;
	prevTime = micros();
	
	long currT = micros();
	float deltaT = ((float)(currT - prevT))/1.0e6;
	float velocity1 = (pos_i - posPrev)/deltaT;
	posPrev = pos_i;
	prevT = currT;
	v1 = velocity1/480*60;
	
	float error = set_speed - v1;
	eIntegral = eIntegral + error * deltaT;
	//eDerivative = (error - prevError)/deltaT;
	u = error*kp + eIntegral*ki /*+ eDerivative*kd*/;
	int dir = 1;
	if (u < 0) {
		dir = -1;
	}
	u = (int)fabs(u);
	if (u > 1023) {
		u = 1023;
	}
	setMotor(1, 80);
	prevError = error;
}

int main() {
	if (wiringPiSetup() == -1) {
		error = errno;
		printf("Error code: %d\t wiringPiSetup\n", error);
		return -1;
	}
	
	if(softPwmCreate(PWM_Pin, PWM_Start, PWM_Range) != 0) {
		error = errno;
		printf("Error code: %d\t softPwmCreate\n");
		return -1;
	}
	int fd = FIFO_Open(filePath, 0);
	int fd_cam = FIFO_Open(filePathCam, 0);
	int ret;
	int16_t angle;
	motorInit();
	
	while(1) {
		ret = read(fd, &packet, sizeof(struct Sensor_Packet));	
		if(ret == -1) {
			error = errno;
			printf("Error code: %d\t reading sensor data\n", error);
			return -1;
		} else if(ret != sizeof(struct Sensor_Packet)) {
			printf("Received only %d of %d bytes\n", ret, sizeof(struct Sensor_Packet));
			return -1;
		}
		do {
			ret = read(fd_cam, &angle, sizeof(int16_t));
			if(ret == -1) {
				error = errno;
				printf("Error code: %d\t reading camera data\n", error);
				return -1;
			}
		} while(ret != 0);
		
		if(packet.S2 < 50) {
			if(packet.S1 < packet.S3) {
				angle = 20;
			} else {
				angle = -20;
			}
		}
		//printf("S1: %f\nS2: %f\nS3: %f\nI1: %d\nI2: %d\n\n", packet.S1, packet.S2, packet.S3, packet.I1, packet.I2);
		printf("%d\n", angle);
		/*if(packet.S2 < 10) {
			set_speed = 0;
		} else {
			set_speed = 100;
		}*/
		
		if(angle > 180 || angle < 0) {
			continue;
		}
		angle -= 90;
		if(angle > 20) {
			angle = 20;
		} else if(angle < -20) {
			angle = -20;
		}
		angle *= 0.25;
		angle += 13;
		printf("temp: %d\n\n", angle);
		softPwmWrite(PWM_Pin, angle);
	}
	softPwmStop(PWM_Pin);
	return 0;
}
