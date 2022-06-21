#include <stdio.h>
#include <wiringPi.h>
#include "libSonar.h"
#include <errno.h>
#include "IPC.h"
#include <unistd.h>

int trigger1 = 7;
int trigger2 = 2;
int trigger3 = 4;
int echo1 = 0;
int echo2 = 3;
int echo3 = 5;
int IRsensor1 = 12;
int IRsensor2 = 13;

char filePath[] = "/tmp/FIFO_Sensors";
int fd;
struct Sensor_Packet packet;

int main()
{
	int error;
	if (wiringPiSetup() == -1) {
		error = errno;
		printf("Error code: %d\t wiringPiSetup\n", error);
		return -1;
	}

	Sonar sonar1;
	sonar1.init(trigger1, echo1);
	Sonar sonar2;
	sonar2.init(trigger2, echo2);
	Sonar sonar3;
	sonar3.init(trigger3, echo3);

	pinMode(IRsensor1, INPUT);
	pinMode(IRsensor2, INPUT);
	
	if (FIFO_Create(filePath) == -1) {
		return -1;
	}
	fd = FIFO_Open(filePath, 1);
	if (fd == -1) {
		return -1;
	}

	while (1) {
		packet.S1 = sonar1.distance(3000); //3000 is the timeout in microseconds
		packet.S2 = sonar2.distance(3000); //3000 is the timeout in microseconds
		packet.S3 = sonar3.distance(3000); //3000 is the timeout in microseconds


		if (digitalRead(IRsensor1) == LOW) {
			packet.I1 = false;
		} else {
			packet.I1 = true;
		}
	
		if (digitalRead(IRsensor2) == LOW) {
			packet.I2 = false;
		} else {
			packet.I2 = true;
		}
        
		if(write(fd, &packet, sizeof(struct Sensor_Packet)) == -1) {
			error = errno;
			printf("Error code: %d\t sending sensor data\n", error);
			return -1;
		}
	}
}
