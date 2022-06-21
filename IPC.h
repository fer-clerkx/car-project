#ifndef _IPC_H
#define _IPC_H

#include <stdbool.h>

struct Sensor_Packet {
	double S1;
	double S2;
	double S3;
	bool I1;
	bool I2;
};

//Create a FIFO at filePath.
//If a FIFO already eists, it will be deleted
//Returns 0 when executed correctly, -1 for an error
int FIFO_Create(char *filePath);

//Open FIFO at filePath
//Return file descriptor
//Write only if mode = true, read only if mode = false
int FIFO_Open(char *filePath, bool mode);

#endif
