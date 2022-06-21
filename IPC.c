#include "IPC.h"

#include <unistd.h>	//acces() and F_OK
#include <errno.h>	//errno and error flags
#include <sys/stat.h>	//mkfifo and its parameters
#include <stdio.h>	//fprint
#include <stdbool.h> //bool type
#include <fcntl.h>	//Open function and parameters

int FIFO_Create(char *filePath) {
	int error;
	if(access(filePath, F_OK) == -1) {
		//FIFO doesn't exist
		if(errno == ENOENT) {
			if(mkfifo(filePath, S_IRUSR | S_IWUSR) == -1) {
				error = errno;
				printf("Error %d, couldn't create FIFO %s\n", error, filePath);
				return -1;
			}
		}
	}
	//FIFO already exists
	else {
		remove(filePath);
		if(mkfifo(filePath, S_IRUSR | S_IWUSR) == -1) {
			error = errno;
			printf("Error %d, couldn't create FIFO %s\n", error, filePath);
			return -1;
		};
	}
	return 0;
}


int FIFO_Open(char *filePath, bool mode) {
	int fd;
	int error;
	if(mode) {
		fd = open(filePath, O_WRONLY);
	} else {
		fd = open(filePath, O_RDONLY);
	}
	
	if(fd == -1) {
		error = errno;
		printf("Error %d, couldn't open FIFO %s\n", error, filePath);
		return -1;
	}
	return fd;
}
