#include "IPC.h"
#include <stdbool.h>

int main() {
	char str[] = "/tmp/test";
	//FIFO_Create(str);
	int fd = FIFO_Open(str, 0);
	return 0;
}
