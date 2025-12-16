#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <sys/time.h>
#include <unistd.h>

#include "log.h"
#include "pin2mmi.h"

int main( int argc, char** argv ) {

	init(SPI_NORMAL,true);
	deInit();
	return 0;
}
