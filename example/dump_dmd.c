#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/time.h>
#include <ctime>
#include <getopt.h>
#include <gpiod.h>
#include "pin2mmi.h"
#include "log.h"

// Make sure we can exit gracefully when Ctrl-C is pressed.
volatile bool interrupt_received = false;
static void InterruptHandler(int signo) {
  interrupt_received = true;
}

static struct option long_options[] = {
	{ "path", required_argument, 0, 'p' },
    { "log-level", required_argument, 0, 'l' },
	{ "screen", no_argument, 0, 's' },
    { "help", no_argument, 0, '?' },

    { 0, 0, 0, 0 } // End marker
};

void usage() {
	printf( "Usage: dump_dmd [options]\n" );
	printf( "Options:\n" );
	printf( "  -p <path>\t\t\t Path to the dump folder\n" );
	printf( "  -s <screen>\t\t\t dump to screen\n" );
	printf( "  -l <loglevel>\t\t\t Set log level (0-5)\n" );
}


int main( int argc, char** argv ) {
	
	struct timespec next_time;
	next_time.tv_sec = time(NULL);
	next_time.tv_nsec = 0;
	struct tm tm;
	
	char fileName [35];
	char filePath [1024] = {};
	char filePathRaw [1024]  = {};
	
	uint8_t* rxbuffer;
		
	uint8_t frameBuffer[16384];

	rxbuffer = (uint8_t*)malloc( RX_BUFFER_SIZE );
	
	signal(SIGTERM, InterruptHandler);
	signal(SIGINT, InterruptHandler);

	if( argc < 2 ) {
		usage();
		exit( 1 );
	}
	
	int opt = 0;
	int longOptIndex = 0;
	bool dumpscreen = false;
	
	while( ( opt = getopt_long( argc, argv, "p:l:s?", long_options, &longOptIndex ) ) != -1 ) {
		switch( opt ) {
		case 'p':
			strcpy(filePath, optarg);
		break;
		case 's':
			dumpscreen = true;
		break;
		case 'l':
			setLogLevel( atoi( optarg ) );
			break;
		case '?':
			usage();
			exit( 0 );
			break;
		default:
			LOGERROR( "Unknown option: %c", opt );
			exit( 1 );
		}
	}
	
	if( argc - optind != 0 || (filePath[0] == 0x00 && dumpscreen == false)) {
		usage();
		exit( 1 );
	}
	
	init(SPI_DUMP);

	if (filePath[0] != 0x00) {
		next_time.tv_sec = time(NULL);
		localtime_r(&next_time.tv_sec, &tm);
		strftime(fileName, sizeof(fileName), "/%d%m%y_%H%M%S_pin2dmd_dump", &tm);
		strcat(filePath,fileName);
		strcpy(filePathRaw, filePath);
		strcat(filePath, ".txt");
		strcat(filePathRaw, ".raw");
		
		dump_file_init(filePath, filePathRaw);
	}
	
	int width = 128;
	int height = 32;
	if (planesize == 2048) {
		width = 256;
		height = 64;
	} else if (planesize == 1536) {
		width = 192;
		height = 64;
	}
	
	while( !interrupt_received ) {

		gpiod_line_request_set_value(gpio2_request, GPIO2, GPIOD_LINE_VALUE_ACTIVE);// get next frame
		while (gpiod_line_request_get_value(gpio1_request, GPIO1) == GPIOD_LINE_VALUE_ACTIVE){};	// wait until next frame is ready
		readSpi(rxbuffer, (numberOfPlanes * planesize));	
		gpiod_line_request_set_value(gpio2_request, GPIO2, GPIOD_LINE_VALUE_INACTIVE);// get next frame

		uint32_t tick = millis();
		create_FrameBuffer( width, height, deviceMode, rxbuffer, (uint8_t*) frameBuffer);

		if (filePath[0] != 0x00)
			dump_file(filePath, filePathRaw, tick, frameBuffer, rxbuffer);
		if (dumpscreen)
			dump_screen(tick, frameBuffer);
		
	}
	
	deInit();
	free( rxbuffer );

	return 0;
}
