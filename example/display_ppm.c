#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "log.h"
#include "pin2mmi.h"
#include <getopt.h>


static struct option long_options[] = {
    { "help", no_argument, 0, '?' },
    { "log-level", required_argument, 0, 'l' },

    { 0, 0, 0, 0 } // End marker
};

void usage() {
	printf( "Usage: display_ppm [options] filename\n" );
	printf( "Options:\n" );
	printf( "  -l <loglevel>\t\t\t Set log level (0-5)\n" );
}

int main( int argc, char** argv ) {
	
	FILE* file_fd;
	
	uint8_t* rgbBuffer = NULL;
	uint8_t* scaleBuffer = NULL;
	uint8_t* displayBuffer = NULL;
	
	setLogDevice( LOG_DEVICE_STDOUT );

	if( argc < 1 ) {
		usage();
		exit( 1 );
	}
	
	int opt = 0;
	int longOptIndex = 0;
	int fileSize = 0;
	
	while( ( opt = getopt_long( argc, argv, "l:?", long_options, &longOptIndex ) ) != -1 ) {
		switch( opt ) {
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
	
	if( argc - optind < 1 ) {
		LOGERROR( "Not enough arguments provided" );
		usage();
		exit( 1 );
	}

	init(SPI_EXTERNAL_RGB);
	
	displayBuffer = (uint8_t*)malloc( DISPLAYBUFFER_SIZE );
	
	const char* filename = (const char*)argv[optind];
	file_fd = fopen(filename, "rb");
	if (file_fd == NULL) {
		LOGERROR( "Failed to open file" );
		exit( 1 );
	}

	// Determine the file size
	fseek(file_fd, 0, SEEK_END);
	fileSize = ftell(file_fd);
	if (fileSize != 12302 && fileSize != 49166) {
		LOGERROR( "Wrong filesize" );
		exit( 1 );
	}
	rewind(file_fd);
	
	//skip ppm header and read pixel data
	fseek(file_fd, 14, SEEK_SET);
	rgbBuffer = (uint8_t*)malloc( fileSize-14 );
	fread(rgbBuffer, 1, fileSize-14, file_fd);
	fclose(file_fd);
	
	if (fileSize == 12302 && deviceType == PIN2DMD_HD) {
		scaleBuffer = (uint8_t*)malloc( DISPLAYBUFFER_SIZE );
		scaleHD(256, 64, rgbBuffer, scaleBuffer, 3);
		create_FrameFromRGB24HD( 256, 64, scaleBuffer, displayBuffer);
	}
	else if (fileSize == 49166 && deviceType == PIN2DMD_HD) {
		create_FrameFromRGB24HD( 256, 64, rgbBuffer, displayBuffer);
	} else {
		create_FrameFromRGB24( 128, 32, (rgb24*) rgbBuffer, displayBuffer);
	}
	
	transferSpi(displayBuffer, NULL);

cleanup:
	free(displayBuffer);
	free(scaleBuffer);
	free(rgbBuffer);
	deInit();

	return 0;
}
