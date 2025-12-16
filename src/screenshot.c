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
	printf( "Usage: screenshot [options] filename\n" );
	printf( "Options:\n" );
	printf( "  -l <loglevel>\t\t\t Set log level (0-5)\n" );
}

uint8_t ppm128[14] {'P','6',0x0a,'1','2','8',' ','3','2',0x0a,'2','5','5',0x0a};
uint8_t ppm256[14] {'P','6',0x0a,'2','5','6',' ','6','4',0x0a,'2','5','5',0x0a};

int main( int argc, char** argv ) {
	
	FILE* file_fd;
	
	uint8_t* displayBuffer = NULL;
	rgb24* rgbBuffer = NULL;
	
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

	init(SPI_RGB);
	
	displayBuffer = (uint8_t*)malloc( DISPLAYBUFFER_SIZE );
	rgbBuffer =  (rgb24*)malloc( DISPLAYBUFFER_SIZE );

	readSpi(displayBuffer, DISPLAYBUFFER_SIZE);
	
	if (deviceType == PIN2DMD_HD)
		create_RGBHDFromFrame(256, 64, displayBuffer, (uint8_t*)rgbBuffer);
	else
		create_RGBFromFrame(128, 32, displayBuffer, (uint8_t*)rgbBuffer);
	
	const char* filename = (const char*)argv[optind];
	file_fd = fopen(filename, "wb");
	if (file_fd == NULL) {
		LOGERROR( "Failed to open file" );
		exit( 1 );
	}

	if (deviceType == PIN2DMD_HD)
		fwrite(ppm256, 1, 14, file_fd);
	else
		fwrite(ppm128, 1, 14, file_fd);
	fwrite(rgbBuffer, 1, DISPLAYBUFFER_SIZE, file_fd);
	fclose(file_fd);
	

cleanup:
	free(displayBuffer);
	free (rgbBuffer);
	deInit();

	return 0;
}
