#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <getopt.h>
#include <gpiod.h>


#include "log.h"
#include "pin2mmi.h"

// Make sure we can exit gracefully when Ctrl-C is pressed.
volatile bool interrupt_received = false;
static void InterruptHandler(int signo) {
  interrupt_received = true;
}

static struct option long_options[] = {
    { "help", no_argument, 0, '?' },
    { "log-level", required_argument, 0, 'l' },
	{ "filename", required_argument, 0, 'f' },
	{ "screenmode", required_argument, 0, 'm' },

    { 0, 0, 0, 0 } // End marker
};

void usage() {
	printf( "Usage: screenmirror [options] filename\n" );
	printf( "Options:\n" );
	printf( "  -m screenmode\t\t\t Set screen mode\n" );
	printf( "  -f <filename>\t\t\t ppm export filename\n" );
	printf( "  -l <loglevel>\t\t\t Set log level (0-5)\n" );
}

uint8_t ppm128[14] {'P','6',0x0a,'1','2','8',' ','3','2',0x0a,'2','5','5',0x0a};
uint8_t ppm192[14] {'P','6',0x0a,'1','9','2',' ','6','4',0x0a,'2','5','5',0x0a};
uint8_t ppm256[14] {'P','6',0x0a,'2','5','6',' ','6','4',0x0a,'2','5','5',0x0a};


int main( int argc, char** argv ) {
	
	FILE* file_fd;
	
	uint8_t* displayBuffer = NULL;
	uint16_t* rgb565Buffer = NULL;
	rgb24* rgbBuffer = NULL;
	int mode = 0;
	
	setLogDevice( LOG_DEVICE_STDOUT );

	if( argc < 1 ) {
		usage();
		exit( 1 );
	}
	
	int opt = 0;
	int longOptIndex = 0;
	int fileSize = 0;
	const char* filename = NULL;
	
	while( ( opt = getopt_long( argc, argv, "m:f:l:?", long_options, &longOptIndex ) ) != -1 ) {
		switch( opt ) {
		case 'f':
			filename = optarg;
			break;
		case 'l':
			setLogLevel( atoi( optarg ) );
			break;
		case 'm':
			mode = atoi( optarg );
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
	
	if( argc - optind != 0 ) {
		LOGERROR( "Not enough arguments provided" );
		usage();
		exit( 1 );
	}
	
	init(SPI_RGB);
	
	displayBuffer = (uint8_t*)malloc( DISPLAYBUFFER_SIZE );
	rgbBuffer = (rgb24*)malloc( DISPLAYBUFFER_SIZE );
	rgb565Buffer =  (uint16_t*) rgbBuffer;
	
	if (filename == NULL){
		struct fb_var_screeninfo vinfo;
		int fb_fd = fb_init(&vinfo);
		fb_clear(fb_fd, vinfo);

		while( !interrupt_received ) {

			gpiod_line_request_set_value(gpio2_request, GPIO2, GPIOD_LINE_VALUE_ACTIVE);// get next frame

			while (gpiod_line_request_get_value(gpio1_request, GPIO1) == GPIOD_LINE_VALUE_ACTIVE){};	// wait until next frame is ready
		
			readSpi(displayBuffer, DISPLAYBUFFER_SIZE);

			gpiod_line_request_set_value(gpio2_request, GPIO2, GPIOD_LINE_VALUE_INACTIVE);// get next frame
			
			int src_w = 128;
			int src_h = 32;

			if (deviceType == PIN2DMD_HD) {
				src_w = 256;
				src_h = 64;
				create_RGBHDFromFrame(256, 64, displayBuffer, (uint8_t*)rgb565Buffer, true);
			} else if (deviceType == PIN2DMD_XL) {
				src_w = 192;
				src_h = 64;
				create_RGBFromFrame(192, 64, displayBuffer, (uint8_t*)rgb565Buffer, true);
			} else {
				create_RGBFromFrame(128, 32, displayBuffer, (uint8_t*)rgb565Buffer, true);
			}
			fb_displayRGB565(fb_fd, vinfo, rgb565Buffer, src_w, src_h, mode);
		}
		
		fb_deInit(fb_fd);
	} else {
		readSpi(displayBuffer, DISPLAYBUFFER_SIZE);

		if (deviceType == PIN2DMD_HD)
			create_RGBHDFromFrame(256, 64, displayBuffer, (uint8_t*)rgbBuffer);
		else if (deviceType == PIN2DMD_XL)
			create_RGBFromFrame(192, 64, displayBuffer, (uint8_t*)rgbBuffer);
		else
			create_RGBFromFrame(128, 32, displayBuffer, (uint8_t*)rgbBuffer);
		
		file_fd = fopen(filename, "wb");
		if (file_fd == NULL) {
			LOGERROR( "Failed to open file" );
			exit( 1 );
		}

		if (deviceType == PIN2DMD_HD)
			fwrite(ppm256, 1, 14, file_fd);
		else if (deviceType == PIN2DMD_XL)
			fwrite(ppm192, 1, 14, file_fd);
		else
			fwrite(ppm128, 1, 14, file_fd);
		fwrite(rgbBuffer, 1, DISPLAYBUFFER_SIZE, file_fd);
		fclose(file_fd);
	}
cleanup:
	
	free(displayBuffer);
	deInit();

	return 0;
}
