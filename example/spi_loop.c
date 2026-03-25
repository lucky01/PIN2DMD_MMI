#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <gpiod.h>
#include "log.h"
#include "pin2mmi.h"
#include <getopt.h>

static struct option long_options[] = {
    { "help", no_argument, 0, '?' },
    { "log-level", required_argument, 0, 'l' },

    { 0, 0, 0, 0 } // End marker
};

// Make sure we can exit gracefully when Ctrl-C is pressed.
volatile bool interrupt_received = false;
static void InterruptHandler(int signo) {
  interrupt_received = true;
}

void usage() {
	printf( "Usage: spi_loop [options]\n" );
	printf( "Options:\n" );
	printf( "  -l <loglevel>\t\t\t Set log level (0-5)\n" );
}


rgb24 palette[] = { { 0x00,0x00,0x00 }, 
					{ 0x11,0x11,0x11 }, 
					{ 0x22,0x22,0x22 }, 
					{ 0x33,0x33,0x33 }, 
					{ 0x44,0x44,0x44 }, 
					{ 0x55,0x55,0x55 }, 
					{ 0x66,0x66,0x66 }, 
					{ 0x77,0x77,0x77 },
					{ 0x88,0x88,0x88 },
					{ 0x99,0x99,0x99 },
					{ 0xAA,0xAA,0xAA },
					{ 0xBB,0xBB,0xBB },
					{ 0xCC,0xCC,0xCC },
					{ 0xDD,0xDD,0xDD },
					{ 0xEE,0xEE,0xEE },
					{ 0xFF,0xFF,0xFF },
				};

uint16_t palette565[] = {0x0000,
						 0x1082,
						 0x2104,
						 0x3186,
						 0x4228,
						 0x52AA,
						 0x632c,
						 0x73ae,
						 0x8c51,
						 0x9cd3,
						 0xad55,
						 0xbdd7,
						 0xce79,
						 0xdefb,
						 0xef7d,
						 0xffff
						 };
	

int main( int argc, char** argv ) {

	uint8_t* rxbuffer;
	uint8_t* displayBuffer;

	rgb24 rgbbuffer[4096];
	rgb24 rgbbufferHD[16384];
	uint16_t rgb565buffer[4096];
	uint16_t rgb565bufferHD[16384];
	
	setLogDevice( LOG_DEVICE_STDOUT );

	signal(SIGTERM, InterruptHandler);
  	signal(SIGINT, InterruptHandler);
	
	int opt = 0;
	int longOptIndex = 0;
	
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

	init(SPI_EXTERNAL_RGB);
	
	rxbuffer = (uint8_t*)malloc( RX_BUFFER_SIZE );
	displayBuffer = (uint8_t*)malloc( DISPLAYBUFFER_SIZE );

	memset(displayBuffer,0,DISPLAYBUFFER_SIZE);

	while( !interrupt_received ) {

		gpiod_line_request_set_value(gpio2_request, GPIO2, GPIOD_LINE_VALUE_ACTIVE);// get next frame

		while (gpiod_line_request_get_value(gpio1_request, GPIO1) == GPIOD_LINE_VALUE_ACTIVE){};	// wait until next frame is ready
		
		transferSpi(displayBuffer, rxbuffer);

		gpiod_line_request_set_value(gpio2_request, GPIO2, GPIOD_LINE_VALUE_INACTIVE);// get next frame

		if (deviceType == PIN2DMD_HD){
			// PIN2DMD EVO256x64
			// use RGB565 for display rendering
			if (planesize == 2048) {
				create_FrameBuffer( 256, 64, deviceMode, rxbuffer, (uint8_t*) rgb565bufferHD,(uint8_t*) palette565, true);
				displayBuffer = create_FrameFromRGB565HD( 256, 64, rgb565bufferHD, displayBuffer );
			} else {
				create_FrameBuffer( 128, 32, deviceMode, rxbuffer, (uint8_t*) rgb565buffer,(uint8_t*) palette565, true);
				scaleHD(256, 64, (uint8_t*) rgb565buffer, (uint8_t*) rgb565bufferHD, 2);
				displayBuffer = create_FrameFromRGB565HD( 256, 64, rgb565bufferHD, displayBuffer );

				// use RGB24 for display rendering
				/*create_FrameBuffer( 128, 32, deviceMode, rxbuffer, (uint8_t*) rgbbuffer,(uint8_t*) palette);
				scaleHD(256, 64, (uint8_t*) rgbbuffer, (uint8_t*) rgbbufferHD, 3);
				displayBuffer = create_FrameFromRGB24HD( 256, 64, (uint8_t*)rgbbufferHD, displayBuffer );*/
			}
			
		} else if (deviceType == PIN2DMD_XL) {
			// PIN2DMD EVO192x64
			create_FrameBuffer( 192, 64, deviceMode, rxbuffer, (uint8_t*) rgbbufferHD,(uint8_t*) palette);
			displayBuffer = create_FrameFromRGB24( 192, 64, rgbbufferHD, displayBuffer );
		} else {
			// PIN2DMD EVO128x32
			create_FrameBuffer( 128, 32, deviceMode, rxbuffer, (uint8_t*) rgbbuffer,(uint8_t*) palette);
			displayBuffer = create_FrameFromRGB24( 128, 32, rgbbuffer, displayBuffer );
		}

		transferSpi(displayBuffer, rxbuffer);

	}
	
cleanup:
	deInit();
	free( displayBuffer );
	free( rxbuffer );
	return 0;
}
