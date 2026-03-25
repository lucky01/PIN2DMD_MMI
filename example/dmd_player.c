#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <sys/time.h>
#include <unistd.h>

#include "frame_reader.h"
#include "log.h"
#include "pin2mmi.h"
#include <getopt.h>

// Make sure we can exit gracefully when Ctrl-C is pressed.
volatile bool interrupt_received = false;
static void InterruptHandler(int signo) {
  interrupt_received = true;
}

struct termios orig_termios;

void set_nonblock_mode( struct termios* orig_termios ) {
	struct termios new_termios;
	tcgetattr( STDIN_FILENO, orig_termios );
	new_termios = *orig_termios;
	new_termios.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &new_termios );
	fcntl( STDIN_FILENO, F_SETFL, O_NONBLOCK );
}

// Restore terminal to original settings
void restore_mode( struct termios* orig_termios ) { tcsetattr( STDIN_FILENO, TCSANOW, orig_termios ); }

// Returns the character pressed, or -1 if none
int nonblock_getchar() {
	unsigned char ch;
	if( read( STDIN_FILENO, &ch, 1 ) == 1 )
		return ch;
	return -1;
}

void restore() { restore_mode( &orig_termios ); }

static struct option long_options[] = {
    { "help", no_argument, 0, '?' },
    { "log-level", required_argument, 0, 'l' },

    { 0, 0, 0, 0 } // End marker
};

void usage() {
	printf( "Usage: dmd_player [options] dumpfile\n" );
	printf( "Options:\n" );
	printf( "  -l <loglevel>\t\t\t Set log level (0-5)\n" );
}

int main( int argc, char** argv ) {
	uint8_t* txbuffer;
	txbuffer = (uint8_t*)malloc( (4 * planesize) + HEADER_SIZE);	
	
	uint8_t* frameBuffer = NULL;

	memset( txbuffer, 0x00, TX_BUFFER_SIZE );

  	signal(SIGTERM, InterruptHandler);
  	signal(SIGINT, InterruptHandler);
	
	setLogDevice( LOG_DEVICE_STDOUT );

	set_nonblock_mode( &orig_termios );
	atexit( restore );
	
	if( argc < 2 ) {
		usage();
		exit( 1 );
	}
	
	int opt = 0;
	int longOptIndex = 0;
	
	frame_reader_t* frame_reader = (frame_reader_t*)NULL;
	
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

	const char* filename = (const char*)argv[optind];
	
	frame_reader = (frame_reader_t*)malloc( sizeof( frame_reader_t ) );
	frame_reader_init( frame_reader, filename );
	
	frameBuffer = (uint8_t*)malloc(dump_width*dump_height);
	free(txbuffer);
	txbuffer = (uint8_t*)malloc( (4 * (dump_width*dump_height/8)) + HEADER_SIZE);
	
	LOGTRACE("dump loaded with width %d height %d\n",dump_width, dump_height);

	init(SPI_NORMAL);
	
	frame_t frame;
	frame.data = (uint8_t*) malloc(dump_width*dump_height);
	uint32_t frame_time = 0;
	if( frame_reader && frame_reader_read_next( frame_reader, &frame, 0 ) == 0 ) {
	}
	
	int skipFrames = 0;

	while( !interrupt_received ) {
		no_update:	
		uint32_t now = millis();

		int c = nonblock_getchar();
			if( c == 'd' )
				skipFrames += 100;
			
		if( frame_reader_has_more( frame_reader ) ) {
			if( now < frame_time ) {
					// not yet time for next frame
					goto no_update;
			}
			LOGTRACE( "frame reader has more frames" );
			if( frame_reader_read_next( frame_reader, &frame, skipFrames ) != 0 ) {
				LOGERROR( "frame reader failed to read next frame" );
				break;
			} else {
				frame_time = millis() + frame.delay;
				if( skipFrames ) {
					skipFrames = 0;
				}
				LOGTRACE( "read frame now: 0x%x", now );
				LOGTRACE( "frame time: 0x%x", frame_time );
			}
		} else {
			LOGTRACE( "frame reader reached end of frames, exiting" );
			break;
		}
		
		memcpy( frameBuffer, frame.data, (dump_width*dump_height) );

		int byteIdx = 0;
		bool shades16 = true;		
		
		if ( deviceType == PIN2DMD_HD && dump_width == 256 && dump_height == 64){
			memcpy (txbuffer, cmd[PLANES_8], HEADER_SIZE);	
			for (int j = 0; j < dump_height; ++j)
				{
					for (int i = 0; i < dump_width; i += 8)
					{
						int bd0, bd1;
						bd0 = 0;
						bd1 = 0;
						for (int v = 7; v >= 0; v--)
						{
							// pixel colour
							int pixel = frameBuffer[j * dump_width + i + v];

							bd0 <<= 1;
							bd1 <<= 1;

							if (pixel & 1)
								bd0 |= 1;
							if (pixel & 2)
								bd1 |= 1;
						}

						txbuffer[byteIdx + HEADER_SIZE] = bd0;
						txbuffer[byteIdx + (dump_width*dump_height/8) + HEADER_SIZE] = bd1;
						byteIdx++;
					}
				}

			// Write to UART
			sendUart(txbuffer, (2 * (dump_width*dump_height/8)) + HEADER_SIZE );
		} else if ( deviceType == PIN2DMD_XL && dump_width == 192 && dump_height == 64){
			memcpy (txbuffer, cmd[PLANES_6], HEADER_SIZE);	
			for (int j = 0; j < dump_height; ++j)
				{
					for (int i = 0; i < dump_width; i += 8)
					{
						int bd0, bd1;
						bd0 = 0;
						bd1 = 0;
						for (int v = 7; v >= 0; v--)
						{
							// pixel colour
							int pixel = frameBuffer[j * dump_width + i + v];

							bd0 <<= 1;
							bd1 <<= 1;

							if (pixel & 1)
								bd0 |= 1;
							if (pixel & 2)
								bd1 |= 1;
						}

						txbuffer[byteIdx + HEADER_SIZE] = bd0;
						txbuffer[byteIdx + (dump_width*dump_height/8) + HEADER_SIZE] = bd1;
						byteIdx++;
					}
				}

			// Write to UART
			sendUart(txbuffer, (2 * (dump_width*dump_height/8)) + HEADER_SIZE );
		} else if ( dump_width == 128 && dump_height == 32) {
			memcpy (txbuffer, cmd[PLANES_4], HEADER_SIZE);		
			for (int j = 0; j < dump_height; ++j)
				{
					for (int i = 0; i < dump_width; i += 8)
					{
						int bd0, bd1, bd2, bd3;
						bd0 = 0;
						bd1 = 0;
						bd2 = 0;
						bd3 = 0;
						for (int v = 7; v >= 0; v--)
						{
							// pixel colour
							int pixel = frameBuffer[j * 128 + i + v];

							bd0 <<= 1;
							bd1 <<= 1;
							bd2 <<= 1;
							bd3 <<= 1;

							if (!shades16)
								if (pixel == 3)
									pixel = 15;
								else if (pixel == 2)
									pixel = 4;

							if (pixel & 1)
								bd0 |= 1;
							if (pixel & 2)
								bd1 |= 1;
							if (pixel & 4)
								bd2 |= 1;
							if (pixel & 8)
								bd3 |= 1;
						}

						txbuffer[byteIdx + HEADER_SIZE] = bd0;
						txbuffer[byteIdx + 512 + HEADER_SIZE] = bd1;
						txbuffer[byteIdx + 1024 + HEADER_SIZE] = bd2;
						txbuffer[byteIdx + 1536 + HEADER_SIZE] = bd3;
						byteIdx++;
					}
				}

			// Write to UART
			sendUart(txbuffer, (4 * planesize) + HEADER_SIZE );
		}
	}
	
cleanup:
	deInit();
	free( txbuffer );

	return 0;
}
