#include <fcntl.h>
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

int32_t millis( void ) {
	struct timeval tv;
	gettimeofday( &tv, NULL );
	return (int32_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
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
		
	uint8_t frameBuffer[4096];

	txbuffer = (uint8_t*)malloc( (4 * planesize) + HEADER_SIZE);
	
	memset( txbuffer, 0x00, TX_BUFFER_SIZE );
	
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

	init(SPI_NORMAL);

	frame_t frame;
	uint32_t frame_offset = 0;
	if( frame_reader && frame_reader_read_next( frame_reader, &frame, 0 ) == 0 ) {
		// on first frame, set offset
		frame_offset = millis() - frame.timestamp;
		LOGDEBUG( "frame offset: 0x%x", frame_offset );
	}
	
	int skipFrames = 0;

	while( 1 ) {
		no_update:	
		uint32_t now = millis();

		int c = nonblock_getchar();
			if( c == 'd' )
				skipFrames += 100;
			
		if( frame_reader_has_more( frame_reader ) ) {
			if( now < ( frame_offset + frame.timestamp ) ) {
					// not yet time for next frame
					goto no_update;
			}
			LOGTRACE( "frame reader has more frames" );
			if( frame_reader_read_next( frame_reader, &frame, skipFrames ) != 0 ) {
				LOGERROR( "frame reader failed to read next frame" );
				break;
			} else {
				if( skipFrames ) {
					frame_offset = millis() - frame.timestamp;
					skipFrames = 0;
				}
				LOGTRACE( "read frame. now: 0x%x", now );
				LOGTRACE( "frame timestamp: 0x%x", frame.timestamp );
			}
		} else {
			LOGTRACE( "frame reader reached end of frames, exiting" );
			break;
		}
		
		memcpy( frameBuffer, frame.data, FRAME_SIZE );
		memcpy (txbuffer, cmd[PLANES_4], HEADER_SIZE);		

		int byteIdx = 0;
		bool shades16 = true;		
		
		for (int j = 0; j < 32; ++j)
			{
				for (int i = 0; i < 128; i += 8)
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
	
cleanup:
	deInit();
	free( txbuffer );

	return 0;
}
