#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <pigpio.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#include "frame_reader.h"
#include "log.h"
#include "pin2mmi.h"
#include "serum-decode.h"
#include <getopt.h>

// 6-bit CIE 1931 lookup table
// Maps 8-bit input (0-255) to 6-bit output (0-63)
static const uint8_t lumConvTab_6bit[256] = {
      0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
      0,     0,     0,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
      1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     2,     2,     2,
      2,     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,     3,     3,     3,
      3,     3,     3,     3,     3,     3,     3,     3,     3,     4,     4,     4,     4,     4,     4,     4,
      4,     4,     5,     5,     5,     5,     5,     5,     5,     5,     5,     6,     6,     6,     6,     6,
      6,     6,     7,     7,     7,     7,     7,     7,     7,     8,     8,     8,     8,     8,     8,     9,
      9,     9,     9,     9,     9,    10,    10,    10,    10,    10,    11,    11,    11,    11,    11,    12,
     12,    12,    12,    12,    13,    13,    13,    13,    13,    14,    14,    14,    14,    15,    15,    15,
     15,    16,    16,    16,    16,    17,    17,    17,    17,    18,    18,    18,    19,    19,    19,    19,
     20,    20,    20,    21,    21,    21,    22,    22,    22,    22,    23,    23,    23,    24,    24,    24,
     25,    25,    25,    26,    26,    27,    27,    27,    28,    28,    28,    29,    29,    30,    30,    30,
     31,    31,    32,    32,    32,    33,    33,    34,    34,    34,    35,    35,    36,    36,    37,    37,
     37,    38,    38,    39,    39,    40,    40,    41,    41,    42,    42,    43,    43,    44,    44,    45,
     45,    46,    46,    47,    47,    48,    48,    49,    49,    50,    51,    51,    52,    52,    53,    53,
     54,    54,    55,    56,    56,    57,    57,    58,    59,    59,    60,    60,    61,    62,    62,    63,
};

uint32_t fWidth = 128, fHeight = 32;

int32_t millis( void ) {
	struct timeval tv;
	gettimeofday( &tv, NULL );
	return (int32_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

typedef struct v1rotations {
	unsigned char firstColor;
	unsigned char numberOfColors;
	unsigned char time;
} v1rotations;

#define MAX_COLOR_ROTATIONS 8

uint32_t updateV1Rotations(v1rotations* rot, rgb24* palette)
{
	uint32_t now = millis();
	static uint32_t nextRot[8]={};
	uint32_t nextRotation = now + 0xff*10;
	rgb24 tmpColor;
	
	for (int i = 0; i < MAX_COLOR_ROTATIONS; i++)
	{
		if (rot[i].firstColor == 255)
		  continue;

		if (now >= nextRot[i])
		{
			memcpy(&tmpColor, &palette[rot[i].firstColor], 3);
			memmove(&palette[rot[i].firstColor], &palette[rot[i].firstColor + 1], (rot[i].numberOfColors - 1) * 3);
			memcpy(&palette[(rot[i].firstColor + rot[i].numberOfColors - 1)], &tmpColor, 3);
			nextRot[i] = now + rot[i].time * 10;
			if (nextRot [i] < nextRotation)
			nextRotation = nextRot[i];
		}
	}
	return nextRotation;
}

bool updateDmdV1( uint8_t* frame, rgb24* palette, uint8_t* displayBuffer ) {
	LOGTRACE( "v1: create RGB24 buffer with palette" );
	static uint16_t* scaleBuffer = NULL;
	static rgb24* serumV1RGBbuffer = NULL;
	
	if( serumV1RGBbuffer == NULL )
		serumV1RGBbuffer = (rgb24*)malloc( fWidth * fHeight * 3 );
	for( int jj = 0; jj < fHeight; jj++ ) {
		for( int ii = 0; ii < fWidth; ii++ ) {
			serumV1RGBbuffer[(jj *fWidth)+ii] = palette[frame[(jj *fWidth)+ii]];
		}
	}
	if (deviceType == PIN2DMD_HD){
		// allocate only once and the reuse
		if( scaleBuffer == NULL )
			scaleBuffer = (uint16_t*)malloc( 256 * 64 * 3 );
		scaleHD(256, 64, (uint8_t*) serumV1RGBbuffer, (uint8_t*) scaleBuffer, 3);
		create_FrameFromRGB24HD( 256, 64, (uint8_t*) scaleBuffer, displayBuffer);
	} else {
		displayBuffer = create_FrameFromRGB24( 128, 32, (rgb24*)serumV1RGBbuffer, displayBuffer );
	}
	LOGTRACE( "set display update flag" );
	return true;
}

bool updateDmd( Serum_Frame_Struc* pSerum, int render32, int render64, uint8_t* displayBuffer ) {
	LOGTRACE( "v%i: render32: %i, render64:%i & width32=%i width64=%i create_FrameFromRGB565", pSerum->SerumVersion,
	          render32, render64, pSerum->width32, pSerum->width64 );
	static uint16_t* scaleBuffer = NULL;
	static rgb24* serumV1RGBbuffer = NULL;
	
	if( render32 && pSerum->width32 > 0 && pSerum->width64 == 0 ) {
		LOGTRACE( "v2: render32 & width32=%i width64=%i create_FrameFromRGB565", pSerum->width32, pSerum->width64 );
		if (deviceType == PIN2DMD_HD){
			// allocate only once and the reuse
			if( scaleBuffer == NULL )
				scaleBuffer = (uint16_t*)malloc( 256 * 64 * 2 );
			scaleHD(pSerum->width32 * 2, 64, (uint8_t*) pSerum->frame32, (uint8_t*) scaleBuffer, 2);
			create_FrameFromRGB24HD( pSerum->width32 * 2, 64, (uint8_t*) scaleBuffer, displayBuffer, 1 );
		} else {
			displayBuffer = create_FrameFromRGB565( pSerum->width32, 32, pSerum->frame32, displayBuffer );
		} 
	} else if( render64 && pSerum->width64 > 0 && pSerum->width32 == 0 ) {
		LOGTRACE( "v2: render64 & width32=%i width64=%i create_FrameFromRGB565", pSerum->width32, pSerum->width64 );
		create_FrameFromRGB24HD( pSerum->width64, 64, (uint8_t*)pSerum->frame64, displayBuffer, 1 );
	} else if( pSerum->width32 > 0 && pSerum->width64 > 0 ) {
		LOGTRACE( "v2: HD render64: %i, render64:%i & width32=%i width64=%i create_FrameFromRGB565", render32, render64,
				  pSerum->width32, pSerum->width64 );
		// hd case see https://github.com/PPUC/libdmdutil/blob/master/src/DMD.cpp#L1143
		create_FrameFromRGB24HD( pSerum->width64, 64, (uint8_t*)pSerum->frame64, displayBuffer, 1 );
	}
	LOGTRACE( "set display update flag" );
	return true;
}

void dump( uint8_t* p ) {
	const char* hex_digits = "0123456789ABCDEF";
	int y = 0, x = 0;
	printf("0x%x\n", millis());
	for( y = 0; y < 32; y++ ) {
		for( x = 0; x < 128; x++ ) {
			printf( "%c", hex_digits[p[y * 128 + x]] );
		}
		printf( "\n" );
	}
	printf( "\n" );
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
    { "dump-frames", no_argument, 0, 'd' },
 	{ "reset",  no_argument, 0, 'r' },
    //{ "output", required_argument, 0, 'o' },
    { "log-level", required_argument, 0, 'l' },
    { "read-file", required_argument, 0, 'f' },
    { "ignore-unknown-frames-timeout", required_argument, 0, 'i' },
    { "maximum-unknown-frames-to-skip", required_argument, 0, 'm' },

    { 0, 0, 0, 0 } // End marker
};

void usage() {
	printf( "Usage: spi-serum [options] <altcolor-path> <rom-name>\n" );
	printf( "Options:\n" );
	printf( "  -d                                Dump serum frames as hex to stdout\n" );
	printf( "  -i <IgnoreUnknownFramesTimeout>   Set ignore unknown frames timeout (ms). default dont set\n" );
	printf( "  -m <MaximumUnknownFramesToSkip>   Set maximum unknown frames to skip. default dont set\n" );
	printf( "  -l <loglevel>                     Set log level (0-5)\n" );
	printf( "  -f <dump-file>\t\t\tRead frames from given file instead of SPI input\n" );
	printf( "  -r					Reset PIN2DMD at startup\n" );
}

int main( int argc, char** argv ) {
	
	GAMMA_TABLE = lumConvTab_6bit;
	
	uint8_t* rxbuffer;      // buffer for receiving dmd frame data
	uint8_t* lastrxbuffer;  // buffer for receiving dmd frame data
	uint8_t* displayBuffer; // buffer for sending display frame data
	uint8_t* serumBuffer;   // buffer for serum input data to colorize
	rgb24* rgbbuffer;       // buffer

	int flags = FLAG_REQUEST_32P_FRAMES; // default request 32P frames from serum

	setLogDevice( LOG_DEVICE_STDOUT );

	set_nonblock_mode( &orig_termios );
	atexit( restore );
	if( argc < 3 ) {
		usage();
		exit( 1 );
	}

	int opt = 0;
	int longOptIndex = 0;
	int setIgnoreUnknownFramesTimeout = -1;
	int setMaximumUnknownFramesToSkip = -1;
	int dumpFrames = 0;
	bool withReset = false;

	frame_reader_t* frame_reader = (frame_reader_t*)NULL;

	while( ( opt = getopt_long( argc, argv, "drl:f:i:m:?", long_options, &longOptIndex ) ) != -1 ) {
		switch( opt ) {
		case 'i':
			setIgnoreUnknownFramesTimeout = atoi( optarg );
			break;
		case 'd':
			dumpFrames = 1;
			break;
		case 'm':
			setMaximumUnknownFramesToSkip = atoi( optarg );
			break;
		case 'l':
			setLogLevel( atoi( optarg ) );
			break;
		case 'r':
			withReset = true;
			break;
		case 'f': {
			frame_reader = (frame_reader_t*)malloc( sizeof( frame_reader_t ) );
			frame_reader_init( frame_reader, optarg );
		} break;
		case '?':
			usage();
			exit( 0 );
			break;
		default:
			LOGERROR( "Unknown option: %c", opt );
			exit( 1 );
		}
	}


	if( argc - optind < 2 ) {
		LOGERROR( "Not enough arguments provided" );
		usage();
		exit( 1 );
	}

	init( SPI_EXTERNAL_RGB, withReset );
	
	rxbuffer = (uint8_t*)malloc( RX_BUFFER_SIZE );
	lastrxbuffer = (uint8_t*)malloc( RX_BUFFER_SIZE );
	displayBuffer = (uint8_t*)malloc( DISPLAYBUFFER_SIZE );
	serumBuffer = (uint8_t*)malloc( fWidth * fHeight );
	rgbbuffer = (rgb24*)malloc( WIDTH * HEIGHT );

	const char* path = (const char*)argv[optind];
	const char* rom = (const char*)argv[optind + 1];

	// TODO maybe set other flags here
	LOGDEBUG( "Start loading serum file %s", rom );
	
	if (deviceType == PIN2DMD_HD){
		flags = FLAG_REQUEST_64P_FRAMES | FLAG_REQUEST_32P_FRAMES;
		LOGDEBUG( "Serum HD flags set");
	}
	
	Serum_Frame_Struc* pSerum = Serum_Load( path, rom, flags );
	if( !pSerum ) {
		LOGERROR( "Failed to load Serum: path=%s, rom=%s", path, rom );
		Serum_Dispose();
		exit( 1 );
	}
	LOGDEBUG( "Serum Version: %i", pSerum->SerumVersion );
	LOGDEBUG( "Serum nocolors: %i", pSerum->nocolors );
	LOGDEBUG( "Serum ntriggers: %i", pSerum->ntriggers );

	if( setIgnoreUnknownFramesTimeout != -1 )
		Serum_SetIgnoreUnknownFramesTimeout( setIgnoreUnknownFramesTimeout );
	if( setMaximumUnknownFramesToSkip != -1 )
		Serum_SetMaximumUnknownFramesToSkip( setMaximumUnknownFramesToSkip );

	uint32_t nextRotation = 0; // timestamp in millis for next color rotation or 0 if none
	uint32_t result;
	int skipFrames = 0;

	LOGDEBUG( "Starting main loop" );
	frame_t frame;
	uint32_t frame_offset = 0;
	if( frame_reader && frame_reader_read_next( frame_reader, &frame, 0 ) == 0 ) {
		// on first frame, set offset
		frame_offset = millis() - frame.timestamp;
		LOGDEBUG( "frame offset: 0x%x", frame_offset );
	}
	
	bool displayUpdate = true;
	bool newFrame = false;
	rgb24* v1palette = NULL;

	while( 1 ) {
		uint32_t now = millis();
		if( frame_reader ) {
			int c = nonblock_getchar();
			if( c == 'd' )
				skipFrames += 100;

			// LOGTRACE( "read frame from frame reader 0x%x", now > (frame_offset + frame.timestamp) );
			if(now > nextRotation > 0 && nextRotation != 0 ) {
				if( pSerum->SerumVersion == SERUM_V1 ) {
					displayUpdate =
						updateDmdV1( pSerum->frame, v1palette, displayBuffer );
					nextRotation = updateV1Rotations((v1rotations*)pSerum->rotations, v1palette);
				} else if ( pSerum->SerumVersion == SERUM_V2 ) {
					uint32_t result = Serum_Rotate();
					displayUpdate =
						updateDmd( pSerum, flags & FLAG_REQUEST_32P_FRAMES, flags & FLAG_REQUEST_64P_FRAMES, displayBuffer );
					if( result > 0 && ( ( result & 0xffff ) < 2048 ) ) {
						nextRotation = now + ( result & 0xffff );
					} else {
						nextRotation = 0;
					}
				}
			}
			if(displayUpdate) {
				transferSpi( displayBuffer, rxbuffer );
				displayUpdate = false;
			}
			if( frame_reader_has_more( frame_reader ) ) {
				if( now < ( frame_offset + frame.timestamp ) ) {
					// not yet time for next frame
					if( nextRotation )
						continue;
					goto colorize;
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
					memcpy( serumBuffer, frame.data, FRAME_SIZE );
					newFrame = true;
					if( dumpFrames )
						dump( serumBuffer );
				}
			} else {
				LOGTRACE( "frame reader reached end of frames, exiting" );
				break;
			}
		} else {

			gpioWrite( GPIO2, true );
			if (gpioRead( GPIO1 ) == 0) 
				newFrame = true;
			if (newFrame || displayUpdate){
				transferSpi( displayBuffer, rxbuffer );
				displayUpdate = false;
			}
			if( now >= nextRotation && nextRotation != 0 ) {
				if( pSerum->SerumVersion == SERUM_V1 ) {
					displayUpdate =
						updateDmdV1( pSerum->frame, v1palette, displayBuffer );
					nextRotation = updateV1Rotations((v1rotations*)pSerum->rotations, v1palette);
				} else if ( pSerum->SerumVersion == SERUM_V2 ) {
					uint32_t result = Serum_Rotate();
					displayUpdate =
						updateDmd( pSerum, flags & FLAG_REQUEST_32P_FRAMES, flags & FLAG_REQUEST_64P_FRAMES, displayBuffer );
					if( result > 0 && ( ( result & 0xffff ) < 2048 ) ) {
						nextRotation = now + ( result & 0xffff );
					} else {
						nextRotation = 0;
					}
				}
			} 
			if (newFrame){
				gpioWrite( GPIO2, false );
				create_FrameBuffer( 128, 32, deviceMode, rxbuffer, (uint8_t*)serumBuffer );			
				if( dumpFrames )
					dump( serumBuffer );
			}
		}
	colorize:
		// colorize with serum
		if (newFrame){
			newFrame = false;
			result = Serum_Colorize( serumBuffer );
			if( result == IDENTIFY_NO_FRAME ) {
				LOGTRACE( "Serum: no frame detected" );
				continue; // if no new frame detected
			} else if( result == 0 ) { // if new frame with no rotation detected
				LOGTRACE( "Serum: new frame detected, no rotation" );
				if( pSerum->SerumVersion == SERUM_V1 ) {
					displayUpdate =
						updateDmdV1( pSerum->frame, (rgb24*)pSerum->palette, displayBuffer );
				} else if( pSerum->SerumVersion == SERUM_V2 ) {
					displayUpdate =
						updateDmd( pSerum, flags & FLAG_REQUEST_32P_FRAMES, flags & FLAG_REQUEST_64P_FRAMES, displayBuffer );
				} else {
					LOGERROR( "Unknown Serum version: %d", pSerum->SerumVersion );
				}
				nextRotation = 0;
			} else if( result > 0 && ( ( result & 0xffff ) < 2048 ) ) {
				if( pSerum->SerumVersion == SERUM_V1 ) {
					if (v1palette == NULL)
						v1palette = (rgb24*)malloc( 64 * 3 );
					memcpy(v1palette, pSerum->palette, 64 * 3);
					nextRotation = updateV1Rotations((v1rotations*)pSerum->rotations, v1palette);
					displayUpdate =
						updateDmdV1( pSerum->frame, (rgb24*)pSerum->palette, displayBuffer );
				} else if( pSerum->SerumVersion == SERUM_V2 ) {
					nextRotation = now + ( result & 0xffff );
					LOGTRACE( "Serum: new frame detected, next rotation in %d ms", ( result & 0xffff ) );
					displayUpdate =
					updateDmd( pSerum, flags & FLAG_REQUEST_32P_FRAMES, flags & FLAG_REQUEST_64P_FRAMES, displayBuffer );
				} else {
					LOGERROR( "Unknown Serum version: %d", pSerum->SerumVersion );
				}
			} 
		}

	} // end main loop

	Serum_Dispose();
	free( serumBuffer );
	free( displayBuffer );
	free( rxbuffer );

	return 0;
}
