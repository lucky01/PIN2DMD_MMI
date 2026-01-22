#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/time.h>
#include "pin2mmi.h"

int32_t millis( void ) {
	struct timeval tv;
	gettimeofday( &tv, NULL );
	return (int32_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

// dump display data to stdout
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

int main( void ) {
	uint8_t* txbuffer;
	uint8_t* rxbuffer;
		
	uint8_t frameBuffer[4096];

	txbuffer = (uint8_t*)malloc( TX_BUFFER_SIZE );
	rxbuffer = (uint8_t*)malloc( RX_BUFFER_SIZE );
	
	memset( txbuffer, 0x00, TX_BUFFER_SIZE );

	init(SPI_NORMAL);
	
	printf( "device mode %d \n", deviceMode );
	printf( "number of planes %d \n", numberOfPlanes );
	printf( "version %s \n", versionString );
	printf( "UID %s \n", UIDString );
	printf( "SPI mode %d \n", Mmi_Status );

	while( 1 ) {

		memcpy (txbuffer, cmd[DUMP], HEADER_SIZE);
		txbuffer[3] = numberOfPlanes;

		// Write to UART
		sendUart(txbuffer, TX_BUFFER_SIZE );
		readUart(rxbuffer, (numberOfPlanes * planesize) ); //uart on pi is not fast enough ready to receive all bytes so use only as trigger here
		readSpi(rxbuffer, (numberOfPlanes * planesize));	
		create_FrameBuffer( 128, 32, deviceMode, rxbuffer, (uint8_t*) frameBuffer);

        dump(frameBuffer);
	}
	
cleanup:
	deInit();
	free( rxbuffer );
	free( txbuffer );

	return 0;
}
