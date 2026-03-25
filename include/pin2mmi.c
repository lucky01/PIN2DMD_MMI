#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <linux/fb.h>
#include <sys/mman.h>

#include "log.h"
#include "pin2mmi.h"

int uart_fd, spi_fd;
configDescriptor config;
char versionString[6] = {};
char UIDString[16] = {};
uint8_t rgbSequence = 0;
DeviceMode deviceMode = VirtualPinball;
DeviceType deviceType = PIN2DMD;
uint8_t numberOfPlanes = 0;
int planesize = 512;
bool rawDump = false;
MMI_STATUS_t Mmi_Status = MMI_SPI_OFF;

// gpiod v2.x variables
struct gpiod_chip *chip = NULL;
struct gpiod_line_request *gpio1_request = NULL;
struct gpiod_line_request *gpio2_request = NULL;

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

const uint8_t GAMMA_LUT[256] = {
    0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,
    2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,
    4,  4,  4,  4,  5,  5,  5,  5,  5,  5,  5,  5,  6,  6,  6,  6,  6,  6,  6,  7,  7,  7,  7,  7,  7,  7,  8,  8,  8,
    8,  8,  9,  9,  9,  9,  9,  9,  10, 10, 10, 10, 11, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 13, 14, 14, 14,
    14, 15, 15, 15, 16, 16, 16, 16, 17, 17, 17, 18, 18, 18, 18, 19, 19, 19, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23,
    23, 24, 24, 24, 25, 25, 25, 26, 26, 27, 27, 27, 28, 28, 29, 29, 29, 30, 30, 31, 31, 31, 32, 32, 33, 33, 34, 34, 35,
    35, 35, 36, 36, 37, 37, 38, 38, 39, 39, 40, 40, 41, 41, 42, 42, 43, 43, 44, 44, 45, 45, 46, 47, 47, 48, 48, 49, 49,
    50, 50, 51, 52, 52, 53, 53, 54, 55, 55, 56, 56, 57, 58, 58, 59, 60, 60, 61, 62, 62, 63, 63, 63 };
	
const uint8_t REVERSE_LUT_8bit[64] = {
      0,   8,  54,  71,  82,  91,  99, 106, 113, 118, 124, 128,
    133, 137, 142, 146, 149, 153, 156, 160, 163, 166, 169, 172,
    175, 178, 181, 183, 186, 188, 191, 193, 196, 198, 200, 202,
    205, 207, 209, 211, 213, 215, 217, 219, 221, 223, 225, 226,
    228, 230, 232, 234, 235, 237, 239, 240, 242, 244, 245, 247,
    248, 250, 251, 255
};

const uint8_t REVERSE_LUT_6bit[64] = {
    0,  10, 15, 18, 20, 22, 24, 26, 27, 29, 30, 31, 32, 33, 35, 35, // 0-15
    36, 37, 38, 39, 40, 41, 41, 42, 43, 44, 44, 45, 46, 46, 47, 48, // 16-31
    48, 49, 49, 50, 51, 51, 52, 52, 53, 53, 54, 54, 55, 55, 56, 56, // 32-47
    57, 57, 58, 58, 59, 59, 60, 60, 60, 61, 61, 62, 62, 62, 63, 63  // 48-63
};

const uint8_t* GAMMA_TABLE = lumConvTab_6bit;

uint8_t cmd[][4] = {
    { 0x02, 0xc3, 0xe8, 0x00 }, // dump with tick
    { 0x01, 0xc3, 0xe8, 0x00 }, // dump
    { 0x81, 0xc3, 0xe7, 0xff }, // config mode
    { 0x01, 0xc3, 0xe7, 0x00 }, // switch palette
    { 0x81, 0xc3, 0xe7, 0x00 }, // 4 planes
    { 0x81, 0xc3, 0xe8, 0x06 }, // 6 planes
    { 0x81, 0xc3, 0xe8, 0x08 }, // 8 planes
    { 0x81, 0xc3, 0xe8, 0x00 }, // n planes
    { 0x81, 0xc3, 0xe9, 0x00 }, // RGB24
    { 0x52, 0x80, 0x20, 0x00 }, // RGB666
    { 'm', 'm', 'i', 0x00 },    // MMI_SPI_NORMAL
    { 'm', 'm', 'i', 0x01 },    // MMI_SPI_RGB
    { 'm', 'm', 'i', 0x02 },    // MMI_SPI_EXTERNAL_RGB
    { 'm', 'm', 'i', 0x03 },    // MMI_SPI_DUMP
};


int setupGpio() {
	LOGDEBUG( "setup GPIO" );
	// Open GPIO chip
	chip = gpiod_chip_open("/dev/gpiochip0");
	if (!chip)
		return 1;
	
	// Configure GPIO1 as input with pull-down (libgpiod v2.x)
	struct gpiod_line_settings *settings1;
	struct gpiod_line_config *line_cfg1;
	struct gpiod_request_config *req_cfg1;
	unsigned int offset1 = GPIO1;
	
	settings1 = gpiod_line_settings_new();
	if (!settings1) {
		gpiod_chip_close(chip);
		return 1;
	}
	
	gpiod_line_settings_set_direction(settings1, GPIOD_LINE_DIRECTION_INPUT);
	gpiod_line_settings_set_bias(settings1, GPIOD_LINE_BIAS_PULL_DOWN);
	
	line_cfg1 = gpiod_line_config_new();
	if (!line_cfg1) {
		gpiod_line_settings_free(settings1);
		gpiod_chip_close(chip);
		return 1;
	}
	
	if (gpiod_line_config_add_line_settings(line_cfg1, &offset1, 1, settings1) < 0) {
		gpiod_line_config_free(line_cfg1);
		gpiod_line_settings_free(settings1);
		gpiod_chip_close(chip);
		return 1;
	}
	
	req_cfg1 = gpiod_request_config_new();
	if (!req_cfg1) {
		gpiod_line_config_free(line_cfg1);
		gpiod_line_settings_free(settings1);
		gpiod_chip_close(chip);
		return 1;
	}
	
	gpiod_request_config_set_consumer(req_cfg1, "pin2mmi");
	
	gpio1_request = gpiod_chip_request_lines(chip, req_cfg1, line_cfg1);
	
	gpiod_request_config_free(req_cfg1);
	gpiod_line_config_free(line_cfg1);
	gpiod_line_settings_free(settings1);
	
	if (!gpio1_request) {
		gpiod_chip_close(chip);
		return 1;
	}
	
	// Configure GPIO2 as output and set to low (libgpiod v2.x)
	struct gpiod_line_settings *settings2;
	struct gpiod_line_config *line_cfg2;
	struct gpiod_request_config *req_cfg2;
	unsigned int offset2 = GPIO2;
	
	settings2 = gpiod_line_settings_new();
	if (!settings2) {
		gpiod_line_request_release(gpio1_request);
		gpiod_chip_close(chip);
		return 1;
	}
	
	gpiod_line_settings_set_direction(settings2, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_output_value(settings2, GPIOD_LINE_VALUE_INACTIVE);
	
	line_cfg2 = gpiod_line_config_new();
	if (!line_cfg2) {
		gpiod_line_settings_free(settings2);
		gpiod_line_request_release(gpio1_request);
		gpiod_chip_close(chip);
		return 1;
	}
	
	if (gpiod_line_config_add_line_settings(line_cfg2, &offset2, 1, settings2) < 0) {
		gpiod_line_config_free(line_cfg2);
		gpiod_line_settings_free(settings2);
		gpiod_line_request_release(gpio1_request);
		gpiod_chip_close(chip);
		return 1;
	}
	
	req_cfg2 = gpiod_request_config_new();
	if (!req_cfg2) {
		gpiod_line_config_free(line_cfg2);
		gpiod_line_settings_free(settings2);
		gpiod_line_request_release(gpio1_request);
		gpiod_chip_close(chip);
		return 1;
	}
	
	gpiod_request_config_set_consumer(req_cfg2, "pin2mmi");
	
	gpio2_request = gpiod_chip_request_lines(chip, req_cfg2, line_cfg2);
	
	gpiod_request_config_free(req_cfg2);
	gpiod_line_config_free(line_cfg2);
	gpiod_line_settings_free(settings2);
	
	if (!gpio2_request) {
		gpiod_line_request_release(gpio1_request);
		gpiod_chip_close(chip);
		return 1;
	}
	
	return 0;
}

void resetUart() {
	// reset UARTRX buffer
	if (!gpio2_request) {
		// GPIO not yet initialized
		return;
	}
	gpiod_line_request_set_value(gpio2_request, GPIO2, GPIOD_LINE_VALUE_ACTIVE);
	usleep( 10000 );
	gpiod_line_request_set_value(gpio2_request, GPIO2, GPIOD_LINE_VALUE_INACTIVE);
	usleep( 10000 );
}

int setupUart() {
	LOGDEBUG( "setup UART" );
	struct termios options;
	// Open UART
	uart_fd = open( UART_DEVICE, O_RDWR | O_NOCTTY );
	if( uart_fd < 0 ) {
		perror( "Failed to open UART" );
		return -1;
	}

	// Configure UART
	tcgetattr( uart_fd, &options );
	cfsetispeed( &options, B1500000 ); // 1500000 baud
	cfsetospeed( &options, B1500000 );

	options.c_cflag |= ( CLOCAL | CREAD );
	options.c_cflag &= ~PARENB; // No parity
	options.c_cflag &= ~CSTOPB; // 1 stop bit
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8; // 8 data bits

	options.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG );
	options.c_iflag &= ~( IXON | IXOFF | IXANY );
	options.c_oflag &= ~OPOST;

	tcsetattr( uart_fd, TCSANOW, &options );
	tcflush(uart_fd, TCIOFLUSH);
	return 0;
}

void sendUart( uint8_t* TransferBuffer, int len ) {
	LOGTRACE( "uart send %d bytes",len);
	resetUart();
	
	size_t total_sent = 0;
	while( total_sent < len ) {
		while( gpiod_line_request_get_value(gpio1_request, GPIO1) == GPIOD_LINE_VALUE_INACTIVE ) {}
		// Perform UART transfer
		int n = write( uart_fd, TransferBuffer + total_sent, TX_BUFFER_SIZE / 2 );
		if( n < 0 ) {
			perror( "UART transfer failed" );
			exit( 1 );
		}
		while( gpiod_line_request_get_value(gpio1_request, GPIO1) == GPIOD_LINE_VALUE_ACTIVE ) {}
		n = write( uart_fd, TransferBuffer + total_sent + ( TX_BUFFER_SIZE / 2 ), TX_BUFFER_SIZE / 2 );
		if( n < 0 ) {
			perror( "UART transfer failed" );
			exit( 1 );
		}
		total_sent += TX_BUFFER_SIZE;
	}
	LOGTRACE( "uart send done");
}

void readUart( uint8_t* TransferBuffer, int len ) {
	LOGTRACE( "uart read %d bytes");
	int total_read = 0;
	while( total_read < len ) {
		int bytes_read = read( uart_fd, TransferBuffer, len );
		total_read += bytes_read;
	}
}

uint32_t speed = 22000000; // 20 MHz
uint8_t bits = 8;

int setupSpi() {
	LOGDEBUG( "setup spi" );
	uint8_t mode = SPI_MODE_0; // CPOL=0, CPHA=0

	// Open SPI device
	spi_fd = open( SPI_DEVICE, O_RDWR );
	if( spi_fd < 0 ) {
		perror( "Failed to open SPI device" );
		return -1;
	}

	// Configure SPI mode
	int ret = ioctl( spi_fd, SPI_IOC_WR_MODE, &mode );
	if( ret < 0 ) {
		perror( "Failed to set SPI mode" );
		return -1;
	}

	// Configure bits per word
	ret = ioctl( spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits );
	if( ret < 0 ) {
		perror( "Failed to set bits per word" );
		return -1;
	}

	// Configure speed
	ret = ioctl( spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed );
	if( ret < 0 ) {
		perror( "Failed to set max speed" );
		return -1;
	}
	return 0;
}

void readSpi( uint8_t* rxBuffer, int len ) {
	LOGTRACE( "spi read of %d bytes", len );
	static uint8_t* spitxbuf = NULL;
	static uint8_t* spirxbuf = NULL;

	// allocate only once and the reuse
	if( spitxbuf == NULL )
		spitxbuf = (uint8_t*)malloc( CHUNK_SIZE );
	if( spirxbuf == NULL )
		spirxbuf = (uint8_t*)malloc( CHUNK_SIZE );

	memset( spitxbuf, 0, CHUNK_SIZE );
	memset( spirxbuf, 0, CHUNK_SIZE );

	// Prepare SPI transfer structure
	struct spi_ioc_transfer transfer;
	transfer.speed_hz = speed;
	transfer.bits_per_word = bits;
	transfer.delay_usecs = 0;
	transfer.len = CHUNK_SIZE;
	transfer.rx_buf = (unsigned long)spirxbuf;
	transfer.tx_buf = (unsigned long)spitxbuf;
	transfer.pad = 0;
	transfer.tx_nbits = 0;
	transfer.rx_nbits = 0;

	size_t total_read = 0;
	int ret = 0;
	while( total_read < DISPLAYBUFFER_SIZE ) {
		// Perform SPI transfer
		ret = ioctl( spi_fd, SPI_IOC_MESSAGE( 1 ), transfer );
		if( ret < 0 ) {
			perror( "SPI transfer failed" );
			exit( 1 );
		}
		if( len > RX_BUFFER_SIZE )
			memcpy( rxBuffer + total_read, spirxbuf, CHUNK_SIZE );
		else
			if (( total_read >= RX_BUFFER_SIZE ) && ( total_read < 2 * RX_BUFFER_SIZE ))
				memcpy( rxBuffer + ( total_read - RX_BUFFER_SIZE ), spirxbuf, len );
		total_read += CHUNK_SIZE;
	}
	LOGTRACE( "spi read done");
}

void transferSpi( uint8_t* txBuffer, uint8_t* rxBuffer ) {
	LOGTRACE( "spi transfer of %d bytes", DISPLAYBUFFER_SIZE );
	static uint8_t* spitxbuf = NULL;
	static uint8_t* spirxbuf = NULL;

	// only allocate once and reuse the buffer
	if( spitxbuf == NULL )
		spitxbuf = (uint8_t*)malloc( CHUNK_SIZE );
	if( spirxbuf == NULL )
		spirxbuf = (uint8_t*)malloc( CHUNK_SIZE );
	memset( spitxbuf, 0, CHUNK_SIZE );
	memset( spirxbuf, 0, CHUNK_SIZE );

	size_t total_received = 0;

	// Prepare SPI transfer structure
	struct spi_ioc_transfer transfer;
	transfer.speed_hz = speed;
	transfer.bits_per_word = bits;
	transfer.delay_usecs = 0;
	transfer.len = CHUNK_SIZE;
	transfer.rx_buf = (unsigned long)spirxbuf;
	transfer.tx_buf = (unsigned long)spitxbuf;
	transfer.pad = 0;
	transfer.tx_nbits = 0;
	transfer.rx_nbits = 0;

	while( total_received < DISPLAYBUFFER_SIZE ) {
		if (txBuffer != NULL){
			memcpy( spitxbuf, txBuffer + total_received, CHUNK_SIZE );
		}

		// Perform SPI transfer
		int ret = ioctl( spi_fd, SPI_IOC_MESSAGE( 1 ), transfer );
		if( ret < 0 ) {
			perror( "SPI transfer failed" );
			exit( 1 );
		}
		if( ( rxBuffer != NULL && total_received >= RX_BUFFER_SIZE ) && ( total_received < 2 * RX_BUFFER_SIZE ) ) {
			memcpy( rxBuffer + ( total_received - RX_BUFFER_SIZE ), spirxbuf, CHUNK_SIZE );
		}
		total_received += CHUNK_SIZE;
	}
	LOGTRACE( "spi transfer done");
}

DeviceMode deviceModeHD[] = {
	VirtualPinball,
	WPC ,
	SAM ,
	WhiteStar ,
	Spike ,
	DataEast ,
	Capcom256 ,
	Inder,
	Sleic,
	HomePin,
	SysXDMD
};

DeviceMode deviceModeXL[] = {
	VirtualPinball,
	SEGA192x64,
	DotMation
};

// use thread to receive UART data since normal UART read misses some bytes at the begining
typedef struct {
    size_t packet_size;
    unsigned char *shared_buffer;
} uart_context_t;

void* uart_receive_thread(void* arg) {
    uart_context_t *ctx = (uart_context_t*)arg;
	size_t received = 0;
	while (received < ctx->packet_size) {
		ssize_t n = read(uart_fd, ctx->shared_buffer + received, ctx->packet_size - received);
		if (n > 0) {
			received += n;
		} 
	}
    return NULL;
}

void getConfig() {

	uint8_t OutputPacketBuffer[2052];
	uint8_t ReceivePacketBuffer[64];
	memset( OutputPacketBuffer, 0, 2052 );
	memset( ReceivePacketBuffer, 0, 64 );

	LOGDEBUG( "getConfig" );

	memcpy( OutputPacketBuffer, cmd[CONFIG], HEADER_SIZE );
	OutputPacketBuffer[4] = SENDSETTINGS;

	// setup receive thread to wait for data
    uart_context_t ctx;
    ctx.packet_size = 62;
    ctx.shared_buffer = ReceivePacketBuffer;
	
    pthread_t thread_id;
    pthread_create(&thread_id, NULL, uart_receive_thread, &ctx);
	
	//now request config data
	sendUart( OutputPacketBuffer, 2052 );
	
	//wait until receive complete
    pthread_join(thread_id, NULL);
	
	if( ReceivePacketBuffer[4] == 0x56 ) { // check leading V of the version string
		memcpy( versionString, ReceivePacketBuffer + 4, 6 );
		memcpy( &config, ReceivePacketBuffer + 10, sizeof( config ) );
		memcpy( UIDString, ReceivePacketBuffer + 48, 15 );
		Mmi_Status = (MMI_STATUS_t)ReceivePacketBuffer[47];
	} else {
		LOGERROR( "UART receive config failed " );
		exit( 1 );
	}

	deviceMode = (DeviceMode)config.mode;
	rgbSequence = config.rgbseq;
	if (UIDString[0] == 'H' || UIDString[0] == 'M'){
		deviceType = PIN2DMD_HD;
		LOGDEBUG( "device type is HD");
		deviceMode = deviceModeHD[deviceMode];
	} 
	else 
	if (UIDString[0] == 'X' || UIDString[0] == 'S'){
		deviceType = PIN2DMD_XL;
		LOGDEBUG( "device type is XL");
		deviceMode = deviceModeXL[deviceMode];
	}
	
	LOGDEBUG( "device mode is %i", deviceMode );
	LOGDEBUG( "rgb sequence is %i", rgbSequence );
	
	switch( deviceMode ) {
	case WPC: // WPC
		numberOfPlanes = 3;
		rawDump = true;
		break;
	case SAM: // SAM
		numberOfPlanes = 4;
		break;
	case WhiteStar: // Whitestar
		numberOfPlanes = 2;
		break;
	case Spike: // Spike
		numberOfPlanes = 4;
		break;
	case DataEast: // DataEast
		numberOfPlanes = 2;
		break;
	case Gottlieb1: // Gottlieb V1
		numberOfPlanes = 12;
		rawDump = true;
		break;
	case Gottlieb2: // Gottlieb V2
		numberOfPlanes = 6;
		rawDump = true;
		break;
	case Gottlieb3: // Gottlieb V3
		numberOfPlanes = 8;
		rawDump = true;
		break;
	case Capcom: // Capcom
		numberOfPlanes = 4;
		rawDump = true;
		break;
	case AlvinG: // AlvinG
		numberOfPlanes = 4;
		rawDump = true;
		break;
	case Spooky: // Spooky
		numberOfPlanes = 0;
		break;
	case DE128x16: // DataEast128x16
		numberOfPlanes = 2;
		break;
	case Inder: // Inder / SPinball
		numberOfPlanes = 3;
		rawDump = true;
		break;
	case Sleic: // Sleic / Petaco
		numberOfPlanes = 2;
		break;
	case HomePin: // HomePin
		numberOfPlanes = 4;
		break;
	case SysXDMD: //  SysXDMD
		numberOfPlanes = 1;
		break;
	case SEGA192x64: //  192x64
	case DotMation: //  192x64
		planesize = 1536;
		numberOfPlanes = 2;
		rawDump = false;
		break;
	case Capcom256: //  Capcom256
		planesize = 2048;
		numberOfPlanes = 4;
		rawDump = true;
		break;
	default:
		break;
	}

}

void reset() {
	LOGTRACE("resetting PIN2DMD device");
	uint8_t* txbuffer;
	txbuffer = (uint8_t*)malloc( (4 * planesize) + HEADER_SIZE);
	memset( txbuffer, 0x00, TX_BUFFER_SIZE );
	memcpy (txbuffer, cmd[CONFIG], HEADER_SIZE);
	txbuffer[4] = RESET;
	// Write to UART
	sendUart(txbuffer, (4 * planesize) + HEADER_SIZE );
	sleep(3);
	free (txbuffer);
}

// default for normal size
int RX_BUFFER_SIZE = 6144;
int DISPLAYBUFFER_SIZE = 12288;
int WIDTH = 128;
int HEIGHT = 32;
int CHUNK_SIZE = 6144;

void init(uint8_t spi_init, bool withReset) {

	setLogDevice( LOG_DEVICE_STDOUT );

	setupGpio();
	setupUart();
	
	if (withReset)
		reset();

	setupSpi();
	resetUart();

	uint8_t* txbuffer;
	txbuffer = (uint8_t*)malloc( CHUNK_SIZE );

	memcpy( txbuffer, cmd[spi_init], HEADER_SIZE ); // set SPI
	sendUart( txbuffer, HEADER_SIZE );

	getConfig();
	
	if( deviceType == PIN2DMD_HD ) {
		RX_BUFFER_SIZE = 8192;
		DISPLAYBUFFER_SIZE = 49152;
		WIDTH = 256;
		HEIGHT = 64;
		CHUNK_SIZE = 8192;
	} else
	if( deviceType == PIN2DMD_XL ) {
		RX_BUFFER_SIZE = 6144;
		DISPLAYBUFFER_SIZE = 36864;
		WIDTH = 192;
		HEIGHT = 64;
		CHUNK_SIZE = 6144;
	}

	free( txbuffer );
}

void deInit() {
	
	tcflush(uart_fd, TCIOFLUSH);
	close( uart_fd );
	close( spi_fd );

	if (gpio1_request)
		gpiod_line_request_release(gpio1_request);
	if (gpio2_request)
		gpiod_line_request_release(gpio2_request);
	if (chip)
		gpiod_chip_close(chip);
	
}

uint8_t* create_FrameBuffer( int width, int height, DeviceMode deviceMode, uint8_t* source, uint8_t* dest,
                             uint8_t* palette, bool rgb565 ) {
	LOGTRACE("creating frame buffer width %d height %d", width, height);
	uint8_t shades4[4] = { 0, 1, 4, 15 };
	rgb24* rgbbuffer = (rgb24*)dest;
	rgb24* palette24 = (rgb24*)palette;
	uint16_t* rgb565buffer = (uint16_t*)dest;
	uint16_t* palette565 = (uint16_t*)palette;
	unsigned char level5[5] = { 0, 1, 2, 3, 3 };                                  // 4 colors
	unsigned char level6[7] = { 0, 1, 2, 2, 2, 2, 3 };                            // 4 colors
	unsigned char level8[9] = { 0, 1, 2, 2, 2, 2, 2, 2, 3 };                      // 4 colors
	unsigned char level12[13] = { 0, 3, 3, 7, 7, 7, 11, 11, 11, 11, 11, 11, 15 }; // 5 colors

	bool shades16 = false;
	if( deviceMode == SAM | deviceMode == Spike | deviceMode == Gottlieb1 | deviceMode == HomePin )
		shades16 = true;

	int pos = 0;
	for( int jj = 0; jj < height; jj++ ) {
		for( int ii = 0; ii < ( width / 8 ); ii++ ) {
			for( int kk = 0; kk < 8; kk++ ) {
				int ll = 0;
				uint8_t val = 0;
				uint8_t tmp = 0;
				switch( deviceMode ) {
				case WPC: // WPC
					for( ll = 0; ll < 3; ll++ )
						val = val + ( ( source[( jj * 16 ) + ii + ( ll * 512 )] >> kk ) & 0x01 );
					break;
				case SAM: // SAM
					val = ( ( source[( jj * 16 ) + ii] >> kk ) & 0x01 ) +
					      ( ( ( source[( jj * 16 ) + ii + 512] >> kk ) & 0x01 ) * 2 ) +
					      ( ( ( source[( jj * 16 ) + ii + 1024] >> kk ) & 0x01 ) * 4 ) +
					      ( ( ( source[( jj * 16 ) + ii + 1536] >> kk ) & 0x01 ) * 8 );
					break;
				case WhiteStar: // Whitestar
					val = ( ( source[( jj * 16 ) + ii] >> kk ) & 0x01 ) +
					      ( ( ( source[( jj * 16 ) + ii + 512] >> kk ) & 0x01 ) * 2 );
					break;
				case Spike: // Spike
					val = ( ( source[( jj * 16 ) + ii] >> kk ) & 0x01 ) +
					      ( ( ( source[( jj * 16 ) + ii + 512] >> kk ) & 0x01 ) * 2 ) +
					      ( ( ( source[( jj * 16 ) + ii + 1024] >> kk ) & 0x01 ) * 4 ) +
					      ( ( ( source[( jj * 16 ) + ii + 1536] >> kk ) & 0x01 ) * 8 );
					break;
				case DataEast: // DataEast
					val = ( ( source[( jj * 16 ) + ii] >> kk ) & 0x01 ) +
					      ( ( ( source[( jj * 16 ) + ii + 512] >> kk ) & 0x01 ) * 2 );
					break;
				case Gottlieb1: // Gottlieb V1
					for( ll = 0; ll < 12; ll++ )
						tmp = tmp + ( ( source[( jj * 16 ) + ii + ( ll * 512 )] >> kk ) & 0x01 );
					val = level12[tmp];
					break;
				case Gottlieb2: // Gottlieb V2
					for( ll = 0; ll < 6; ll++ )
						int tmp = tmp + ( ( source[( jj * 16 ) + ii + ( ll * 512 )] >> kk ) & 0x01 );
					val = level6[tmp];
					break;
				case Gottlieb3: // Gottlieb V3
					for( ll = 0; ll < 8; ll++ )
						tmp = tmp + ( ( source[( jj * 16 ) + ii + ( ll * 512 )] >> kk ) & 0x01 );
					val = level8[tmp];
					break;
				case Capcom: // Capcom
					tmp = ( ( source[( jj * 16 ) + ii] >> kk ) & 0x01 ) + ( ( source[( jj * 16 ) + ii + 512] >> kk ) & 0x01 ) +
					      ( ( source[( jj * 16 ) + ii + 1024] >> kk ) & 0x01 ) +
					      ( ( source[( jj * 16 ) + ii + 1536] >> kk ) & 0x01 );
					val = level5[tmp];
					break;
				case AlvinG: // AlvinG
					val = ( ( source[( jj * 16 ) + ii] >> kk ) & 0x01 ) + ( ( source[( jj * 16 ) + ii + 512] >> kk ) & 0x01 ) +
					      ( ( source[( jj * 16 ) + ii + 1024] >> kk ) & 0x01 ) +
					      ( ( source[( jj * 16 ) + ii + 1536] >> kk ) & 0x01 );
					break;
				case Spooky: // Spooky
					val = ( ( source[( jj * 16 ) + ii] >> kk ) & 0x01 ) +
					      ( ( ( source[( jj * 16 ) + ii + 512] >> kk ) & 0x01 ) * 2 ) +
					      ( ( ( source[( jj * 16 ) + ii + 1024] >> kk ) & 0x01 ) * 4 ) +
					      ( ( ( source[( jj * 16 ) + ii + 1536] >> kk ) & 0x01 ) * 8 );
					break;
				case DE128x16: // DataEast128x16
					val = ( ( source[( jj * 16 ) + ii] >> kk ) & 0x01 ) +
					      ( ( ( source[( jj * 16 ) + ii + 512] >> kk ) & 0x01 ) * 2 );
					break;
				case Inder: // Inder / SPinball
					for( ll = 0; ll < 3; ll++ )
						val = val + ( ( source[( jj * 16 ) + ii + ( ll * 512 )] >> kk ) & 0x01 );
					break;
				case Sleic: // Sleic
					val = ( ( source[( jj * 16 ) + ii] >> kk ) & 0x01 ) +
					      ( ( ( source[( jj * 16 ) + ii + 512] >> kk ) & 0x01 ) * 2 );
					break;
				case HomePin: // HomePin
					val = ( ( source[( jj * 16 ) + ii] >> kk ) & 0x01 ) +
					      ( ( ( source[( jj * 16 ) + ii + 512] >> kk ) & 0x01 ) * 2 ) +
					      ( ( ( source[( jj * 16 ) + ii + 1024] >> kk ) & 0x01 ) * 4 ) +
					      ( ( ( source[( jj * 16 ) + ii + 1536] >> kk ) & 0x01 ) * 8 );
					break;
				case SysXDMD: // SysXDMD
					val = ( ( source[( jj * 16 ) + ii] >> kk ) & 0x01 ) + ( ( ( source[( jj * 16 ) + ii] >> kk ) & 0x01 ) * 2 );
					break;
				case SEGA192x64: // 192x64
				case DotMation: 
					val = ( ( source[( jj * 24 ) + ii] >> kk ) & 0x01 ) +
					      ( ( ( source[( jj * 24 ) + ii + 1536] >> kk ) & 0x01 ) * 2 );
					break;
				case Capcom256: // Capcom256
					tmp = ( ( source[( jj * 32 ) + ii] >> kk ) & 0x01 ) + ( ( source[( jj * 32 ) + ii + 2048] >> kk ) & 0x01 ) +
					      ( ( source[( jj * 32 ) + ii + 4096] >> kk ) & 0x01 ) +
					      ( ( source[( jj * 32 ) + ii + 6144] >> kk ) & 0x01 );
					val = level5[tmp];
					break;
				default:
					break;
				}
				if( palette != NULL ) {
					if( rgb565 ) {
						if( !shades16 )
							rgb565buffer[pos++] = palette565[shades4[val]];
						else
							rgb565buffer[pos++] = palette565[val];
					} else {
						if( !shades16 )
							rgbbuffer[pos++] = palette24[shades4[val]];
						else
							rgbbuffer[pos++] = palette24[val];
					}
				} else
					dest[pos++] = val;
			}
		}
	}

	return dest;
}

uint8_t* create_FrameFromRGB24( uint16_t width, uint16_t height, rgb24* rgb24source, uint8_t* dest ) {
	LOGTRACE("creating display buffer from RGB24 (width=%i, height=%i)", width, height);
	int elements = width * height / 2;
	int frameSize = ( elements * 6 );
	memset( dest, 0, frameSize );

	int pixel_r, pixel_g, pixel_b, pixel_rl, pixel_gl, pixel_bl;
	for( int i = 0; i < elements; i++ ) {
		switch( rgbSequence ) {
		case PANEL_RBG: // RGB panels
			pixel_r = rgb24source[i].red;
			pixel_b = rgb24source[i].blue;
			pixel_g = rgb24source[i].green;
			// lower half of display
			pixel_rl = rgb24source[i + elements].red;
			pixel_bl = rgb24source[i + elements].blue;
			pixel_gl = rgb24source[i + elements].green;
			break;
		case PANEL_GRB: // BGR panels
			pixel_b = rgb24source[i].red;
			pixel_g = rgb24source[i].blue;
			pixel_r = rgb24source[i].green;
			// lower half of display
			pixel_bl = rgb24source[i + elements].red;
			pixel_gl = rgb24source[i + elements].blue;
			pixel_rl = rgb24source[i + elements].green;
			break;
		case PANEL_GBR:
			pixel_g = rgb24source[i].red;
			pixel_b = rgb24source[i].blue;
			pixel_r = rgb24source[i].green;
			// lower half of display
			pixel_gl = rgb24source[i + elements].red;
			pixel_bl = rgb24source[i + elements].blue;
			pixel_rl = rgb24source[i + elements].green;
			break;
		case PANEL_BRG:
			pixel_b = rgb24source[i].red;
			pixel_r = rgb24source[i].blue;
			pixel_g = rgb24source[i].green;
			// lower half of display
			pixel_bl = rgb24source[i + elements].red;
			pixel_rl = rgb24source[i + elements].blue;
			pixel_gl = rgb24source[i + elements].green;
			break;
		case PANEL_BGR:
			pixel_g = rgb24source[i].red;
			pixel_r = rgb24source[i].blue;
			pixel_b = rgb24source[i].green;
			// lower half of display
			pixel_gl = rgb24source[i + elements].red;
			pixel_rl = rgb24source[i + elements].blue;
			pixel_bl = rgb24source[i + elements].green;
			break;
		default: // RGB panels
			pixel_r = rgb24source[i].red;
			pixel_g = rgb24source[i].blue;
			pixel_b = rgb24source[i].green;
			// lower half of display
			pixel_rl = rgb24source[i + elements].red;
			pixel_gl = rgb24source[i + elements].blue;
			pixel_bl = rgb24source[i + elements].green;
			break;
		}

		// color correction
		pixel_r = GAMMA_TABLE[pixel_r];
		pixel_g = GAMMA_TABLE[pixel_g];
		pixel_b = GAMMA_TABLE[pixel_b];

		pixel_rl = GAMMA_TABLE[pixel_rl];
		pixel_gl = GAMMA_TABLE[pixel_gl];
		pixel_bl = GAMMA_TABLE[pixel_bl];

		int target_idx = i;

		for( int k = 0; k < BITS_PER_CHANNEL; k++ ) {
			int val = ( ( pixel_gl & 1 ) << 5 ) | ( ( pixel_bl & 1 ) << 4 ) | ( ( pixel_rl & 1 ) << 3 ) |
			          ( ( pixel_g & 1 ) << 2 ) | ( ( pixel_b & 1 ) << 1 ) | ( ( pixel_r & 1 ) << 0 );
			dest[target_idx] = val;
			pixel_r >>= 1;
			pixel_g >>= 1;
			pixel_b >>= 1;
			pixel_rl >>= 1;
			pixel_gl >>= 1;
			pixel_bl >>= 1;
			target_idx += elements;
		}
	}
	return dest;
};

#define RED_RGB565( pixel ) GAMMA_TABLE[( ( ( ( pixel ) >> 8 ) & 0xF8 ) | ( ( ( pixel ) >> 13 ) & 0x07 ) )]
#define GREEN_RGB565( pixel ) GAMMA_TABLE[( ( ( ( pixel ) >> 3 ) & 0xFC ) | ( ( ( pixel ) >> 9 ) & 0x03 ) )]
#define BLUE_RGB565( pixel ) GAMMA_TABLE[( ( ( ( pixel ) << 3 ) & 0xF8 ) | ( ( ( pixel ) >> 2 ) & 0x07 ) )]

uint8_t* create_FrameFromRGB565( uint16_t width, uint16_t height, uint16_t* rgb565source, uint8_t* dest ) {
	LOGTRACE("creating display buffer from RGB565 (width=%i, height=%i)", width, height);
	int elements = width * height / 2;
	int frameSize = ( elements * 6 );
	memset( dest, 0, frameSize );

	int pixel_r, pixel_g, pixel_b, pixel_rl, pixel_gl, pixel_bl;
	for( int i = 0; i < elements; i++ ) {
		uint16_t raw_pixel = rgb565source[i];
		uint16_t raw_pixel_lower = rgb565source[i + elements];
		switch( rgbSequence ) {
		case PANEL_RBG: // RGB panels
			pixel_r = RED_RGB565( raw_pixel );
			pixel_b = BLUE_RGB565( raw_pixel );
			pixel_g = GREEN_RGB565( raw_pixel );
			// lower half of display
			pixel_rl = RED_RGB565( raw_pixel_lower );
			pixel_bl = BLUE_RGB565( raw_pixel_lower );
			pixel_gl = GREEN_RGB565( raw_pixel_lower );
			break;
		case PANEL_GRB: // BGR panels
			pixel_b = RED_RGB565( raw_pixel );
			pixel_g = BLUE_RGB565( raw_pixel );
			pixel_r = GREEN_RGB565( raw_pixel );
			// lower half of display
			pixel_bl = RED_RGB565( raw_pixel_lower );
			pixel_gl = BLUE_RGB565( raw_pixel_lower );
			pixel_rl = GREEN_RGB565( raw_pixel_lower );
			break;
		case PANEL_GBR:
			pixel_g = RED_RGB565( raw_pixel );
			pixel_b = BLUE_RGB565( raw_pixel );
			pixel_r = GREEN_RGB565( raw_pixel );
			// lower half of display
			pixel_gl = RED_RGB565( raw_pixel_lower );
			pixel_bl = BLUE_RGB565( raw_pixel_lower );
			pixel_rl = GREEN_RGB565( raw_pixel_lower );
			break;
		case PANEL_BRG:
			pixel_b = RED_RGB565( raw_pixel );
			pixel_r = BLUE_RGB565( raw_pixel );
			pixel_g = GREEN_RGB565( raw_pixel );
			// lower half of display
			pixel_bl = RED_RGB565( raw_pixel_lower );
			pixel_rl = BLUE_RGB565( raw_pixel_lower );
			pixel_gl = GREEN_RGB565( raw_pixel_lower );
			break;
		case PANEL_BGR:
			pixel_g = RED_RGB565( raw_pixel );
			pixel_r = BLUE_RGB565( raw_pixel );
			pixel_b = GREEN_RGB565( raw_pixel );
			// lower half of display
			pixel_gl = RED_RGB565( raw_pixel_lower );
			pixel_rl = BLUE_RGB565( raw_pixel_lower );
			pixel_bl = GREEN_RGB565( raw_pixel_lower );
			break;
		default: // RGB panels
			pixel_r = RED_RGB565( raw_pixel );
			pixel_g = BLUE_RGB565( raw_pixel );
			pixel_b = GREEN_RGB565( raw_pixel );
			// lower half of display
			pixel_rl = RED_RGB565( raw_pixel_lower );
			pixel_gl = BLUE_RGB565( raw_pixel_lower );
			pixel_bl = GREEN_RGB565( raw_pixel_lower );
			break;
		}

		int target_idx = i;

		for( int k = 0; k < BITS_PER_CHANNEL; k++ ) {
			int val = ( ( pixel_gl & 1 ) << 5 ) | ( ( pixel_bl & 1 ) << 4 ) | ( ( pixel_rl & 1 ) << 3 ) |
			          ( ( pixel_g & 1 ) << 2 ) | ( ( pixel_b & 1 ) << 1 ) | ( ( pixel_r & 1 ) << 0 );
			dest[target_idx] = val;
			pixel_r >>= 1;
			pixel_g >>= 1;
			pixel_b >>= 1;
			pixel_rl >>= 1;
			pixel_gl >>= 1;
			pixel_bl >>= 1;
			target_idx += elements;
		}
	}
	return dest;
}

uint8_t* scaleDouble( uint16_t width, uint16_t height, uint8_t* src, uint8_t* dest, int byteSize ) {
	LOGTRACE("scaling double to (width=%i, height=%i)", width, height);
	int targetIdx = 0;
	for( int i = 0; i < height; ++i ) {
		int iUnscaled = i / 2;
		for( int j = 0; j < width; ++j ) {
			int jUnscaled = j / 2;
			for( int k = 0; k < byteSize; k++ ) {
				dest[targetIdx++] = src[iUnscaled * ( width / 2 ) * byteSize + jUnscaled * byteSize + k];
			}
		}
	}
	return dest;
}

uint32_t getPixel(uint8_t* src, int x, int y, int width, int byteSize) {
	uint32_t val = 0;
		for (int i = 0; i < byteSize; i++) {
			val = val << 8;
			val |= src[(x * byteSize) + (y * width * byteSize) + i] & 0xff;
		}
	return val;
}

void setPixel(uint8_t* dest, uint32_t val, int x, int y, int width, int byteSize) {
	uint8_t k = byteSize - 1;
	for (int i = 0; i < byteSize; i++) {
		dest[(x * byteSize) + (y * width * byteSize) + k] = (val >> (i * 8)) & 0xff;
		k--;
	}
}

uint8_t* scale2x( uint16_t destWidth, uint16_t destHeight, uint8_t* src, uint8_t* dest , int byteSize)  {
	LOGTRACE("scaling 2X to (width=%i, height=%i)", destWidth, destHeight);

	int srcWidth = destWidth / 2;
	int srcHeight = destHeight / 2;

	for( int y = 0; y < srcHeight; y++ ) {
		for( int x = 0; x < srcWidth; x++) {
			uint32_t p = getPixel( src, x, y, srcWidth, byteSize);
			uint32_t p1 = p;
			uint32_t p2 = p;
			uint32_t p3 = p;
			uint32_t p4 = p;
			uint32_t a = getPixel( src, x, y-1, srcWidth, byteSize );
			uint32_t b = getPixel( src, x+1, y, srcWidth, byteSize );
			uint32_t c = getPixel( src, x-1, y, srcWidth, byteSize );
			uint32_t d = getPixel( src, x, y+1, srcWidth, byteSize );

			if( c == a && c!=d && a!= b ) p1 = a;
			if( a == b && a!=c && b!= d ) p2 = b;
			if( d == c && d!=b && c!= a ) p3 = c;
			if( b == d && b!=a && d!= c ) p4 = d;
			setPixel( dest, p1, x*2, y*2, destWidth , byteSize );
			setPixel( dest, p2, x*2+1, y*2, destWidth, byteSize );
			setPixel( dest, p3, x*2, y*2+1, destWidth, byteSize );
			setPixel( dest, p4, x*2+1, y*2+1, destWidth, byteSize );
		}
	}

	return dest;
}

uint8_t* scaleHD( uint16_t destWidth, uint16_t destHeight, uint8_t* src, uint8_t* dest , int byteSize){
	if (config.scaler == 1)
		return scale2x(destWidth, destHeight, src, dest , byteSize);
	return scaleDouble(destWidth, destHeight, src, dest , byteSize);
}

uint8_t* create_FrameFromRGB24HD( uint16_t width, uint16_t height, uint8_t* rgbsource, uint8_t* dest, bool rgb565 ) {
	int elements = width * height / 2;
	int frameSize = ( elements * 6 );
	memset( dest, 0, frameSize );
	uint8_t* tmp = (uint8_t*)malloc( frameSize );

	if( rgb565 )
		tmp = create_FrameFromRGB565( width, height, (uint16_t*)rgbsource, tmp );
	else
		tmp = create_FrameFromRGB24( width, height, (rgb24*)rgbsource, tmp );

	int dest_idx = 0;
	int tmp_idx = 0;
	
	LOGTRACE("sorting HD buffer to buffermode %d",config.buffermode);

	if( config.buffermode == 0 ) {
		for( int l = 0; l < frameSize / 2; l++ ) {
			dest[dest_idx] = tmp[tmp_idx + ( width / 2 )];
			dest[dest_idx + 1] = tmp[tmp_idx] << 1;
			dest_idx += 2;
			tmp_idx++;
			if( ( dest_idx ) % width == 0 )
				tmp_idx += width / 2;
		}
	} else {
		uint8_t val;
		for( int i = 0; i < 32; i++ ) {    // 32 rows of source as we split into upper and lower half
			for( int j = 0; j < 16; j++ ) {  // 16 channels per driver IC with duplicate pixel
				for( int k = 0; k < 8; k++ ) { // 8 led driver ICs per module
					for( int l = 5; l >= 0; l-- ) {
						tmp_idx = k * 16 + j + i * 256;
						val = tmp[tmp_idx + ( width / 2 ) + ( width * height / 2 * l )];
						dest[dest_idx++] = val;
						val = tmp[tmp_idx + ( width * height / 2 * l )] << 1;
						dest[dest_idx++] = val;
					}
				}
			}
		}
	}
	free( tmp );
	return dest;
}

uint8_t* create_FrameFromRGB565HD( uint16_t width, uint16_t height, uint16_t* rgbsource, uint8_t* dest ) {
	return create_FrameFromRGB24HD( width, height, (uint8_t*)rgbsource, dest, true );
}

uint8_t* create_RGBFromFrame(uint16_t width, uint16_t height, uint8_t* src, uint8_t* dest, bool rgb565 ) {
	LOGTRACE("create RGB24 from display buffer (width=%i, height=%i)", width, height);
	int elements = width * height / 2;
	int frameSizeRGB24 = (elements * 6);
	int frameSizeRGB565 = (elements * 4);
	rgb24* rgb24dest = (rgb24*) dest;
	uint16_t* rgb565dest = (uint16_t*) dest;
	if (!rgb565)
		memset(rgb24dest, 0, frameSizeRGB24);
	else
		memset(rgb565dest, 0, frameSizeRGB565);

	int pixel_r, pixel_g, pixel_b, pixel_rl, pixel_gl, pixel_bl;
	int red, blue, green, lred, lblue, lgreen;
	
	for (int i = 0; i < elements; i++) {
		int src_idx = i;
		pixel_r = pixel_g = pixel_b = pixel_rl = pixel_gl = pixel_bl = 0;

		for (int k = 0; k < BITS_PER_CHANNEL; k++) {
			int val = src[src_idx];
			pixel_r  |= ((val >> 0) & 1) << k;
            pixel_b  |= ((val >> 1) & 1) << k;
            pixel_g  |= ((val >> 2) & 1) << k;
            pixel_rl |= ((val >> 3) & 1) << k;
            pixel_bl |= ((val >> 4) & 1) << k;
            pixel_gl |= ((val >> 5) & 1) << k;
			src_idx += elements;
		}
		
		switch (rgbSequence) {
		case PANEL_RBG: // RGB panels
			red = pixel_r;
			blue = pixel_b;
			green = pixel_g;
			// lower half of display
			lred = pixel_rl;
			lblue = pixel_bl;
			lgreen = pixel_gl;
			break;
		case PANEL_GRB: // BGR panels
			red = pixel_b;
			blue = pixel_g;
			green = pixel_r;
			// lower half of display
			lred = pixel_bl;
			lblue = pixel_gl;
			lgreen = pixel_rl;
			break;
		case PANEL_GBR:
			red = pixel_g;
			blue = pixel_b;
			green = pixel_r;
			// lower half of display
			lred = pixel_gl;
			lblue = pixel_bl;
			lgreen = pixel_rl;
			break;
		case PANEL_BRG:
			red = pixel_b;
			blue = pixel_r;
			green = pixel_g;
			// lower half of display
			lred = pixel_bl;
			lblue = pixel_rl;
			lgreen = pixel_gl;
			break;
		case PANEL_BGR:
			red = pixel_g;
			blue = pixel_r;
			green = pixel_b;
			// lower half of display
			lred = pixel_gl;
			lblue = pixel_rl;
			lgreen = pixel_bl;
			break;
		default: // RGB panels
			red = pixel_r;
			blue = pixel_g;
			green = pixel_b;
			// lower half of display
			lred = pixel_rl;
			lblue = pixel_gl;
			lgreen = pixel_bl;
			break;
		}
		
		if (!rgb565){
			rgb24dest[i].red = REVERSE_LUT_8bit[red];
			rgb24dest[i].green = REVERSE_LUT_8bit[green];
			rgb24dest[i].blue = REVERSE_LUT_8bit[blue];
			rgb24dest[i + elements].red = REVERSE_LUT_8bit[lred];
			rgb24dest[i + elements].green = REVERSE_LUT_8bit[lgreen];
			rgb24dest[i + elements].blue = REVERSE_LUT_8bit[lblue];
		} else {
			rgb565dest[i] = (uint16_t) ((REVERSE_LUT_6bit[red] & 0b111110) << 10) | ((REVERSE_LUT_6bit[green] & 0b111111) << 5) | ((REVERSE_LUT_6bit[blue] & 0b111110) >> 1); 
			rgb565dest[i + elements] = (uint16_t)((REVERSE_LUT_6bit[lred] & 0b111110) << 10) | ((REVERSE_LUT_6bit[lgreen] & 0b111111) << 5) | ((REVERSE_LUT_6bit[lblue] & 0b111110) >> 1) ; 
		}
	}
	return dest;
};

uint8_t* create_RGBHDFromFrame(uint16_t width, uint16_t height, uint8_t* src, uint8_t* dest, bool rgb565) {
	LOGTRACE("presorting RGB24 HD from display buffer (width=%i, height=%i, buffermode=%i)", width, height, config.buffermode);

	int elements = width * height / 2;
	int frameSize = (elements * 6);
	uint8_t* tmp = (uint8_t*)malloc(frameSize);

	int src_idx = 0;
	int tmp_idx = 0;
	
	if (config.buffermode == 0) {
		for (int l = 0; l < frameSize / 2; l++) {
			tmp[tmp_idx + (width / 2)] = src[src_idx];
			tmp[tmp_idx] = src[src_idx + 1] >> 1 ;
			src_idx += 2;
			tmp_idx++;
			if ((src_idx) % width == 0)
				tmp_idx += width / 2;
		}
	}
	else {
		uint8_t val;
		for (int i = 0; i < 32; i++) {    // 32 rows of source as we split into upper and lower half
			for (int j = 0; j < 16; j++) {  // 16 channels per driver IC with duplicate pixel
				for (int k = 0; k < 8; k++) { // 8 led driver ICs per module
					for (int l = 5; l >= 0; l--) {
						tmp_idx = k * 16 + j + i * 256;
						val = src[src_idx++];
						tmp[tmp_idx + (width / 2) + (width * height / 2 * l)] = val;
						val = src[src_idx++] >> 1;
						tmp[tmp_idx + (width * height / 2 * l)] = val;
					}
				}
			}
		}
	}
	
	dest = create_RGBFromFrame(width, height, tmp, dest, rgb565);

	free(tmp);
	return dest;
}

int fb_init ( struct fb_var_screeninfo* vinfo ){
	int fb_fd = open("/dev/fb0", O_RDWR);
    if (fb_fd == -1) { perror("Fehler: /dev/fb0"); exit(1); }
	
    ioctl(fb_fd, FBIOGET_VSCREENINFO, vinfo);
    // map 16-bit framebuffer
	return fb_fd;
}

void fb_clear( int fb_fd, struct fb_var_screeninfo vinfo ){
	
	long screensize = vinfo.yres_virtual * vinfo.xres_virtual * 2;
	uint16_t* fbp = (uint16_t*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, 0);
	// clear screen
    memset(fbp, 0, screensize);
	munmap(fbp, screensize);
	
}

void fb_displayRGB565( int fb_fd, struct fb_var_screeninfo vinfo , uint16_t* img, int src_w, int src_h, int mode){
	
	long screensize = vinfo.yres_virtual * vinfo.xres_virtual * 2;
    uint16_t* fbp = (uint16_t*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, 0);
	
	int target_w = vinfo.xres;
	int target_h = vinfo.yres;
	switch (mode) {
		case 1:{	//4:1
			target_h = target_w / 4; 
		}
		break;
		case 2:{	//2:1
			target_h = target_w / 2; 
		}
		break;
		case 3:{	//3:1
			target_h = target_w / 3; 
		}
		break;
		default:
		break;
	}
	LOGTRACE("screen x = %d screen y = %d target w = %d target h %d\n", vinfo.xres, vinfo.yres, target_w, target_h);
	
    // limit if calculated target_h is larger than screen
    if (target_h > vinfo.yres) {
        target_h = vinfo.yres;
    }

    int offset_x = (vinfo.xres - target_w) / 2;
    int offset_y = (vinfo.yres - target_h) / 2;
	
	for (int y = 0; y < target_h; y++) {
		for (int x = 0; x < target_w; x++) {
			// map destination coordinates back to 128x32 source
			int src_x = (x * src_w) / target_w;
			int src_y = (y * src_h) / target_h;
			
			// write pixel with offset for centering
			fbp[(x + offset_x) + (y + offset_y) * vinfo.xres] = img[src_y * src_w + src_x];;
		}
	}
	
	munmap(fbp, screensize);
}

void fb_displayRGB( int fb_fd, struct fb_var_screeninfo vinfo , rgb24* img, int src_w, int src_h, int mode){
	
	long screensize = vinfo.yres_virtual * vinfo.xres_virtual * 2;
    uint16_t* fbp = (uint16_t*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, 0);

    // --- scaling logic for 4:2 (2:1) ---
    int target_w = vinfo.xres;
    int target_h = vinfo.yres;
	switch (mode) {
		case 1:{	//4:1
			target_h = target_w / 4; 
		}
		break;
		case 2:{	//2:1
			target_h = target_w / 2; 
		}
		break;
		case 3:{	//3:1
			target_h = target_w / 3; 
		}
		break;
		default:
		break;
	}

    // limit if calculated target_h is larger than screen
    if (target_h > vinfo.yres) {
        target_h = vinfo.yres;
    }

    int offset_x = (vinfo.xres - target_w) / 2;
    int offset_y = (vinfo.yres - target_h) / 2;

    for (int y = 0; y < target_h; y++) {
        for (int x = 0; x < target_w; x++) {
			// map destination coordinates back to 128x32 source
            int src_x = (x * src_w) / target_w;
            int src_y = (y * src_h) / target_h;
            
            rgb24 p = img[src_y * src_w + src_x];

            // convert 888 -> 565
            uint16_t color16 = ((p.red >> 3) << 11) | ((p.green >> 2) << 5) | (p.blue >> 3);

            // write pixel with offset for centering
            fbp[(x + offset_x) + (y + offset_y) * vinfo.xres] = color16;
        }
    }

    munmap(fbp, screensize);
}

void fb_deInit (int fb_fd){
    close(fb_fd);
}

// dump display data to stdout
void dump_screen( uint32_t tick, uint8_t* p ) {
	int width = 128;
	int height = 32;
	if (planesize == 2048){
		width = 256; height = 64;
	} else if (planesize == 1536){
		width = 192; height = 64;
	}
	const char* hex_digits = "0123456789ABCDEF";
	int y = 0, x = 0;
	printf("0x%x\n", tick);
	for( y = 0; y < height; y++ ) {
		for( x = 0; x < width; x++ ) {
			printf( "%c", hex_digits[p[y * width + x]] );
		}
		printf( "\n" );
	}
	printf( "\n" );
}

void dump_file_init(char* filePath, char* filePathRaw){
	int width = 128;
	int height = 32;
	if (planesize == 2048){
		width = 256; height = 64;
	} else if (planesize == 1536){
		width = 192; height = 64;
	}
	LOGDEBUG("writing dump to %s\n",filePath);
	if (rawDump) 
	{	//write rawdump header
			LOGDEBUG("writing raw dump to %s\n",filePathRaw);
			FILE* fileRaw = fopen(filePathRaw, "ab");
			if (fileRaw)
			{
				fputc(0x52, fileRaw); //R
				fputc(0x41, fileRaw); //A
				fputc(0x57, fileRaw); //W
				fputc(0x00, fileRaw); //
				fputc(0x01, fileRaw); // version
				fputc(width, fileRaw);  // width
				fputc(height, fileRaw);   // height
				fputc(numberOfPlanes, fileRaw);
				fclose(fileRaw);
			}
	}
}

int32_t millis( void ) {
	struct timeval tv;
	gettimeofday( &tv, NULL );
	return (int32_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

void dump_file(char* filePath, char* filePathRaw, uint32_t tick , uint8_t* p, uint8_t* raw) {
	int width = 128;
	int height = 32;
	if (planesize == 2048){
		width = 256; height = 64;
	} else if (planesize == 1536){
		width = 192; height = 64;
	}
	LOGTRACE("write text dump width %d height %d\n", width, height);
	FILE* file = fopen(filePath, "a");
	const char* hex_digits = "0123456789ABCDEF";
	int y = 0, x = 0;
	fprintf(file, "0x%x%c%c", tick,0x0D,0x0A);
	for( y = 0; y < height; y++ ) {
		for( x = 0; x < width; x++ ) {
			fprintf(file, "%c", hex_digits[p[y * width + x]] );
		}
		fprintf(file, "%c%c",0x0D,0x0A);
	}
	fprintf(file, "%c%c",0x0D,0x0A);
	fclose(file);
	
	if (rawDump) {
		FILE* fileRaw = fopen(filePathRaw, "ab");
		if (fileRaw){
			fwrite(&tick, 1, 4, fileRaw);
			fwrite(raw, 1, numberOfPlanes * planesize, fileRaw);
			fclose(fileRaw);
		}
	}
}
