
#pragma once

#include <stdint.h>

#define UART_DEVICE "/dev/ttyAMA0" // or /dev/serial0
#define SPI_DEVICE "/dev/spidev0.0"

#define BITS_PER_CHANNEL 6

#define PANEL_RGB 0
#define PANEL_RBG 1
#define PANEL_GRB 2
#define PANEL_GBR 3
#define PANEL_BRG 4
#define PANEL_BGR 5

#define DUMP_WITH_TICK 0
#define DUMP 1
#define CONFIG 2
#define PALETTE 3
#define PLANES_4 4
#define PLANES_N 5
#define RGB 6
#define RAW 7
#define SPI_NORMAL 8
#define SPI_RGB 9
#define SPI_EXTERNAL_RGB 10

#define HEADER_SIZE 4
#define TX_BUFFER_SIZE 256
#define GPIO2 25
#define GPIO1 7

typedef struct rgb24 {
	unsigned char red;
	unsigned char green;
	unsigned char blue;
} rgb24;


typedef enum {
	MMI_SPI_OFF = 0,
	MMI_SPI_NORMAL,
	MMI_SPI_RGB,
	MMI_SPI_EXTERNAL_RGB
}MMI_STATUS_t;

typedef enum { 	
	VirtualPinball,
	WPC ,
	SAM ,
	WhiteStar ,
	Spike ,
	DataEast ,
	Gottlieb1 ,
	Gottlieb2 ,
	Gottlieb3 ,
	Capcom ,
	AlvinG ,
	Spooky,
	DE128x16,
	Inder,
	Sleic,
	HomePin,
	SysXDMD,
	Capcom256 = 30
} DeviceMode;
	
typedef enum { 	
	PIN2DMD = 0,
	PIN2DMD_XL ,
	PIN2DMD_HD
} DeviceType;
	

#define	RESET 0x00
#define SWITCHMODE 0x02
#define SENDSETTINGS 0x10
#define	READSETTINGS 0x20
#define	DISPLAYUID 0xFF

typedef struct configDescriptor {
	unsigned char mode = VirtualPinball;
	unsigned char palette = 0x00;
	unsigned char smartDMD[8] = {0x0f, 0x0a, 0x0f, 0x0c, 0x0f, 0x00, 0x0f, 0x0f};
	unsigned char foo[6];
	unsigned char buffermode = 0x00;
	unsigned char scaler = 0x00;
	unsigned char copyPalToFlash = 0x00;
	unsigned char mmi = 0x00;
	unsigned char rgbseq = 0x01;
	unsigned char enhancer = 0x01;
	unsigned char rgbbrightness = 0x1E;
} configDescriptor;

int setupGpio();
int setupUart();
void sendUart( uint8_t* TransferBuffer, int len );
void readUart( uint8_t* TransferBuffer, int len );
int setupSpi();
void readSpi( uint8_t* rxBuffer, int len );
void transferSpi( uint8_t* txBuffer, uint8_t* rxBuffer );

int getConfig();

void init( uint8_t spi_init, bool withReset = false );
void deInit();

uint8_t* create_FrameBuffer( int width, int height, DeviceMode deviceMode, uint8_t* source, uint8_t* dest,
                             uint8_t* palette = (uint8_t*)0, bool rgb565 = false );

uint8_t* create_FrameFromRGB24( uint16_t width, uint16_t height, rgb24* rgb24source, uint8_t* dest );
uint8_t* create_FrameFromRGB24HD( uint16_t width, uint16_t height, uint8_t* rgbsource, uint8_t* dest, bool rgb565 = false);
uint8_t* create_FrameFromRGB565( uint16_t width, uint16_t height, uint16_t* rgb565source, uint8_t* dest );
uint8_t* create_FrameFromRGB565HD( uint16_t width, uint16_t height, uint16_t* rgbsource, uint8_t* dest );
								  
uint8_t* scaleHD( uint16_t destWidth, uint16_t destHeight, uint8_t* src, uint8_t* dest , int byteSize);
uint8_t* scaleDouble( uint16_t destWidth, uint16_t destHeight, uint8_t* src, uint8_t* dest , int byteSize);

uint8_t* create_RGBFromFrame(uint16_t width, uint16_t height, uint8_t* src, uint8_t* dest, bool rgb565 = false );
uint8_t* create_RGBHDFromFrame(uint16_t width, uint16_t height, uint8_t* src, uint8_t* dest, bool rgb565 = false);

extern DeviceMode deviceMode;
extern uint8_t numberOfPlanes;
extern int planesize;
extern const uint8_t* GAMMA_TABLE;
extern uint8_t cmd[][4];
extern char UIDString[16];
extern MMI_STATUS_t Mmi_Status;
extern char versionString[6];
extern DeviceType deviceType;

extern int RX_BUFFER_SIZE;
extern int DISPLAYBUFFER_SIZE;
extern int WIDTH;
extern int HEIGHT;
extern int CHUNK_SIZE;
