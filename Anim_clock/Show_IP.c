//
// This requires an external dependency, so install these first before you
// can call `make Show_IP
//   sudo apt-get update
//   sudo apt-get install libgraphicsmagick++-dev libwebp-dev -y
//   make Show_IP

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <getopt.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>

#include "log.h"
#include "pin2mmi.h"

#include <exception>
#include <Magick++.h>
#include <magick/image.h>

// Make sure we can exit gracefully when Ctrl-C is pressed.
volatile bool interrupt_received = false;
static void InterruptHandler(int signo) {
  interrupt_received = true;
}

using ImageVector = std::vector<Magick::Image>;

void ShowImage(const Magick::Image &image) {
	
  static rgb24* rgbBuffer = NULL;
  static uint8_t* displayBuffer = NULL;
  static uint8_t* scaleBuffer = NULL;
  
  if (rgbBuffer == NULL)
	  rgbBuffer = (rgb24*)malloc( image.rows()*image.columns()*3 );
  if (displayBuffer == NULL)
	  displayBuffer = (uint8_t*)malloc( DISPLAYBUFFER_SIZE );
	
  for (size_t y = 0; y < image.rows(); ++y) {
    for (size_t x = 0; x < image.columns(); ++x) {
      const Magick::Color &c = image.pixelColor(x, y);
      if (c.alphaQuantum() < 256) {
		rgbBuffer[x + (y * image.columns())].red = ScaleQuantumToChar(c.redQuantum());
		rgbBuffer[x + (y * image.columns())].green = ScaleQuantumToChar(c.greenQuantum());
		rgbBuffer[x + (y * image.columns())].blue = ScaleQuantumToChar(c.blueQuantum());
      }
    }
  }
  if (image.columns() == 128 && image.rows() == 32 && deviceType == PIN2DMD_HD) {
	if (scaleBuffer == NULL)
		scaleBuffer = (uint8_t*)malloc( DISPLAYBUFFER_SIZE );
	scaleHD(256, 64, (uint8_t*)rgbBuffer, scaleBuffer, 3);
	create_FrameFromRGB24HD( 256, 64, scaleBuffer, displayBuffer);
  }
  else if (image.columns() == 256 && image.rows() == 64 && deviceType == PIN2DMD_HD) {
	create_FrameFromRGB24HD( 256, 64,(uint8_t*) rgbBuffer, displayBuffer);
  } else {
	create_FrameFromRGB24( 128, 32, (rgb24*) rgbBuffer, displayBuffer);
  }
  transferSpi(displayBuffer, NULL);
}

char* print_interface_status(const char *if_target, char* text_buffer) {
    struct ifaddrs *ifaddr, *ifa;
    int found = 0;

	if (getifaddrs(&ifaddr) == -1) {
        sprintf(text_buffer, "%-6s: error", if_target);
        return text_buffer;
    }
	
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL || strcmp(ifa->ifa_name, if_target) != 0)
            continue;

        // Nur IPv4 Adressen berücksichtigen
        if (ifa->ifa_addr->sa_family == AF_INET) {
            char ip[INET_ADDRSTRLEN];
            struct sockaddr_in *pAddr = (struct sockaddr_in *)ifa->ifa_addr;
            inet_ntop(AF_INET, &pAddr->sin_addr, ip, sizeof(ip));
            
			if (ifa->ifa_flags & IFF_RUNNING)
				sprintf(text_buffer,"%-6s: %-15s", if_target, ip);
			else 
				sprintf(text_buffer,"%-6s: not connected", if_target);
			
            found = 1;
            break; 
        }
    }

    if (!found) {
        sprintf(text_buffer,"%-6s: not available", if_target);
    }
	
	return text_buffer;

    freeifaddrs(ifaddr);
}

static struct option long_options[] = {
    { "help", no_argument, 0, '?' },
    { "log-level", required_argument, 0, 'l' },
	{ "led-cols=", required_argument, 0, 'w' },
	{ "led-chain=", required_argument, 0, 'c' },
	{ "led-rows=", required_argument, 0, 'h' },

    { 0, 0, 0, 0 } // End marker
};

void usage() {
	printf( "Usage: Show_IP [options]\n" );
	printf( "Options:\n" );
	printf( "    -h\t\t\t: Panel rows. Typically 8, 16, 32 or 64. (Default: 32).\n" );
    printf( "    -w\t\t\t: Panel columns. Typically 32 or 64. (Default: 32).\n" );
	printf( "    -l <log-level>\t: Set log level (0-5)\n" );
}

int main(int argc, char *argv[]) {
	
  int width = 128;
  int height = 32;
  int chain = 1;
  
  opterr = 0;
  
  Magick::InitializeMagick(*argv);

  init(SPI_EXTERNAL_RGB);
  
  int opt = 0;
  int longOptIndex = 0;
  
  while( ( opt = getopt_long( argc, argv, "l:w:h:c?", long_options, &longOptIndex ) ) != -1 ) {
	switch( opt ) {
	case 'c':
		chain = atoi( optarg );
		break;
	case 'w':
		width = atoi( optarg );
		break;
	case 'h':
		height = atoi( optarg );
		break;
	case 'l':
		setLogLevel( atoi( optarg ) );
		break;
	case '?':
		if (argc == 2){			
			usage();
			exit(0);
		}			
		break;
	default:
		LOGERROR( "Unknown option: %c", opt );
	}
  }

  if (argc - optind != 0 ){
	LOGERROR( "not enough arguments provided");
	usage();
	exit( 1 );
  }

  width = width * chain;
  char text_buffer[25];
 
  signal(SIGTERM, InterruptHandler);
  signal(SIGINT, InterruptHandler);
  
  Magick::Image display(Magick::Geometry(128, 32), Magick::Color("black"));

  display.fillColor("red");       
  display.fontPointsize(8);         
  display.font("fixedsys.ttf");     
  display.antiAlias(false);
  display.annotate("IP Configuration:", Magick::Geometry("128x32+0-10"), Magick::CenterGravity);
  display.annotate(print_interface_status("wlan0", text_buffer), Magick::Geometry("+0+0"),  Magick::CenterGravity);
  display.annotate(print_interface_status("eth0", text_buffer), Magick::Geometry("+0+10"), Magick::CenterGravity);

  ShowImage(display);
  
  sleep(5);
  
  printf("That\'s all folks !\n");

  deInit();

  return 0;
}
