//
// This requires an external dependency, so install these first before you
// can call `Show_Image`
//   sudo apt-get update
//   sudo apt-get install libgraphicsmagick++-dev libwebp-dev -y
//   make Show_Image

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <getopt.h>

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

// Given the filename, load the image and scale to the size of the
// matrix.
// // If this is an animated image, the resutlting vector will contain multiple.
static ImageVector LoadImage(const char *filename) {
  ImageVector result;

  ImageVector frames;
  try {
    readImages(&frames, filename);
  } catch (std::exception &e) {
    if (e.what())
      fprintf(stderr, "%s\n", e.what());
    return result;
  }

  if (frames.empty()) {
    fprintf(stderr, "No image found.");
    return result;
  }

  // Animated images have partial frames that need to be put together
  if (frames.size() > 1) {
    Magick::coalesceImages(&result, frames.begin(), frames.end());
  } else {
    result.push_back(frames[0]); // just a single still image.
  }

  return result;
}

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

void ShowAnimatedImage(const ImageVector &images, int width, int height) {
	
	Magick::Image scaledImage;
	
    for (const auto &image : images) {
		if (interrupt_received) break;
		scaledImage = image;
		if (!(image.rows() == 32 && image.columns() == 128)){
			scaledImage.scale(Magick::Geometry(width, height));
		}
		ShowImage(scaledImage);
		usleep(image.animationDelay() * 10000);  // 1/100s converted to usec
    }
}

static struct option long_options[] = {
    { "help", no_argument, 0, '?' },
    { "log-level", required_argument, 0, 'l' },
	{ "led-cols=", required_argument, 0, 'w' },
	{ "led-chain=", required_argument, 0, 'c' },
	{ "led-rows=", required_argument, 0, 'h' },
	{ "file", required_argument, 0, 'F' },
	{ "timeout", required_argument, 0, 'T' },
	{ "center", no_argument, 0, 'C' },

    { 0, 0, 0, 0 } // End marker
};

void usage() {
	printf( "Usage: Show_Image [options] -F filename\n" );
	printf( "Options:\n" );
	printf( "  -F <file>\t\t: Animation/Image file.\n" );
    printf( "  -C\t\t\t: Center images.\n" );
	printf( "  -T\t\t\t: Timeout.\n" );
	printf( "  -h\t\t\t: Panel height. Typically 32 or 64. (Default: 32).\n" );
    printf( "  -w\t\t\t: Panel width. Typically 128 or 256. (Default: 128).\n" );
 	printf( "  -l <log-level>\t: Set log level (0-5)\n" );
}

int main(int argc, char *argv[]) {
	
  int width = 128;
  int height = 32;
  int chain = 1;
  int timeout = 3000;
  
  Magick::InitializeMagick(*argv);

  init(SPI_EXTERNAL_RGB);
  
  opterr = 0;
  int opt = 0;
  int longOptIndex = 0;
  const char *filename = "boot.gif";
  
  while( ( opt = getopt_long( argc, argv, "F:l:w:h:c:T:C?", long_options, &longOptIndex ) ) != -1 ) {
		switch( opt ) {
		case 'F':
			filename = optarg;
		break;
		case 'C':
		break;
		case 'c':
			chain = atoi( optarg );
			break;
		case 'w':
			width = atoi( optarg );
			break;
		case 'h':
			height = atoi( optarg );
			break;
		case 'T':
			timeout = atoi( optarg );
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
 
  signal(SIGTERM, InterruptHandler);
  signal(SIGINT, InterruptHandler);
  
  ImageVector images = LoadImage(filename);
  Magick::Image scaledImage;

  switch (images.size()) {
  case 0:   // failed to load image.
    break;
  case 1:   // Simple example: one image to show
	scaledImage = images[0];
	if (!(images[0].rows() == 32 && images[0].columns() == 128))
		scaledImage.scale(Magick::Geometry(width, height));
	ShowImage(scaledImage);
	usleep(timeout*1000);
    break;
  default:  // More than one image: this is an animation.
	ShowAnimatedImage(images, width, height);
    break;
  }

  deInit();
  
  printf("That\'s all folks !\n");

  return 0;
}
