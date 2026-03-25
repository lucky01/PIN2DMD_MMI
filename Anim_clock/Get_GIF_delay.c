//
// This requires an external dependency, so install these first before you
// can call `Show_Image`
//   sudo apt-get update
//   sudo apt-get install libgraphicsmagick++-dev libwebp-dev -y
//   make Show_Image

#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <getopt.h>

#include "log.h"

#include <exception>
#include <Magick++.h>
#include <magick/image.h>

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
    printf("N/A\n");
    return result;
  }

  if (frames.empty()) {
    printf("N/A\n");
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

void PrintAnimatedImageDelay(const ImageVector &images) {

	int delay = 0;
    for (const auto &image : images) {
      delay += image.animationDelay() * 10;
    }
	printf( "Delay=%d\n",delay );
}

static struct option long_options[] = {
    { "help", no_argument, 0, '?' },
    { "log-level", required_argument, 0, 'l' },
	{ "file", required_argument, 0, 'F' },

    { 0, 0, 0, 0 } // End marker
};

void usage() {
	printf( "Usage: Get_GIF_delay [options] -F filename\n" );
	printf( "Options:\n" );
	printf( "  -F <file>\t\t: Animation/Image file.\n" );
 	printf( "  -l <log-level>\t: Set log level (0-5)\n" );
}

int main(int argc, char *argv[]) {
	
  Magick::InitializeMagick(*argv);

  opterr = 0;
  int opt = 0;
  int longOptIndex = 0;
  const char *filename;
  
  while( ( opt = getopt_long( argc, argv, "F:l:w:h:c:C?", long_options, &longOptIndex ) ) != -1 ) {
		switch( opt ) {
		case 'F':
			filename = optarg;
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
  
  ImageVector images = LoadImage(filename);

  switch (images.size()) {
  case 0:   // failed to load image.
    break;
  case 1:   // Simple example: one image to show
	printf("N/A\n");
    break;
  default:  // More than one image: this is an animation.
    PrintAnimatedImageDelay(images);
    break;
  }

  return 0;
}
