//
// This requires an external dependency, so install these first before you
// can call `make Anim_clock
//   sudo apt-get update
//   sudo apt-get install libgraphicsmagick++-dev libwebp-dev -y
//   make Anim_clock

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
  static int rgbBufferSize = 0;
  static uint8_t* displayBuffer = NULL;
  static uint8_t* scaleBuffer = NULL;
  
  if (rgbBuffer != NULL && (image.rows()*image.columns()*3) != rgbBufferSize){
	free(rgbBuffer);
	rgbBuffer = NULL;
  }
  if (rgbBuffer == NULL){
	rgbBufferSize = image.rows()*image.columns()*3;
	rgbBuffer = (rgb24*)malloc( rgbBufferSize );
  }
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

Magick::Image create_text_overlay(const char* text, const char* font_path, double pointsize, Magick::Image& pattern_frame) {

	// create transparent image for text
	Magick::Image text_image(Magick::Geometry(pattern_frame.columns(), pattern_frame.rows()), Magick::Color("transparent"));
	text_image.depth(8);
	
	// font settings
	text_image.font(font_path);
	text_image.fontPointsize(pointsize);
	
	// draw in white (as mask)
	text_image.fillColor("white");
	text_image.strokeColor("transparent");
	//text_image.draw(Magick::DrawableText(0, 0, text));
	text_image.annotate(text, MagickLib::CenterGravity);
	
	// copy pattern frame
	Magick::Image pattern_copy = pattern_frame;
	
	// use text as mask for pattern
	// copy alpha channel as mask for pattern
	text_image.negate(); // invert for correct mask
	pattern_copy.composite(text_image, 0, 0, MagickLib::CopyOpacityCompositeOp);
	
	return pattern_copy;
	
}

Magick::Image create_text_with_pattern_fill(const char* text, const char* font_path, double pointsize, int alignment, Magick::Image& pattern_frame) {
        
	Magick::Image pattern_copy = pattern_frame;
	
	// create text mask
	Magick::Image text_mask(Magick::Geometry(pattern_frame.columns(), pattern_frame.rows()), Magick::Color("black"));
	text_mask.depth(8);
	text_mask.font(font_path);
	text_mask.fontPointsize(pointsize);
	text_mask.fillColor("white");
	text_mask.strokeColor("transparent");
	text_mask.strokeAntiAlias(false);
	text_mask.strokeWidth(1);
	text_mask.antiAlias(false);
	//text_mask.draw(Magick::DrawableText(0, 32, text));
	text_mask.annotate(text, (Magick::GravityType) alignment); 

	
	// composite pattern with text mask
	Magick::Image result = pattern_copy;
	result.composite(text_mask, 0, 0, Magick::MultiplyCompositeOp);
	
	// set black pixel to transparent
	result.transparent(Magick::Color("black"));

	return result;
        
}


Magick::Image composite_frame(Magick::Image bgimage, Magick::Image patimage, int xpos, int ypos, const char* text, const char* font_path, double pointsize, int alignment) {
						 
	// get current background frame
	Magick::Image output = bgimage;
	
	int y = ypos;
	const int alignment_map[9] = {0,3,6,1,4,7,2,5,8};
	int align = alignment_map[alignment];
	if (align < 3) {		
		y = y - ((patimage.rows() - pointsize) / 2); //move to top
		align += 4;
	} else if (align > 5) { 
		y = y + ((patimage.rows() - pointsize) / 2); //move to bottom
		align -= 2;
	} else {					
		align += 1;
	}
	
	// create text overlay with pattern
	Magick::Image text_overlay = create_text_with_pattern_fill(
		text, font_path, pointsize, align, patimage);
	
	// composite text over background
	output.composite(text_overlay, xpos, y, Magick::OverCompositeOp);
	
	// send to display
	return output;
        
}

void showClock(const char* patternFile, const char* backgroundFile, int width, int height, int alignment, int xpos, int ypos, uint32_t displayTime, const char* font, int fontSize, const char* locale, const char* format, Magick::Color color){
		
	char text_buffer[256];
	struct timespec next_time;
	next_time.tv_sec = time(NULL);
	next_time.tv_nsec = 0;
	struct tm tm;
	
	uint32_t bgDelay = 0xFFFFFFFF;
	uint32_t patDelay = 0xFFFFFFFF;
	uint32_t refresh = 0xFFFFFFFF;
	
	bool update = true;
	uint32_t now = millis();
	uint32_t stopDisplay = now + displayTime;	

	ImageVector patimages = LoadImage(patternFile);
	if (!patimages.empty())
		patDelay = now + patimages[0].animationDelay();
	int patidx = 0;
	
	ImageVector bgimages = LoadImage(backgroundFile);
	if (!bgimages.empty())
		bgDelay = now + bgimages[0].animationDelay();
	int bgidx = 0;

	Magick::Image background (Magick::Geometry(width, height), Magick::Color("black"));
	Magick::Image pattern (Magick::Geometry(width, height), color);
	
	refresh = now + 100;
	
	while (!interrupt_received && now < stopDisplay) {
		
		now = millis();	

		if (now >= bgDelay){
			bgidx++;
			if (bgidx == bgimages.size())
				bgidx = 0;
			bgDelay = now + bgimages[bgidx].animationDelay();
			update = true;
		}
		if (now >= patDelay){
			patidx++;
			if (patidx == patimages.size())
				patidx = 0;
			patDelay = now + patimages[patidx].animationDelay();
			update = true;
		}
		if (now >= refresh){
			update = true;
			refresh = now + 100;
		}
		
		//
  
		if (update){
			update = false;
			next_time.tv_sec = time(NULL);
			localtime_r(&next_time.tv_sec, &tm);
			strftime(text_buffer, sizeof(text_buffer), format, &tm);
			// Compositing and rendering
			if(!patimages.empty()) {
				LOGTRACE("no background\n");
				pattern = patimages[patidx];
				pattern.scale(Magick::Geometry(width, height));
			} 
			if(!bgimages.empty()) {
				LOGTRACE("no pattern\n");
				background = bgimages[bgidx];
				background.scale(Magick::Geometry(width, height));
			}
			ShowImage(composite_frame(background, pattern, xpos, ypos, text_buffer, font, fontSize, alignment));
		}
	}
}	

static struct option long_options[] = {
	{ "led-rows=", required_argument, 0, 'h' },
	{ "led-cols=", required_argument, 0, 'w' },
	{ "led-chain=", required_argument, 0, 'c' },
	{ "animation-file", required_argument, 0, 'g' },
	{ "wait-time", required_argument, 0, 'W' },
	{ "time-format", required_argument, 0, 'T' },
	{ "clock_timeout", required_argument, 0, 't' },
	{ "x-origin", required_argument, 0, 'x' },
	{ "y-origin", required_argument, 0, 'y' },
	{ "font-file", required_argument, 0, 'f' },
	{ "font-size", required_argument, 0, 's' },
	{ "pattern-file", required_argument, 0, 'p' },
	{ "background-file", required_argument, 0, 'b' },
	{ "text-alignment", required_argument, 0, 'a' },
	{ "date-format", required_argument, 0, 'D' },
	{ "date_timeout", required_argument, 0, 'd' },
	{ "X-origin", required_argument, 0, 'X' },
	{ "Y-origin", required_argument, 0, 'Y' },
	{ "Font-file", required_argument, 0, 'F' },
	{ "Font-Size", required_argument, 0, 'S' },
	{ "Pattern-file", required_argument, 0, 'P' },
	{ "Background-file", required_argument, 0, 'B' },
	{ "Text-Alignment", required_argument, 0, 'A' },
	{ "Locale", required_argument, 0, 'L' },
	{ "Color", required_argument, 0, 'C' },
    { "log-level", required_argument, 0, 'l' },
    { "help", no_argument, 0, '?' },

    { 0, 0, 0, 0 } // End marker
};

void usage() {
	printf( "usage: ./Anim_clock [options]\n" );
	printf( "Options:\n" );
	printf( "    -h\t\t\t\t: Panel height in pixel. Typically 32 or 64. (Default: 32).\n" );
    printf( "    -w\t\t\t\t: Panel width. Typically 128 or 256. (Default: 128).\n" );
    printf( "    -g <animation-file>\t\t: Animation/Image file.\n" );
    printf( "    -W <wait time>\t\t: Wait time after animation/Image file.\n" );
	printf( "    -l <log-level>\t\t: Set log level (0-5)\n" );
	printf( "Clock options :\n" );
    printf( "    -T <time-format>\t\t: Format '%%H:%%M:%%S'. See strftime(), If not specified, clock will not be displayed\n");
    printf( "    -t <clock_timeout>\t\t: Display time in ms, If not specified, clock will not be displayed\n" );
    printf( "    -x <x-origin>\t\t: X-Origin of clock text (Default: 0)\n" );
    printf( "    -y <y-origin>\t\t: Y-Origin of clock text (Default: 0)\n" );
    printf( "    -f <font-file>\t\t: Font file to use for clock.\n" );
    printf( "    -s <font-size>\t\t: Font size to use for clock.\n" );
    printf( "    -p <pattern-file>\t\t: Use pattern file for clock characters.\n" );
    printf( "    -b <background-file>\t: Background image for clock display.\n" );
    printf( "    -a <text-alignment>\t\t: 0 : Top Left, 1: Left, 2: Bottom Left, 3: Top Middle, 4: Middle, 5: Bottom Middle, 6: Top Right, 7: Right, 8: Bottom Right.\n" );
	printf( "Date options :\n" );
    printf( "    -D <time-format>\t\t: Format '%%d/%%m/%%Y'. See strftime(), If not specified, date will not be displayed\n");
    printf( "    -d <date_timeout>\t\t: Display time in ms, If not specified, date will not be displayed\n" );
    printf( "    -X <x-origin>\t\t: X-Origin of date text (Default: 0)\n" );
    printf( "    -Y <y-origin>\t\t: Y-Origin of date text (Default: 0)\n" );
    printf( "    -F <font-file>\t\t: Font file to use for date.\n" );
    printf( "    -S <font-size>\t\t: Font size to use for date.\n" );
    printf( "    -P <pattern-file>\t\t: Use pattern file for date characters.\n" );
    printf( "    -B <background-file>\t: Background image for date display.\n" );
    printf( "    -A <text-alignment>\t\t: 0 : Top Left, 1: Left, 2: Bottom Left, 3: Top Middle, 4: Middle, 5: Bottom Middle, 6: Top Right, 7: Right, 8: Bottom Right.\n" );
    printf( "    -L <locale>\t\t\t: Text localisation (Default : fr_FR)\n" );
    printf( "    -C <r,g,b>\t\t\t: Color. Default 255,255,0\n" );
}


int main(int argc, char **argv) {

	int width = 128;
	int height = 32;
	int chain = 1;
	
	opterr = 0;
	
	Magick::InitializeMagick(*argv);
	
	init(SPI_EXTERNAL_RGB);
    
	int opt = 0;
	int longOptIndex = 0;
	
	const char *animationFile = NULL;
	int waitTime = 0;
	
	const char *clockFormat = "%H:%M:%S";
	int clockX = 0;
	int clockY = 0;
	int clockAlignment = 4;
	int clockDisplayTime = 3000;
	const char *clockBackgroundFile = NULL;
	const char *clockPatternFile = NULL;
	const char *clockFontFile = NULL;
	int clockFontSize = 20;
	const char *dateFormat = "%d %b %y";
	int dateX = 0;
	int dateY = 0;
	int dateAlignment = 4;
	int dateDisplayTime = 2500;
	const char *dateBackgroundFile = NULL;
	const char *datePatternFile = NULL;
	const char *dateFontFile = NULL;
	int dateFontSize = 13;
	const char *textLocale = NULL;
	std::string colorSpec;
	std::string rgbInput;
	
	while( ( opt = getopt_long( argc, argv, "h:w:c:g:l:W:T:t:x:y:f:s:p:b:a:D:d:X:Y:F:S:P:B:A:L:C:?", long_options, &longOptIndex ) ) != -1 ) {
		switch( opt ) {
		case 'h':
			height = atoi( optarg );
			break;
		case 'w':
			width = atoi( optarg );
			break;
		case 'c':
			chain = atoi( optarg );
			break;
		case 'g':
			animationFile = optarg;
			break;
		case 'W':
			waitTime = atoi( optarg );
			break;
		case 'T':
			clockFormat = optarg;
			break;
		case 't':
			clockDisplayTime = atoi( optarg );
			break;
		case 'x':
			clockX = atoi( optarg );
			break;
		case 'y':
			clockY = atoi( optarg );
			break;
		case 'f':
			clockFontFile = optarg;
			break;
		case 's':
			clockFontSize = atoi( optarg );
			break;
		case 'p':
			clockPatternFile = optarg;
			break;
		case 'b':
			clockBackgroundFile = optarg;
			break;
		case 'a':
			clockAlignment = atoi( optarg );;
			break;
		case 'D':
			dateFormat = optarg;
			break;
		case 'd':
			dateDisplayTime = atoi( optarg );
			break;
		case 'X':
			dateX = atoi( optarg );
			break;
		case 'Y':
			dateY = atoi( optarg );
			break;
		case 'F':
			dateFontFile = optarg;
			break;
		case 'S':
			dateFontSize = atoi( optarg );
			break;
		case 'P':
			datePatternFile = optarg;
			break;
		case 'B':
			dateBackgroundFile = optarg;
			break;
		case 'A':
			dateAlignment = atoi( optarg );;
			break;
		case 'L':
			textLocale = optarg;
			break;
		case 'C':
			rgbInput = optarg;
			colorSpec = "rgb(" + rgbInput + ")"; // Forms "rgb(255,0,0)"
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

	Magick::Color color(colorSpec);
	width = width * chain;
	
	signal(SIGTERM, InterruptHandler);
	signal(SIGINT, InterruptHandler);
	
	ImageVector bgimages = LoadImage(animationFile);
	Magick::Image background;
	int bgidx = 0;
	int patidx = 0;
	
	//display animation
	while (!interrupt_received) {
		background = bgimages[bgidx];
		if (!(background.rows() == 32 && background.columns() == 128))
			background.scale(Magick::Geometry(width, height));
		ShowImage(background);
		usleep(bgimages[bgidx].animationDelay() * 10000);  // 1/100s converted to usec
		bgidx++;
		if (bgidx == bgimages.size())
			break;
	}

	usleep(waitTime * 10000);  // 1/100s converted to usec
	
	//display date
	showClock(datePatternFile, dateBackgroundFile, width, height, dateAlignment, dateX, dateY, dateDisplayTime, dateFontFile, dateFontSize, textLocale, dateFormat, color);
	
	//display clock
	showClock(clockPatternFile, clockBackgroundFile, width, height, clockAlignment, clockX, clockY, clockDisplayTime, clockFontFile, clockFontSize, textLocale, clockFormat, color);
	
	printf("That\'s all folks !\n");

	deInit();

    return 0;
}
