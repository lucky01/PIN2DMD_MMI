#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <curl/curl.h>
#include <json-c/json.h>
#include <math.h>
#include <time.h>
#include <Magick++.h>
#include <string>
#include <getopt.h>

#define WIDTH 128
#define HEIGHT 32
#define API_URL "http://api.openweathermap.org/data/2.5/forecast?q=%s,%s&appid=%s&units=%s"

// Structure for hourly weather data
struct HourlyWeather {
    int hour;           // Hour (0-23)
    double temp;        // Temperature in °C
    char description[64]; // Weather description
    char icon[4];       // Weather icon code
};

// Structure for HTTP Response
struct MemoryStruct {
    char *memory;
    size_t size;
};

// Pixel buffer for the graphic
unsigned char display[HEIGHT][WIDTH];

// Callback function for CURL
static size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp) {
    size_t realsize = size * nmemb;
    struct MemoryStruct *mem = (struct MemoryStruct *)userp;
    
    char *ptr = (char*)realloc(mem->memory, mem->size + realsize + 1);
    if(!ptr) {
        printf("Not enough memory!\n");
        return 0;
    }
    
    mem->memory = ptr;
    memcpy(&(mem->memory[mem->size]), contents, realsize);
    mem->size += realsize;
    mem->memory[mem->size] = 0;
    
    return realsize;
}

// Initialize display
void init_display() {
    for(int y = 0; y < HEIGHT; y++) {
        for(int x = 0; x < WIDTH; x++) {
            display[y][x] = 0;
        }
    }
}

// Set pixel
void set_pixel(int x, int y, unsigned char value) {
    if(x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
        display[y][x] = value;
    }
}

// Draw text (simple 3x5 pixel font)
void draw_char(int x, int y, char c, unsigned char value) {
    // Very simple bitmap font for numbers and some letters
    unsigned char font[128][5] = {0};
    
    // Digit '0'
    font['0'][0] = 0b01110; font['0'][1] = 0b10001; font['0'][2] = 0b10001; 
    font['0'][3] = 0b10001; font['0'][4] = 0b01110;
    
    // Additional digits
    font['1'][0] = 0b00100; font['1'][1] = 0b01100; font['1'][2] = 0b00100; 
    font['1'][3] = 0b00100; font['1'][4] = 0b01110;
    
    font['2'][0] = 0b01110; font['2'][1] = 0b10001; font['2'][2] = 0b00110; 
    font['2'][3] = 0b01000; font['2'][4] = 0b11111;
    
    font['3'][0] = 0b11110; font['3'][1] = 0b00001; font['3'][2] = 0b01110; 
    font['3'][3] = 0b00001; font['3'][4] = 0b11110;
    
    font['4'][0] = 0b10001; font['4'][1] = 0b10001; font['4'][2] = 0b11111; 
    font['4'][3] = 0b00001; font['4'][4] = 0b00001;
    
    font['5'][0] = 0b11111; font['5'][1] = 0b10000; font['5'][2] = 0b11110; 
    font['5'][3] = 0b00001; font['5'][4] = 0b11110;
    
    font['6'][0] = 0b01110; font['6'][1] = 0b10000; font['6'][2] = 0b11110; 
    font['6'][3] = 0b10001; font['6'][4] = 0b01110;
    
    font['7'][0] = 0b11111; font['7'][1] = 0b00001; font['7'][2] = 0b00010; 
    font['7'][3] = 0b00100; font['7'][4] = 0b01000;
    
    font['8'][0] = 0b01110; font['8'][1] = 0b10001; font['8'][2] = 0b01110; 
    font['8'][3] = 0b10001; font['8'][4] = 0b01110;
    
    font['9'][0] = 0b01110; font['9'][1] = 0b10001; font['9'][2] = 0b01111; 
    font['9'][3] = 0b00001; font['9'][4] = 0b01110;
    
    font['-'][0] = 0b00000; font['-'][1] = 0b00000; font['-'][2] = 0b11111; 
    font['-'][3] = 0b00000; font['-'][4] = 0b00000;
    
    font[':'][0] = 0b00000; font[':'][1] = 0b00100; font[':'][2] = 0b00000; 
    font[':'][3] = 0b00100; font[':'][4] = 0b00000;
    
	font['C'][0] = 0b10000; font['C'][1] = 0b00111; font['C'][2] = 0b01000; 
    font['C'][3] = 0b01000; font['C'][4] = 0b00111;
    
    font['%'][0] = 0b11001; font['%'][1] = 0b00010; font['%'][2] = 0b00100; 
    font['%'][3] = 0b01000; font['%'][4] = 0b10011;
    
	font['.'][0] = 0b00000; font['.'][1] = 0b00000; font['.'][2] = 0b00000; 
    font['.'][3] = 0b00000; font['.'][4] = 0b00100;
    
    for(int row = 0; row < 5; row++) {
        for(int col = 0; col < 5; col++) {
            if(font[(int)c][row] & (1 << (4 - col))) {
                set_pixel(x + col, y + row, value);
            }
        }
    }
}

// Draw string
void draw_string(int x, int y, const char *str, unsigned char value) {
    int offset = 0;
    while(*str) {
        draw_char(x + offset, y, *str, value);
        offset += 6; // 5 pixel width + 1 pixel spacing
        str++;
    }
}

// Fetch weather data from API (forecast for next 4 hours)
int fetch_weather_data(const char *zip, const char *country, const char *api_key, const char *units, HourlyWeather *hours, int count) {
    CURL *curl;
    CURLcode res;
    struct MemoryStruct chunk;
    char url[512];
    
    chunk.memory = (char*)malloc(1);
    chunk.size = 0;
    
    snprintf(url, sizeof(url), API_URL, zip, country, api_key, units);
    
    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();
    
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&chunk);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, "weather-display/1.0");
        
        res = curl_easy_perform(curl);
        
        if(res != CURLE_OK) {
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
            curl_easy_cleanup(curl);
            free(chunk.memory);
            curl_global_cleanup();
            return -1;
        }
        
        curl_easy_cleanup(curl);
    }
    
    curl_global_cleanup();
    
    // Parse JSON
    struct json_object *parsed_json;
    struct json_object *list_array;
    
    parsed_json = json_tokener_parse(chunk.memory);
    
    if(parsed_json == NULL) {
        fprintf(stderr, "Error parsing JSON data\n");
        free(chunk.memory);
        return -1;
    }
    
    // Extract forecast list
    if(!json_object_object_get_ex(parsed_json, "list", &list_array)) {
        fprintf(stderr, "No forecast data found\n");
        json_object_put(parsed_json);
        free(chunk.memory);
        return -1;
    }
    
    int array_len = json_object_array_length(list_array);
    if(array_len < count) count = array_len;
    
    // Extract data for each hour
    for(int i = 0; i < count; i++) {
        struct json_object *item = json_object_array_get_idx(list_array, i);
        struct json_object *main_obj, *weather_array, *weather_obj;
        struct json_object *temp_obj, *desc_obj, *icon_obj, *dt_obj;
        
        // Get timestamp and calculate hour
        if(json_object_object_get_ex(item, "dt", &dt_obj)) {
            time_t timestamp = json_object_get_int64(dt_obj);
            struct tm *timeinfo = localtime(&timestamp);
            hours[i].hour = timeinfo->tm_hour;
        }
        
        // Get temperature
        if(json_object_object_get_ex(item, "main", &main_obj)) {
            json_object_object_get_ex(main_obj, "temp", &temp_obj);
            hours[i].temp = json_object_get_double(temp_obj);
        }
        
        // Get weather description and icon
        if(json_object_object_get_ex(item, "weather", &weather_array)) {
            weather_obj = json_object_array_get_idx(weather_array, 0);
            
            json_object_object_get_ex(weather_obj, "description", &desc_obj);
            strncpy(hours[i].description, json_object_get_string(desc_obj), 63);
            hours[i].description[63] = '\0';
            
            json_object_object_get_ex(weather_obj, "icon", &icon_obj);
            strncpy(hours[i].icon, json_object_get_string(icon_obj), 3);
            hours[i].icon[3] = '\0';
        }
    }
    
    json_object_put(parsed_json);
    free(chunk.memory);
    
    return 0;
}

// Visualize 4-hour weather forecast
void visualize_weather(HourlyWeather *hours, int count) {
    init_display();
    
    // Each column is 32 pixels wide (128/4)
    int col_width = 32;
    
    for(int i = 0; i < count && i < 4; i++) {
        int x_offset = i * col_width;
        
        // Draw time (centered in column)
        char time_str[8];
        snprintf(time_str, sizeof(time_str), "%02d:00", hours[i].hour);
        draw_string(x_offset + 2, 2, time_str, 1);

        // Draw temperature (centered)
        char temp_str[8];
        snprintf(temp_str, sizeof(temp_str), "%.0fC", hours[i].temp);
        int temp_x = x_offset + (col_width - strlen(temp_str) * 6) / 2;
        draw_string(temp_x, 25, temp_str, 1);
    }
}

// Save display as PNG with Magick++ and composite colored weather icons
void save_as_png_magick(const char *filename, HourlyWeather *hours, int count, std::string colorSpec, int width, int height) {
    try {
        // Create image with white background
        Magick::Image image(Magick::Geometry(WIDTH, HEIGHT), Magick::Color("black"));
        
        // Set pixel format
        image.type(Magick::TrueColorType);
        image.depth(8);
        
        // Set pixels from display buffer (text and fallback icons)
        for(int y = 0; y < HEIGHT; y++) {
            for(int x = 0; x < WIDTH; x++) {
                if(display[y][x]) {
                    // Black pixels for set bits
                    image.pixelColor(x, y, Magick::Color(colorSpec));
                }
            }
        }
		
		if(width == 256 && height == 64){
			image.scale(Magick::Geometry(width, height));
		}
				
        // Composite colored weather icons from images/ directory
        
        for(int i = 0; i < count && i < 4; i++) {
            // Construct filename: weather_icons/[icon_code].png
            // e.g., weather_icons/01d.png or weather_icons/10n.png
			// Icon codes from OpenWeatherMap:
			// 01d/01n = clear sky
			// 02d/02n = few clouds
			// 03d/03n = scattered clouds
			// 04d/04n = broken clouds
			// 09d/09n = shower rain
			// 10d/10n = rain
			// 11d/11n = thunderstorm
			// 13d/13n = snow
			// 50d/50n = mist
            char icon_file[128];
            snprintf(icon_file, sizeof(icon_file), "weather_icons/%s.png", hours[i].icon);
            
            try {
                // Load the colored icon from local directory
                Magick::Image icon(icon_file);
				
				icon.antiAlias(false);
                
				int x_pos, y_pos, col_width = 0;
                // Resize icon to fit (approximately 20x20 pixels)
				if(width == 256 && height == 64){
					col_width = 64;
					icon.resize(Magick::Geometry(60, 60));
                
					// Calculate position (centered in column, middle area)
					x_pos = i * col_width + (col_width - 60) / 2;
					y_pos = 2;  // Position between time and temperature
				}
				else {
					col_width = 32;
					icon.resize(Magick::Geometry(30, 30));
                
					// Calculate position (centered in column, middle area)
					x_pos = i * col_width + (col_width - 30) / 2;
					y_pos = 1;  // Position between time and temperature
				}
                
                // Composite the icon onto the main image
                image.composite(icon, x_pos, y_pos, Magick::OverCompositeOp);
                
                //printf("Icon %s composited successfully from %s\n", hours[i].icon, icon_file);
                
            } catch(Magick::Exception &icon_error) {
                fprintf(stderr, "Failed to load/composite icon from %s: %s\n", 
                        icon_file, icon_error.what());
                fprintf(stderr, "Using fallback monochrome icon instead\n");
                // Fallback icon already drawn in display buffer
            }
        }
        
        // Set PNG options
        image.magick("PNG");
        image.quality(100);
        
        // Save image
        image.write(filename);
        
        printf("PNG graphic saved with Magick++ as: %s\n", filename);
        
    } catch(Magick::Exception &error) {
        fprintf(stderr, "Magick++ error: %s\n", error.what());
    }
}

static struct option long_options[] = {
    { "help", no_argument, 0, '?' },
 	{ "width=", required_argument, 0, 'X' },
	{ "height=", required_argument, 0, 'Y' },
	{ "id=", required_argument, 0, 'i' },
	{ "zip-code=", required_argument, 0, 'z' },
	{ "counry=", required_argument, 0, 'c' },
	{ "units=", required_argument, 0, 'u' },
	{ "color=", required_argument, 0, 'C' },
	{ "filename=", required_argument, 0, 'F' },
	

    { 0, 0, 0, 0 } // End marker
};

void usage() {
	printf( "Usage: Weather [options]\n" );
	printf( "Options:\n" );
	printf( "    -i           : OpenWeather ID\n" );
	printf( "    -c           : Country\n" );
	printf( "    -z           : Zip Code\n" );
	printf( "    -u           : Units\n" );
	printf( "    -C           : Text Color\n" );
	printf( "    -Y           : Height\n" );
    printf( "    -X           : Width\n" );
	printf( "    -F           : Filename (default /tmp/meteo.png)\n" );
}

int main(int argc, char *argv[]) {
	
	int width = 128;
	int height = 32;
	
	opterr = 0;
	
	int opt = 0;
	int longOptIndex = 0;
	
	const char *filename = "/tmp/meteo.png";
	const char* country = NULL;
	const char* zip = NULL;
	const char* api_key = NULL;
	const char* units = "metric";
	std::string colorSpec;
	std::string rgbInput;
	
	while( ( opt = getopt_long( argc, argv, "c:z:i:u:F:C:X:Y:?", long_options, &longOptIndex ) ) != -1 ) {
		switch( opt ) {
		case 'c':
			country = optarg;
			break;
		case 'z':
			zip = optarg;
			break;
		case 'i':
			api_key = optarg;
			break;
		case 'u':
			units = optarg;
			break;
		case 'C':
			rgbInput = optarg;
			colorSpec = "rgb(" + rgbInput + ")"; // Forms "rgb(255,0,0)"
			break;
		case 'X':
			width = atoi( optarg );
			break;
		case 'Y':
			height = atoi( optarg );
			break;
		case 'F':
			filename = optarg;
			break;
		case '?':
			if (argc == 2){			
				usage();
				exit(0);
			}			
			break;
		default:
			fprintf(stderr, "Unknown option: %c\n", opt );
		}
	}
  
	if (argc - optind != 0 ){
		fprintf(stderr, "Not enough arguments provided\n");
		usage();
		exit( 1 );
	}

    // Initialize Magick++
    Magick::InitializeMagick(*argv);
    
    HourlyWeather hours[4];
    
    if(fetch_weather_data(zip, country, api_key, units, hours, 4) != 0) {
        fprintf(stderr, "Error fetching weather data\n");
        return 1;
    }
    
    visualize_weather(hours, 4);
    
	save_as_png_magick(filename, hours, 4, colorSpec, width, height);
    
    return 0;
}
