#include "frame_reader.h"
#include "log.h"
#include <stdlib.h>
#include <string.h>

int dump_width = 0;
int dump_height = 0;
char line[260];

/**
 * Convert hex character to binary value
 */
static uint8_t hex_char_to_byte(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    } else if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return 0;
}

/**
 * Convert two hex characters to one byte
 */
static uint8_t hex_to_byte(char high, char low) {
    return (hex_char_to_byte(high) << 4) | hex_char_to_byte(low);
}

int frame_reader_init(frame_reader_t *reader, const char *filename) {
    if (!reader || !filename) {
        return -1;
    }
    
    reader->file = fopen(filename, "r");
    if (!reader->file) {
        return -1;
    }

    fgets(line, sizeof(line), reader->file); // read timestamp
    fgets(line, sizeof(line), reader->file); // read first row of frame
    
    dump_width = strlen(line);
	
	if ( dump_width > 256 ){
		dump_width = 256;
		dump_height = 64;
	} else if ( dump_width > 192 ){
		dump_width = 192;
		dump_height = 64;
	} else {
		dump_width = 128;
		dump_height = 32;
	}

    fseek(reader->file, 0, SEEK_SET);

    reader->has_more_frames = 1;
    return 0;
}

int frame_reader_read_next(frame_reader_t *reader, frame_t *frame, int skipFrames) {
    do {
        if (!reader || !frame || !reader->file || !reader->has_more_frames) {
            return -1;
        }
        
	uint32_t timestamp_current, timestamp_next = 0;
        
        // Read timestamp line
        if (!fgets(line, sizeof(line), reader->file)) {
            reader->has_more_frames = 0;
            return -1;
        }
        
		
        // Parse timestamp (format: 0x12345678)
        if (sscanf(line, "0x%x", &timestamp_current ) != 1) {
            reader->has_more_frames = 0;
            return -1;
        }
	
        // Read all lines of frame data 
        for (int row = 0; row < dump_height; row++) {
            if (!fgets(line, sizeof(line), reader->file)) {
                reader->has_more_frames = 0;
                return -1;
            }
            
            if( strlen(line) < 128 ) {
                LOGERROR("unexpected frame line: %s", line);
            } else {
                // Convert hex string to binary data
                for (int col = 0; col < dump_width; col++ ) {
                    frame->data[row * dump_width + col] = 
                        hex_to_byte(0, line[col]);
                }
            }
        }

        // Skip empty line between frames (if present)
        fgets(line, sizeof(line), reader->file);
        
        // Check if there's another frame by trying to peek at the next line
        long pos = ftell(reader->file);
        if (!fgets(line, sizeof(line), reader->file)) {
            reader->has_more_frames = 0;
        } else {
            // Rewind to the position before the peek
            fseek(reader->file, pos, SEEK_SET);
            // Check if the line looks like a timestamp
            if (strncmp(line, "0x", 2) != 0) {
                reader->has_more_frames = 0;
            } else {
				sscanf(line, "0x%x", &timestamp_next );
				frame->delay = timestamp_next - timestamp_current;
			}

        }
    } while( skipFrames-- > 0 );
    
    return 0;
}

int frame_reader_has_more(frame_reader_t *reader) {
    if (!reader) {
        return 0;
    }
    return reader->has_more_frames;
}

void frame_reader_close(frame_reader_t *reader) {
    if (reader && reader->file) {
        fclose(reader->file);
        reader->file = NULL;
        reader->has_more_frames = 0;
    }
}
