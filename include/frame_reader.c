#include "frame_reader.h"
#include "log.h"
#include <stdlib.h>
#include <string.h>

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
    
    reader->has_more_frames = 1;
    return 0;
}

int frame_reader_read_next(frame_reader_t *reader, frame_t *frame, int skipFrames) {
    do {
        if (!reader || !frame || !reader->file || !reader->has_more_frames) {
            return -1;
        }
        
        char line[256];
        
        // Read timestamp line
        if (!fgets(line, sizeof(line), reader->file)) {
            reader->has_more_frames = 0;
            return -1;
        }
        
        // Parse timestamp (format: 0x12345678)
        if (sscanf(line, "0x%x", &frame->timestamp ) != 1) {
            reader->has_more_frames = 0;
            return -1;
        }
        
        // Read 32 lines of frame data (128 hex chars each)
        for (int row = 0; row < FRAME_HEIGHT; row++) {
            if (!fgets(line, sizeof(line), reader->file)) {
                reader->has_more_frames = 0;
                return -1;
            }
            
            if( strlen(line) < 128 ) {
                LOGERROR("unexpected frame line: %s", line);
            } else {
                // Convert hex string to binary data
                for (int col = 0; col < FRAME_WIDTH; col++ ) {
                    frame->data[row * FRAME_WIDTH + col] = 
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
