#ifndef FRAME_READER_H
#define FRAME_READER_H

#include <stdio.h>
#include <stdint.h>

extern int dump_width;
extern int dump_height;

typedef struct {
    uint32_t delay;
    uint8_t* data;
} frame_t;

typedef struct {
    FILE *file;
    int has_more_frames;
} frame_reader_t;

/**
 * Initialize frame reader with a file
 * Returns 0 on success, -1 on error
 */
int frame_reader_init(frame_reader_t *reader, const char *filename);

/**
 * Read the next frame from the file
 * Returns 0 on success, -1 on error or end of file
 */
int frame_reader_read_next(frame_reader_t *reader, frame_t *frame, int skipFrames);

/**
 * Check if there are more frames to read
 * Returns 1 if more frames available, 0 otherwise
 */
int frame_reader_has_more(frame_reader_t *reader);

/**
 * Rewind to first frame
*/
int frame_reader_rewind(frame_reader_t *reader);

/**
 * Close the frame reader and cleanup resources
 */
void frame_reader_close(frame_reader_t *reader);

#endif /* FRAME_READER_H */
