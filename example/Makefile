# Compiler and flags
CC              = g++
CFLAGS          = -g $(INCDIR)
LDFLAGS         = -lpigpio

# Directories
HOME_DIR        = $(HOME)
BUILD_DIR       = ../build
INC_DIRS        = /usr/local/include ../include
INCDIR          = $(foreach dir,$(INC_DIRS),-I$(dir))
LIBDIR          = /usr/local/lib/

# Source files and objects
SRCS            = ../include/pin2mmi.c \
                  ../include/log.c \
                  ../include/frame_reader.c
OBJS            = $(SRCS:.c=.o)

# Targets
TARGETS         = $(BUILD_DIR)/dmd_player \
                  $(BUILD_DIR)/dump_dmd \
                  $(BUILD_DIR)/display_ppm \
                  $(BUILD_DIR)/screenshot \
                  $(BUILD_DIR)/spi_loop \
                  $(BUILD_DIR)/reset

# Default target
all: $(BUILD_DIR) $(TARGETS)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Build targets
$(BUILD_DIR)/dmd_player: dmd_player.o ../include/frame_reader.o ../include/pin2mmi.o ../include/log.o
	$(CC) -I$(INCDIR) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/dump_dmd: dump_dmd.o ../include/pin2mmi.o ../include/log.o
	$(CC) -I$(INCDIR) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/display_ppm: display_ppm.o ../include/pin2mmi.o ../include/log.o
	$(CC) -I$(INCDIR) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/screenshot: screenshot.o ../include/pin2mmi.o ../include/log.o
	$(CC) -I$(INCDIR) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/spi_loop: spi_loop.o ../include/pin2mmi.o ../include/log.o
	$(CC) -I$(INCDIR) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/reset: reset.o ../include/pin2mmi.o ../include/log.o
	$(CC) -I$(INCDIR) -o $@ $^ $(LDFLAGS)

# Generic rule for object files
%.o: %.c
	$(CC) -c $< $(CFLAGS) -o $@

# Clean build artifacts
clean:
	rm -f *.o $(TARGETS)

.PHONY: all clean
