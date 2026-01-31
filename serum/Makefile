# Directories
BUILD_DIR = ../build
INCLUDE_DIR = ../include
LIBSERUM_DIR = ../../libserum

# Compiler and flags
CC  = g++
CFLAGS  = $(INCDIR) -I$(LIBSERUM_DIR)/src -I$(INCLUDE_DIR)
LDFLAGS = -L../../libserum/build -Wl,-Bstatic -lserum -Wl,-Bdynamic -lpigpio

# Source files and objects
SRCS            = ../include/pin2mmi.c \
                  ../include/log.c \
                  ../include/frame_reader.c
OBJS            = $(SRCS:.c=.o)

# Target executable
TARGET = $(BUILD_DIR)/spi-serum

# Default target
all: $(BUILD_DIR) $(TARGET)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Build targets
$(BUILD_DIR)/spi-serum: spi-serum.o ../include/frame_reader.o ../include/pin2mmi.o ../include/log.o
	$(CC) $(INCDIR) -o $@ $^ $(LDFLAGS)

# Generic rule for object files
%.o: %.c
	$(CC) -c $< $(CFLAGS) -o $@

# Clean build artifacts
clean:
	rm -f $(OBJS) $(TARGET) *.o

# Phony targets
.PHONY: all clean
