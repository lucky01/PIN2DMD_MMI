CC = g++
HOME_DIR = $(HOME)
LDFLAGS = -lpigpio
BUILD_DIR = ../build
INCDIR = -I/usr/local/include/ -I../include
LIBDIR = /usr/local/lib/
CFLAGS = -g $(INCDIR)
SRCS = ../include/pin2mmi.c ../include/log.c ../include/frame_reader.c
OBJS = $(SRCS:.c=.o)

TARGETS = $(BUILD_DIR)/dmd_player $(BUILD_DIR)/dump_dmd $(BUILD_DIR)/display_ppm $(BUILD_DIR)/screenshot $(BUILD_DIR)/spi_loop $(BUILD_DIR)/reset

all: $(BUILD_DIR) $(TARGETS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/dmd_player: dmd_player.o ../include/frame_reader.o ../include/pin2mmi.o ../include/log.o
	$(CC) -I$(INCDIR) -o $(BUILD_DIR)/$@ $^ $(LDFLAGS)

$(BUILD_DIR)/dump_dmd: dump_dmd.o ../include/pin2mmi.o ../include/log.o
	$(CC) -I$(INCDIR) -o $(BUILD_DIR)/$@ $^ $(LDFLAGS)

$(BUILD_DIR)/display_ppm: display_ppm.o ../include/pin2mmi.o ../include/log.o
	$(CC) -I$(INCDIR) -o $(BUILD_DIR)/$@ $^ $(LDFLAGS)

$(BUILD_DIR)/screenshot: screenshot.o ../include/pin2mmi.o ../include/log.o
	$(CC) -I$(INCDIR) -o $(BUILD_DIR)/$@ $^ $(LDFLAGS)

$(BUILD_DIR)/spi_loop: spi_loop.o ../include/pin2mmi.o ../include/log.o
	$(CC) -I$(INCDIR) -o $(BUILD_DIR)/$@ $^ $(LDFLAGS)

$(BUILD_DIR)/reset: reset.o ../include/pin2mmi.o ../include/log.o
	$(CC) -I$(INCDIR) -o $(BUILD_DIR)/$@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) -c $< $(CFLAGS) -o $@

clean:
	    rm -f *.o $(TARGETS)

.PHONY: all clean

