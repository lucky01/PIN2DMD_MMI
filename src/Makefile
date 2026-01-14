CC = g++
HOME_DIR = $(HOME)
LDFLAGS = -lpigpio

BUILD_DIR = ../build

SRCS = pin2mmi.src log.c frame_reader.c
OBJS = $(SRCS:.c=.o)

TARGETS = $(BUILD_DIR)/dmd_player $(BUILD_DIR)/dump_dmd $(BUILD_DIR)/display_ppm $(BUILD_DIR)/screenshot $(BUILD_DIR)/spi_loop $(BUILD_DIR)/reset

all: $(BUILD_DIR) $(TARGETS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/dmd_player: dmd_player.o frame_reader.o pin2mmi.o log.o
	$(CC) -o $(BUILD_DIR)/$@ $^ $(LDFLAGS)

$(BUILD_DIR)/dump_dmd: dump_dmd.o pin2mmi.o log.o
	$(CC) -o $(BUILD_DIR)/$@ $^ $(LDFLAGS)

$(BUILD_DIR)/display_ppm: display_ppm.o pin2mmi.o log.o
	$(CC) -o $(BUILD_DIR)/$@ $^ $(LDFLAGS)

$(BUILD_DIR)/screenshot: screenshot.o pin2mmi.o log.o
	$(CC) -o $(BUILD_DIR)/$@ $^ $(LDFLAGS)

$(BUILD_DIR)/spi_loop: spi_loop.o pin2mmi.o log.o
	$(CC) -o $(BUILD_DIR)/$@ $^ $(LDFLAGS)

$(BUILD_DIR)/reset: reset.o pin2mmi.o log.o
	$(CC) -o $(BUILD_DIR)/$@ $^ $(LDFLAGS)

$(BUILD_DIR)/spi-serum: spi-serum.o pin2mmi.o log.o
	$(CC) -o $(BUILD_DIR)/$@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) -c $< -o $@

clean:
	    rm -f *.o $(TARGETS)

.PHONY: all clean

