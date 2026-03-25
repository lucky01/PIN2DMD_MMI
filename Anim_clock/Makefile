CXX=g++
CFLAGS=-Wall -O3 -march=native -mtune=native -flto=2 -g -Wextra -Wno-unused-parameter
CXXFLAGS=$(CFLAGS)
LDFLAGS = -lgpiod
INCDIR= -I/usr/local/include/ -I../include
LIBDIR = /usr/local/lib/
CFLAGS = -g $(INCDIR)

# To compile Show_Image Anim_clock Show_IP
MAGICK_CXXFLAGS?=$(shell GraphicsMagick++-config --cppflags --cxxflags)
MAGICK_LDFLAGS?=$(shell GraphicsMagick++-config --ldflags --libs)

TARGETS = Show_Image Show_IP Anim_clock Get_GIF_delay Weather

all: $(TARGETS)

Weather.o : Weather.c
	$(CXX) $(INCDIR) $(CXXFLAGS) $(MAGICK_CXXFLAGS) -c -o $@ $<

Weather: Weather.o
	$(CXX) $< ../include/pin2mmi.o ../include/log.o $(INCDIR) $(LDFLAGS) -lcurl -ljson-c -L$(LIBDIR) $(MAGICK_LDFLAGS) -o $@

Show_Image.o : Show_Image.c
	$(CXX) $(INCDIR) $(CXXFLAGS) $(MAGICK_CXXFLAGS) -c -o $@ $<

Show_Image: Show_Image.o
	$(CXX) $< ../include/pin2mmi.o ../include/log.o $(INCDIR) $(LDFLAGS) -L$(LIBDIR) $(MAGICK_LDFLAGS) -o $@ 

Get_GIF_delay.o : Get_GIF_delay.c
	$(CXX) $(INCDIR) $(CXXFLAGS) $(MAGICK_CXXFLAGS) -c -o $@ $<

Get_GIF_delay: Get_GIF_delay.o
	$(CXX) $< ../include/pin2mmi.o ../include/log.o $(INCDIR) $(LDFLAGS) -L$(LIBDIR) $(MAGICK_LDFLAGS) -o $@ 

Show_IP.o : Show_IP.c
	$(CXX) $(INCDIR) $(CXXFLAGS) $(MAGICK_CXXFLAGS) -c -o $@ $<

Show_IP: Show_IP.o
	$(CXX) $< ../include/pin2mmi.o ../include/log.o $(INCDIR) $(LDFLAGS) -L$(LIBDIR) $(MAGICK_LDFLAGS) -o $@ 

Anim_clock.o : Anim_clock.c
	$(CXX) $(INCDIR) $(CXXFLAGS) $(MAGICK_CXXFLAGS) -c -o $@ $<

Anim_clock: Anim_clock.o
	$(CXX) $< ../include/pin2mmi.o ../include/log.o $(INCDIR) $(LDFLAGS) -L$(LIBDIR) $(MAGICK_LDFLAGS) -o $@ 

%.o: %.c
	$(CC) -c $< $(CFLAGS) -o $@

# Clean build artifacts
clean:
	rm -f $(TARGETS) *.o


FORCE:
.PHONY: FORCE
