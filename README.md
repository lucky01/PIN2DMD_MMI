
# MMI tools for pin2dmd

Collection of tools that can be used together with the MMI <-> PIN2DMD bridge

Prerequisites for Pi Zero 2W:
- OS like DietPi Trixie installed
- uart activated (enable_uart=1 in config.txt)
- hardware uart (dev/ttyAMA0) mapped to UART pins. (dtoverlay=disable-bt in config.txt)
- uart console disabled (remove the console= part from cmdline.txt)
- stop getty for ttyAMA0 (systemctl mask serial-getty@ttyAMA0.service)
- spi interface activated (e.g. dtparam=spi=on and dtoverlay=spi0-1cs in config.txt)
- spi message size set to 8192 (spidev.bufsiz=8192 added to cmdline.txt)
- gpiod installed (apt install libgpiod-dev)

## Serum - a libserum implementation for PIN2DMD_MMI
see README.md in serum folder for details

## Anim_clock - a animation clock toolbox for PIN2DMD_MMI
see README.md in Anim_clock folder for details


## Example applications in the example folder :
### dmd_player - playback pinmame txt dump files on PIN2DMD

```
Usage: dmd_player [options] filename
Options:
  -l <loglevel>                  Set log level (0-5)
```

### display_ppm - display 128x32 (256x64 on HD) ppm picture file on PIN2DMD

```
Usage: display_ppm [options] filename
Options:
  -l <loglevel>                  Set log level (0-5)
```

### screenmirror - mirror display content to framebuffer (HDMI) or save screenshot as 128x32 (256x64 on HD) ppm file

```
Usage: screenmirror [options] filename
Options:
  -m screenmode                  Set screen mode (0 - fullscreen, 1 - 4:2, 2 - 4:1, 3 - 3:1)
  -f <filename>                  ppm export filename
  -l <loglevel>                  Set log level (0-5)
```

### dmd_dump - dump pinball input stream to file or stdout

```
Usage: dump_dmd [options]
Options:
  -p <path>                      Path to the dump folder
  -s <screen>                    dump to screen
  -l <loglevel>                  Set log level (0-5)
```

### spi_loop - read real pinball input data and send back to pin2dmd display

```
Usage: spi_loop [options]
Options:
  -l <loglevel>                  Set log level (0-5)
```

## reset - reset PIN2DMD

