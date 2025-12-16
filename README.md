
# MMI tools for pin2dmd

Collection of tools that can be used together with the MMI <-> PIN2DMD bridge

Prerequisites for Pi Zero 2W:
- OS like DietPi installed
- uart activated (enable_uart=1 in config.txt)
- hardware uart (dev/ttyAMA0) mapped to UART pins. (dtoverlay=disable-bt in config.txt)
- uart console disabled (remove the console= part from cmdline.txt)
- spi interface activated (e.g. dtparam=spi=on in config.txt)
- spi message size set to 8192 (spidev.bufsiz=8192 added to cmdline.txt)
- pigpio installed (apt install pigpio-tools)

## dmd_player - playback pinmame txt dump files on PIN2DMD

```
Usage: dmd_player [options] filename
Options:
  -l <loglevel>                  Set log level (0-5)
```

## display_ppm - display 128x32 (256x64 on HD) ppm picture file on PIN2DMD

```
Usage: display_ppm [options] filename
Options:
  -l <loglevel>                  Set log level (0-5)
```

## screenshot - save display content as 128x32 (256x64 on HD) ppm file

```
Usage: screenshot [options] filename
Options:
  -l <loglevel>                  Set log level (0-5)
```

## spi_loop - read real pinball input data and send back to pin2dmd display

```
Usage: spi_loop [options]
Options:
  -l <loglevel>                  Set log level (0-5)
```

## reset - reset PIN2DMD

