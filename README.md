
# MMI tools for pin2dmd

Collection of tools that can be used together with the MMI <-> PIN2DMD bridge

Prerequisites for Pi Zero 2W:
- OS like DietPi Bookworm installed
- uart activated (enable_uart=1 in config.txt)
- hardware uart (dev/ttyAMA0) mapped to UART pins. (dtoverlay=disable-bt in config.txt)
- uart console disabled (remove the console= part from cmdline.txt)
- spi interface activated (e.g. dtparam=spi=on in config.txt)
- spi message size set to 8192 (spidev.bufsiz=8192 added to cmdline.txt)
- pigpio installed (apt install pigpio)

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

## spi-serum Usage

```
Usage: spi-serum [options] <altcolor-path> <rom-name>

Options:
  -x                                Enable HD (64-line) output mode
  -d                                Dump serum frames as hex to stdout
  -i <IgnoreUnknownFramesTimeout>   Set ignore unknown frames timeout (ms). default dont set
  -m <MaximumUnknownFramesToSkip>   Set maximum unknown frames to skip. default dont set
  -w <width>                        Set DMD width (default: 128)
  -h <height>                       Set DMD height (default: 32)
  -r <lines-to-request>             Set lines to request for serum (32 or 64 (HD), default: 32)
  -l <loglevel>                     Set log level (0-5)
  -f <dump-file>                    Read frames from given file instead of SPI input


## License

This project is licensed under the GNU General Public License v2.0 - see the [LICENSE](LICENSE) file for details.
