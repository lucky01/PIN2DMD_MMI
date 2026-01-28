This is the implementation of libserum on PIN2DMD_MMI by [sker65](https://github.com/sker65).
[LibSerum](https://github.com/ppuc/libserum) is a cross-platform library for decoding Serum files, a colorization format for pinball ROMs.

## Compiling libserum

#### Linux (aarch64)
```shell
cmake -DPLATFORM=linux -DARCH=aarch64 -DCMAKE_BUILD_TYPE=Release -B build
cmake --build build
```

## spi-serum Usage

```
Usage: spi-serum [options] <altcolor-path> <rom-name>
Options:
  -d                                Dump serum frames as hex to stdout
  -i <IgnoreUnknownFramesTimeout>   Set ignore unknown frames timeout (ms). default dont set
  -m <MaximumUnknownFramesToSkip>   Set maximum unknown frames to skip. default dont set
  -l <loglevel>                     Set log level (0-5)
  -f <dump-file>                        Read frames from given file instead of SPI input
  -r                                    Reset PIN2DMD at startup
```

## License

This project is licensed under the GNU General Public License v2.0 - see the [LICENSE](LICENSE) file for details.


