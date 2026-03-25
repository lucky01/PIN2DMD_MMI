# MMI tools for AnimationClock

Collection of tools that can be used to build a RPI2DMD like animation clock on PIN2DMD_MMI.
RPI2DMD is a charityware / careware project which displays clock, date and weather information together with pixelart gif animations.
You can find all the information about RPI2DMD on the Neo Arcadia forum.
https://www.neo-arcadia.com/forum/viewtopic.php?f=14&t=67065

## AnimClock - playback animated gif followed by date and time display
```
usage: ./Anim_clock [options]
Options:
    -h                          : Panel height in pixel. Typically 32 or 64. (Default: 32).
    -w                          : Panel width. Typically 128 or 256. (Default: 128).
    -g <animation-file>         : Animation/Image file.
    -W <wait time>              : Wait time after animation/Image file.
    -l <log-level>              : Set log level (0-5)
Clock options :
    -T <time-format>            : Format '%H:%M:%S'. See strftime(), If not specified, clock will not be displayed
    -t <clock_timeout>          : Display time in ms, If not specified, clock will not be displayed
    -x <x-origin>               : X-Origin of clock text (Default: 0)
    -y <y-origin>               : Y-Origin of clock text (Default: 0)
    -f <font-file>              : Font file to use for clock.
    -s <font-size>              : Font size to use for clock.
    -p <pattern-file>           : Use pattern file for clock characters.
    -b <background-file>        : Background image for clock display.
    -a <text-alignment>         : 0 : Top Left, 1: Left, 2: Bottom Left, 3: Top Middle, 4: Middle, 5: Bottom Middle, 6: Top Right, 7: Right, 8: Bottom Right.
Date options :
    -D <time-format>            : Format '%d/%m/%Y'. See strftime(), If not specified, date will not be displayed
    -d <date_timeout>           : Display time in ms, If not specified, date will not be displayed
    -X <x-origin>               : X-Origin of date text (Default: 0)
    -Y <y-origin>               : Y-Origin of date text (Default: 0)
    -F <font-file>              : Font file to use for date.
    -S <font-size>              : Font size to use for date.
    -P <pattern-file>           : Use pattern file for date characters.
    -B <background-file>        : Background image for date display.
    -A <text-alignment>         : 0 : Top Left, 1: Left, 2: Bottom Left, 3: Top Middle, 4: Middle, 5: Bottom Middle, 6: Top Right, 7: Right, 8: Bottom Right.
    -L <locale>                 : Text localisation (Default : fr_FR)
    -C <r,g,b>                  : Color. Default 255,255,0
```

## Get_GIF_delay - display total time of GIF animation
```
Usage: Get_GIF_delay [options] -F filename
Options:
  -F <file>             : Animation/Image file.
  -l <log-level>        : Set log level (0-5)
```

## Show_Image - display animation or still image (looks for boot.gif animation if no filename option is given)
```
Usage: Show_Image [options] -F filename
Options:
  -F <file>             : Animation/Image file.
  -C                    : Center images.
  -T                    : Timeout.
  -h                    : Panel height. Typically 32 or 64. (Default: 32).
  -w                    : Panel width. Typically 128 or 256. (Default: 128).
  -l <log-level>        : Set log level (0-5)
```

## Show_IP - display IP address information of wlan0 and eth0
```
Usage: Show_IP [options]
Options:
    -h                  : Panel rows. Typically 8, 16, 32 or 64. (Default: 32).
    -w                  : Panel columns. Typically 32 or 64. (Default: 32).
    -l <log-level>      : Set log level (0-5)
```

## Weather - fetch weather forecast from OpenWeather and save as 128x32 png image
```
Usage: Weather [options]
Options:
    -i           : OpenWeather ID
    -c           : Country
    -z           : Zip Code
    -u           : Units
    -C           : Text Color
    -Y           : Height
    -X           : Width
    -F           : Filename (default /tmp/meteo.png)
```
