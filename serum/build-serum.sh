#!/bin/bash

mkdir ../build

g++ -c spi-serum.c -I ~/git/libserum/src -I../include
g++ -c ../include/log.c
g++ -c ../include/frame_reader.c
g++ -c ../include/pin2mmi.c
g++ -o ../build/spi-serum spi-serum.o log.o frame_reader.o pin2mmi.o -L ~/git/libserum/build -Wl,-Bstatic -lserum -Wl,-Bdynamic -lpigpio

systemctl stop serum
cp ../build/spi-serum /usr/local/bin/.
systemctl start serum

