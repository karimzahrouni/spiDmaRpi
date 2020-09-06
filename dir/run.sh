rm dma.o
gcc -Wall -c dma.c
rm libtest.a
ar -cvq libtest.a dma.o
gcc -o spidma spidev_test.c libtest.a
