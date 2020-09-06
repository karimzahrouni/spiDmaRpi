/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "dma.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
char *device = "/dev/spidev0.0";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay;

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static void err(int fd){
	
	int ret = 0;
	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");
	}

static void transfer(char *d , char *tx)
{
	int ret;
	int fd;

	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");
	err(fd);
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)tx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

	for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
		printf("%.2X \n", tx[ret]);
	}

	close(fd);
}
static void dmaInit(){
            // Ensure cleanup if user hits ctrl-C
    signal(SIGINT, terminate);

    // Map GPIO, DMA registers into virtual mem (user space)
    virt_gpio_regs = map_segment((void *)GPIO_BASE, PAGE_SIZE);
    virt_dma_regs = map_segment((void *)DMA_BASE, PAGE_SIZE);
    virt_clk_regs = map_segment((void *)CLK_BASE, PAGE_SIZE);
    enable_dma();

    // Set LED pin as output, and set high
    gpio_mode(LED_PIN, GPIO_OUT);
    gpio_out(LED_PIN, 1);

    // Use mailbox to get uncached memory for DMA decriptors and buffers
    mbox_fd = open_mbox();
    if ((dma_mem_h = alloc_vc_mem(mbox_fd, DMA_MEM_SIZE, DMA_MEM_FLAGS)) <= 0 ||
        (bus_dma_mem = lock_vc_mem(mbox_fd, dma_mem_h)) == 0 ||
        (virt_dma_mem = map_segment(BUS_PHYS_ADDR(bus_dma_mem), DMA_MEM_SIZE)) == 0)
            FAIL("Error: can't allocate uncached memory\n");
    printf("VC mem handle %u, phys %p, virt %p\n", dma_mem_h, bus_dma_mem, virt_dma_mem);
}

int main()
{
	uint8_t tx[] ={0xF5,0xF6};
	char *data ;
	dmaInit();
	data  = dma_test_mem_transfer((char*)tx);
	transfer(device,data);
	dmaInit();
    	dma_test_mem_transfer((char*)data);
	terminate(0);

	return 0;
}
