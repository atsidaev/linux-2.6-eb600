/*
 * arch/arm/mach-imx/prs505.c
 *
 * Initially based on:
 *	linux-2.6.25.rc8-imx/arch/arm/mach-imx/Mx1ads.c
 *	Zhang Wenjie <zwjsq@vip.sina.com>
 *	This is a part of project openinkpot
 * 2004 (c) MontaVista Software, Inc.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/page.h>

#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <mach/mmc.h>
#include <mach/imx-uart.h>
#include <linux/interrupt.h>
#include "generic.h"

/*the bitwise definition of UCR1*/
#define SNDBRK		0x00000010
#define UART_CLK_EN	0x00000004
#define UARTEN		0x00000001
#define RRDYEN		0x00000200
#define IDEN		0x00001000

/*the bitwise definition of UCR2*/
#define IRTS	0x00004000
#define CTSC	0x00002000
#define CTS     0x00001000
#define PREN	0x00000100
#define PROE	0x00000080
#define STPB	0x00000040
#define WS      0x00000020
#define TXEN	0x00000004
#define RXEN	0x00000002
#define SRST	0x0000fffe

/*the interrupt enable bit*/
#define UCR1_TRDYEN		0x00002000
#define UCR1_TXMPTYEN	0x00000040

/*  08/5/27 */
#define _reg_PORTC_GIUS			GIUS(2)
#define _reg_PORTC_GPR			GPR(2)
#define _reg_UART1_URXD		__REG(IMX_UART1_BASE + 0x0)
#define _reg_UART1_UTXD		__REG(IMX_UART1_BASE + 0x40)
#define _reg_UART1_UCR1		__REG(IMX_UART1_BASE + 0x80)
#define _reg_UART1_UCR2		__REG(IMX_UART1_BASE + 0x84)
#define _reg_UART1_UCR3		__REG(IMX_UART1_BASE + 0x88)
#define _reg_UART1_UCR4		__REG(IMX_UART1_BASE + 0x8C)
#define _reg_UART1_UFCR		__REG(IMX_UART1_BASE + 0x90)
#define _reg_UART1_USR1		__REG(IMX_UART1_BASE + 0x94)
#define _reg_UART1_USR2		__REG(IMX_UART1_BASE + 0x98)
#define _reg_UART1_UBIR			__REG(IMX_UART1_BASE + 0xA4)
#define _reg_UART1_UBMR		__REG(IMX_UART1_BASE + 0xA8)
#define _reg_AITC_INTSRCL		__REG(IMX_AITC_BASE + 0x4C)

/*dma transmitting*/
#define _reg_DMA_CCR1           CCR(1)
#define _reg_DMA_RSSR1         RSSR(1) 
#define _reg_DMA_BLR1           BLR(1)
#define _reg_DMA_RTOR1       	RTOR(1)
#define _reg_DMA_BUCR1        BUCR(1) 

#define _reg_DMA_DAR1          DAR(1) 
#define _reg_DMA_SAR1          SAR(1) 
#define _reg_DMA_CNTR1        CNTR(1)  

/*regsiters for dma*/
#define _reg_DMA_DIMR		DIMR

static struct imxuart_platform_data uart_pdata = {
	.flags = IMXUART_HAVE_RTSCTS,
};

static struct resource imx_uart1_resources[] = {
	[0] = {
		.start	= 0x00206000,
		.end	= 0x002060FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (UART1_MINT_RX),
		.end	= (UART1_MINT_RX),
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= (UART1_MINT_TX),
		.end	= (UART1_MINT_TX),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device imx_uart1_device = {
	.name		= "imx-uart",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(imx_uart1_resources),
	.resource	= imx_uart1_resources,
	.dev = {
		.platform_data = &uart_pdata,
	}
};

static struct resource imx_uart2_resources[] = {
	[0] = {
		.start	= 0x00207000,
		.end	= 0x002070FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (UART2_MINT_RX),
		.end	= (UART2_MINT_RX),
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= (UART2_MINT_TX),
		.end	= (UART2_MINT_TX),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device imx_uart2_device = {
	.name		= "imx-uart",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(imx_uart2_resources),
	.resource	= imx_uart2_resources,
	.dev = {
		.platform_data = &uart_pdata,
	}
};

static struct resource ebook_usb_s1r72v17_resources[] = {
	[0] = {
		.start	= 0x15E00000,
		.end	= 0x15E10000,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (IRQ_GPIOC(3)),
		.end	= (IRQ_GPIOC(3)),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device ebook_usb_s1r72v17_device = {
	.name		= "s1r72v17",
	.num_resources	= ARRAY_SIZE(ebook_usb_s1r72v17_resources),
	.resource	= ebook_usb_s1r72v17_resources,
};

static struct platform_device *devices[] __initdata = {
//	&imx_uart1_device,
	&imx_uart2_device,
	&ebook_usb_s1r72v17_device,
};

static void ebook_power_off(void)
{
    int i = 0;
    int trans_buf[5];
	unsigned long j;	/* 03/07/28 */

    printk(KERN_ERR "Shutdown ebook_power_off_new\n");  /* 03/07/28 */

    /* Clear the FIFO buffers and disable them  (they will be reenabled in change_speed())*/
    _reg_UART1_UCR2 &= SRST;
    /* delay for the reset*/
    for (i = 0; i < 1000; i++);

    _reg_UART1_UCR1 |= UARTEN;              /* Enable the UART */
    _reg_UART1_UCR2 |= (RXEN | TXEN);       /* 02/11/15 */
    _reg_UART1_UCR2 |= PREN;                /* 02/11/15 */
    _reg_UART1_UCR2 |= WS;                  /* 02/11/15 */

    /* not ignore RTS line */
    _reg_UART1_UCR2 &= 0x0000bfff;      /* IRTS=0 02/11/15 */
    /* CTS line control */
    _reg_UART1_UCR2 &= 0x0000dfff;      /* CTSC=0 02/11/15 */
    _reg_UART1_UCR2 |= CTS;             /* 02/11/15 */

    /*this is funny that UCR4 must be set as follows*/
    _reg_UART1_UCR4 = 1;

    /* and set the speed of the serial port */
    _reg_UART1_UBIR = 0xf;
    _reg_UART1_UBMR = 8000000 / 38400;  /* baudrate=38400 */
    _reg_UART1_UFCR = 0x00000A01;       /* 02/11/15 */
//    _reg_UART1_UFCR = 0x00000A81;

    /* delay for transmit data*/
    for (i = 0; i < 50000; i++);

    j = jiffies + 2;        /* 20ms wait */ /* 03/07/28 */
    while(jiffies < j){schedule();}         /* 03/07/28 */

    /* HOST Shutdown Complite COMMAND */
    trans_buf[0] = 0x000000D3;
    trans_buf[1] = 0x00000000;
    trans_buf[2] = 0x00000000;
    trans_buf[3] = 0x00000000;
    trans_buf[4] = 0x0000002C;

    for(i=0; i<5; i++){
        _reg_UART1_UTXD=trans_buf[i];
    };

    while(1);
}


static void __init
mx1_init(void)
{
#ifdef CONFIG_LEDS
	imx_gpio_mode(GPIO_PORTA | GPIO_OUT | 2);
#endif
	MPCTL0 = 0x003F1437;
	SPCTL0 = 0x123638ad;

/*MMC SD card CS*/
	__REG(IMX_EIM_BASE + 0x18) = 0x00001800;
	__REG(IMX_EIM_BASE + 0x1c) = 0xcccc0D01;

/*set up the CS0-5 GPIO PIN*/
	GIUS(0) = (GIUS(0)) & 0x001ffffe;
	GPR(0) = (GPR(0)) & 0x001ffffe;

	imx_gpio_mode(GPIO_PORTA|GPIO_GIUS|GPIO_OUT|GPIO_DR|18);
	imx_gpio_mode(GPIO_PORTA|GPIO_GIUS|GPIO_IN|GPIO_AIN|GPIO_AOUT|GPIO_BOUT|19);
	imx_gpio_mode(GPIO_PORTA|GPIO_GIUS|GPIO_OUT|GPIO_DR|20);

	imx_gpio_mode(PC9_PF_UART1_CTS);
	imx_gpio_mode(PC10_PF_UART1_RTS);
	imx_gpio_mode(PC11_PF_UART1_TXD);
	imx_gpio_mode(PC12_PF_UART1_RXD);

	imx_gpio_mode(PB28_PF_UART2_CTS);
	imx_gpio_mode(PB29_PF_UART2_RTS);
	imx_gpio_mode(PB30_PF_UART2_TXD);
	imx_gpio_mode(PB31_PF_UART2_RXD);

	platform_add_devices(devices, ARRAY_SIZE(devices));

	pm_power_off = ebook_power_off;
}

static void __init
mx1_map_io(void)
{
	imx_map_io();
}

MACHINE_START(SONY_PRS505, "Sony PRS-505 (Motorola DragonBall MX1)")
	/* Maintainer: Yauhen Kharuzhy */
	.phys_io	= 0x00200000,
	.io_pg_offst	= ((0xe0000000) >> 18) & 0xfffc,
	.boot_params	= 0x08000100,
	.map_io		= mx1_map_io,
	.init_irq	= imx_init_irq,
	.timer		= &imx_timer,
	.init_machine	= mx1_init,
MACHINE_END

