/*
 *  drivers/mtd/nand/ebook-nand.c
 *
     Based on the  original sony ebook-nand.c
     Zhang Wenjie <zwjsq@vip.sina.com>
 *  Copyright (C) 2002 Sony Corporation.
 *  Copyright (C) 2000 Steven J. Hill (sjhill@cotw.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <mach/gpio.h>

#include <mach/hardware.h>

/* NOT USED.  Using STATUS_READ */
//#undef USE_BUSYLINE 
#define USE_BUSYLINE 

/* ctl GPIOC-PIN*/
#define	NAND_GPIO_PORT     						2  /*PORT C*/
#define   EBOOK_NAND_BUSY                                        15
#define   EBOOK_NAND_CE0                                         14
#define   EBOOK_NAND_CLE                                         16
#define   EBOOK_NAND_ALE                                         17

/*
 * MTD structure for EBOOK board
 */
static struct mtd_info *ebook_mtd = NULL;

/*
 * Values specific to the EBOOK board
 */
static unsigned long ebook_fio_base = IMX_CS1_PHYS;/* Where the flash mapped */

/*
 * Define partitions for flash device
 */
#ifdef CONFIG_MTD_PARTITIONS

static struct mtd_partition ebook_partition_info[] = {
	{ .name	= "SDM table ",
	   .size	= 0x80000,
	   .offset	= 0, },
	{ .name	= "nblconfig ",
	   .size	= 0x80000,
	   .offset	= 0x80000, },   
	{ .name	= "Linux ",
	   .size	= 0x1A0000,
	   .offset	= 0x100000, },  
	{ .name	= "Linux0 ",
	   .size	= 0x1A0000,
	   .offset	= 0x2A0000, }, 
	{ .name	= "Rootfs2 ",
	   .size	= 0x7E0000,
	   .offset	= 0x440000, }, 
	{ .name	= "Rootfs ",
	   .size	= 0x980000,
	   .offset	= 0xC20000, }, 
	{ .name	= "Fsk ",
	   .size	= 0x4600000,
	   .offset	= 0x15A0000, }, 
	{ .name	= "Unknow0 ",
	   .size	= 0x3E0000,
	   .offset	= 0x1A00000, }, 
	{ .name	= "Data ",
	   .size	= 0xD260000,
	   .offset	= 0x1DE0000, }, 
	{ .name	= "Opt0 ",
	   .size	= 0x300000,
	   .offset	= 0xF040000, }, 
	{  .name = "Unkown1",
	    .size =	   MTDPART_SIZ_FULL,
	    .offset = 0xF340000
	}
};

#define NUM_PARTITIONS  ARRAY_SIZE(ebook_partition_info)
#endif
/* 
 *	hardware specific access to control-lines
 */

/* for CS1 */
static void ebook_hwcontrol(struct mtd_info *mtd, int cmd,
			    unsigned int ctrl)
{
    struct nand_chip *chip = mtd->priv;
	
    if (ctrl & NAND_CTRL_CHANGE) {
		u32 bits;

		bits = ((ctrl & NAND_NCE) ? 0x0: 0x1 )<< EBOOK_NAND_CE0;
		bits |= ((ctrl & NAND_CLE) ? 0x1: 0x0 )<< EBOOK_NAND_CLE;
		bits |= ((ctrl & NAND_ALE) ? 0x1: 0x0 )<< EBOOK_NAND_ALE;

		DR(NAND_GPIO_PORT) =((DR(NAND_GPIO_PORT))& ~(1<<EBOOK_NAND_ALE)& ~(1<<EBOOK_NAND_CLE)& ~(1<<EBOOK_NAND_CE0))| bits;	
    }
    
   if (cmd != NAND_CMD_NONE)
   	writeb(cmd, chip->IO_ADDR_W);
}

#ifdef USE_BUSYLINE
/*
 *	read device ready pin
 */
int ebook_device_ready(void)
{
	if ((SSR(NAND_GPIO_PORT))& (1<<EBOOK_NAND_BUSY))
		return 1;
	return 0;
}
#endif

static int __init ebook_init_chip (void)
{
	struct nand_chip *this;
	void __iomem *nandaddr;

	/* Allocate memory for MTD device structure and private data */
	ebook_mtd = kmalloc (sizeof(struct mtd_info) + sizeof (struct nand_chip),
		       GFP_KERNEL);
	if (ebook_mtd == NULL) {
		printk ("Unable to allocate EBOOK NAND MTD device "
			"structure.\n");
		return -ENOMEM;
	}
	/* Get pointer to private data */
	this = (struct nand_chip *) (ebook_mtd+1);

	/* Initialize structures */
	memset((char *) ebook_mtd, 0, sizeof(struct mtd_info));
	memset((char *) this, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	ebook_mtd->priv = this;
	ebook_mtd->owner = THIS_MODULE;

	/* Set address of NAND IO lines */
	nandaddr = ioremap(ebook_fio_base, IMX_CS1_SIZE);
	if (!nandaddr) {
                printk("Failed to ioremap nand flash.\n");
                return -ENOMEM;
        }

	this->IO_ADDR_R = this->IO_ADDR_W = nandaddr;
	/* Set address of hardware control function */
	this->cmd_ctrl = ebook_hwcontrol;
	this->ecc.mode = NAND_ECC_SOFT;
#ifdef USE_BUSYLINE
	this->dev_ready = ebook_device_ready;

        /* board without fix to busy line does not work */
        if (!ebook_device_ready()) {
                printk("Error in NAND busy line!  "
		       "Has board fix applied properly?\n");
		kfree (ebook_mtd);
		iounmap((void *)nandaddr);
		return -ENXIO;
        }
#endif

	/* Scan to find existence of the device */
	if (nand_scan (ebook_mtd,1)) {
		printk(KERN_NOTICE "No NAND device - returning -ENXIO\n");
		kfree (ebook_mtd);
		iounmap((void *)nandaddr);
		return -ENXIO;
	}

	/* Register the partitions */
	add_mtd_partitions(ebook_mtd, ebook_partition_info, NUM_PARTITIONS);

	/* Return happy */
	return 0;
}


/*
 * Main initialization routine
 */
static int __init ebook_nand_init(void)
{
        int err;

        printk("MTD(NAND): %s() %s\n", __FUNCTION__, __TIME__);
        /* 
         * initialise EIM / GPIO appropriate for access
         */

        /* busy */
	 imx_gpio_mode(GPIO_PORTC|EBOOK_NAND_BUSY|GPIO_IN|GPIO_PUEN|GPIO_GIUS); 
        /* other pins */
	 imx_gpio_mode(GPIO_PORTC|EBOOK_NAND_CE0|GPIO_OUT|GPIO_DR|GPIO_GIUS|GPIO_PUEN); 
	 imx_gpio_mode(GPIO_PORTC|EBOOK_NAND_CLE|GPIO_OUT|GPIO_DR|GPIO_GIUS|GPIO_PUEN); 
        imx_gpio_mode(GPIO_PORTC|EBOOK_NAND_ALE|GPIO_OUT|GPIO_DR|GPIO_GIUS|GPIO_PUEN);
		
        err = ebook_init_chip();
        if (err)  goto errgpio;
		
	 return 0;

 errgpio:
        printk("MTD(NAND): GPIO allocation failed\n");
        return err;
}
module_init(ebook_nand_init);

static void __exit ebook_cleanup_chip (struct mtd_info *mtd)
{
	struct nand_chip *this = (struct nand_chip *)&mtd[1];
	/* Release resources, unregister device(s) */
	nand_release(mtd);
	
	/* Release io resource */
	iounmap((void *)this->IO_ADDR_W);

	/* Free the MTD device structure */
	kfree (mtd);
	
       return;
}

/*
 * Clean up routine
 */
static void __exit ebook_cleanup (void)
{
	ebook_cleanup_chip(ebook_mtd);
	
}
module_exit(ebook_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Zhang Wenjie");
MODULE_DESCRIPTION("Board-specific glue layer for NAND flash on eBook board");
