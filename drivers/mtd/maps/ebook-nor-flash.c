/*
 * NOR Flash support routine for Sony PRS-505 eReader.
 * Zhang Wenjie <zwjsq@vip.sina.com>
 *
 * This code is based on Mpu110_series-flash.c.
 * This code is based on physmap.c.
 *
 *  
 *   Normal mappings of chips in physical memory
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/slab.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>	
#include <mach/hardware.h>
#include <asm/io.h>

static struct mtd_info *mymtd;

static struct map_info ebook_nor_map = {
	.name =		"ebook-nor-flash",
	.bankwidth =	2,
	.size =		PRS505_FLASH_SIZE,
	.phys =		PRS505_FLASH_PHYS,
};

static int __init ebook_nor_flash_init(void)
{
	printk("MTD(NOR): %s() %s\n", __FUNCTION__, __TIME__);
       printk(KERN_NOTICE "  Flash device: 0x%lx[bytes] at 0x%lx\n", ebook_nor_map.size, ebook_nor_map.phys);

	ebook_nor_map.virt =  ioremap(PRS505_FLASH_PHYS, PRS505_FLASH_SIZE);
	if (!ebook_nor_map.virt) {
		printk(KERN_ERR "Ebook-Nor-Flash: ioremap failed\n");
		return -EIO;
	}	
	
	simple_map_init(&ebook_nor_map);

       mymtd = do_map_probe("cfi_probe", &ebook_nor_map);
       if (mymtd == NULL)  mymtd = do_map_probe("jedec_probe", &ebook_nor_map);	
	if (mymtd) {
		mymtd->owner = THIS_MODULE;

		add_mtd_device(mymtd);
		return 0;
	}

	iounmap((void *)ebook_nor_map.virt);
	return -ENXIO;
	
}

static void __exit ebook_nor_flash_cleanup(void)
{
	if (mymtd) {
		del_mtd_device(mymtd);
		map_destroy(mymtd);
	}

	if (ebook_nor_map.virt) {
		iounmap((void *)ebook_nor_map.virt);
		ebook_nor_map.virt = 0;
	}
}

module_init(ebook_nor_flash_init);
module_exit(ebook_nor_flash_cleanup);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Zhang Wenjie");
MODULE_DESCRIPTION("MTD map driver for Sony PRS-505 eReader");
