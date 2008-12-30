/*
 * linux/drivers/video/prs505.c -- Platform device for sony prs-505 eink
 *
 * Copyright (C) 2008, Zhang Wenjie
 * Copyright (C) 2008, Yauhen Kharuzhy <jekhor@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>

#include <mach/gpio.h>

#include <video/metronomefb.h>

#define ETRACKFB_FB_WIDTH	600
#define ETRACKFB_FB_HEIGHT	800

#define FBIO_EINK_GET_TEMPERATURE		0x46A1	//Returns temperature in degree Celsius
#define FBIO_EINK_DISP_PIC				0x46A2	//Displays picture
#define FBIO_EINK_INIT					0x46A3	//Initializes the controller
#define FBIO_EINK_ENABLE_ERROR_CHECK	0x46A4	//Enables the error checking
#define FBIO_EINK_DISABLE_ERROR_CHECK	0x46A5	//Disables the error checking
#define FBIO_EINK_GET_STATUS			0x46A6	//Gets controller status
#define FBIO_EINK_GET_VEROFCONT		0x46B0	//Gets controller hardware version
#define FBIO_EINK_GET_DISPSTATUS		0x46B4	//Gets display status (BUSY=1, NOTBUSY=0)
#define FBIO_EINK_GOTO_NORMAL			0x46E0	//Transitions controller from sleep to normal/operational mode
#define FBIO_EINK_GOTO_SLEEP			0x46E2	//Transitions controller from normal to sleep mode
#define FBIO_EINK_GOTO_STANDBY		0x46F0	//Transitions controller from sleep to standby mode
#define FBIO_EINK_WAKE_UP				0x46F1	//Transitions controller from standby to sleep mode
#define FBIO_EINK_SET_ORIENTATION		0x46F2	//Sets picture orientation angle (0 for 0, 1 for 90, 2 for 180, 3 for 270 degrees)
#define FBIO_EINK_GET_ORIENTATION		0x46F3	//Gets picture orientation angle (0 for 0, 1 for 90, 2 for 180, 3 for 270 degrees)
#define FBIO_EINK_SET_BORDER_COLOR	0x46F4  //Sets the border gray color (0-15)
#define FBIO_EINK_GET_BORDER_COLOR	0x46F5  //Gets the border gray color
#define FBIO_EINK_GET_MODE			       0x46F7  //Gets 8track power mode
#define FBIO_EINK_SET_BORDER_ONOFF	0x46F8  //Sets the border ON = 1 OFF = 0

#define OP_MODE_INIT 0 
#define OP_MODE_MU 1
#define OP_MODE_GU 2
#define OP_MODE_GC 3

#define POWER_MODE_NORMAL  1
#define POWER_MODE_SLEEP   2
#define POWER_MODE_STANDBY 3

/* GPIO and some default value*/
#define GPIO_INTR (IRQ_GPIOD(9))
#define reg_A 0
#define reg_B 1
#define reg_C 2
#define reg_D 3
#define PD8_CLS ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|8 )
#define PD7_ERR ( GPIO_GIUS | GPIO_PORTD | GPIO_IN |GPIO_AIN|GPIO_AOUT|GPIO_BOUT|7 )
#define PD9_INT ( GPIO_GIUS | GPIO_PORTD | GPIO_IN |GPIO_AIN|GPIO_AOUT|GPIO_BOUT|9 )
#define PD10_STBYL ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|10 )
#define PD11_RSTL ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|11 )
#define PA2_VIO ( GPIO_GIUS | GPIO_PORTA | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|2 )
#define PA4_VCORE ( GPIO_GIUS | GPIO_PORTA | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|4 )
#define PD6_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|6 )
#define PD12_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|12 )
#define PD13_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|13 )
#define PD14_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|14 )
#define PD15_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|15 )
#define PD16_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|16 )
#define PD17_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|17 )
#define PD18_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|18 )
#define PD19_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|19 )
#define PD20_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|20 )
#define PD21_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|21 )
#define PD22_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|22)
#define PD23_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|23 )
#define PD24_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|24 )
#define PD25_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|25 )
#define PD26_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|26 )
#define PD27_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|27 )
#define PD28_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|28 )
#define PD29_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|29 )
#define PD30_LCDC ( GPIO_GIUS | GPIO_PORTD | GPIO_OUT |GPIO_DR |GPIO_AOUT_1|GPIO_BOUT_1|GPIO_PUEN|30 )

#define BCLK_DIV_BITP  10
#define BCLK_DIV_MASK  0xf

#define PCLK_DIV2_BITP 4
#define PCLK_DIV2_MASK 0xf

#define GPIO_LSCLK_BITP 6
#define GPIO_LCDC_BITP  12

#define GPIO_VCORE_PORT 0
#define GPIO_VCORE_BITP 4

#define GPIO_VIO_PORT 0
#define GPIO_VIO_BITP 2

#define GPIO_RSTL_PORT 3
#define GPIO_RSTL_BITP 11

#define GPIO_STBYL_PORT 3
#define GPIO_STBYL_BITP 10

#define GPIO_RDY_PORT 3
#define GPIO_RDY_BITP 9

#define GPIO_ERR_PORT 3
#define GPIO_ERR_BITP 7

#define PANEL_WIDTH   800
#define PANEL_HEIGHT  600

#define DATA_WIDTH  (PANEL_WIDTH/2)

#define DDW  (DATA_WIDTH-1)
#define DHW  23
#define DBW  19
#define DEW  19

#define GAP            (DHW+DBW+DEW+3)
#define BUF_LINE_LEN   (DATA_WIDTH*2+GAP)
#define BUF_LINE_NUM   312
#define IMG_START_LINE 22
#define IMG_SIZE       (PANEL_WIDTH*PANEL_HEIGHT)

#define CMD_CS_POS 32
#define WFM_CS_POS (BUF_LINE_LEN*10+DATA_WIDTH+GAP+192)
#define IMG_CS_POS (BUF_LINE_LEN*(BUF_LINE_NUM-1))

#define EB_WIDTH   (BUF_LINE_LEN*2)
#define EB_HEIGHT  BUF_LINE_NUM

#define VSW 15  /* V_WIDTH  */
#define VBW 15  /* V_WAIT_2 */
#define VEW 15  /* V_WAIT_1 */

#define HSW 9  /* H_WIDTH  */
#define HBW 8  /* H_WAIT_2 */
#define HEW 10 /* H_WAIT_1 */

#define ONE_FRAME_LINES  (PANEL_HEIGHT+VSW+VBW+VEW)

#define PWR1 (ONE_FRAME_LINES/2)
#define PWR2 (ONE_FRAME_LINES/2)
#define PWR3 100

#define SDLEW 4
#define SDOSZ 2
#define SDOR  0
#define SDCES 0
#define SDCER 0
#define GDSPL (IMG_START_LINE+VSW+VBW-4-1)
#define GDRL  1
#define SDSHR 1
#define GDSPP 0
#define GDSPW 18
#define DISPC 0
#define VDLC (PANEL_HEIGHT-1)
#define DSI  0
#define DSIC 0

/* ======================================================================= *
 * LM75 stuff                                                              *
 * ======================================================================= */
#define I2C_IFDR 0x04
#define I2C_I2CR 0x08
#define I2C_I2SR 0x0C
#define I2C_I2DR 0x10

#define I2C_IFDR_VAL __REG(IMX_I2C_BASE + I2C_IFDR)
#define I2C_I2CR_VAL __REG(IMX_I2C_BASE + I2C_I2CR )
#define I2C_I2SR_VAL __REG(IMX_I2C_BASE + I2C_I2SR )
#define I2C_I2DR_VAL __REG(IMX_I2C_BASE + I2C_I2DR )


static int lm75_init_regs( void );
static int lm75_read_config( unsigned int * v );
static int lm75_read_bytes( int n, unsigned int * vals );
static int lm75_receive_data( unsigned int * val );
static int lm75_write_bytes( int n, unsigned int * vals );
static int lm75_send_data( unsigned int val );
static int lm75_stop( void );
static int lm75_start( void );
static int lm75_ack_received( void );
static int lm75_wait_for_iif( int timeout );
static int lm75_read_temp( struct metronomefb_par * );
static int lm75_power_down( void );
static int lm75_grab_bus( int timeout );

unsigned long flags;

static void prs505_set_vcore(struct metronomefb_par *par, int v )
{
	if ( v > 0 ) DR(reg_A) |= 1 << GPIO_VCORE_BITP;
	else         DR(reg_A) &= ~( 1 << GPIO_VCORE_BITP );
}

static void prs505_set_vio(struct metronomefb_par *par, int v )
{
	if ( v > 0 ) DR(reg_A) |= 1 << GPIO_VIO_BITP;
	else         DR(reg_A) &= ~( 1 << GPIO_VIO_BITP );
}
static void prs505_init_gpio_regs (struct metronomefb_par *par)
{
	imx_gpio_mode(PD9_INT);  // register RDY
	imx_gpio_mode(PD7_ERR);  // register ERR
	imx_gpio_mode(PD10_STBYL);  // register STBYL
	imx_gpio_mode(PD11_RSTL);  // register RSTL
	imx_gpio_mode(PA2_VIO);  // register VIO
	imx_gpio_mode(PA4_VCORE);  // register VCOR
	imx_gpio_mode(PD8_CLS);  // register CLS


	/* register port_D LCDC pins */
	imx_gpio_mode(PD30_LCDC);
	imx_gpio_mode(PD29_LCDC);
	imx_gpio_mode(PD28_LCDC);
	imx_gpio_mode(PD27_LCDC);
	imx_gpio_mode(PD26_LCDC);
	imx_gpio_mode(PD25_LCDC);
	imx_gpio_mode(PD24_LCDC);
	imx_gpio_mode(PD23_LCDC);
	imx_gpio_mode(PD22_LCDC);
	imx_gpio_mode(PD21_LCDC);
	imx_gpio_mode(PD20_LCDC);
	imx_gpio_mode(PD19_LCDC);
	imx_gpio_mode(PD18_LCDC);
	imx_gpio_mode(PD17_LCDC);
	imx_gpio_mode(PD16_LCDC);
	imx_gpio_mode(PD15_LCDC);
	imx_gpio_mode(PD14_LCDC);
	imx_gpio_mode(PD13_LCDC);
	imx_gpio_mode(PD12_LCDC);
	imx_gpio_mode(PD6_LCDC);

	DR(reg_A) &= ~( 1 << GPIO_VCORE_BITP );
	DR(reg_A) &= ~( 1 << GPIO_VIO_BITP );

	DR(reg_D) &= ~( 1 << GPIO_RSTL_BITP );
	DR(reg_D) &= ~( 1 << GPIO_STBYL_BITP );
	DR(reg_D) |= ( 1 << 8 );

	DR(reg_D) &= ~( 1 << GPIO_LSCLK_BITP );
	DR(reg_D) &= ~( 0x7ffff << GPIO_LCDC_BITP );

	GPR(reg_D) &= ~( 1 << GPIO_LSCLK_BITP );
	GPR(reg_D) &= ~( 0x7ffff << GPIO_LCDC_BITP );
//	IMR(reg_D) &= ~(0x00000200);

	GIUS(reg_D)  &= ~( 1 << GPIO_LSCLK_BITP );
	GIUS(reg_D)  &= ~( 0x7ffff << GPIO_LCDC_BITP );

	prs505_set_vcore( par, 1 );
	prs505_set_vio( par, 1 );
	mdelay(10);
}

static void prs505_init_lcdc_regs(struct metronomefb_par *par)
{
	unsigned int val = 0;

	if ( ( ( CSCR >> BCLK_DIV_BITP ) & BCLK_DIV_MASK ) != 0 ) {
		printk( "<<< ERROR: BCLK_DIV != 0\n" );
		return ;
	}

	PCDR &= ~( PCLK_DIV2_MASK << PCLK_DIV2_BITP );
	PCDR |= 1 << PCLK_DIV2_BITP;

	LCDC_RMCR= 0x0 ; // disable LCDC

//	val = par->info->fix.smem_start;
	LCDC_SSA = par->metromem_dma;

//	val = ( ( BUF_LINE_LEN / 16 ) << 20 ) | BUF_LINE_NUM;
	val = ((par->info->var.xres / 16) << 20) | par->info->var.yres;
	LCDC_SIZE= val ;

//	val = BUF_LINE_LEN / 2;
	val = par->info->var.xres / 2;
	LCDC_VPW= val ;

	val = HBW | ( HEW << 8 ) | ( HSW << 26 );
	LCDC_HCR= val ;

	val = VBW | ( VEW << 8 ) | ( VSW << 26 );
	LCDC_VCR= val ;

	val = ( 1 << 31 ) /* TFT */
		| ( 1 << 30 ) /* COLOR */
		| ( 0 << 28 ) /* PBSIZ */
		| ( 4 << 25 ) /* BPIX */
		| ( 0 << 24 ) /* PIXPOL */
		| ( 0 << 23 ) /* FLMPOL */
		| ( 0 << 22 ) /* LPPOL */
		| ( 1 << 21 ) /* CLKPOL */
		| ( 0 << 20 ) /* OEPOL */
		| ( 1 << 19 ) /* SCLKIDLE */
		| ( 0 << 18 ) /* END_SEL */
		| ( 0 << 17 ) /* SWAP_SEL */
		| ( 0 << 16 ) /* REV_VS */
		| ( 0 << 15 ) /* ACDSEL */
		| ( 0 << 8 )  /* ACD */
		| ( 1 << 7 )  /* SCLKSEL */
		| ( 0 << 6 )  /* SHARP */
		| ( 2 << 0 ); /* PCD */
	LCDC_PCR= val ;

	val = ( 3 << 16 ) | 10;
	LCDC_DMACR= val ;

}

static int prs505_init_lm75_regs(struct metronomefb_par *par)
{
	int res;
	unsigned int temperature_val;

	res = lm75_init_regs( );
	if (res < 0) {
		return res;
	}

	res = lm75_read_config( &temperature_val );
	if (res < 0) {
		return res;
	}

	if ( ( temperature_val & 0x1 ) == 0 ) {
		res = lm75_power_down( );
		if (res < 0) {
			return res;
		}
	}

	return 0;	
}

static void prs505_set_rst(struct metronomefb_par *par, int v)
{
	if ( v > 0 ) DR(reg_D) |= 1 << GPIO_RSTL_BITP;
	else         DR(reg_D) &= ~( 1 << GPIO_RSTL_BITP );
	printk("%s\n", __FUNCTION__);
}
static void prs505_set_stdby(struct metronomefb_par *par, int v)
{
	if ( v > 0 ) DR(reg_D) |= 1 << GPIO_STBYL_BITP;
	else         DR(reg_D) &= ~( 1 << GPIO_STBYL_BITP );
	printk("%s\n", __FUNCTION__);
}
static void prs505_set_lcd(struct metronomefb_par *par , int v)
{
	if (v>0) {
		GIUS(reg_D)  &= ~( 1 << GPIO_LSCLK_BITP );
		GIUS(reg_D)  &= ~( 0x7ffff << GPIO_LCDC_BITP );
		LCDC_RMCR = RMCR_LCDC_EN;
	}else{
		LCDC_RMCR = 0;

		GIUS(reg_D)  |= 1 << GPIO_LSCLK_BITP;
		GIUS(reg_D) |= 0x7ffff << GPIO_LCDC_BITP;
	}
}
static unsigned int prs505_get_rdy(struct metronomefb_par *par)
{
	unsigned int val = SSR(reg_D);
	val = ( val >> GPIO_RDY_BITP ) & 0x1;
//	printk("%s: val = %d\n", __FUNCTION__, val);
	return val;
}
static unsigned int prs505_get_err(struct metronomefb_par *par)
{
	return ( SSR(reg_D) >> GPIO_ERR_BITP ) & 0x1;
}

void check_ebuff( struct metronomefb_par *par );
void check_dmaur( void );

static int prs505_wait_event(struct metronomefb_par *par)
{
	int ret;
	ret = wait_event_timeout(par->waitq, prs505_get_rdy(par), HZ*3);
	printk("%s: ret = %d, prs505_get_rdy(par) = %d, ERR = %d\n", __FUNCTION__, ret, prs505_get_rdy(par), prs505_get_err(par));
	if (ret == HZ*3) {
		check_ebuff(par);
		check_dmaur();
	}
	return ret;
}

static int prs505_wait_event_intr(struct metronomefb_par *par)
{
	int ret;
	ret = wait_event_interruptible_timeout(par->waitq, prs505_get_rdy(par), HZ*3);
	printk("%s: ret = %d, prs505_get_rdy(par) = %d\n", __FUNCTION__, ret, prs505_get_rdy(par));
	return ret;
//	return wait_event_interruptible_timeout(par->waitq, prs505_get_rdy(par), HZ);
}


/* FIXME: write not brain-damaged implementation */
static int prs505_wait_for_rdy(struct metronomefb_par *par , int v, int timeout)
{
	int res;
	while ( prs505_get_rdy( par ) != v ) {
		for(res=0; res<500000; res++){} // wait 10000usec (10 millisecond)
		if( timeout-- < 0 ) return -1;
		schedule();	
		printk("wait_for_rdy\n");
	}
	return 0;
}

void check_ebuff( struct metronomefb_par *par )
{ 
	unsigned short * bp = (unsigned short *) par->metromem;
	unsigned short cs = 0;
	int i;
	int p;
	int x;

	for ( i = 0; i < 32; i++ ) cs += bp[i];

	if ( bp[32] == cs ) printk( "+++ CMD CS --- ok\n" );
	else                printk( "+++ CMD CS --- err\n" );

	p = 464;
	cs = 0;
	x = 0;

	for ( i = 0; i < 8192; i++ ) {
		cs += bp[p++];
		x++;
		if ( x == 800 ) { p += 64; x = 0; };
	}

	printk("p = %d, cs = %u\n", p, cs);
	if ( bp[p] == cs ) printk( "+++ WFM CS --- ok\n" );
	else               printk( "+++ WFM CS --- err\n" );

	p = 864 * 11;
	x = 400;
	cs = 0;

	for ( i = 0; i < 240000; i++ ) {
		cs += bp[p++];
		x++;
		if ( x == 800 ) { p += 64; x = 0; }
	}

	if ( bp[p] == cs ) printk( "+++ IMG CS --- ok\n" );
	else               printk( "+++ IMG CS --- err\n" );
}


void check_dmaur( void )
{
	if ( LCDC_LCDISR & 0x8 ) printk( "+++ DMA UR --- yes\n" );
	else             printk( "+++ DMA UR --- no\n" );
}

#if 0
static void prs505_umask_irq(void)
{
	IMR(reg_D) |= 0x00000200;
}
static void prs505_mask_irq(void)
{
	IMR(reg_D) &= ~0x00000200;
}
#endif
/* 
 * metronome_ready_high() -- interrupt handler 
 *
 */
static irqreturn_t prs505_ready_high(int irq, void *dev_id)
{
	int res;
	struct fb_info *info = dev_id;
	struct metronomefb_par *par = info->par;


	if (prs505_get_rdy( par ) != 1) {
		printk("EROR RDY != 1\n");
	}
	printk("RDY intr\n");
#if 0 /*HO*/
	if ( prs505_get_err( par ) ) {
		printk( "++++++++++++ ERR=1\n" );
		check_ebuff( info );
		check_dmaur( );
	}
#endif
	printk("IMR(reg_D) is 0x%x\n", (IMR(reg_D)));
	printk("ISR(reg_D) is 0x%x\n", (ISR(reg_D)));
#if 0
	local_irq_save(flags);
	prs505_mask_irq();
	local_irq_restore(flags);

	/* goto sleep*/	
	prs505_set_rst( par, 0);
	prs505_set_stdby( par, 0);
	for(res=0; res<500000; res++){} // wait 10000usec (10 millisecond)
	prs505_set_lcd( par, 0);

	if ( res < 0 ) {
		panic("Unable to put 8track controller to sleep mode");
	}
//	par->prs505_power_mode = POWER_MODE_SLEEP;

	/* goto standby*/
	prs505_set_vcore( par, 0 );
	prs505_set_vio( par, 0 );

	if ( res < 0 ) {
		panic("Unable to put 8track controller to standby mode");
	}

//	par->prs505_power_mode = POWER_MODE_STANDBY;

//	par->prs505_standby = 1;
//	par->prs505_display_status = 0;
#endif
	wake_up_interruptible(&par->waitq);
	return IRQ_HANDLED;
}

static void prs505_post_dma_setup(struct metronomefb_par *par)
{
	par->metromem_desc->mFDADR0 = par->metromem_desc_dma;
	par->metromem_desc->mFSADR0 = par->metromem_dma;
	par->metromem_desc->mFIDR0 = 0;
	par->metromem_desc->mLDCMD0 = par->info->var.xres
					* par->info->var.yres;
	LCDC_RMCR = RMCR_LCDC_EN;
	mdelay(10);
	printk("%s: RDY = %d\n", __FUNCTION__, prs505_get_rdy(par));
}

static int prs505_setup_irq(struct fb_info *info)
{
	int res;
	res = set_irq_type(GPIO_INTR, IRQ_TYPE_EDGE_RISING);
	if(res){
		printk("cannot set type\n");
		return res;
	}

	/* set RDY=1 interrupt */
	res = request_irq(GPIO_INTR, prs505_ready_high, IRQF_SHARED, "e-ink", info);
	if (res < 0) {
		printk("error: request_irq %d\n", res);
		return res;
	}

	printk("IMR(reg_D) is 0x%x\n", (IMR(reg_D)));
	printk("ISR(reg_D) is 0x%x\n", (ISR(reg_D)));
	printk("%s: done\n", __FUNCTION__);
	return res;
}
static void prs505_free_irq(struct fb_info *info)
{
	free_irq(GPIO_INTR, info);
}

#if 0
/* ======================================================================= *
 * LM75                                                                    *
 * ======================================================================= */
static int lm75_read_temp( struct metronomefb_par * par )
{
	int res;
	unsigned int vals[2];

	vals[0] = 1;
	vals[1] = 0x81;
	res = lm75_write_bytes( 2, vals );
	if ( res < 0 ) return res;

	vals[0] = 0;
	res = lm75_write_bytes( 1, vals );
	if ( res < 0 ) return res;

	res = lm75_read_bytes( 2, vals );
	if ( res < 0 ) return res;

//	par->metronome_temperature = vals[0];

	return 0;
}

static int lm75_power_down( void )
{
	int res;
	unsigned int vals[3];
	unsigned int val;

	vals[0] = 1;
	vals[1] = 1;
	res = lm75_write_bytes( 2, vals );
	if ( res < 0 ) return -1;

	res = lm75_read_config( &val );
	if ( res < 0 ) return res;

	if ( ( val & 0x1 ) == 0 ) {
		printk( "<<< ERROR: failed to power down LM75\n" );
		return -1;
	}

	return 0;
}

static int lm75_receive_data( unsigned int * val )
{
	int res;

	*val = I2C_I2DR_VAL; // dummy read

	I2C_I2SR_VAL = 0 ;

	res = lm75_wait_for_iif( 10000 );

	return res;
}

static int lm75_read_bytes( int n, unsigned int * vals )
{
	int res;
	int i;
	unsigned int val;

	res = lm75_start( );
	if ( res < 0 ) return res;

	res = lm75_send_data( 0x91 );
	if ( res < 0 ) return res;

	I2C_I2CR_VAL &= ~( 1 << 4 ); // MTX = 0
	if ( n == 1 ) I2C_I2CR_VAL |= ( 1 << 3 ); // TXAK = 1;
	else          I2C_I2CR_VAL &= ~( 1 << 3 ); // TXAK = 0;
	I2C_I2CR_VAL &= ~( 1 << 3 ); // TXAK = 0;

	res = lm75_receive_data( &val ); // dummy
	if ( res < 0 ) return res;

	for ( i = 0; i < n; i++ ) {

		if ( i == ( n-1 ) ) {
			I2C_I2CR_VAL |= ( 1 << 3 ); // TXAK = 1;
		}

		res = lm75_receive_data( &vals[i] );
		if ( res < 0 ) return res;
	} 

	res = lm75_stop( );

	return res;
}

static int lm75_wait_for_iif( int timeout )
{
	while ( 1 ) {
		if ( ( I2C_I2SR_VAL >> 1 ) & 0x1 ) break;
		timeout--;
		if ( timeout < 0 ) {
			printk( "<<< ERROR: timeout in waiting for iif\n" );
			return -1;
		}
	}
	if ( ( ( I2C_I2SR_VAL >> 7 ) & 0x1 ) == 0 ) {
		printk( "<<< ERROR: data transfer not complete\n" );
		return -1;
	}

	return 0;
}

static int lm75_ack_received( void )
{
	if ( ( I2C_I2SR_VAL & 0x1 ) != 0 ) {
		printk( "<<< ERROR: no ack was not received from lm75\n" );
		return -1;
	}
	return 0;
}

static int lm75_send_data( unsigned int val )
{
	int res;

	I2C_I2DR_VAL = val ;
	I2C_I2SR_VAL = 0   ;

	res = lm75_wait_for_iif( 10000 );
	if ( res < 0 ) return res;

	res = lm75_ack_received( );

	return res;
}

static int lm75_start( void )
{
	int res;

	res = lm75_grab_bus( 100 );
	if ( res < 0 ) return res;

	I2C_I2CR_VAL |= 1 << 5; // MSTA = 1
	I2C_I2CR_VAL |= 1 << 4; // MTX = 1
	I2C_I2CR_VAL |= 1 << 3; // TXAK = 1
	I2C_I2CR_VAL &= ~( 1 << 2 ); // RSTA = 0

	return 0;
}

static int lm75_stop( void )
{
	I2C_I2CR_VAL &= ~( 1 << 5 ); // MSTA = 0;
	return 0;
}

static int lm75_write_bytes( int n, unsigned int * vals )
{
	int res;
	int i;

	res = lm75_start( );
	if ( res < 0 ) return res;

	res = lm75_send_data( 0x90 );
	if ( res < 0 ) return res;

	for ( i = 0; i < n; i++ ) {
		res = lm75_send_data( vals[i] );
		if ( res < 0 ) return res;
	} 

	res = lm75_stop( );

	return res;
}

static int lm75_read_config( unsigned int * v )
{
	int res;
	unsigned int val;

	val = 1;
	res = lm75_write_bytes( 1, &val );
	if ( res < 0 ) return -1;

	res = lm75_read_bytes( 1, &val );
	if ( res < 0 ) return -1;

	*v = val;

	return 0;
}

static void lm75_sleep( int t )
{
	int s = 0;
	int n = t * 1000;
	int i;
	for ( i = 0; i < n; i++ ) s += 1;
}

static int lm75_grab_bus( int timeout )
{

	while ( timeout >= 0 ) {
		if ( ( I2C_I2SR_VAL & 0x30 ) == 0 ) break;
		timeout--;
		lm75_sleep( 10 );
	}
	if ( timeout < 0 ) {
		printk( "<<< ERROR: failed to grab the i2c bus : restart LM75\n");
		lm75_stop();
		return 0;
	}
	return 0;
}

static int lm75_init_regs( void )
{
	unsigned int val;
	int res;

	GIUS(reg_A) &= ~( 3 << 15 );
	val = GIUS(reg_A);
	GPR(reg_A) &= ~( 3 << 15 );
	val = GPR(reg_A);

	val = 0x07;  // HCLK Divider (72) --- 1.3 MHz
	I2C_IFDR_VAL = val ;
	val = I2C_IFDR_VAL;

	val = 0;
	val |= 1 << 7; // IEN = 1
	I2C_I2CR_VAL = val ;
	val = I2C_I2CR_VAL;

	res = lm75_grab_bus( 10000 );

	return 0;
}
#endif


static struct metronome_board prs505_board = {
	.owner			= THIS_MODULE,
	.free_irq		= prs505_free_irq,
	.setup_irq		= prs505_setup_irq,
	.init_gpio_regs		= prs505_init_gpio_regs,
	.init_lcdc_regs		= prs505_init_lcdc_regs,
	.post_dma_setup		= prs505_post_dma_setup,
//	.init_lm75_regs 	= prs505_init_lm75_regs,
	.set_rst		= prs505_set_rst,
	.set_stdby		= prs505_set_stdby,
//	.get_rdy		= prs505_get_rdy,
//	.set_lcd	= prs505_set_lcd,
//	.set_vcore	= prs505_set_vcore,
//	.set_vio	= prs505_set_vio,
//	.lm75_read_temp = lm75_read_temp,
	.met_wait_event	= prs505_wait_event,
	.met_wait_event_intr	= prs505_wait_event_intr,
//	.umask_irq	= prs505_umask_irq,
//	.mask_irq	= prs505_mask_irq,
//	.get_err	= prs505_get_err,
};

static struct platform_device *prs505_device;

static int __init prs505_init(void)
{
	int ret;

	/* request our platform independent driver */
	request_module("metronomefb");

	prs505_device = platform_device_alloc("metronomefb", -1);
	if (!prs505_device)
		return -ENOMEM;

	platform_device_add_data(prs505_device, &prs505_board,
			sizeof(prs505_board));

	/* this _add binds metronomefb to sony 505 */
	ret = platform_device_add(prs505_device);

	if (ret)
		platform_device_put(prs505_device);

	return ret;
}

static void __exit prs505_exit(void)
{
	platform_device_unregister(prs505_device);
}

module_init(prs505_init);
module_exit(prs505_exit);

MODULE_DESCRIPTION("board driver for sony prs-505 eink");
MODULE_AUTHOR("Zhang Wenjie");
MODULE_LICENSE("GPL");

