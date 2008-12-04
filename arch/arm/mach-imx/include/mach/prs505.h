/*
 * arch/arm/mach-imx/include/mach/prs505.h
 *
 * Copyright (C) 2008, Yauhen Kharuzhy <jekhor@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef __ASM_ARCH_PRS505_H
#define __ASM_ARCH_PRS505_H

/* ------------------------------------------------------------------------ */
/* Memory Map for the Sony PRS-505		                            */
/* ------------------------------------------------------------------------ */

#define PRS505_FLASH_PHYS		0x10000000
#define PRS505_FLASH_SIZE		(2*1024*1024)

#define IMX_FB_PHYS			(0x0C000000 - 0x40000)

#define CLK32 32768

#endif /* __ASM_ARCH_PRS505_H */

