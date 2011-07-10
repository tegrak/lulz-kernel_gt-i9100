/* ------------------------------------------------------------------------- */
/* 									     */
/* s5p-thermal.h - definitions of s5pv310 specific thermal interface	     */
/* 									     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2010 Samsung Electronics Co. ltd.
 
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.		     */
/* ------------------------------------------------------------------------- */

#ifndef _S5PV310_THERMAL_H
#define _S5PV310_THERMAL_H


struct tmu_data {
	unsigned int	t1;				/* cooling stop temperature */
	unsigned int	t2;				/* trimming start temperature */
	unsigned int 	thr_temp;		/* tmu threshold value */
	int				mode;			/* compensation mode */
};

struct tmu_platform_device {
	int 				id;
	void __iomem		*tmu_base;
	struct device		*dev;
	struct tmu_data		data;
	int	   dvfs_flag;
};

extern void s5p_tmu_set_platdata(struct tmu_data *pd);
extern struct tmu_platform_device *s5p_tmu_get_platdata(void);
extern int s5p_tmu_get_irqno(int num);

#endif /* _S5PV310_THERMAL_H */
