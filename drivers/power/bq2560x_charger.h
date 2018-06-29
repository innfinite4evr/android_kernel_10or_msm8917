/*
 * BQ2560x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef _LINUX_BQ2560X_I2C_H
#define _LINUX_BQ2560X_I2C_H

#include <linux/power_supply.h>

/* if open the otg, need config the phy_irq and otg_mode in dts*/
#define BQ2415X_OTG_FUNC_EN	1

/********** reset state for all registers**********/

#define BQ2415X_RESET_STATUS		BIT(6)
#define BQ2415X_RESET_CONTROL		(BIT(4)|BIT(5))
#define BQ2415X_RESET_VOLTAGE		(BIT(1)|BIT(3))
#define BQ2415X_RESET_CURRENT		(BIT(0)|BIT(3)|BIT(7))

/************* status register 00**************/
#define BQ2415X_REG00_STATUS		0x00

#define BQ2415X_HIGH_OTGSTAT		1
#define BQ2415X_LOW_OTGSTAT		0
#define BQ2415X_SHIFT_OTGSTAT		7
#define BQ2415X_MASK_OTGSTAT		BIT(7)


#define BQ2415X_ENABLE_STAT		1
#define BQ2415X_DISABLE_STAT		0
#define BQ2415X_SHIFT_STAT		6
#define BQ2415X_MASK_STAT		BIT(6)


#define BQ2415X_READY_CHGSTAT		0
#define BQ2415X_CHARGING_CHGSTAT	1
#define BQ2415X_DONE_CHGSTAT		2
#define BQ2415X_FAULT_CHGSTAT		3
#define BQ2415X_SHIFT_CHGSTAT		4
#define BQ2415X_MASK_CHGSTAT		(BIT(4)|BIT(5))


#define BQ2415X_IN_BOOST		1
#define BQ2415X_NO_BOOST		0
#define BQ2415X_SHIFT_BOOST		3
#define BQ2415X_MASK_BOOST		BIT(3)

#define BQ2415X_MASK_FAULT		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_FAULT 		0

/*************** control register 01****************/
#define BQ2415X_REG01_CONTROL 		0x01

#define BQ2415X_100_IIN_LIMIT		0
#define BQ2415X_500_IIN_LIMIT		1
#define BQ2415X_800_IIN_LIMIT		2
#define BQ2415X_NO_IIN_LIMIT		3
#define BQ2415X_MASK_IIN_LIMIT		(BIT(6)|BIT(7))
#define BQ2415X_SHIFT_IIN_LIMIT		6

#define BQ2415X_MASK_VLOWV		(BIT(4)|BIT(5))
#define BQ2415X_SHIFT_VLOWV		4

#define BQ2415X_ENABLE_TERM		1
#define BQ2415X_DISABLE_TERM		0
#define BQ2415X_SHIFT_TERM		3
#define BQ2415X_MASK_TERM		BIT(3)



#define BQ2415X_ENABLE_CHARG		0
#define BQ2415X_DISABLE_CHARG		1
#define BQ2415X_SHIFT_CHARG		2
#define BQ2415X_MASK_CHARG		BIT(2)


#define BQ2415X_NOTIN_HZ		0
#define BQ2415X_IN_HZ			1
#define BQ2415X_SHIFT_HZ		1
#define BQ2415X_MASK_HZ			BIT(1)

#define BQ2415X_CHARGER_MODE		0
#define BQ2415X_BOOST_MODE		1
#define BQ2415X_SHIFT_OPA_MODE		0
#define BQ2415X_MASK_OPA_MODE		BIT(0)

/*************** voltage register 02**************/
#define BQ2415X_REG02_VOLTAGE		0x02

#define BQ2415X_4400MV_VO		0x2d
#define BQ2415X_MASK_VO 		(BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7))
#define BQ2415X_SHIFT_VO		2

#define BQ2415X_BIT_OTG_PL		1

#define BQ2415X_ENABLE_OTG		1
#define BQ2415X_DISABLE_OTG		0
#define BQ2415X_SHIFT_OTG		0
#define BQ2415X_MASK_OTG		BIT(0)


/***************** vender register 03**************/
#define BQ2415X_REG03_VENDER		0x03

#define BQ2415X_MASK_VENDER		(BIT(5)|BIT(6)|BIT(7))
#define BQ2415X_SHIFT_VENDER		5

#define BQ2415X_MASK_PN			(BIT(3)|BIT(4))
#define BQ2415X_SHIFT_PN		3

#define BQ2415X_MASK_REVISION		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_REVISION		0

/**************** current register 04***************/
#define BQ2415X_REG04_CURRENT		0x04

#define BQ2415X_CHG_RESET		1
#define BQ2415X_MASK_RESET		BIT(7)
#define BQ2415X_SHIFT_RESET		7


#define BQ2415X_950MA_ICHRG		4
#define BQ2415X_MASK_ICHRG		(BIT(4)|BIT(5)|BIT(6))
#define BQ2415X_SHIFT_ICHRG		4

		/* N/A					BIT(3) */

#define BQ2415X_100MA_ITERM		1
#define BQ2415X_MASK_ITERM		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_ITERM		0


/**************** adaptive register 05*************/
#define BQ2415X_REG05_ADAPTIVE		0x05

#define BQ2415X_EN_LOWCHG		1
#define BQ2415X_DIS_LOWCHG		0

#define BQ2415X_MASK_LOWCHG 	(BIT(5))
#define BQ2415X_SHIFT_LOWCHG		5


/************** current register 06****************/
#define BQ2415X_REG06_LIMIT		0x06

#define BQ2415X_MASK_IMAX_LIMIT		(BIT(7)|BIT(6)|BIT(5)|BIT(4))
#define BQ2415X_SHIFT_IMAX_LIMIT		4


#define BQ2415X_MASK_VMAX_LIMIT		(BIT(3)|BIT(2)|BIT(1)|BIT(0))
#define BQ2415X_SHIFT_VMAX_LIMIT		0


struct bq2415x_charge_param {
	int vlim;
	int ilim;
	int ichg;
	int vreg;
};

enum stat_ctrl {
	STAT_CTRL_STAT,
	STAT_CTRL_ICHG,
	STAT_CTRL_INDPM,
	STAT_CTRL_DISABLE,
};

struct bq2415x_platform_data {
	struct bq2415x_charge_param usb;
	struct bq2415x_charge_param atl;
	int iprechg;
	int iterm;

	enum stat_ctrl statctrl;

};

#endif
