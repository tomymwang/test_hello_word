// SPDX-License-Identifier: GPL-2.0
/* clk-idtxp.c - Program xp family settings via I2C.
 *
 * Copyright (C) 2018, Integrated Device Technology, Inc. <@idt.com>
 *
 * See https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html
 * This program is distributed "AS IS" and  WITHOUT ANY WARRANTY;
 * including the implied warranties of MERCHANTABILITY, FITNESS FOR
 * A PARTICULAR PURPOSE, or NON-INFRINGEMENT.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/debugfs.h>

#define NUM_CONFIG_REGISTERS		256
#define NUM_FREQ_REGISTERS		6
#define NUM_MISCELLANEOUS_REGISTERS	8

#define DEBUGFS_ROOT_DIR_NAME		"idtxp_pro_xo"
#define DEBUGFS_I2C_FILE_NAME		"i2c"

/* Frequency0 */
#define IDTXP_REG_DIVO_7_0			0x10
#define IDTXP_REG_DIVO_8_DIVN_INT_6_0		0x11
#define IDTXP_REG_ICP_DIVN_INT_8_7_MODE		0x12
#define IDTXP_REG_DIVN_FRAC_7_0			0x13
#define IDTXP_REG_DIVN_FRAC_15_8		0x14
#define IDTXP_REG_DIVN_FRAC_23_16		0x15

/* Miscellaneous settings */
#define IDTXP_REG_HSPI2C_CMOS			0x50
#define IDTXP_REG_DBLR_DIS_VDD			0x51
#define IDTXP_REG_VCXO				0x52
#define IDTXP_REG_OE_POL_DRV_TYPE		0x53
//#define IDTXP_REG_I2C_ADDR			0x54
#define IDTXP_REG_XO_0				0x55
#define IDTXP_REG_XO_1				0x56
#define IDTXP_REG_XO_2				0x57

/* Active trigger control commands */
#define IDTXP_REG_CONTROL			0x60
#define IDTXP_REG_FREQ_CHG			0x62

/* Masks */
#define IDTXP_DIVO_8_MASK			0x80
#define IDTXP_DIVN_INT_6_0_MASK			0x7F
#define IDTXP_ICP_OFFSET_EN_MASK		0x40
#define IDTXP_DIVN_INT_8_7_MASK			0x30
#define IDTXP_ICP_VALUE_MASK			0x0E
#define IDTXP_PLL_MODE_MASK			0x01
#define IDTXP_HSPI2C_EN				0x10
#define IDTXP_CMOS_EN				0x08
#define IDTXP_DBLR_DIS_MASK			0x80
#define IDTXP_VDD_DEF_MASK			0x60
#define IDTXP_VCXO_EN_MASK			0x04
#define IDTXP_VCXO_BW_MASK			0x03
#define IDTXP_GSLOPE_MASK			0x80
#define IDTXP_GEXP_MASK				0x70
#define IDTXP_GSCALE_MASK			0x0F
#define IDTXP_OE_POL_EN				0x80
#define IDTXP_DRV_TYPE				0x70
#define IDTXP_OT_GM_MASK			0xC0
#define IDTXP_XO_CAP_MASK			0x3F
#define IDTXP_XO_AMPSLICE_MASK			0xF0
#define IDTXP_BYPASS_MASK			0x08
#define IDTXP_CAP_X2_MASK			0x07
#define IDTXP_OT_DIS_MASK			0x80
#define IDTXP_OT_RES_MASK			0x70
#define IDTXP_NVMCP_TO_NVM_MASK			0x20
#define IDTXP_LOCK_PLL_MASK			0x01
#define IDTXP_SMALL_FREQ_CHG_MASK		0x02
#define IDTXP_LARGE_FREQ_CHG_MASK		0x01

/* Limits */
#define DIVO_MIN    		4
#define DIVO_MAX    		511

#define DIVN_MIN    		41
#define DIVN_MAX   		216

#define FVCO_MIN    		6860000000LL
#define FVCO_MAX    		8650000000LL

#define IDTXP_MIN_FREQ          16000000LL
#define IDTXP_MAX_FREQ          2100000000LL
#define IDTXP_HCSL_MAX_FREQ     725000000LL

/**
 * @hsp_i2c_en:		high speed i2c enable
 * @cmos_en:		cmos output enable
 * @dblr_dis:		XO frequency doubler disable
 * @vdd_def:		power supply voltage
 * @vcxo_dis:		vcxo disabler
 * @vcxo_bw:		vcxo modulation bandwidth
 * @vcxo_gslope:	vcxo gain slope
 * @vcxo_gexp:		vcxo gain expoentially
 * @vcxo_gscale:	vcxo gain scale
 * @oe_pol_en:		output enable polarity
 * @drv_type:		output logic type
 * @gm:			XO amplifier gm overtone
 * @cap_x1:		XO load capacitance trim value, x1 pin
 * @ampslice:		XO amplifier slice
 * @bypass:		bypass the XO oscillator
 * @cap_x2:		XO load capacitance trim value, x2 pin
 * @ot_dis:		overtone operation disable
 * @ot_res:		overtone filter resistor value
 */
struct clk_xo_setting {
	bool hsp_i2c_en;
	bool cmos_en;
	bool dblr_dis;
	u8 vdd_def;
	bool vcxo_dis;
	u8 vcxo_bw;
	bool vcxo_gslope;
	u8 vcxo_gexp;
	u8 vcxo_gscale;
	bool oe_pol_en;
	u8 drv_type;
	u8 gm;
	u8 cap_x1;
	u8 ampslice;
	bool bypass;
	u8 cap_x2;
	bool ot_dis;
	u8 ot_res;
};

/**
 * struct clk_idtxp:
 * @hw:			clock hw struct
 * @regmap:		register map used to perform i2c writes to the chip
 * @i2c_client:		I2C client pointer
 * @has_settings:	true if settings array is valid
 * @settings:		full register map from device tree
 * @min_freq:		mininum frequency for this device
 * @max_freq:		maximum frequency for this device
 * @xo:			struct for the miscellaneous settings and XO mode
 * @fxtal:		factory xtal frequency
 * @divo:		output clock divider
 * @divnint:		int component of feedback divider for VCO
 * @divnfrac:		fractional component of feedback divider for VCO
 * @req_freq:		request output frequency (in Hz)
 * @act_freq:		actual output clock frequency (in Hz)
 * @icp_offset_en:	charge pump offset enable
 * @icp_value:		charge pump value
 * @pll_mode:		pll mode
 * @divo_8:		stores the value for setting register
 * @divnint_8_7:	stores the value for setting register
 * @divnfrac_15_8:	stores the value for setting register
 * @divnfrac_23_18:	stores the value for setting register
 * @debugfs_root_dir:	the directory of debugfs
 * @debugfs_i2c_file:	read and write the registers through the i2c
 */
struct clk_idtxp {
	struct clk_hw hw;
	struct regmap *regmap;
	struct i2c_client *i2c_client;

	bool has_settings;
	u8 settings[NUM_CONFIG_REGISTERS];

	u64 min_freq;
	u64 max_freq;

	struct clk_xo_setting xo;

	u32 fxtal;
	u64 fvco;
	u16 divo;
	u16 divnint;
	u32 divnfrac;
	u32 req_freq;
	u32 act_freq;
	bool icp_offst_en;
	u8 icp_value;
	bool pll_mode;
	
	u8 divo_8;
	u8 divnint_8_7;
	u8 divnfrac_15_8;
	u8 divnfrac_23_16;

	struct dentry *debugfs_root_dir, *debugfs_i2c_file;
};
#define to_clk_idtxp(_hw)	container_of(_hw, struct clk_idtxp, hw)

enum clk_idtxp_variant {
	idtxp_xo
};

/**
 * bit_to_shift() - Number of bits to shift given specified mask.
 * @mask:	32-bit word input to count zero bits on right.
 * 
 * Return: Number of bits to shift.
 */
static int bit_to_shift(unsigned int mask)
{
	unsigned int c = 32;

	mask &= ~mask + 1;
	if (mask)
		c--;
	if (mask & 0x0000FFFF)
		c -= 16;
	if (mask & 0x00FF00FF)
		c -= 8;
	if (mask & 0x0F0F0F0F)
		c -= 4;
	if (mask & 0x33333333)
		c -= 2;
	if (mask & 0x55555555)
		c -= 1;
	return c;
}

static void set_to_reg(u8 *reg, u8 val, unsigned int mask)
{
	*reg = ((*reg) & ~mask ) | (val << bit_to_shift(mask));
}

static void get_from_reg(u8 reg, u8 *val, unsigned int mask)
{
	*val = (reg & mask) >> bit_to_shift(mask);
}


/**
 * update_divis_regs() - Update the value for registers.
 * @data: 	The clock device structure that contains all the requested
 * 		data for outputting frequency.
 * 
 */
static void update_divis_regs(struct clk_idtxp* data)
{
	data->divo_8 = ((data->divo & (0x1 << 8)) >> 8);
	data->divnint_8_7 = ((data->divo & (0x2 << 7)) >> 7);
	data->divnfrac_15_8 = ((data->divnfrac & (0xFF << 8)) >> 8);
	data->divnfrac_23_16 = ((data->divnfrac & (0xFF << 16)) >> 16);
}

/**
 * idtxp_get_xo_settings() - Read in miscellaneous settings from registers.
 * @data: 	The clock device structure that contains all the requested
 * 		data for outputting frequency.
 * 
 * Return: 0 on success, negative errno otherwise.
 */
static int idtxp_get_xo_settings(struct clk_idtxp *data)
{
	int err;
	u8 reg[8];
	struct i2c_client *client = data->i2c_client;

	err = regmap_bulk_read(data->regmap, IDTXP_REG_HSPI2C_CMOS,
			reg, ARRAY_SIZE(reg));
	if (err)
		return err;

	get_from_reg(reg[0], (u8*)&data->xo.hsp_i2c_en, IDTXP_HSPI2C_EN);
	get_from_reg(reg[0], (u8*)&data->xo.cmos_en, IDTXP_CMOS_EN);
	get_from_reg(reg[1], (u8*)&data->xo.dblr_dis, IDTXP_DBLR_DIS_MASK);
	get_from_reg(reg[1], &data->xo.vdd_def, IDTXP_VDD_DEF_MASK);
	get_from_reg(reg[1], &data->xo.vcxo_bw, IDTXP_VCXO_BW_MASK);
	get_from_reg(reg[2], (u8*)&data->xo.vcxo_gslope, IDTXP_GSLOPE_MASK);
	get_from_reg(reg[2], &data->xo.vcxo_gexp, IDTXP_GEXP_MASK);
	get_from_reg(reg[2], &data->xo.vcxo_gscale, IDTXP_GSCALE_MASK);
	get_from_reg(reg[3], (u8*)&data->xo.oe_pol_en, IDTXP_OE_POL_EN);
	get_from_reg(reg[3], &data->xo.drv_type, IDTXP_DRV_TYPE);
	get_from_reg(reg[5], &data->xo.gm, IDTXP_OT_GM_MASK);
	get_from_reg(reg[5], &data->xo.cap_x1, IDTXP_XO_CAP_MASK);
	get_from_reg(reg[6], &data->xo.ampslice, IDTXP_XO_AMPSLICE_MASK);
	get_from_reg(reg[6], (u8*)&data->xo.bypass, IDTXP_BYPASS_MASK);
	get_from_reg(reg[6], &data->xo.cap_x2, IDTXP_CAP_X2_MASK);
	get_from_reg(reg[7], (u8*)&data->xo.ot_dis, IDTXP_OT_DIS_MASK);
	get_from_reg(reg[7], &data->xo.ot_res, IDTXP_OT_RES_MASK);

	dev_info(&client->dev,
		 "idtxp_get_xo_settings: [dblr_dis] %d\n", 
		 data->xo.dblr_dis);
	dev_info(&client->dev,
		 "idtxp_get_xo_settings: [0x50-0x57] %02x %02x %02x %02x \
		 %02x %02x %02x %02x\n",
		 reg[0], reg[1], reg[2], reg[3],
		 reg[4], reg[5], reg[6], reg[7]);

	return 0;
}

/**
 * idtxp_get_divs_and_icp() - Read in dividers value from registers.
 * @data: 	The clock device structure that contains all the requested
 * 		data for outputting frequency.
 * 
 * Return: 0 on success, negative errno otherwise.
 */
static int idtxp_get_divs_and_icp(struct clk_idtxp *data)
{
	int err;
	u8 reg[NUM_FREQ_REGISTERS];
	struct i2c_client *client = data->i2c_client;

	err = regmap_bulk_read(data->regmap, IDTXP_REG_DIVO_7_0,
			reg, ARRAY_SIZE(reg));
	if (err)
		return err;

	get_from_reg(reg[0], (u8*)&data->divo, 0xFF);
	get_from_reg(reg[1], &data->divo_8, IDTXP_DIVO_8_MASK);
	get_from_reg(reg[1], (u8*)&data->divnint, IDTXP_DIVN_INT_6_0_MASK);
	get_from_reg(reg[2],
		     (u8*)&data->icp_offst_en,
		     IDTXP_ICP_OFFSET_EN_MASK);
	get_from_reg(reg[2], &data->divnint_8_7, IDTXP_DIVN_INT_8_7_MASK);
	get_from_reg(reg[2], &data->icp_value, IDTXP_ICP_VALUE_MASK);
	get_from_reg(reg[2], (u8*)&data->pll_mode, IDTXP_PLL_MODE_MASK);
	get_from_reg(reg[3], (u8*)&data->divnfrac, 0xFF);
	get_from_reg(reg[4], &data->divnfrac_15_8, 0XFF);
	get_from_reg(reg[5], &data->divnfrac_23_16, 0xFF);

	data->divo |= (data->divo_8 << 8);
	data->divnint |= (data->divnint_8_7);
	data->divnfrac |= (data->divnfrac_15_8 << 8) |
			(data->divnfrac_23_16 << 16);

	dev_info(&client->dev, "idtxp_get_divs_and_icp: [0x10-0x15] \
			%02x %02x %02x %02x %02x %02x\n",
			reg[0], reg[1], reg[2], reg[3], reg[4], reg[5]);
	return 0;
}

/**
 * idtxp_get_defaults() - Read in default values from registers.
 * @data: 	The clock device structure that contains all the requested
 * 		data for outputting frequency.
 * 
 * Return: 0 on success, negative errno otherwise.
 */
static int idtxp_get_defaults(struct clk_idtxp *data)
{
	int err;
	
	err = idtxp_get_divs_and_icp(data);
	if (err)
		return err;

	err = idtxp_get_xo_settings(data);
	if (err)
		return err;

	return 0;
}

/**
 * idtxp_calc_divs() - Calculates the values of dividers.
 * @data: 	The clock device structure that contains all the requested
 * 		data for outputting frequency.
 * 
 * Return: 0 on success, negative errno otherwise.
 */
static int idtxp_calc_divs(struct clk_idtxp *data)
{
	//int err;
	bool is_int = false;
	u32 pfd;
	u64 rem;
	struct i2c_client *client = data->i2c_client;

	/*
	 * Output Divider = INT(1 + 6860 / Fout)
	 * Fvco = Fout * Output Divider
	 * Feedback Divider = Fvco / (Fcrystal * Doubler)
	 */
	data->divnfrac = 0;
	pfd = data->fxtal * (data->xo.dblr_dis ? 1 : 2);
	dev_info(&client->dev, "idtxp_calc_divs: [pfd] %u\n", pfd);
	data->divo = 1 + div_u64(FVCO_MIN, data->req_freq);
	while (data->divo < DIVO_MAX) {
		data->fvco = (u64)data->req_freq * (u64)data->divo;

		dev_info(&client->dev,
			 "idtxp_calc_divs: [fvco] %llu\n", 
			 data->fvco);

		if (data->fvco > FVCO_MAX)
			break;
		data->divnint = div64_u64_rem(data->fvco, pfd, &rem);

		dev_info(&client->dev,
			 "idtxp_calc_divs: [divnint] %u\n", 
			 data->divnint);

		if (data->divnint < DIVN_MIN)
			continue;
		if (data->divnint > DIVN_MAX)
			break;
		if (rem == 0) {
			is_int = true;
			break;
		}
		data->divo++;
	}

	/*
	 * FBInt = INT(Feedback Divider)
	 * FBFrac = Feedback Divider - INT(Feedback Divider)
	 * FBFrac bits = INT(0.5 + FBFrac * 2 ^ 24)
	 * 
	 * FBFrac <  0.5 -> FBInt
	 * FBFrac >= 0.5 -> FBInt + 1
	 */
	if (!is_int) {
		dev_info(&client->dev, "IS_FRAC\n");
		data->divo = 1 + div_u64(FVCO_MIN, data->req_freq);
		data->fvco = (u64)data->req_freq * (u64)data->divo;

		dev_info(&client->dev,
			 "idtxp_calc_divs: [fvco] %llu\n", 
			 data->fvco);

		data->divnint = div64_u64_rem(data->fvco, pfd, &rem);

		dev_info(&client->dev,
			 "idtxp_calc_divs: [divnint] %u\n", 
			 data->divnint);

		data->divnfrac = div64_u64_rem(rem << 24, pfd, &rem);
		if (div64_u64(rem * 10, pfd) >= 5) {
			data->divnfrac += 1;
		}
		if ((data->divnfrac * 10) >> 24 >= 5) {
			data->divnint += 1;
		}
	}

	update_divis_regs(data);

	dev_info(&client->dev,
		 "idtxp_calc_divs: [req_freq] %u\n",
		 data->req_freq);
	dev_info(&client->dev, 
		 "idtxp_calc_divs: [divo] %u\n", 
		 data->divo);
	dev_info(&client->dev,
		 "idtxp_calc_divs: [fvco] %llu\n", 
		 data->fvco);
	dev_info(&client->dev,
		 "idtxp_calc_divs: [divnint] %u\n", 
		 data->divnint);
	dev_info(&client->dev,
		 "idtxp_calc_divs: [divnfrac] %u\n", 
		 data->divnfrac);

	return 0;
}

/**
 * idtxp_calc_charge_pump() - Calculate the charge pump values.
 * 			
 * @data: 	The clock device structure that contains all the requested
 * 		data for outputting frequency.
 * 
 * Return: 0 on success, negative errno otherwise.
 */
static int idtxp_calc_charge_pump(struct clk_idtxp *data)
{
	struct i2c_client *client = data->i2c_client;

	if (data->fvco < 7000000000LL) {
		data->icp_value = 5;
	} else if (7000000000LL <= data->fvco &&
		data->fvco < 7400000000LL) {
		data->icp_value = 4;
	} else if (7400000000LL <= data->fvco &&
		data->fvco < 7800000000LL) {
		data->icp_value = 3;
	} else {
		data->icp_value = 2;
	}

	dev_info(&client->dev,
		 "idtxp_calc_charge_pump: [icp_value] %u\n",
		 data->icp_value);

	return 0;
}

/**
 * idtxp_calc_xo_settings() - Calculate the XO settings.
 * @data: 	The clock device structure that contains all the requested
 * 		data for outputting frequency.
 * 
 * Return: 0 on success, negative errno otherwise.
 */
static int idtxp_calc_xo_settings(struct clk_idtxp *data)
{
	int err = 0;
	struct i2c_client *client = data->i2c_client;

	if (40000000 <= data->fxtal && data->fxtal <= 80000000) {
		data->xo.dblr_dis = 0x0;
		data->xo.gm = 0x2;
		data->xo.cap_x1 = 0x3C;
		data->xo.ampslice = 0x1;
		data->xo.cap_x2 = 0x2;
		data->xo.ot_dis = 0x1;
		data->xo.ot_res = 0x0;
	} else if (100000000 <= data->fxtal && data->fxtal < 140000000) {
		data->xo.dblr_dis = 0x1;
		data->xo.gm = 0x2;
		data->xo.cap_x1 = 0x15;
		data->xo.ampslice = 0x0C;
		data->xo.cap_x2 = 0x5;
		data->xo.ot_dis = 0x0;
		data->xo.ot_res = 0x5;
	} else if (140000000 <= data->fxtal && data->fxtal <= 166000000) {
		data->xo.dblr_dis = 0x1;
		data->xo.gm = 0x3;
		data->xo.cap_x1 = 0x15;
		data->xo.ampslice = 0x0C;
		data->xo.cap_x2 = 0x5;
		data->xo.ot_dis = 0x0;
		data->xo.ot_res = 0x3;
	} else {
		dev_err(&client->dev, "Error: wrong XO frequency");
		err = -EINVAL;
	}

	return err;
}

/**
 * idtxp_setup() - Write the ram registers into the device settings
 * @data: 	The clock device structure that contains all the requested
 * 		data for outputting frequency.
 * 
 * Return: 0 on success, negative errno otherwise.
 */
static int idtxp_setup(struct clk_idtxp *data)
{
	int err;

	err = regmap_write(data->regmap, IDTXP_REG_CONTROL, 0x00);
	if (err)
		return err;
	err = regmap_write(data->regmap, IDTXP_REG_CONTROL, 0x20);
	if (err)
		return err;
	err = regmap_write(data->regmap, IDTXP_REG_CONTROL, 0x00);
	if (err)
		return err;
	err = regmap_write(data->regmap, IDTXP_REG_CONTROL, 0x01);
	if (err)
		return err;
	err = regmap_write(data->regmap, IDTXP_REG_CONTROL, 0x00);
	if (err)
		return err;

	return 0;
}

/**
 * idtxp_write_divs_settings() - Write dividers value into registers
 * @data: 	The clock device structure that contains all the requested
 * 		data for outputting frequency.
 * 
 * Return: 0 on success, negative errno otherwise.
 */
static int idtxp_write_divs_settings(struct clk_idtxp *data)
{
	int err, i;
	u8 reg[NUM_FREQ_REGISTERS];
	struct reg_sequence reg_seq[NUM_FREQ_REGISTERS];
	struct i2c_client *client = data->i2c_client;

	err = regmap_bulk_read(data->regmap, IDTXP_REG_DIVO_7_0,
		reg, ARRAY_SIZE(reg));
	
	update_divis_regs(data);
	
	set_to_reg(&reg[0], (u8)data->divo, 0xFF);
	set_to_reg(&reg[1], data->divo_8, IDTXP_DIVO_8_MASK);
	set_to_reg(&reg[1], (u8)data->divnint, IDTXP_DIVN_INT_6_0_MASK);
	set_to_reg(&reg[2], (u8)data->icp_offst_en, IDTXP_ICP_OFFSET_EN_MASK);
	set_to_reg(&reg[2], data->divnint_8_7, IDTXP_DIVN_INT_8_7_MASK);
	set_to_reg(&reg[2], data->icp_value, IDTXP_ICP_VALUE_MASK);
	set_to_reg(&reg[2], (u8)data->pll_mode, IDTXP_PLL_MODE_MASK);
	set_to_reg(&reg[3], (u8)data->divnfrac, 0xFF);
	set_to_reg(&reg[4], data->divnfrac_15_8, 0xFF);
	set_to_reg(&reg[5], data->divnfrac_23_16, 0xFF);

	dev_info(&client->dev, "idtxp_write_divs_settings: [0x10-0x15] \
			%02x %02x %02x %02x %02x %02x\n",
			reg[0], reg[1], reg[2], reg[3], reg[4], reg[5]);

	for (i = 0; i < NUM_FREQ_REGISTERS; i++) {
		reg_seq[i].reg = IDTXP_REG_DIVO_7_0 + i;
		reg_seq[i].def = reg[i];
	}
	err = regmap_multi_reg_write(data->regmap, reg_seq, NUM_FREQ_REGISTERS);
	if (err)
		return err;

	dev_info(&client->dev, "idtxp_write_divs_settings: okay\n");

	return 0;
}

/**
 * idtxp_write_xo_settings() - Write XO settings into registers
 * @data: 	The clock device structure that contains all the requested
 * 		data for outputting frequency.
 * 
 * Return: 0 on success, negative errno otherwise.
 */
static int idtxp_write_xo_settings(struct clk_idtxp *data)
{
	int err, i;
	u8 reg[NUM_MISCELLANEOUS_REGISTERS];
	struct reg_sequence reg_seq[NUM_MISCELLANEOUS_REGISTERS];
	struct i2c_client *client = data->i2c_client;

	err = regmap_bulk_read(data->regmap, IDTXP_REG_HSPI2C_CMOS,
		reg, ARRAY_SIZE(reg));
	
	set_to_reg(&reg[0], (u8)data->xo.hsp_i2c_en, IDTXP_HSPI2C_EN);
	set_to_reg(&reg[0], (u8)data->xo.cmos_en, IDTXP_CMOS_EN);
	set_to_reg(&reg[1], (u8)data->xo.dblr_dis, IDTXP_DBLR_DIS_MASK);
	set_to_reg(&reg[1], data->xo.vdd_def, IDTXP_VDD_DEF_MASK);
	set_to_reg(&reg[1], data->xo.vcxo_bw, IDTXP_VCXO_BW_MASK);
	set_to_reg(&reg[2], (u8)data->xo.vcxo_gslope, IDTXP_GSLOPE_MASK);
	set_to_reg(&reg[2], data->xo.vcxo_gexp, IDTXP_GEXP_MASK);
	set_to_reg(&reg[2], data->xo.vcxo_gscale, IDTXP_GSCALE_MASK);
	set_to_reg(&reg[3], (u8)data->xo.oe_pol_en, IDTXP_OE_POL_EN);
	set_to_reg(&reg[3], data->xo.drv_type, IDTXP_DRV_TYPE);
	set_to_reg(&reg[5], data->xo.gm, IDTXP_OT_GM_MASK);
	set_to_reg(&reg[5], data->xo.cap_x1, IDTXP_XO_CAP_MASK);
	set_to_reg(&reg[6], data->xo.ampslice, IDTXP_XO_AMPSLICE_MASK);
	set_to_reg(&reg[6], (u8)data->xo.bypass, IDTXP_BYPASS_MASK);
	set_to_reg(&reg[6], data->xo.cap_x2, IDTXP_CAP_X2_MASK);
	set_to_reg(&reg[7], (u8)data->xo.ot_dis, IDTXP_OT_DIS_MASK);
	set_to_reg(&reg[7], data->xo.ot_res, IDTXP_OT_RES_MASK);

	dev_info(&client->dev, "idtxp_write_xo_settings: [0x50-0x57] \
			%02x %02x %02x %02x %02x %02x %02x %02x\n",
			reg[0], reg[1], reg[2], reg[3],
			reg[4], reg[5], reg[6], reg[7]);

	for (i = 0; i < NUM_MISCELLANEOUS_REGISTERS; i++) {
		reg_seq[i].reg = IDTXP_REG_HSPI2C_CMOS + i;
		reg_seq[i].def = reg[i];
	}
	err = regmap_multi_reg_write(data->regmap,
				reg_seq,
				NUM_MISCELLANEOUS_REGISTERS);
	if (err)
		return err;

	dev_info(&client->dev, "idtxp_write_xo_settings: okay\n");

	return 0;
}

/**
 * idtxp_write_all_settings() - Write settings array into registers
 * @data: 	The clock device structure that contains all the requested
 * 		data for outputting frequency.
 * 
 * Return: 0 on success, negative errno otherwise.
 */
static int idtxp_write_all_settings(struct clk_idtxp *data)
{
	int err, i;
	struct reg_sequence reg_seq[NUM_CONFIG_REGISTERS];

	for (i = 0; i < NUM_CONFIG_REGISTERS; i++) {
		reg_seq[i].reg = i;
		reg_seq[i].def = data->settings[i];
	}
	err = regmap_multi_reg_write(data->regmap,
				     reg_seq,
				     NUM_CONFIG_REGISTERS);

	return err;
}

/**
 * idtxp_recalc_rate() - Return the frequency being provided by the clock.
 * @hw:			Handle between common and hardware-specific interfaces	
 * @parent_rate:	Clock frequency of parent clock	
 * 
 * Return: the frequency of the specified clock.
 */
static unsigned long idtxp_recalc_rate(struct clk_hw *hw, 
			unsigned long parent_rate)
{
	struct clk_idtxp *output = to_clk_idtxp(hw);

	return output->req_freq;
}

/**
 * idtxp_round_rate() - Get valid rate that is closest to the requested rate.
 * @hw:			Handle between common and hardware-specific interfaces
 * @rate:		The rate (in Hz) for the specified clock.
 * @parent_rate:	Clock frequency of parent clock
 * 
 * Return: frequency closest to @rate the hardware can generate.
 */
static long idtxp_round_rate(struct clk_hw *hw, unsigned long rate,
                        unsigned long *parent_rate)
{
	return rate;
}

/**
 * idtxp_large_frequency_change() - 
 * @data:	The clock device structure that contains all the requested
 * 		data for outputting frequency.
 * @frequency:	The rate (in Hz) for the specified clock.
 * 
 * Return: 0 on success, negative errno otherwise.
 */
static int idtxp_large_frequency_change(struct clk_idtxp *data,
					unsigned long frequency)
{
	int err;
	struct i2c_client *client = data->i2c_client;

	dev_info(&client->dev, "idtxp_large_frequency_change\n");

	err = idtxp_calc_divs(data);
	if (err)
		return err;

	err = idtxp_calc_charge_pump(data);
	if (err)
		return err;

	err = idtxp_write_divs_settings(data);
	if (err)
		return err;
	
	err = idtxp_setup(data);
	if (err)
		return err;

	/* update the frequency with PLL lock */
	regmap_write(data->regmap, IDTXP_REG_FREQ_CHG, 0x01);
	regmap_write(data->regmap, IDTXP_REG_FREQ_CHG, 0x00);

	data->act_freq= data->req_freq;

	return 0;
}

/**
 * idtxp_small_frequency_change() - 
 * @data:	The clock device structure that contains all the requested
 * 		data for outputting frequency.
 * @frequency:	The rate (in Hz) for the specified clock.
 * 
 * Return: 0 on success, negative errno otherwise.
 */
static int idtxp_small_frequency_change(struct clk_idtxp *data,
					unsigned long frequency)
{
	int err;
	struct i2c_client *client = data->i2c_client;

	dev_info(&client->dev, "idtxp_small_frequency_change\n");

	err = idtxp_calc_divs(data);
	if (err)
		return err;

	err = idtxp_calc_charge_pump(data);
	if (err)
		return err;

	err = idtxp_write_divs_settings(data);
	if (err)
		return err;

	err = idtxp_setup(data);
	if (err)
		return err;
	
	/* update the frequency without PLL lock */
	regmap_write(data->regmap, IDTXP_REG_FREQ_CHG, 0x02);
	regmap_write(data->regmap, IDTXP_REG_FREQ_CHG, 0x00);

	data->act_freq= data->req_freq;

	return 0;
}

/**
 * idtxp_set_rate() - Return the frequency being provided by the clock.
 * @hw:			Handle between common and hardware-specific interfaces
 * @rate:		The rate (in Hz) for the specified clock.
 * @parent_rate:	Clock frequency of parent clock
 * 
 * Return: 0 on success, negative errno otherwise.
 */
static int idtxp_set_rate(struct clk_hw *hw, unsigned long rate,
                        unsigned long parent_rate)
{
	struct clk_idtxp *data = to_clk_idtxp(hw);
	struct i2c_client *client = data->i2c_client;

	dev_info(&client->dev, "idtxp_set_rate: in\n");

	if (rate < data->min_freq || rate > data->max_freq) {
		dev_err(&client->dev,
			"request frequency %lu Hz is out of range\n", rate);
		return -EINVAL;
	}

	data->req_freq = rate;

	if (div64_u64(abs(rate - data->act_freq) * 10000LL,
				data->act_freq) < 5) 
		return idtxp_small_frequency_change(data, rate);
	else
		return idtxp_large_frequency_change(data, rate);
}

static const struct clk_ops idtxp_clk_ops = {
	.recalc_rate = idtxp_recalc_rate,
	.round_rate = idtxp_round_rate,
	.set_rate = idtxp_set_rate,
};

static bool idtxp_regmap_is_volatile(struct device *dev, unsigned int reg)
{
	return false;
}

static bool idtxp_regmap_is_writeable(struct device *dev, unsigned int reg)
{
	return true;
}

static const struct regmap_config idtxp_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.max_register = 256,
	.writeable_reg = idtxp_regmap_is_writeable,
	.volatile_reg = idtxp_regmap_is_volatile,
};

/**
 * idtxp_read_all_settings() - Read in registers and print to a buffer
 * @data:	The clock device structure that contains all the requested
 * 		data for outputting frequency.
 * @buf:	The printed out buffer.
 * @count:	The size of the buffer.
 * 
 * Return: 0 on success, negative errno otherwise.
 */
static int idtxp_read_all_settings(struct clk_idtxp *data,
				   char *buf,
				   int count)
{
	int i;
	int err = 0;
	u8 settings[NUM_CONFIG_REGISTERS];
	char tmp[10];

	err = regmap_bulk_read(data->regmap,
			       0,
			       settings,
			       NUM_CONFIG_REGISTERS);
	sprintf(buf, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	if (!err) {
		for (i = 0; i < NUM_CONFIG_REGISTERS; i++) {
			if (i % 16 == 0) {
				sprintf(tmp, "%02x  ", i);
				strcat(buf, tmp);
			}
			sprintf(tmp, "%02x ", settings[i]);
			strcat(buf, tmp);
			if ((i + 1) % 16 == 0)
				strcat(buf, "\n");
		}
	}
	strcat(buf, "\n");
	return err;
}

static int debugfs_i2c_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

/**
 * debugfs_i2c_read() - Read in all the device registers.
 * @filp:		Open file to invoke ioctl method on.
 * @user_buffer:	Buffer to read data from.
 * @count:		Size of the buffer.
 * @ppos:		Offset within the file.
 * 
 * Return: 0 on success, negative errno otherwise.
 */
static ssize_t debugfs_i2c_read(struct file *filp, char __user *user_buffer,
				size_t count, loff_t *ppos)
{
	int err;
	struct clk_idtxp *data = (struct clk_idtxp*)filp->private_data;
	char *buf = kzalloc(5000, GFP_KERNEL);

	err = idtxp_read_all_settings(data, buf, 5000);
	if (err) {
		dev_err(&data->i2c_client->dev,
			"error calling idtxp_read_all_settings (%i)\n", err);
		return 0;
	}

	err = simple_read_from_buffer(
		user_buffer, count, ppos, buf, strlen(buf));
	kfree(buf);
	return err;
}

/**
 * debugfs_i2c_write() - Write an value into register.
 * @filp:		Open file to invoke ioctl method on.
 * @user_buffer:	Buffer to read data from.
 * @count:		Size of the buffer.
 * @ppos:		Offset within the file.
 * 
 * Return: 0 on success, negative errno otherwise.
 */
static ssize_t debugfs_i2c_write(struct file *filp, 
		const char __user *user_buffer,	size_t count, loff_t *ppos)
{
	int err, written, i, num;
	struct clk_idtxp *data = (struct clk_idtxp*)filp->private_data;
	char *buf = kzalloc(10, GFP_KERNEL);
	char *next_ptr;
	u8 settings[2];

	written = simple_write_to_buffer(buf, 5000,
		ppos, user_buffer, count);

	dev_info(&data->i2c_client->dev, "echo: %s", buf);
	
	if (written > 10) {
		dev_info(&data->i2c_client->dev, "written too much");
		return written;
	}

	for (i = 0, num = 0, next_ptr = buf; i < count && num < 2; i++) {
		if (buf[i] == ' ' || buf[i] == '\n') {
			buf[i] = '\0';
			err = kstrtou8(next_ptr, 16, settings + num);
			if (err) {
				dev_err(&data->i2c_client->dev, 
					"parsing error");
				return written;
			}
			next_ptr = buf + i + 1;
			num++;
		}
	}

	dev_info(&data->i2c_client->dev, 
		 "addr: %u val: %u", 
		 settings[0], 
		 settings[1]);

	err = regmap_write(data->regmap, settings[0], settings[1]);
	if (err) {
		dev_err(&data->i2c_client->dev, "error writing to register");
		return err;
	}
	dev_info(&data->i2c_client->dev, "writing successful");

	return written;
}

struct file_operations debugfs_i2c_ops = {
	.owner = THIS_MODULE,
	.open = debugfs_i2c_open,
	.read = debugfs_i2c_read,
	.write = debugfs_i2c_write,
};

/**
 * idtxp_probe() - Main entry point for ccf driver.
 * @client:	Pointer to i2c_client structure
 * @id:		Pointer to i2c_device_id structure
 * 
 * Return: 0 for success.
 */
static int idtxp_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct clk_idtxp *data;
	struct clk_init_data init;
	int err;
	enum clk_idtxp_variant variant = id->driver_data;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	init.ops = &idtxp_clk_ops;
	init.flags = 0;
	init.num_parents = 0;
	data->hw.init = &init;
	data->i2c_client = client;

	data->max_freq = IDTXP_MAX_FREQ;
	data->min_freq = IDTXP_MIN_FREQ;
	data->act_freq = IDTXP_MIN_FREQ;

	if (of_property_read_string(client->dev.of_node, "clock-output-names",
				&init.name))
		init.name = client->dev.of_node->name;

	err = of_property_read_u32(client->dev.of_node, "factory-fout",
			&data->fxtal);
	if (err) {
		dev_err(&client->dev, "'factory-fout' property missing\n");
		return err;
	}
	dev_info(&client->dev, "registered, XO frequency %u Hz\n",
			data->fxtal);

	err = of_property_read_u8_array(
		client->dev.of_node, "settings", data->settings,
		ARRAY_SIZE(data->settings));
	if (!err) {
		dev_info(&client->dev, "settings property specified in DT");
		data->has_settings = true;
	} else {
		if (err == -EOVERFLOW) {
			dev_alert(&client->dev,
				  "EOVERFLOW error trying to read the \
				  settings. ARRAY_SIZE: %zu",
				  ARRAY_SIZE(data->settings));
			return err;
		}
		data->has_settings = false;
		dev_info(&client->dev,
			"settings property not specified in DT \
			(or there was an error that can be ignored: %i). \
			The settings property is optional.",
			err);
	}

	data->regmap = devm_regmap_init_i2c(client, &idtxp_regmap_config);
	if (IS_ERR(data->regmap)) {
		dev_err(&client->dev, "failed to allocate register map\n");
		return PTR_ERR(data->regmap);
	}

	i2c_set_clientdata(client, data);
	err = idtxp_get_defaults(data);
	if (err)
		return err;
	
	/* Write in all the settings, if there is a setting array */
	if (data->has_settings) {
		err = idtxp_write_all_settings(data);
		if (err) {
			dev_err(&client->dev,
				"error writing all settings to chip (%i)\n",
				err);
			return err;
		}
		dev_info(&client->dev, 
			 "successfully wrote full settings array");
	}

	if (variant == idtxp_xo) {
		regmap_write(data->regmap, IDTXP_REG_HSPI2C_CMOS, 0x15);
		regmap_write(data->regmap, IDTXP_REG_VCXO, 0x2A);
	}

	/* Read all the values from hw */
	err = idtxp_get_defaults(data);
	if (err) {
		dev_err(&client->dev,
			"failed calling idtxp_get_defaults (%i)\n", err);
		return err;
	}

	err = devm_clk_hw_register(&client->dev, &data->hw);
	if (err) {
		dev_err(&client->dev, "clock registration failed\n");
		return err;
	}
	err = of_clk_add_hw_provider(client->dev.of_node, 
				     of_clk_hw_simple_get,
				     &data->hw);
	if (err) {
		dev_err(&client->dev, "unable to add clk provider\n");
		return err;
	}
 
	/* Read the power supply voltage from device tree */
	if (!of_property_read_u8(client->dev.of_node, "power-supply-voltage",
				&data->xo.vdd_def)) {
		if (data->xo.vdd_def >= 0 && data->xo.vdd_def < 3) {
			dev_info(&client->dev,
				 "vdd_def: %u",
				 data->xo.vdd_def);
			dev_info(&client->dev, 
				 "registered, power supply voltage is %s\n",
				 (data->xo.vdd_def == 0 ? "1.8V" :
				 (data->xo.vdd_def == 1 ? "2.5V" : "3.3V")));
		} else {
			dev_info(&client->dev, 
				 "The value for power supply voltage \
				 must 0, 1 or 2\n");
		}
	}

	idtxp_calc_xo_settings(data);
	idtxp_write_xo_settings(data);

	/* Read the requested initial output frequency from device tree */
	if (!of_property_read_u32(client->dev.of_node, "clock-frequency",
				&data->req_freq)) {
		err = clk_set_rate(data->hw.clk, data->req_freq);
		if (err) {
			of_clk_del_provider(client->dev.of_node);
			return err;
		}
		dev_info(&client->dev,
			 "registered, current frequency %u Hz\n",
			 data->act_freq);
	}


	/* Create the debugfs for driver test */
	data->debugfs_root_dir = debugfs_create_dir(DEBUGFS_ROOT_DIR_NAME,
			 			    NULL);
	data->debugfs_i2c_file = debugfs_create_file(DEBUGFS_I2C_FILE_NAME,
	 					     0644,
						     data->debugfs_root_dir,
						     data, &debugfs_i2c_ops);

	return 0;
}

static int idtxp_remove(struct i2c_client *client)
{
	struct clk_idtxp *data = 
		(struct clk_idtxp*)i2c_get_clientdata(client);
		
	of_clk_del_provider(client->dev.of_node);
	debugfs_remove_recursive(data->debugfs_root_dir);
	return 0;
}

static const struct i2c_device_id idtxp_id[] = {
	{ "idtxp_pro_xo", idtxp_xo },
	{ }
};
MODULE_DEVICE_TABLE(i2c, idtxp_id);

static const struct of_device_id clk_idtxp_of_match[] = {
	{ .compatible = "idt,idtxp_pro_xo" },
	{ },
};
MODULE_DEVICE_TABLE(of, clk_idtxp_of_match);

static struct i2c_driver idtxp_driver = {
	.driver = {
		.name = "idtxp",
		.of_match_table = clk_idtxp_of_match,
	},
	.probe		= idtxp_probe,
	.remove		= idtxp_remove,
	.id_table	= idtxp_id,
};
module_i2c_driver(idtxp_driver);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("IDT XP family driver");
MODULE_LICENSE("GPL");