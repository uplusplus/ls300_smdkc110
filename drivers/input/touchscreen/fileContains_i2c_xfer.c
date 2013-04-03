
extern int get_ite_i2c_nostop(void);

static int tegra_i2c_xfer(struct i2c_adapter *adap,
	struct i2c_msg msgs[], int num)
{
	...

	if ( (msgs[0].addr == 0x46) && (get_ite_i2c_nostop() == 1) )
	{
		msgs[0].flags = I2C_M_NOSTART ;
	}

	...
}
