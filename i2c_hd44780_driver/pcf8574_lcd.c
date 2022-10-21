#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/types.h>

#include "pcf8574_lcd.h"



static inline u8 pcf8574_lcd_regwrite(struct lcd_data *lcd,
										u8 value)
{
	u8 rval;
	dev_dbg(&lcd->client->dev, 
			"writing 0x%x to instruction register", value);
	
	rval =  i2c_smbus_write_byte(lcd->client, value);
	if (rval < 0) {
		dev_err(&lcd->client->dev, "error writing on i2c-2 bus: %d", rval);
	}

	return rval;	
	

}	

static inline u8 pcf8574_lcd_regread(struct lcd_data *lcd) 
{
	u8 rval;

	rval = i2c_smbus_read_byte(lcd->client);
	if (rval < 0) {
		dev_err(&lcd->client->dev, "error reading on i2c-2 bus: %d", rval);
		
		return rval;
	}
	dev_info(&lcd->client->dev, "read byte: 0x%x", rval);
	return rval;
}

static int pcf8574_lcd_display_on(struct lcd_data *lcd)
{
	u8 rval;
	u8 regread;
	
	regread = pcf8574_lcd_regread(lcd);
	regread &= 
		~(PCF8574_LCD_BLINK_FLAG | PCF8574_LCD_CURSOR_FLAG | 
		PCF8574_LCD_DISPLAY_FLAG | PCF8574_LCD_DISPLAY_CTRL_BIT); 
	
	rval = pcf8574_lcd_regwrite(lcd, regread);	

	return rval;
}


static ssize_t pcf8574_lcd_write(struct file *file, const char __user *userbuf,
	   						size_t count, loff_t *ppos) 

{
	int ret;
	u8 data_ctrl_byte;
	struct lcd_data *lcd;

	lcd = container_of(file->private_data, 
					   struct lcd_data, 
			           pcf8574_misc_device);
	pr_info("checking contents of lcd data struct -- owner: %s", lcd->name);
	dev_info(&lcd->client->dev, "pcf8574_lcd_write() entered on %s\n", lcd->name);
	dev_info(&lcd->client->dev, "write callback fn recv'd %d chars\n", count);
	
	ret = kstrtou8_from_user(userbuf, count, 0, &data_ctrl_byte); 

	if(ret) {
		dev_err(&lcd->client->dev, 
				"%s:%u: userspace copy conversion failed on minor=%d, errno %d",
				__func__, __LINE__, MISC_DYNAMIC_MINOR, ret);
				
		return ret;
	}
	
	
	dev_info(&lcd->client->dev, "will write val %d to lcd controller", data_ctrl_byte);
	
	ret = i2c_smbus_write_byte(lcd->client, data_ctrl_byte);

	if (ret < 0) {
		dev_err(&lcd->client->dev, "i2c bus write failed\n");
	}

	dev_info(&lcd->client->dev, "byte written to i2c bus: %d\n", data_ctrl_byte);

	return count;

}

static long pcf8574_lcd_ioctl(struct file* filp, unsigned int cmd,
							unsigned long data)
{

	struct lcd_data *lcd;
	void __user *arg = NULL;
	int rval = -EINVAL;

	lcd = container_of(filp->private_data, 
						struct lcd_data,
						pcf8574_misc_device); 
	
	/* guard against wrong ioctl magics */
	if (_IOC_TYPE(cmd) != PCF8574_LCD_MAGIC) {
		return -ENOTTY;
	}
	
	/* check if a valid value was actually passed to ioctl from userspace */
	if (_IOC_DIR(cmd) != _IOC_NONE) {
		arg = (void __user * ) data;
		if (!arg) {
			return rval;
		}
	}

	switch (cmd) {
	
		case PCF8574_LCD_DISPLAY_ON:
			pcf8574_lcd_display_on(lcd);	

	}

	return 1;
}

static const struct file_operations pcf8574_lcd_fops = {
	.owner = THIS_MODULE,
	.write = pcf8574_lcd_write,
	.unlocked_ioctl = pcf8574_lcd_ioctl
};

static int pcf8574_lcd_remove(struct i2c_client *client) 
{
	
	struct lcd_data *lcd;

	lcd = i2c_get_clientdata(client);
	dev_info(&client->dev, "removing pcf8574 lcd device");
	

	misc_deregister(&lcd->pcf8574_misc_device);
	dev_info(&client->dev, "pcf8574 lcd remove exited\n");

	return 0;
}

static int pcf8574_lcd_probe(struct i2c_client *client,
	   					const struct i2c_device_id *id)	
{	

	int err;
	struct lcd_data *lp;
	
	dev_info(&client->dev, "probing pcf8574 i2c lcd controller\n");
 	
	lp = devm_kzalloc(&client->dev, sizeof(struct lcd_data), GFP_KERNEL); 
	if (!lp) {
		pr_err("devm_kzalloc() failed on minor=%d\n", MISC_DYNAMIC_MINOR);
		return -ENOMEM;
	}
	
	/* set our client data to driver_data in the i2c client device struct */
	i2c_set_clientdata(client, lp);
	
	/* store our i2c client pointer in our private struct */
	lp->client = client;	
	
	sprintf(lp->name, DRIVER_NAME);

	lp->pcf8574_misc_device.name = DRIVER_NAME;
	lp->pcf8574_misc_device.minor = MISC_DYNAMIC_MINOR;
	lp->pcf8574_misc_device.fops = &pcf8574_lcd_fops;
	
	/* all flags are set because of quasi-directional i/o on pcf8574
	 * all pins are set HIGH on power on */

	lp->flags = PCF8574_LCD_ALL_FLAGS_SET; 
	lp->regmode = PCF8574_LCD_DATA;
	err = misc_register(&lp->pcf8574_misc_device);
	if (err) {
		dev_err(&client->dev, "%s:%u: failed misc_register on dynamic minor=%d\n",
			   	__func__, __LINE__, err);
		return err;

	}


	dev_info(&client->dev, "pcf8574 i2c probe exit");
	return 0;
}


static struct of_device_id pcf8574_dt_idtable[] = {
	{.compatible = "pd,pcf8574" },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, pcf8574_dt_idtable);

static struct i2c_device_id pcf8574_idtable[] = {
	{ .name = DRIVER_NAME, },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(i2c, pcf8574_idtable);

static struct i2c_driver pcf8574_lcd_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = pcf8574_dt_idtable,
	},
	.probe	= pcf8574_lcd_probe,
	.remove = pcf8574_lcd_remove,
	.id_table = pcf8574_idtable,
};

module_i2c_driver(pcf8574_lcd_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chris Strong");
MODULE_DESCRIPTION("HD44780 LCD driver connected to PCF8574 I/O expander");
