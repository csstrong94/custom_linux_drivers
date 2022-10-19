#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/mutex.h>

#define DRIVER_NAME 						"pcf8574_lcd"

#define PCF8574_LCD_ADDR					0x27
#define PCF8574_LCD_4BIT_OPERATING_MODE		0x20
#define PCF8574_LCD_DISPLAY_ON				0xE0
#define PCF8574_LCD_ENTRY_MODE_SET			0x60

#define PCF8574_LCD_BACKLIGHT_BIT			(1 << 3)

struct lcd_controller_data {
	
	struct i2c_client *client;
	struct miscdevice pcf8574_misc_device;
	struct mutex bus_write_lock;
	char name[64];

	/* LCD controller parameters */
	unsigned char register_select;
	unsigned char read_write_mode;
	unsigned char enable;
	unsigned char data_hi[4];
	unsigned char backlight;

};

/* write a byte to the io expander, which will write the data out
 * as an i/o octal consisting of RS,RW,E, BLA, D4-D7 */

static int __maybe_unused write_ioexpander(struct lcd_controller_data* lcd, char data)
{
	pr_info("writing i2c to pcf8574\n");

	return 1;
}

static ssize_t pcf8574_lcd_write(struct file *file, const char __user *userbuf,
	   						size_t count, loff_t *ppos) 

{
	int ret;
	u8 data_ctrl_byte;
	struct lcd_controller_data *lcd;

	lcd = container_of(file->private_data, 
					   struct lcd_controller_data, 
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
	
	mutex_lock(&lcd->bus_write_lock);
	ret = i2c_smbus_write_byte(lcd->client, data_ctrl_byte);
	mutex_unlock(&lcd->bus_write_lock);

	if (ret < 0) {
		dev_err(&lcd->client->dev, "i2c bus write failed\n");
	}

	dev_info(&lcd->client->dev, "byte written to i2c bus: %d\n", data_ctrl_byte);

	return count;

}

static const struct file_operations pcf8574_lcd_fops = {
	.owner = THIS_MODULE,
	.write = pcf8574_lcd_write,

};

static struct miscdevice pcf8574_miscdev = {
	.name = DRIVER_NAME,
	.minor = MISC_DYNAMIC_MINOR,
	.fops = &pcf8574_lcd_fops,
};

static int pcf8574_lcd_remove(struct i2c_client *client) 
{
	
	struct lcd_controller_data *lcd;

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
	struct lcd_controller_data *lp;
	
	dev_info(&client->dev, "probing pcf8574 i2c lcd controller\n");
 	
	lp = devm_kzalloc(&client->dev, sizeof(struct lcd_controller_data), GFP_KERNEL); 
	if (!lp) {
		pr_err("devm_kzalloc() failed on minor=%d\n", MISC_DYNAMIC_MINOR);
		return -ENOMEM;
	}
	
	/* set our client data to driver_data in the i2c client device struct */
	i2c_set_clientdata(client, lp);
	
	/* store our i2c client pointer in our private struct */
	lp->client = client;	
	
	sprintf(lp->name, DRIVER_NAME);
	mutex_init(&lp->bus_write_lock);

	lp->pcf8574_misc_device.name = DRIVER_NAME;
	lp->pcf8574_misc_device.minor = MISC_DYNAMIC_MINOR;
	lp->pcf8574_misc_device.fops = &pcf8574_lcd_fops;
	
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
