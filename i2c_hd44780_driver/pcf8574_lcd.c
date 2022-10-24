#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/string.h>

#include "pcf8574_lcd.h"


#define PCF8574_LCD_CLOCK_STROBE	1


enum pcf8574_onoff
{
	PCF8574_LCD_OFF = 0,
	PCF8574_LCD_ON = 4,
};

enum pcf8574_fontsize 
{
	PCF8574_LCD_FONT_SMALL = 0,
	PCF8574_LCD_FONT_LARGE,

};

enum pcf8574_scrollshift 
{
	PCF8574_LCD_LEFT_SHIFT = 0,
	PCF8574_LCD_RIGHT_SHIFT,

};

enum pcf8574_regselect
{
	PCF8574_LCD_INSTRUCTION = 0,
	PCF8574_LCD_DATA,

};

enum pcf8574_shiftcursor
{
	PCF8574_LCD_CURSOR_MOVE = 0,
	PCF8574_LCD_DISPLAY_SHIFT,

};

enum pcf8574_backlight
{
	PCF8574_LCD_BACKLIGHT_OFF = 0,
	PCF8574_LCD_BACKLIGHT_ON = 8,

};

enum pcf8574_rwmode
{
	PCF8574_LCD_WRITE_SELECT = 0,
	PCF8574_LCD_READ_SELECT,
	
};

enum pcf8574_blinkstate 
{
	PCF8574_LCD_BLINK_OFF = 0,
	PCF8574_LCD_BLINK_ON,

};

enum pcf8574_cursoronoff
{
	PCF8574_LCD_CURSOR_OFF = 0,
	PCF8574_LCD_CURSOR_ON = 2,

};


struct lcd_data {
	
	struct i2c_client *client;
	struct miscdevice pcf8574_misc_device;
	char name[32];
	__u8 ddram[80];
	
	/* LCD controller parameters */
	enum pcf8574_onoff display_state;
	enum pcf8574_cursoronoff cursor_onoff;
	enum pcf8574_blinkstate cursor_blink;
	enum pcf8574_fontsize font;
	enum pcf8574_scrollshift dir;
	enum pcf8574_shiftcursor sc_mode;
	enum pcf8574_regselect regmode;
	enum pcf8574_rwmode rwmode;
	enum pcf8574_backlight backlight;
};


static int pcf8574_lcd_clock_pulse(struct lcd_data *lcd)
{
	u8 regread;
	dev_dbg(&lcd->client->dev, 
			"pulsing clock");

	regread = i2c_smbus_read_byte(lcd->client) | lcd->backlight;

	regread &= ~(PCF8574_LCD_ENABLE_BIT);
	

	/* drive clock high */
	regread |= PCF8574_LCD_ENABLE_CMD;
	i2c_smbus_write_byte(lcd->client, regread);
	udelay(4);
	
	/* drive clock low and delay 37 us */
	regread &= ~(PCF8574_LCD_ENABLE_BIT);
	i2c_smbus_write_byte(lcd->client, regread);
	udelay(37);
	
	return 0;
}

static inline u8 pcf8574_lcd_regwrite(struct lcd_data *lcd,
										u8 value)
{
	u8 rval;
	dev_dbg(&lcd->client->dev, 
			"writing 0x%x to io pins", value);

	
	rval = i2c_smbus_write_byte(lcd->client, value);	

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

static inline u8 pcf8574_lcd_rwselect(struct lcd_data *lcd, int rw) 
{
	
	u8 regread, rval;
	
	regread = i2c_smbus_read_byte(lcd->client);

	/* if rw is high we are trying to read, so set bit 0 */
	if (rw) {
		rw = 1;
		rval = i2c_smbus_write_byte(lcd->client, (regread | (rw << 1)));
		return rval;	
	}

	else {
		rw = 0;

		/* we drive the read line low) */
		regread &= ~(PCF8574_LCD_READ_SELECT << 1);
		rval = i2c_smbus_write_byte(lcd->client, regread);
	}

	lcd->rwmode = rw;
	return rval;


}

static inline u8 pcf8574_lcd_regselect(struct lcd_data *lcd, int reg)
{

	u8 regread, rval;
	
	regread = i2c_smbus_read_byte(lcd->client);
	
	if (reg) {
		/* normalize to 1 in case userland gives us something bigger */
		reg = 1;
		rval = i2c_smbus_write_byte(lcd->client, (regread | reg));
	}
	else {

		/* normalize to zero otherwise */
		reg = 0;
		regread &= ~(PCF8574_LCD_DATA);
		rval = i2c_smbus_write_byte(lcd->client, regread);
	}

	lcd->regmode = reg;
	return rval;
}

static int pcf8574_lcd_exec_8bit_cmd(struct lcd_data *lcd,
								u8 value, u8 regselect) 

{
	u8 upper_nibble, lower_nibble;

	pcf8574_lcd_rwselect(lcd, PCF8574_LCD_WRITE_SELECT);
	pcf8574_lcd_regselect(lcd, regselect);

	
	dev_dbg(&lcd->client->dev, "value requested: 0x%02x", value);
	upper_nibble = (value & 0xF0);
	dev_dbg(&lcd->client->dev, "writing upper nibble: %02x", upper_nibble);
	lower_nibble = ((value & 0x0F) << 4);
	
	dev_dbg(&lcd->client->dev, "writing lower nibble: %02x", lower_nibble);


	pcf8574_lcd_regwrite(lcd, upper_nibble | 
			lcd->backlight | lcd->regmode | lcd->rwmode);
	pcf8574_lcd_clock_pulse(lcd);
	udelay(37);

	pcf8574_lcd_regwrite(lcd, lower_nibble | 
			lcd->backlight | lcd->regmode | lcd->rwmode);
	pcf8574_lcd_clock_pulse(lcd);

	return 0;
}

static int pcf8574_lcd_exec_4bit_cmd(struct lcd_data *lcd,
										u8 value, u8 regselect, u8 pulse)
{

	u8 upper_nibble;
	
	pcf8574_lcd_rwselect(lcd, PCF8574_LCD_WRITE_SELECT);
	pcf8574_lcd_regselect(lcd, PCF8574_LCD_INSTRUCTION);
	udelay(1000);
	
	upper_nibble = ((value & (0x0F)) << 4) | lcd->backlight 
			| lcd->regmode | lcd->rwmode;
	
	pcf8574_lcd_regwrite(lcd, upper_nibble);
	if (pulse) {
		pcf8574_lcd_clock_pulse(lcd);
	}
	return 0;
}



static int pcf8574_lcd_initialize(struct lcd_data *lcd) 
{
	
	u8 instr_byte;
	__maybe_unused u8 data_byte;

	instr_byte = 0x3;
	pcf8574_lcd_exec_4bit_cmd(lcd, instr_byte, 
			PCF8574_LCD_INSTRUCTION, PCF8574_LCD_CLOCK_STROBE);
	mdelay(5);

	instr_byte = 0x3;
	pcf8574_lcd_exec_4bit_cmd(lcd, instr_byte, 
			PCF8574_LCD_INSTRUCTION, PCF8574_LCD_CLOCK_STROBE);
	udelay(150);
	
	instr_byte = 0x3;
	pcf8574_lcd_exec_4bit_cmd(lcd, instr_byte, PCF8574_LCD_INSTRUCTION, 1);
	mdelay(5);

	instr_byte = 0x2;
	pcf8574_lcd_exec_4bit_cmd(lcd, instr_byte, PCF8574_LCD_INSTRUCTION, 1);
	mdelay(5);

	/* set 4-bit mode, 2 line, small font mode */
	instr_byte = 0x28;
	pcf8574_lcd_exec_8bit_cmd(lcd, instr_byte, PCF8574_LCD_INSTRUCTION);
	mdelay(5);	
	
		
	instr_byte = 0x8;
	pcf8574_lcd_exec_8bit_cmd(lcd, instr_byte, PCF8574_LCD_INSTRUCTION);
	mdelay(5);

	instr_byte = 0x1;
	pcf8574_lcd_exec_8bit_cmd(lcd, instr_byte, PCF8574_LCD_INSTRUCTION);
	mdelay(10);

	instr_byte = 0x6;
	pcf8574_lcd_exec_8bit_cmd(lcd, instr_byte, PCF8574_LCD_INSTRUCTION);
	mdelay(5);

	/* display on, cursor on, cursor blink on */
	instr_byte = 0xF;
	pcf8574_lcd_exec_8bit_cmd(lcd, instr_byte, PCF8574_LCD_INSTRUCTION);
	mdelay(5);
	
	/* return cursor to home position, init complete */
	instr_byte = 0x2;
	pcf8574_lcd_exec_8bit_cmd(lcd, instr_byte, PCF8574_LCD_INSTRUCTION);
	udelay(37);
	
	lcd->backlight = PCF8574_LCD_BACKLIGHT_ON;
	lcd->cursor_blink = PCF8574_LCD_BLINK_ON;

	dev_info(&lcd->client->dev, "LCD controller initialization complete\n");
	
	return 0;
	
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



static ssize_t pcf8574_lcd_read(struct file *file, char __user *userbuf, 
								size_t count, loff_t *ppos) 
{

	u8 ret;
	int size;
	char buf[3];
	struct lcd_data *lcd;

	lcd = container_of(file->private_data, 
					   struct lcd_data, 
			           pcf8574_misc_device);
	pr_info("checking contents of lcd data struct -- owner: %s", lcd->name);
	dev_info(&lcd->client->dev, "pcf8574_lcd_read() entered on %s\n", lcd->name);
	dev_info(&lcd->client->dev, "read callback fn recv'd %d chars\n", count);
	


	dev_info(&lcd->client->dev, "will read pin state on io expander");
	
	ret = i2c_smbus_read_byte(lcd->client);

	if (ret < 0) {
		dev_err(&lcd->client->dev, "i2c bus read failed\n");
		return ret;
	}
	dev_info(&lcd->client->dev, "byte read from i2c bus: %d\n", ret );
	
	size = sprintf(buf, "%02x", ret);
	buf[size] = '\n';
	if (*ppos == 0) {
		if(copy_to_user(userbuf, buf, size+1)) {
			dev_err(&lcd->client->dev, "failed to return i2c read data to userspace");
			return -EFAULT;

		}
		*ppos+=1;
		return size + 1;
	}
	return 0;



}

static int pcf8574_lcd_write_string(struct lcd_data *lcd, void __user *arg)
{

	int count;
	char to_ddram[81];
	int i;
	u8 data_byte;

	/* strncpy from userspace */
	count = strncpy_from_user(to_ddram, arg, 80);
	
	if (count < 0) 
	{
		dev_err(&lcd->client->dev, "userspace strncpy errno: %d", EFAULT);
		return -EFAULT;
	}
	
	/* null-terminate */
	to_ddram[count] = '\0';
	dev_info(&lcd->client->dev, "string copied from user: %s, %d bytes", to_ddram, count);
	
	i = 0;
	while(to_ddram[i]) {
		data_byte = to_ddram[i];
		
		pcf8574_lcd_exec_8bit_cmd(lcd, data_byte, PCF8574_LCD_DATA);
		i++;
	}


	return 0;
}

static int pcf8574_lcd_clear_display(struct lcd_data *lcd)
{

	u8 instr_byte;
	instr_byte = PCF8574_LCD_DISPLAY_CLEAR_BIT;

	pcf8574_lcd_exec_8bit_cmd(lcd, instr_byte, PCF8574_LCD_INSTRUCTION);

	return 0;	


}

static int pcf8574_lcd_clear_cursor_mode(struct lcd_data *lcd)
{

	u8 disp_ctrl_byte;
	
	disp_ctrl_byte = (1 << 3) | lcd->display_state | lcd->cursor_onoff | lcd->cursor_blink;
	disp_ctrl_byte &= ~(lcd->cursor_blink);
	dev_info(&lcd->client->dev, "sending: %02x to lcd controller IR\n", disp_ctrl_byte);
	pcf8574_lcd_exec_8bit_cmd(lcd, disp_ctrl_byte, PCF8574_LCD_INSTRUCTION); 

	return 0;

}


static int pcf8574_lcd_set_cursor_mode(struct lcd_data *lcd)
{

	u8 disp_ctrl_byte;
	
	disp_ctrl_byte = (1 << 3) | lcd->display_state | lcd->cursor_onoff | lcd->cursor_blink;
	disp_ctrl_byte |= lcd->cursor_blink;
	dev_info(&lcd->client->dev, "sending: %02x to lcd controller IR\n", disp_ctrl_byte);
	pcf8574_lcd_exec_8bit_cmd(lcd, disp_ctrl_byte, PCF8574_LCD_INSTRUCTION); 

	return 0;

}





static int pcf8574_lcd_toggle_cursor_mode(struct lcd_data *lcd)
{

	u8 disp_ctrl_byte;
	
	disp_ctrl_byte = (1 << 3) | lcd->display_state | lcd->cursor_onoff | lcd->cursor_blink;
	disp_ctrl_byte ^= lcd->cursor_blink;
	dev_info(&lcd->client->dev, 
			"curr blink: %02x sending: %02x to lcd controller IR\n", lcd->cursor_blink, disp_ctrl_byte);
	pcf8574_lcd_exec_8bit_cmd(lcd, disp_ctrl_byte, PCF8574_LCD_INSTRUCTION); 

	return 0;

}

static long pcf8574_lcd_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	struct lcd_data *lcd;
	void __user *arg = NULL;
	int rval = -EINVAL;
	
			
	pr_info("pcf8574_lcd in ioctl function");
	lcd = container_of(filp->private_data, 
						struct lcd_data,
						pcf8574_misc_device); 
	

	dev_info(&lcd->client->dev,"ioctl: request recv'd\n");


	/* guard against wrong ioctl magics */
	if (_IOC_TYPE(cmd) != PCF8574_LCD_MAGIC) {
		return -ENOTTY;
	}

	if (_IOC_NR(cmd) > PCF8574_MAX_NR) {
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
			dev_info(&lcd->client->dev,"ioctl: display on request recv'd\n");
			break;
		case PCF8574_LCD_WRITE_STRING:
			dev_info(&lcd->client->dev, "ioctl: ddram write request recv'd\n");
			pcf8574_lcd_write_string(lcd,arg);
			break;
		case PCF8574_LCD_CLEAR_DISPLAY:
			dev_info(&lcd->client->dev, "ioctl: clear display request received\n");
			pcf8574_lcd_clear_display(lcd);
			break;
		case PCF8574_LCD_TOGGLE_CURSOR_MODE:
			dev_info(&lcd->client->dev, "ioctl: toggle cursor blink mode req recv'd\n");
			pcf8574_lcd_toggle_cursor_mode(lcd);
			break;
	}

	return 1;
}

static int pcf8574_lcd_open(struct inode *iptr, 
							struct file* filp) 
{

	return 0;
}

static int pcf8574_lcd_release(struct inode *iptr,
								struct file* filp) 

{

	return 0;

}


static const struct file_operations pcf8574_lcd_fops = {
	.owner = THIS_MODULE,
	.write = pcf8574_lcd_write,
	.read = pcf8574_lcd_read,
	.release = pcf8574_lcd_release,
	.open = pcf8574_lcd_open,
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

	/* update private device data struct */

	lp->pcf8574_misc_device.name = DRIVER_NAME;
	lp->pcf8574_misc_device.minor = MISC_DYNAMIC_MINOR;
	lp->pcf8574_misc_device.fops = &pcf8574_lcd_fops;
	lp->backlight = PCF8574_LCD_BACKLIGHT_ON;
	lp->cursor_blink = PCF8574_LCD_BLINK_ON;
	lp->cursor_onoff = PCF8574_LCD_CURSOR_ON;
	lp->display_state = PCF8574_LCD_ON;

	lp->regmode = PCF8574_LCD_DATA;

	err = misc_register(&lp->pcf8574_misc_device);
	if (err) {
		dev_err(&client->dev, "%s:%u: failed misc_register on dynamic minor=%d\n",
			   	__func__, __LINE__, err);
		return err;

	}
	
	/* init the hardware */	
	pcf8574_lcd_initialize(lp);


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
