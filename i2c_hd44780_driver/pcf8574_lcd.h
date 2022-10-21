#ifndef __PCF8574_LCD__
#define __PCF8574_LCD__

#include <linux/miscdevice.h>
#include <asm/ioctl.h>
#include <linux/types.h>



#define DRIVER_NAME 						"pcf8574_lcd"
#define PCF8574_LCD_MAGIC					'!'
#define PCF8574_LCD_I2C_ADDR				0x27

#define PCF8574_LCD_BLINK_FLAG				(1 << 0x0)
#define PCF8574_LCD_CURSOR_FLAG				(1 << 0x1)
#define PCF8574_LCD_SHIFT_MODE				(1 << 0x2)
#define PCF8574_LCD_DISPLAY_FLAG			(1 << 0x3)

/* PCF8574 pin 3 is wired to a transistor to 
 * control the backlight 
 */
#define PCF8574_LCD_BACKLIGHT_FLAG			(1 << 0x3)


#define PCF8574_LCD_SHIFT_DIR				(1 << 0x3)


#define PCF8574_LCD_DISPLAY_CLEAR_BIT		(1 << 0x0)
#define PCF8574_LCD_RETURN_HOME_BIT			(1 << 0x1)
#define PCF8574_LCD_ENTRY_MODE_BIT			(1 << 0x2)
#define PCF8574_LCD_DISPLAY_CTRL_BIT		(1 << 0x3)
#define PCF8574_LCD_CURSOR_DISP_SHIFT_BIT	(1 << 0x4)
#define PCF8574_LCD_FUNCTION_SET_BIT		(1 << 0x5)
#define PCF8574_LCD_SET_CGRAM_ADDR_BIT		(1 << 0x6)
#define PCF8574_LCD_SET_DDRAM_ADDR_BIT		(1 << 0x7)
#define PCF8574_LCD_READ_BUSY_FLAG_BIT		(1 << 0x8)
#define PCF8574_LCD_RAM_WRITE_DATA			(1 << 0x9)
#define PCF8574_LCD_RAM_READ_DATA			(0x600)



#define PCF8574_LCD_ALL_FLAGS_SET			(0xff)



/* ioctl */
#define PCF8574_LCD_DISPLAY_ON				_IOW(PCF8574_LCD_MAGIC, 0, __u8)
#define PCF8574_LCD_SET_BACKLIGHT			_IOW(PCF8574_LCD_MAGIC, 1, __u8)




enum pcf8574_onoff
{
	PCF8574_LCD_OFF = 0,
	PCF8574_LCD_ON,
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
	PCF8574_LCD_DATA

};

enum pcf8574_shiftcursor
{
	PCF8574_LCD_CURSOR_MOVE = 0,
	PCF8574_LCD_DISPLAY_SHIFT,

};

struct lcd_data {
	
	struct i2c_client *client;
	struct miscdevice pcf8574_misc_device;
	__u8 name[64];

	/* LCD controller parameters */
	__u8 flags;
	enum pcf8574_onoff display_state;
	enum pcf8574_fontsize font;
	enum pcf8574_scrollshift dir;
	enum pcf8574_shiftcursor sc_mode;
	enum pcf8574_regselect regmode;

};




#endif /* __PCF8574_LCD__ */
