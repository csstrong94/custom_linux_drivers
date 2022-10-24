#ifndef __PCF8574_LCD__
#define __PCF8574_LCD__

#ifdef __KERNEL__
	
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/types.h>

#define DRIVER_NAME 						"pcf8574_lcd"
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


#define PCF8574_LCD_ENABLE_CMD				(0x0C)
#define PCF8574_LCD_ENABLE_BIT				(1 << 2)
#define PCF8574_LCD_4BIT_MODE				(0x2)

#else
#include <sys/ioctl.h>
#endif /* __KERNEL__ */

/* ioctl */
#define PCF8574_MAX_NR						15
#define PCF8574_LCD_MAGIC					'4'
#define PCF8574_LCD_DISPLAY_ON				_IO(PCF8574_LCD_MAGIC, 0 )
#define PCF8574_LCD_TOGGLE_BACKLIGHT			_IO(PCF8574_LCD_MAGIC, 1)
#define PCF8574_LCD_CLEAR_DISPLAY			_IO(PCF8574_LCD_MAGIC, 2)
#define PCF8574_LCD_TOGGLE_CURSOR_MODE		_IO(PCF8574_LCD_MAGIC,3)
#define PCF8574_LCD_WRITE_STRING			_IOW(PCF8574_LCD_MAGIC,4, const char*)




#endif /* __PCF8574_LCD__ */
