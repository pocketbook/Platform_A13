#ifndef __LINUX_FT5X_TS_H__
#define __LINUX_FT5X_TS_H__

// gpio base address
#define CONFIG_FT5X0X_MULTITOUCH     (1)
#define CALIBRATION  (1)
#define UPGRADE   (5)

#define I2C_MINORS 	256
#define I2C_MAJOR 	125
                                
struct ft5x_ts_platform_data{
	u16	intr;		/* irq number	*/
};

enum ft5x_ts_regs {
	FT5X0X_REG_PMODE	= 0xA5,	/* Power Consume Mode		*/	
};

//FT5X0X_REG_PMODE
#define PMODE_ACTIVE        0x00
#define PMODE_MONITOR       0x01
#define PMODE_STANDBY       0x02
#define PMODE_HIBERNATE     0x03

//suspend mode
#define SUSPEND_MODE 	0x1 //wake up by wake pin
#define POWEROFF_MODE	0x2 //need reset configuration of touch chip

#ifndef ABS_MT_TOUCH_MAJOR
    #define ABS_MT_TOUCH_MAJOR	0x30	/* touching ellipse */
    #define ABS_MT_TOUCH_MINOR	0x31	/* (omit if circular) */
    #define ABS_MT_WIDTH_MAJOR	0x32	/* approaching ellipse */
    #define ABS_MT_WIDTH_MINOR	0x33	/* (omit if circular) */
    #define ABS_MT_ORIENTATION	0x34	/* Ellipse orientation */
    #define ABS_MT_POSITION_X	0x35	/* Center X ellipse position */
    #define ABS_MT_POSITION_Y	0x36	/* Center Y ellipse position */
    #define ABS_MT_TOOL_TYPE	0x37	/* Type of touching device */
    #define ABS_MT_BLOB_ID		0x38	/* Group set of pkts as blob */
#endif /* ABS_MT_TOUCH_MAJOR */


#endif

