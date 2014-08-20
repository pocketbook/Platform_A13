
#include "lcd_panel_cfg.h"

//delete this line if you want to use the lcd para define in sys_config1.fex
//#define LCD_PARA_USE_CONFIG

#ifdef LCD_PARA_USE_CONFIG
static __u8 g_gamma_tbl[][2] = 
{
//{input value, corrected value}
    {0, 0},
    {15, 15},
    {30, 30},
    {45, 45},
    {60, 60},
    {75, 75},
    {90, 90},
    {105, 105},
    {120, 120},
    {135, 135},
    {150, 150},
    {165, 165},
    {180, 180},
    {195, 195},
    {210, 210},
    {225, 225},
    {240, 240},
    {255, 255},
};

static void LCD_cfg_panel_info(__panel_para_t * info)
{
    __u32 i = 0, j=0;
    
    memset(info,0,sizeof(__panel_para_t));

    info->lcd_x             = 258;
    info->lcd_y             = 620;
    info->lcd_dclk_freq     = 14;       //MHz
    
    info->lcd_pwm_not_used  = 0;
    info->lcd_pwm_ch        = 0;
    info->lcd_pwm_freq      = 10000;     //Hz
    info->lcd_pwm_pol       = 0;

    info->lcd_if            = 0;        //0:hv(sync+de); 1:8080; 2:ttl; 3:lvds

    info->lcd_hbp           = 2;//14;      //hsync back porch
    info->lcd_ht            = info->lcd_hbp + (info->lcd_x + 1) + 10;//271     //hsync total cycle
    info->lcd_hv_hspw       = 22;//1;//10;        //hsync plus width
    info->lcd_vbp           = 2;//8;       //vsync back porch
    info->lcd_vt            = (info->lcd_vbp + 1 + info->lcd_y + 1 + 2)*2 + 10;//1262  //vysnc total cycle *2
    info->lcd_hv_vspw       = 2;//1;//5;        //vysnc plus width

    info->lcd_hv_if         = 0;        //0:hv parallel 1:hv serial 
    info->lcd_hv_smode      = 0;        //0:RGB888 1:CCIR656
    info->lcd_hv_s888_if    = 0;        //serial RGB format
    info->lcd_hv_syuv_if    = 0;        //serial YUV format

    info->lcd_cpu_if        = 0;        //0:18bit 4:16bit
    info->lcd_frm           = 0;        //0: disable; 1: enable rgb666 dither; 2:enable rgb656 dither

    info->lcd_lvds_ch       = 0;        //0:single channel; 1:dual channel
    info->lcd_lvds_mode     = 0;        //0:NS mode; 1:JEIDA mode
    info->lcd_lvds_bitwidth = 0;        //0:24bit; 1:18bit
    info->lcd_lvds_io_cross = 0;        //0:normal; 1:pn cross

    info->lcd_io_cfg0       = 0x04000000;//0x10000000;//0x20000000;//0x24000000;//0; //0x04000000;
    
    info->lcd_gamma_correction_en = 0;
    if(info->lcd_gamma_correction_en)
    {
        __u32 items = sizeof(g_gamma_tbl)/2;
        
        for(i=0; i<items-1; i++)
        {
            __u32 num = g_gamma_tbl[i+1][0] - g_gamma_tbl[i][0];

            //__inf("handling{%d,%d}\n", g_gamma_tbl[i][0], g_gamma_tbl[i][1]);
            for(j=0; j<num; j++)
            {
                __u32 value = 0;

                value = g_gamma_tbl[i][1] + ((g_gamma_tbl[i+1][1] - g_gamma_tbl[i][1]) * j)/num;
                info->lcd_gamma_tbl[g_gamma_tbl[i][0] + j] = (value<<16) + (value<<8) + value;
                //__inf("----gamma %d, %d\n", g_gamma_tbl[i][0] + j, value);
            }
        }
        info->lcd_gamma_tbl[255] = (g_gamma_tbl[items-1][1]<<16) + (g_gamma_tbl[items-1][1]<<8) + g_gamma_tbl[items-1][1];
        //__inf("----gamma 255, %d\n", g_gamma_tbl[items-1][1]);
    }
}
#endif

static __s32 LCD_open_flow(__u32 sel)
{
	//LCD_OPEN_FUNC(sel, LCD_power_on, 50);   //open lcd power, and delay 50ms

    //modified by heyihang
	//LCD_OPEN_FUNC(sel, TCON_open, 500);     //open lcd controller, and delay 500ms

    LCD_OPEN_FUNC(sel, LCD_eink_main_pwr_on, 1);
    LCD_OPEN_FUNC(sel, LCD_eink_vneg_on, 5);
    LCD_OPEN_FUNC(sel, LCD_eink_vpos_on, 0);
    LCD_OPEN_FUNC(sel, LCD_eink_ud_on, 0);
    LCD_OPEN_FUNC(sel, LCD_eink_rl_on, 0);
    LCD_OPEN_FUNC(sel, LCD_eink_vgg_on, 4);
    LCD_OPEN_FUNC(sel, LCD_eink_vcom_on, 1);
    
	LCD_OPEN_FUNC(sel, TCON_open, 0);     //open lcd controller, and delay 0ms
	//LCD_OPEN_FUNC(sel, LCD_bl_open, 0);     //open lcd backlight, and delay 0ms

	return 0;
}

static __s32 LCD_close_flow(__u32 sel)
{	
    //modified by heyihang
    //LCD_CLOSE_FUNC(sel, LCD_bl_close, 0);       //close lcd backlight, and delay 0ms
	//LCD_CLOSE_FUNC(sel, LCD_bl_close, 100);       //close lcd backlight, and delay 100ms
	LCD_CLOSE_FUNC(sel, TCON_close, 0);         //close lcd controller, and delay 0ms
    
    LCD_CLOSE_FUNC(sel, LCD_eink_vcom_off, 1);

    LCD_CLOSE_FUNC(sel, LCD_eink_vneg_off, 0);
    LCD_CLOSE_FUNC(sel, LCD_eink_vpos_off, 0);
    LCD_CLOSE_FUNC(sel, LCD_eink_ud_off, 0);
    LCD_CLOSE_FUNC(sel, LCD_eink_rl_off, 0);
    LCD_CLOSE_FUNC(sel, LCD_eink_vgg_off, 0);

    LCD_CLOSE_FUNC(sel, LCD_eink_main_pwr_off, 0);
    
	//LCD_CLOSE_FUNC(sel, LCD_power_off, 1000);   //close lcd power, and delay 1000ms

	return 0;
}

//sel: 0:lcd0; 1:lcd1
static __s32 LCD_user_defined_func(__u32 sel, __u32 para1, __u32 para2, __u32 para3)
{
    return 0;
}

void LCD_get_panel_funs_0(__lcd_panel_fun_t * fun)
{
#ifdef LCD_PARA_USE_CONFIG
    fun->cfg_panel_info = LCD_cfg_panel_info;//delete this line if you want to use the lcd para define in sys_config1.fex
#endif
    fun->cfg_open_flow = LCD_open_flow;
    fun->cfg_close_flow = LCD_close_flow;
    fun->lcd_user_defined_func = LCD_user_defined_func;
}

