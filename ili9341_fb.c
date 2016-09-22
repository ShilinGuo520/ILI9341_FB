#include <linux/init.h>  
#include <linux/module.h>  
#include <linux/moduleparam.h>  
#include <linux/kernel.h>  
#include <linux/device.h> 
#include <mach/platform.h>       
#include <linux/platform_device.h>
#include <linux/types.h>  
#include <linux/fs.h>   
#include <linux/fb.h>
#include <linux/ioctl.h>  
#include <linux/cdev.h>  
#include <linux/delay.h>  
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>  
#include <linux/delay.h>  
#include <asm/irq.h>  
#include <linux/mm.h>  
#include <linux/delay.h>  
#include <linux/moduleparam.h>  
#include <linux/slab.h>  
#include <linux/errno.h>  
#include <linux/cdev.h>  
#include <linux/string.h>  
#include <linux/list.h>  
#include <linux/pci.h>  
#include <asm/uaccess.h>  
#include <asm/atomic.h>  
#include <asm/unistd.h>  
#include <asm/io.h>  
#include <asm/uaccess.h>  
#include <linux/ioport.h>  
#include <linux/workqueue.h>
#include "bcm2835.h"  


static struct fb_info *clb_fbinfo;

static void gpio_reset(int num)
{
	unsigned int reset;
	reset = (1 << num);
	writel(reset, __io_address(GPIO_BASE+0x28));
}

static void gpio_set(int num)
{
	unsigned int set;
	set =  (1 << num);
	writel(set, __io_address(GPIO_BASE+0x1C));
}

static void gpio_output(int num)
{
	unsigned int func;
	func = readl(__io_address(GPIO_BASE));
	func = (func | (1 << (num * 3)));
	writel(func, __io_address(GPIO_BASE));
}

static void tft_gpio_init(void)
{
	gpio_output(2);
	gpio_output(3);
	gpio_output(4);
	gpio_output(5);
	gpio_output(6);
}

#define LED_L	gpio_reset(2);
#define LED_H	gpio_set(2);
#define RES_L	gpio_reset(3);
#define RES_H	gpio_set(3);
#define DC_L	gpio_reset(4);
#define DC_H	gpio_set(4);
#define SDI_L	gpio_reset(5);
#define SDI_H	gpio_set(5);
#define CLK_L	gpio_reset(6);
#define CLK_H   gpio_set(6);


/***********************************/
u16 BACK_COLOR, POINT_COLOR;

#define LCD_W 320
#define LCD_H 240

#define delayms msleep

void LCD_Writ_Bus(char da)
{
	char bitdata = da;
	int i;
	for(i = 0; i < 8; i++) {
		if(bitdata & 0x80)
			SDI_H
		else
			SDI_L
		CLK_L
		bitdata = bitdata << 1;
		CLK_H
	}
} 

void LCD_WR_DATA8(char da) 
{
    DC_H;
	LCD_Writ_Bus(da);
}  

void LCD_WR_DATA(int da)
{
    DC_H;
	LCD_Writ_Bus(da>>8);
	LCD_Writ_Bus(da);
}

void LCD_WR_REG(char da)	 
{
    DC_L;
	LCD_Writ_Bus(da);
}

void LCD_WR_REG_DATA(int reg,int da)
{
    LCD_WR_REG(reg);
	LCD_WR_DATA(da);
}

void Address_set(	unsigned int x1, unsigned int y1, \
					unsigned int x2, unsigned int y2)
{
	LCD_WR_REG(0x2a);
	LCD_WR_DATA8(x1>>8);
	LCD_WR_DATA8(x1);
	LCD_WR_DATA8(x2>>8);
	LCD_WR_DATA8(x2);

	LCD_WR_REG(0x2b);
	LCD_WR_DATA8(y1>>8);
	LCD_WR_DATA8(y1);

	LCD_WR_DATA8(y2>>8);
	LCD_WR_DATA8(y2);

	LCD_WR_REG(0x2C);
}

void Lcd_Init(void)
{
	RES_L
	delayms(20);

	RES_H
	delayms(20);
	LED_H

	LCD_WR_REG(0xCB);  
    LCD_WR_DATA8(0x39); 
    LCD_WR_DATA8(0x2C); 
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0x34); 
    LCD_WR_DATA8(0x02); 

    LCD_WR_REG(0xCF);  
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0XC1); 
    LCD_WR_DATA8(0X30); 

	LCD_WR_REG(0xE8);  
    LCD_WR_DATA8(0x85); 
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0x78); 

	LCD_WR_REG(0xEA);  
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0x00); 

	LCD_WR_REG(0xED);  
    LCD_WR_DATA8(0x64); 
    LCD_WR_DATA8(0x03); 
    LCD_WR_DATA8(0X12); 
    LCD_WR_DATA8(0X81); 

    LCD_WR_REG(0xF7);  
    LCD_WR_DATA8(0x20); 

    LCD_WR_REG(0xC0);    //Power control 
    LCD_WR_DATA8(0x23);   //VRH[5:0] 

    LCD_WR_REG(0xC1);    //Power control 
    LCD_WR_DATA8(0x10);   //SAP[2:0];BT[3:0] 

    LCD_WR_REG(0xC5);    //VCM control 
    LCD_WR_DATA8(0x3e); //¶Ô±È¶Èµ÷œÚ
    LCD_WR_DATA8(0x28); 

    LCD_WR_REG(0xC7);    //VCM control2 
    LCD_WR_DATA8(0x86);  //--

    LCD_WR_REG(0x36);    // Memory Access Control 
    LCD_WR_DATA8(0x28); //	   //48 68ÊúÆÁ//28 E8 ºáÆÁ

    LCD_WR_REG(0x3A);    
    LCD_WR_DATA8(0x55); 

    LCD_WR_REG(0xB1);    
    LCD_WR_DATA8(0x00);  
    LCD_WR_DATA8(0x18); 

    LCD_WR_REG(0xB6);    // Display Function Control 
    LCD_WR_DATA8(0x08); 
    LCD_WR_DATA8(0x82);
    LCD_WR_DATA8(0x27);  

	LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
    LCD_WR_DATA8(0x00); 

	LCD_WR_REG(0x26);    //Gamma curve selected 
    LCD_WR_DATA8(0x01); 

	LCD_WR_REG(0xE0);    //Set Gamma 
    LCD_WR_DATA8(0x0F); 
    LCD_WR_DATA8(0x31); 
    LCD_WR_DATA8(0x2B); 
    LCD_WR_DATA8(0x0C); 

    LCD_WR_DATA8(0x0E); 
    LCD_WR_DATA8(0x08); 
    LCD_WR_DATA8(0x4E); 
    LCD_WR_DATA8(0xF1); 
    LCD_WR_DATA8(0x37); 
    LCD_WR_DATA8(0x07); 
    LCD_WR_DATA8(0x10); 
    LCD_WR_DATA8(0x03); 
    LCD_WR_DATA8(0x0E); 
    LCD_WR_DATA8(0x09); 
    LCD_WR_DATA8(0x00); 

    LCD_WR_REG(0XE1);    //Set Gamma 
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0x0E); 
    LCD_WR_DATA8(0x14); 
    LCD_WR_DATA8(0x03); 
    LCD_WR_DATA8(0x11); 
    LCD_WR_DATA8(0x07); 
    LCD_WR_DATA8(0x31); 
    LCD_WR_DATA8(0xC1); 
    LCD_WR_DATA8(0x48); 
    LCD_WR_DATA8(0x08); 
    LCD_WR_DATA8(0x0F); 
    LCD_WR_DATA8(0x0C); 
    LCD_WR_DATA8(0x31); 
    LCD_WR_DATA8(0x36); 
    LCD_WR_DATA8(0x0F); 

    LCD_WR_REG(0x11);    //Exit Sleep 
    delayms(120); 

	LCD_WR_REG(0x29);    //Display on 
    LCD_WR_REG(0x2c); 
}


void LCD_Clear(u16 Color)
{
	u8 VH,VL;
	u16 i,j;
	VH=Color>>8;
	VL=Color;	

	Address_set(0,0,LCD_W-1,LCD_H-1);

	for(i=0; i<LCD_W; i++) {
		for (j=0; j<LCD_H; j++) {
			LCD_WR_DATA8(VH);
			LCD_WR_DATA8(VL);	
		}
	}
}
/***********************************/

static void tft_init(void)
{
	tft_gpio_init();
	Lcd_Init();
}

void update_tft(void)
{
	u8 *fb_base ; 
	u16 i,j;

	fb_base = clb_fbinfo->screen_base;

	Address_set(0,0,LCD_W-1,LCD_H-1);	
	for(i=0; i<LCD_W; i++) {
		for (j=0; j<LCD_H; j++) {
			LCD_WR_DATA8(*(fb_base + 1));
			LCD_WR_DATA8(*fb_base);
			fb_base++;
			fb_base++;
		}
	}
}

static void defense_work_handler(struct work_struct *work);
static DECLARE_DELAYED_WORK(defense_work, defense_work_handler);

static void defense_work_handler(struct work_struct *work)
{
	update_tft();
	schedule_delayed_work(&defense_work, (1 * HZ) / 20);
}

static int ili9341_map(struct fb_info *info, struct vm_area_struct *vma)
{    
    unsigned long page;
    unsigned long start = (unsigned long)vma->vm_start;
    unsigned long size = (unsigned long)(vma->vm_end - vma->vm_start);

	printk("func %s line %d \n", __func__, __LINE__);
    //得到物理地址
    page = info->fix.smem_start;
    //将用户空间的一个vma虚拟内存区映射到以page开始的一段连续物理页面上
    if(remap_pfn_range(vma,start,page>>PAGE_SHIFT,size,PAGE_SHARED)) { //第三个参数是页帧号，由物理地址右移PAGE_SHIFT得到
		printk("error:func %s", __func__);
		return -1;
	}
    return 0;
}

//帧缓冲操作函数
static struct fb_ops ili9341_lcdfb_ops =
{
    .owner          = THIS_MODULE,
	.fb_mmap		= ili9341_map,
	.fb_fillrect    = cfb_fillrect,  //画一个矩形
    .fb_copyarea    = cfb_copyarea,  //数据拷贝
    .fb_imageblit   = cfb_imageblit, //图像填充
};

static int __init ili9341_init(void)  
{  
    int ret;  
	int mem_size;
	/* 1.分配一个fb_info */
    clb_fbinfo = framebuffer_alloc(0 , NULL);

    /* 2. 设置 */
    /* 2.1 设置固定的参数 */
    strcpy(clb_fbinfo->fix.id, "ili9341_lcd");
    clb_fbinfo->fix.smem_len = 320 * 240 * 2;
    clb_fbinfo->fix.type = FB_TYPE_PACKED_PIXELS;
    clb_fbinfo->fix.visual = FB_VISUAL_TRUECOLOR;
    clb_fbinfo->fix.line_length = 320 * 2;

    /* 2.2 设置可变的参数 */
    clb_fbinfo->var.xres = 320;
    clb_fbinfo->var.yres = 240;
    clb_fbinfo->var.xres_virtual   = 320;
    clb_fbinfo->var.yres_virtual   = 240;
    clb_fbinfo->var.bits_per_pixel = 16;

    clb_fbinfo->var.red.offset = 11;
    clb_fbinfo->var.red.length = 5;

    clb_fbinfo->var.green.offset = 5;
    clb_fbinfo->var.green.length = 6;

    clb_fbinfo->var.blue.offset = 0;
    clb_fbinfo->var.blue.length = 5;

	clb_fbinfo->flags = FBINFO_FLAG_DEFAULT | FBINFO_VIRTFB;

    clb_fbinfo->var.activate = FB_ACTIVATE_NOW;

    /* 2.3 设置操作函数 */
    clb_fbinfo->fbops = &ili9341_lcdfb_ops;

    /* 2.4 其他的设置 */
    /* 2.4.1 设置显存的大小 */
    clb_fbinfo->screen_size =  320 * 240 * 2;

	mem_size = 320 * 240 * 2 ;
	clb_fbinfo->screen_base = (unsigned char *)kmalloc(mem_size,GFP_KERNEL);
	clb_fbinfo->fix.smem_start = virt_to_phys(clb_fbinfo->screen_base);
	memset(clb_fbinfo->screen_base,0xf0,mem_size);

    /* 2.4.3 设置显存的虚拟起始地址 */

	tft_init();
	schedule_delayed_work(&defense_work, 1 * HZ);
	ret = register_framebuffer(clb_fbinfo);

    printk("ili9341init.\n");  
    return ret;  
}  
  
static void ili9341_exit(void)  
{  
	ClearPageReserved(virt_to_page(clb_fbinfo->screen_base));
	kfree(clb_fbinfo->screen_base);
	unregister_framebuffer(clb_fbinfo);
	cancel_delayed_work_sync(&defense_work);       

	LED_L 
    printk("ili9341_exit\n");  
	framebuffer_release(clb_fbinfo);
}  
  
module_init(ili9341_init);  
module_exit(ili9341_exit);  
  
MODULE_AUTHOR("Guo Shilin");
MODULE_LICENSE("GPL");

