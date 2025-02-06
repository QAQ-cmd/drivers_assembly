/*
 *  qiyang_reset_power.c 
 *
 *  Author          wangtg
 *  Email           wangtg@qiyangtech.com
 *  Create time     2019-10-29
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include <linux/fs.h>
#include <linux/delay.h>
#include <asm/ioctl.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/delay.h>


static int gpio_cs = -1;
static int gpio_sda = -1;
static int gpio_sck = -1;


 #define CS1239_DELAY	  25

static void cs(bool val)
{
    gpio_set_value(gpio_cs,!val);
    
}

static void sda_out(int val)
{
    
    gpio_set_value(gpio_sda, val);
}

static int sda_in(void)
{
    gpio_direction_input(gpio_sda);
    return gpio_get_value(gpio_sda);
}

static void sck(int val)
{
    gpio_set_value(gpio_sck, val);
}


static void spi_start(void)
{
    sck(1);	
	cs(1);
	udelay(CS1239_DELAY);

	
	sck(0);
	udelay(CS1239_DELAY);
	udelay(CS1239_DELAY);
	cs(0);
}


static void spi_end(void)
{
	udelay(CS1239_DELAY);
	udelay(CS1239_DELAY);
	cs(1);
	udelay(CS1239_DELAY);
	sck(1);
	//udelay(CS1239_DELAY);
	//sck(0);
}


//SPI Send byte
void spi_send_byte(unsigned char data)
{
	int i;

	for(i=0;i<8;i++){
		if((data&0x80) >0)
			sda_out(1);
		else
			sda_out(0);
		data<<=1;
		
		sck(1);
		udelay(CS1239_DELAY);
		sck(0);
		udelay(CS1239_DELAY);
	}
}


unsigned char spi_recv_byte(void)
{
	char i;

    unsigned char data = 0;
	gpio_direction_input(gpio_sda);
	udelay(CS1239_DELAY);
	for(i=0;i<8;i++)
	{
		if(i != 0)
			data<<=1; 
		sck(1);
		udelay(CS1239_DELAY);
		if(sda_in())	
			data|=1;
		sck(0);
		udelay(CS1239_DELAY);
	}

	gpio_direction_output(gpio_sda,0);
	return data;
}


static void spi_write(unsigned char reg,unsigned char val)
{
	spi_start();
	spi_send_byte(0x80|reg);	//WRITE ADDRESS
	spi_send_byte(val);	//WRITE DATA
	spi_end();
}

static char spi_read(unsigned char reg)
{
    unsigned char data = 0;
	spi_start();
	spi_send_byte(reg);	//WRITE ADDRESS
	data = spi_recv_byte();
	spi_end();
    return data;
}

// return : 1,ok; 0,fail
static unsigned int spi_get_adc(void)
{
	unsigned char vad1,vad2,vad3;
    unsigned long   adc_val = 0;
	spi_start();
	spi_send_byte(0x09);	//WRITE ADDRESS
	vad1 = spi_recv_byte();
	vad2 = spi_recv_byte();
	vad3 = spi_recv_byte();
	spi_end();
	adc_val = (vad1<<16)+(vad2<<8)+vad3;
	adc_val = adc_val*2973/1000;
    return (unsigned int)adc_val;
}


static int cs1239_write(unsigned char reg,unsigned char val)
{
   int i;
    for(i=0;i<100;i++){
        spi_write(reg,val);
        udelay(1000);
        if(spi_read(reg) == val){
            return 1;
        }
        udelay(1000);
    }
	printk("error:%s,wirte fail\n",__func__);
    return -1;

}

static int cs1239_init(void)
{
	cs1239_write(6,0x0);
	cs1239_write(5,0x41);	
	cs1239_write(4,0);	
	cs1239_write(2,0xc0);	
	cs1239_write(1,0x18);	
	cs1239_write(0,0x17);	

    return 0;
}

int cs1239_set_ch(int ch)
{
    int ret = -1;
    switch(ch){
        case 1:
            ret = cs1239_write(1,0x18);	//set ADC P: AIN0; N: AIN4;
            if(ret < 0){
                printk("error:%s,wirte fail\n",__func__);
                return -1;
            }
            break;
        case 2:
            ret = cs1239_write(1,0x19);	//set ADC P: AIN1; N: AIN4;
            if(ret < 0){
                printk("error:%s,wirte fail\n",__func__);
                return -1;
            }
            break;
        case 3:
            ret = cs1239_write(1,0x1a);	//set ADC P: AIN2; N: AIN4;
            if(ret < 0){
                printk("error:%s,wirte fail\n",__func__);
                return -1;
            }
            break;
        case 4:
            ret = cs1239_write(1,0x1b);	//set ADC P: AIN3; N: AIN4;
            if(ret < 0){
                printk("error:%s,wirte fail\n",__func__);
                return -1;
            }
            break;
        

    }
    return 0;

}

static unsigned int cs1239_get_AIN1(void)
{
	cs1239_set_ch(1);
	msleep(20);
	return spi_get_adc();
}

static unsigned int cs1239_get_AIN2(void)
{
	cs1239_set_ch(2);
	msleep(20);
	return spi_get_adc();
}

static unsigned int cs1239_get_AIN3(void)
{
	cs1239_set_ch(4);
	msleep(20);
	return spi_get_adc();
}

static unsigned int cs1239_get_AIN4(void)
{
    int ret = -2;
	ret = cs1239_set_ch(3);
	msleep(20);
	return spi_get_adc();
}


static ssize_t show_ain1_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%d\n",cs1239_get_AIN1());
}

static ssize_t show_ain2_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%d\n",cs1239_get_AIN2());
}

static ssize_t show_ain3_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%d\n",cs1239_get_AIN3());
}

static ssize_t show_ain4_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%d\n",cs1239_get_AIN4());
}

static DEVICE_ATTR(ain1,S_IRUGO, show_ain1_value, NULL);
static DEVICE_ATTR(ain2,S_IRUGO, show_ain2_value, NULL);
static DEVICE_ATTR(ain3,S_IRUGO, show_ain3_value, NULL);
static DEVICE_ATTR(ain4,S_IRUGO,show_ain4_value,NULL);


static struct attribute *cs1239_attributes[] = {	

	&dev_attr_ain1.attr,
	&dev_attr_ain2.attr,
	&dev_attr_ain3.attr,
	&dev_attr_ain4.attr,
	NULL
};

static struct attribute_group cs1239_attribute_group = {
	//.name = "qmi8658",
	.attrs = cs1239_attributes
};

static int probe(struct platform_device *pdev)
{
    int ret = -1;
    struct device_node *np = pdev->dev.of_node;
    // enum of_gpio_flags flags;
	
    gpio_sda = of_get_named_gpio(np, "sda-gpios", 0);
    if (gpio_sda == -EPROBE_DEFER){
		dev_err(&pdev->dev, "failed to get of_get_named_gpio sda : %d\n", gpio_sda);
        return gpio_sda;
	}

    gpio_sck = of_get_named_gpio(np, "sck-gpios", 0);
    if (gpio_sck == -EPROBE_DEFER){
		dev_err(&pdev->dev, "failed to get of_get_named_gpio sdk : %d\n", gpio_sck);
        return gpio_sck;
	}

	gpio_cs = of_get_named_gpio(np, "cs-gpios", 0);
    if (gpio_cs == -EPROBE_DEFER){
		dev_err(&pdev->dev, "failed to get of_get_named_gpio cs : %d\n", gpio_cs);
        return gpio_cs;
	}

    
    if (!gpio_is_valid(gpio_cs))
        return -ENODEV;
    ret = devm_gpio_request_one(&pdev->dev,gpio_cs,1,"cs1239_cs");
    if (ret) {
        dev_err(&pdev->dev, "failed to get qiyang-power-gpios: %d\n", ret);
        return ret;
    }
    gpio_direction_output(gpio_cs,1);

    if (!gpio_is_valid(gpio_sda))
        return -ENODEV;
    ret = devm_gpio_request_one(&pdev->dev,gpio_sda,1,"cs1239_sda");
    if (ret) {
        dev_err(&pdev->dev, "failed to get qiyang-power-gpios: %d\n", ret);
        return ret;
    }
    gpio_direction_output(gpio_sda,1);

    if (!gpio_is_valid(gpio_sck))
        return -ENODEV;
    ret = devm_gpio_request_one(&pdev->dev,gpio_sck,1,"cs1239_sck");
    if (ret) {
        dev_err(&pdev->dev, "failed to get qiyang-power-gpios: %d\n", ret);
        return ret;
    }
    gpio_direction_output(gpio_sck,1);
	msleep(100);

	
    cs1239_init();
	//printk("spi_get_adc() %d\n",spi_get_adc());
	

    ret = sysfs_create_group(&pdev->dev.kobj, &cs1239_attribute_group);
	if (ret < 0) {
		printk("%s: create group fail!\n", __func__);
		return ret;
	}

    return 0;
}

static int remove(struct platform_device *pdev)
{

    // if(gpio_cs > 0)
    //     devm_gpio_free(&pdev->dev, gpio_cs);
    // if(gpio_sda > 0)
    //     devm_gpio_free(&pdev->dev, gpio_sda);
    // if(gpio_sck > 0)
    //     devm_gpio_free(&pdev->dev, gpio_sck);
    return 0;
}



static const struct of_device_id cs1239_dt_match[] = {
    { .compatible = "qiyang,cs1239" },
    { }
};
MODULE_DEVICE_TABLE(of, cs1239_dt_match);

static struct platform_driver cs1239_driver = {
    .driver ={
        .name = "cs1239",
        .of_match_table = of_match_ptr(cs1239_dt_match),
    },
    .probe = probe,
    .remove = remove,
};

module_platform_driver(cs1239_driver);

MODULE_AUTHOR("lijun");
MODULE_DESCRIPTION("IMX cs1239");
MODULE_LICENSE("GPL v2");

