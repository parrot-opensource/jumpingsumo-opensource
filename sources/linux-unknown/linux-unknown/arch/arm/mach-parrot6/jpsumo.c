/*
 *  linux/arch/arm/mach-parrot6/parrot6idev.c
 *
 *  Copyright (C) 2008 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2008-11-05
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/mmc/host.h>
#include <linux/i2c.h>
#include <linux/reboot.h>

/* Functions to sleep */
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/platform.h>
#include <mach/map.h>
#include <mach/parrot.h>
#include <mach/gpio.h>
#include <mach/gpio_parrot.h>
#include <mach/mmc.h>
#include <mach/usb.h>
#include <mach/regs-pwm-p6.h>
#include <mach/i2c.h>
#include <mach/aai.h>

#include <media/soc_camera.h>
#include <media/soc_camera_platform.h>

#include "timer.h"
#include "devs.h"
#include "pm.h"
#include "parrot6.h"
#include "parrot6i.h"


// ----------------------------------- JPSUMO pins etc... -----------------------------------
typedef enum _jpsumo_hw_version_t
{
    JPSUMO_HW_A1 = 0x04,    // HW 01    ( The 3 bits have been inverted => HW Bin 001 => Bin 100 => 4 )
    JPSUMO_HW_A2 = 0x02,    // HW 02
    JPSUMO_HW_A3 = 0x06,    // HW 03
    JPSUMO_HW_B1 = 0x01,    // HW 04
    JPSUMO_HW_B2 = 0x05,    // HW 05
    JPSUMO_HW_06 = 0x05,    // HW 06    // HW 6 board, were used fir DV1, but the version number was still 0x05
    JPSUMO_HW_07 = 0x07,    // HW 07
} jpsumo_hw_version_t;


// --------------------------------
// -------  JPSUMO_PROTO_A1  ------
// --------------------------------

// JUMPING SUMO GPIOs:
#define JS_HSIS_A1__GPIOPIN_LED_DEBUG               GPIO_043
#define JS_HSIS_A1__GPIO_LED_DEBUG                  43

#define JS_HSIS_A1__GPIOPIN_SWITCH_POSITION_1       GPIO_050
#define JS_HSIS_A1__GPIO_SWITCH_POSITION_1          50
#define JS_HSIS_A1__GPIOPIN_SWITCH_POSITION_2       GPIO_051
#define JS_HSIS_A1__GPIO_SWITCH_POSITION_2          51

#define JS_HSIS_A1__GPIOPIN_1V8_P6i_EN              P6I_GPIO_007
#define JS_HSIS_A1__GPIO_1V8_P6i_EN                 7
#define JS_HSIS_A1__GPIOPIN_ALIM_OFF                GPIO_058
#define JS_HSIS_A1__GPIO_ALIM_OFF                   58

#define JS_HSIS_A1__GPIOPIN_JUMP_CTRL_1             GPIO_038
#define JS_HSIS_A1__GPIO_JUMP_CTRL_1                38
#define JS_HSIS_A1__GPIOPIN_JUMP_CTRL_2             GPIO_039
#define JS_HSIS_A1__GPIO_JUMP_CTRL_2                39

// CHARGE_STATUS        Input... Charge status...
#define JS_HSIS_A1__GPIOPIN_CHARGE_STATUS           GPIO_053
#define JS_HSIS_A1__GPIO_CHARGE_STATUS              53

// wifi...
#define JS_HSIS_A1__GPIOPIN_WLAN_nPWD               GPIO_040
#define JS_HSIS_A1__GPIO_WLAN_nPWD                  40
#define JS_HSIS_A1__GPIOPIN_WLAN_WARM_RST           GPIO_041
#define JS_HSIS_A1__GPIO_WLAN_WARM_RST              41

#define JS_HSIS_A1__GPIOPIN_nFAULT_WHEEL            GPIO_054	// input from wheel driver
#define JS_HSIS_A1__GPIO_nFAULT_WHEEL               54
#define JS_HSIS_A1__GPIOPIN_nSLEEP_WHEEL            GPIO_055	// output to kill wheel driver
#define JS_HSIS_A1__GPIO_nSLEEP_WHEEL               55

#define JS_HSIS_A1__GPIOPIN_PWMGEN_nOE              P6I_GPIO_004    // XXX This is a patch that should be on all Proto A1 boards.
#define JS_HSIS_A1__GPIO_PWMGEN_nOE                 4


// INT_MAGNETO      GPIO_056        // interupt from magneto    // unused

// INT_INERT : interupt from MPU6050
#define JS_HSIS_A1__GPIOPIN_INT_INERT               GPIO_057
#define JS_HSIS_A1__GPIO_INT_INERT                  57

// Unused on proto A1
#define JS_HSIS_A1__PWMPIN_PWMSIP6_0                P6I_PWM_00a
//#define JS_HSIS_A1__PWMPIN_PWMSIP6_1              P6I_PWM_01a		// Right now, we use this GPIO for something else. // XXX replaced by JS_HSIS_A1__GPIOPIN_PWMGEN_nOE on the same pin.
#define JS_HSIS_A1__PWMPIN_PWMSIP6_2                P6I_PWM_02a
#define JS_HSIS_A1__PWMPIN_PWMSIP6_3                P6I_PWM_03a

#define JS_HSIS_A1__GPIOPIN_nAMP_PWDN               GPIO_052
#define JS_HSIS_A1__GPIO_nAMP_PWDN                  52



// --------------------------------
// -------  JPSUMO_PROTO_A2  ------
// --------------------------------

// JUMPING SUMO GPIOs:
#define JS_HSIS_A2__GPIOPIN_LED_IR                  P6I_GPIO_010
#define JS_HSIS_A2__GPIO_LED_IR                     10

#define JS_HSIS_A2__GPIOPIN_NRST_CAM_1              GPIO_040
#define JS_HSIS_A2__GPIO_NRST_CAM_1                 40
#define JS_HSIS_A2__GPIOPIN_NRST_CAM                GPIO_043
#define JS_HSIS_A2__GPIO_NRST_CAM                   43

#define JS_HSIS_A2__GPIOPIN_DEV_GPIO_1              GPIO_048
#define JS_HSIS_A2__GPIO_DEV_GPIO_1                 48
#define JS_HSIS_A2__GPIOPIN_DEV_GPIO_2              GPIO_049
#define JS_HSIS_A2__GPIO_DEV_GPIO_2                 49
#define JS_HSIS_A2__GPIOPIN_DEV_GPIO_3              GPIO_051
#define JS_HSIS_A2__GPIO_DEV_GPIO_3                 51
#define JS_HSIS_A2__GPIOPIN_DEV_GPIO_4              GPIO_050
#define JS_HSIS_A2__GPIO_DEV_GPIO_4                 50

#define JS_HSIS_A2__GPIOPIN_1V8_P6i_EN              P6I_GPIO_007
#define JS_HSIS_A2__GPIO_1V8_P6i_EN                 7

#define JS_HSIS_A2__GPIOPIN_JUMP_CTRL_1             GPIO_038
#define JS_HSIS_A2__GPIO_JUMP_CTRL_1                38
#define JS_HSIS_A2__GPIOPIN_JUMP_CTRL_2             GPIO_039
#define JS_HSIS_A2__GPIO_JUMP_CTRL_2                39

#define JS_HSIS_A2__GPIOPIN_CHARGE_STATUS           GPIO_053    // XXX Test point, should not be necessary
#define JS_HSIS_A2__GPIO_CHARGE_STATUS              53

#define JS_HSIS_A2__GPIOPIN_nFAULT_WHEEL            GPIO_054	// input from wheel driver
#define JS_HSIS_A2__GPIO_nFAULT_WHEEL               54
#define JS_HSIS_A2__GPIOPIN_nSLEEP_WHEEL            GPIO_055	// output to kill wheel driver
#define JS_HSIS_A2__GPIO_nSLEEP_WHEEL               55

#define JS_HSIS_A2__GPIOPIN_PWMGEN_nOE              GPIO_042
#define JS_HSIS_A2__GPIO_PWMGEN_nOE                 42

// INT_INERT : interupt from MPU6050
#define JS_HSIS_A2__GPIOPIN_INT_INERT               GPIO_057
#define JS_HSIS_A2__GPIO_INT_INERT                  57


#define JS_HSIS_A2__PWMPIN_PWMSIP6_0                P6I_PWM_00a
#define JS_HSIS_A2__PWMPIN_PWMSIP6_1                P6I_PWM_01a
#define JS_HSIS_A2__PWMPIN_PWMSIP6_2                P6I_PWM_02a
#define JS_HSIS_A2__PWMPIN_PWMSIP6_3                P6I_PWM_03a

#define JS_HSIS_A2__GPIOPIN_nAMP_PWDN               GPIO_052
#define JS_HSIS_A2__GPIO_nAMP_PWDN                  52


// -------------------------------------------
// -------  JPSUMO_PROTO_A3 (and HW>=3) ------
// -------------------------------------------

// JUMPING SUMO GPIOs:
#define JS_HSIS_A3__GPIOPIN_LED_IR                  P6I_GPIO_010
#define JS_HSIS_A3__GPIO_LED_IR                     10

#define JS_HSIS_A3__GPIOPIN_LEFT_EYE_LED            P6I_GPIO_011
#define JS_HSIS_A3__GPIO_LEFT_EYE_LED               11
#define JS_HSIS_A3__GPIOPIN_RIGHT_EYE_LED           P6I_GPIO_012
#define JS_HSIS_A3__GPIO_RIGHT_EYE_LED              12

#define JS_HSIS_A3__GPIOPIN_NRST_CAM                GPIO_040
#define JS_HSIS_A3__GPIO_NRST_CAM                   40

#define JS_HSIS_A3__GPIOPIN_DEV_GPIO_1              GPIO_048
#define JS_HSIS_A3__GPIO_DEV_GPIO_1                 48
#define JS_HSIS_A3__GPIOPIN_DEV_GPIO_2              GPIO_049
#define JS_HSIS_A3__GPIO_DEV_GPIO_2                 49
#define JS_HSIS_A3__GPIOPIN_DEV_GPIO_3              GPIO_051
#define JS_HSIS_A3__GPIO_DEV_GPIO_3                 51

#define JS_HSIS_A3__GPIOPIN_1V8_P6i_EN              P6I_GPIO_007
#define JS_HSIS_A3__GPIO_1V8_P6i_EN                 7

#define JS_HSIS_A3__GPIOPIN_JUMP_CTRL_1             GPIO_038
#define JS_HSIS_A3__GPIO_JUMP_CTRL_1                38
#define JS_HSIS_A3__GPIOPIN_JUMP_CTRL_2             GPIO_039
#define JS_HSIS_A3__GPIO_JUMP_CTRL_2                39

#define JS_HSIS_A3__GPIOPIN_CHARGE_STATUS           GPIO_053    // XXX Test point, should not be necessary
#define JS_HSIS_A3__GPIO_CHARGE_STATUS              53

#define JS_HSIS_A3__GPIOPIN_nFAULT_WHEEL            GPIO_054	// input from wheel driver
#define JS_HSIS_A3__GPIO_nFAULT_WHEEL               54
#define JS_HSIS_A3__GPIOPIN_nSLEEP_WHEEL            GPIO_055	// output to kill wheel driver
#define JS_HSIS_A3__GPIO_nSLEEP_WHEEL               55

#define JS_HSIS_A3__GPIOPIN_PWMGEN_nOE              GPIO_042
#define JS_HSIS_A3__GPIO_PWMGEN_nOE                 42

// INT_INERT : interupt from MPU6050
#define JS_HSIS_A3__GPIOPIN_INT_INERT               GPIO_057
#define JS_HSIS_A3__GPIO_INT_INERT                  57


#define JS_HSIS_A3__PWMPIN_PWMSIP6_0                P6I_PWM_00a
#define JS_HSIS_A3__PWMPIN_PWMSIP6_1                P6I_PWM_01a
#define JS_HSIS_A3__PWMPIN_PWMSIP6_2                P6I_PWM_02a
#define JS_HSIS_A3__PWMPIN_PWMSIP6_3                P6I_PWM_03a

#define JS_HSIS_A3__GPIOPIN_nAMP_PWDN               GPIO_035
#define JS_HSIS_A3__GPIO_nAMP_PWDN                  35

#define JS_HSIS_A3__GPIOPIN_MUX_GPIO_ADC_IN0        GPIO_033
#define JS_HSIS_A3__GPIO_MUX_GPIO_ADC_IN0           33
#define JS_HSIS_A3__GPIOPIN_MUX_GPIO_ADC_IN1_2      GPIO_034
#define JS_HSIS_A3__GPIO_MUX_GPIO_ADC_IN1_2         34


// ------------------------------------------------------------------------------------------


static char * aai_clock_mode = "";
static char * aai_sync_freq = "";
static struct parrot_aai_platform_data aai_platform_data;

// ************************************

static unsigned int pins_init_p6i_jpsumo_A1[] = {
    /* gpios */
    JS_HSIS_A1__GPIOPIN_LED_DEBUG,
    JS_HSIS_A1__GPIOPIN_SWITCH_POSITION_1   ,
    JS_HSIS_A1__GPIOPIN_SWITCH_POSITION_2   ,
    JS_HSIS_A1__GPIOPIN_JUMP_CTRL_1         ,
    JS_HSIS_A1__GPIOPIN_JUMP_CTRL_2         ,
    JS_HSIS_A1__GPIOPIN_CHARGE_STATUS       ,
    JS_HSIS_A1__GPIOPIN_WLAN_nPWD           ,
    JS_HSIS_A1__GPIOPIN_WLAN_WARM_RST       ,
    JS_HSIS_A1__GPIOPIN_nFAULT_WHEEL        ,
    JS_HSIS_A1__GPIOPIN_nSLEEP_WHEEL        ,
    JS_HSIS_A1__GPIOPIN_PWMGEN_nOE          ,
    JS_HSIS_A1__GPIOPIN_INT_INERT           ,
    JS_HSIS_A1__GPIOPIN_nAMP_PWDN           ,
    /* uart */
    P6I_UART1_DEFAULT                       ,
    /* i2c */
    P6I_I2CM0_DEFAULT                       ,
    /* sdio */
    P6I_SD0_DEFAULT                         ,
    /* rtc pwr on */
    P6I_RTC_PWR_ON                          ,
    0                                       ,
};

static unsigned int pins_init_p6i_jpsumo_A2[] = {
    /* gpios */
    JS_HSIS_A2__GPIOPIN_LED_IR          ,
    JS_HSIS_A2__GPIOPIN_NRST_CAM_1      ,
    JS_HSIS_A2__GPIOPIN_NRST_CAM        ,
    JS_HSIS_A2__GPIOPIN_DEV_GPIO_1      ,
    JS_HSIS_A2__GPIOPIN_DEV_GPIO_2      ,
    JS_HSIS_A2__GPIOPIN_DEV_GPIO_3      ,
    JS_HSIS_A2__GPIOPIN_DEV_GPIO_4      ,
    JS_HSIS_A2__GPIOPIN_JUMP_CTRL_1     ,
    JS_HSIS_A2__GPIOPIN_JUMP_CTRL_2     ,
    JS_HSIS_A2__GPIOPIN_CHARGE_STATUS   ,
    JS_HSIS_A2__GPIOPIN_nFAULT_WHEEL    ,
    JS_HSIS_A2__GPIOPIN_nSLEEP_WHEEL    ,
    JS_HSIS_A2__GPIOPIN_PWMGEN_nOE      ,
    JS_HSIS_A2__GPIOPIN_INT_INERT       ,
    JS_HSIS_A2__GPIOPIN_nAMP_PWDN       ,
    /* pwms */
    JS_HSIS_A2__PWMPIN_PWMSIP6_0        ,
    JS_HSIS_A2__PWMPIN_PWMSIP6_1        ,
    JS_HSIS_A2__PWMPIN_PWMSIP6_2        ,
    JS_HSIS_A2__PWMPIN_PWMSIP6_3        ,
    /* uart */
    P6I_UART1_DEFAULT                   ,
    /* i2c */
    P6I_I2CM0_DEFAULT                   ,
    /* rtc pwr on */
    P6I_RTC_PWR_ON                      ,
    0                                   ,
};

static unsigned int pins_init_p6i_jpsumo_A3[] = {
    /* gpios */
    JS_HSIS_A3__GPIOPIN_LED_IR              ,
    JS_HSIS_A3__GPIOPIN_LEFT_EYE_LED        ,
    JS_HSIS_A3__GPIOPIN_RIGHT_EYE_LED       ,
    JS_HSIS_A3__GPIOPIN_NRST_CAM            ,
    JS_HSIS_A3__GPIOPIN_DEV_GPIO_1          ,
    JS_HSIS_A3__GPIOPIN_DEV_GPIO_2          ,
    JS_HSIS_A3__GPIOPIN_DEV_GPIO_3          ,
    JS_HSIS_A3__GPIOPIN_JUMP_CTRL_1         ,
    JS_HSIS_A3__GPIOPIN_JUMP_CTRL_2         ,
    JS_HSIS_A3__GPIOPIN_CHARGE_STATUS       ,
    JS_HSIS_A3__GPIOPIN_nFAULT_WHEEL        ,
    JS_HSIS_A3__GPIOPIN_nSLEEP_WHEEL        ,
    JS_HSIS_A3__GPIOPIN_PWMGEN_nOE          ,
    JS_HSIS_A3__GPIOPIN_INT_INERT           ,
    JS_HSIS_A3__GPIOPIN_nAMP_PWDN           ,
    JS_HSIS_A3__GPIOPIN_MUX_GPIO_ADC_IN0    ,
    JS_HSIS_A3__GPIOPIN_MUX_GPIO_ADC_IN1_2  ,
    /* pwms */
    JS_HSIS_A3__PWMPIN_PWMSIP6_0            ,
    JS_HSIS_A3__PWMPIN_PWMSIP6_1            ,
    JS_HSIS_A3__PWMPIN_PWMSIP6_2            ,
    JS_HSIS_A3__PWMPIN_PWMSIP6_3            ,
    /* uart */
    P6I_UART1_DEFAULT                       ,
    /* i2c */
    P6I_I2CM0_DEFAULT                       ,
    /* rtc pwr on */
    P6I_RTC_PWR_ON                          ,
    0                                       ,
};


static struct platform_device *p6i_jpsumo_A1_devices[] __initdata = {
    &p6_uart1_device,
    &p6_nand_device,
    &p6_aai_device,
    &p6_i2cm0_device,
    &p6_sdhci0_device,
    &p6i_usb0_device,
	/* virtual device */
    &p6_dmac_device,
    &p6_gpio,
};

static struct platform_device *p6i_jpsumo_A2plus_devices[] __initdata = {
    &p6_uart1_device,
    &p6_nand_device,
    &p6_aai_device,
    &p6_us_device,
    &p6_i2cm0_device,
    &p6i_usb0_device,
	/* virtual device */
    &p6_dmac_device,
    &p6_gpio,
};

static struct parrot_mmc_platform_data p6i_mmc_wifi_platform_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	/* wifi : not cd and wp pins */
};

// I2C 0 :
static struct parrot5_i2cm_platform i2cm_platform_js_i2c0 = {
	.bus_freq	= 400*1000,		// use i2c at 400kHz
	.retries	= 1,
};

static void init_aai(char *aai_clock_mode)
{
	int auto_mclk = 0;
	int auto_clk = 0;
	/*Auto : let aai driver select/unselect both pins clocks */
	if(!strcmp(aai_clock_mode,"auto_clock"))
		auto_mclk = auto_clk = 1;
	/*Auto_no_master : let aai driver select/unselect only bit clock pin
	  master clock remains unselected */
	else if(!strcmp(aai_clock_mode,"auto_clock_no_master"))
		auto_clk = 1;
	/*Default : bit and master clock always selected*/
	else {
		parrot_select_pin(P6I_I2S_CLK);
		parrot_select_pin(P6I_MCLK);
	}

	if(!strcmp(aai_sync_freq,"44100"))
		aai_platform_data.sync_freq = 44100;
	else
		aai_platform_data.sync_freq = 48000;


	aai_platform_data.i2s_bit_clock_control = auto_clk;
	aai_platform_data.i2s_master_clock_control = auto_mclk;

	p6_aai_device.dev.platform_data = &aai_platform_data;
}

static void __init p6idev_init(void)
{
    jpsumo_hw_version_t jpsumo_hw_version = -1;
    
	printk("Initializing JUMPING SUMO board ...\n");
	
    sip6_init(0);
    jpsumo_hw_version = parrot_board_rev();
    printk("parrot_board_rev = 0x%2X\n", jpsumo_hw_version);
    
    switch ( jpsumo_hw_version )
    {
        case JPSUMO_HW_A1:
            printk("=> JPSUMO_HW_A1\n");
            parrot_init_pin(pins_init_p6i_jpsumo_A1);
            
	        // Turn Off Jump H-Bridge
            gpio_direction_input(JS_HSIS_A1__GPIO_JUMP_CTRL_1);
        	gpio_direction_input(JS_HSIS_A1__GPIO_JUMP_CTRL_2);
            
            // JS : Turn Off PWM Generator
            gpio_direction_output(JS_HSIS_A1__GPIO_PWMGEN_nOE, 1);
            
            // JS : LED Debug: ( 0 is On ; 1 is Off )
            gpio_direction_output(JS_HSIS_A1__GPIO_LED_DEBUG, 0);
            
            gpio_direction_input(JS_HSIS_A1__GPIO_SWITCH_POSITION_1);
            gpio_direction_input(JS_HSIS_A1__GPIO_SWITCH_POSITION_2);
            
            gpio_direction_input(JS_HSIS_A1__GPIO_CHARGE_STATUS);
            
            gpio_direction_output(JS_HSIS_A1__GPIO_WLAN_nPWD, 1);
            gpio_direction_output(JS_HSIS_A1__GPIO_WLAN_WARM_RST, 0);
            
            gpio_direction_output(JS_HSIS_A1__GPIO_nSLEEP_WHEEL, 0);
            gpio_direction_input(JS_HSIS_A1__GPIO_nFAULT_WHEEL);
            
            /* Interrupt pin for pal_gpio_irq */
            gpio_direction_input(JS_HSIS_A1__GPIO_INT_INERT);
            gpio_interrupt_register(JS_HSIS_A1__GPIO_INT_INERT, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
            
            gpio_direction_output(JS_HSIS_A1__GPIO_nAMP_PWDN, 0); /* Speaker amplifier */
			
            /* INIT AAI */
            init_aai(aai_clock_mode);
            
            // I2C :
            p6_i2cm0_device.dev.platform_data = &i2cm_platform_js_i2c0;
            p6i_set_pads_i2c(0);
            p6i_set_i2c_drive_strength(0, 0x0);
	        
            // SDIO ....
            p6i_set_pads_sdcard(50000000);	// Set SDIO frequency at 50MHz & do set drive strength
            
            printk("Initializing Ethernet JTAG ...\n");
            p6i_eth_on_jtag_init();
	        
            p6_sdhci0_device.dev.platform_data = &p6i_mmc_wifi_platform_data;
            
            platform_add_devices( p6i_jpsumo_A1_devices, ARRAY_SIZE(p6i_jpsumo_A1_devices) );
            
            break;
        
        case JPSUMO_HW_A2:
            printk("=> JPSUMO_HW_A2\n");
            parrot_init_pin(pins_init_p6i_jpsumo_A2);
            
	        // Turn Off Jump H-Bridge
            gpio_direction_input(JS_HSIS_A2__GPIO_JUMP_CTRL_1);
        	gpio_direction_input(JS_HSIS_A2__GPIO_JUMP_CTRL_2);
            
            // JS : Turn Off PWM Generator
            gpio_direction_output(JS_HSIS_A2__GPIO_PWMGEN_nOE, 1);
            
            // Turn Off LED IR:
            gpio_direction_output(JS_HSIS_A2__GPIO_LED_IR, 0);
            
            gpio_direction_input(JS_HSIS_A2__GPIO_CHARGE_STATUS);
            
            gpio_direction_output(JS_HSIS_A2__GPIO_nSLEEP_WHEEL, 0);
            gpio_direction_input(JS_HSIS_A2__GPIO_nFAULT_WHEEL);
            
            /* Interrupt pin for pal_gpio_irq */
            gpio_direction_input(JS_HSIS_A2__GPIO_INT_INERT);
            gpio_interrupt_register(JS_HSIS_A2__GPIO_INT_INERT, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
            
            gpio_direction_output(JS_HSIS_A2__GPIO_nAMP_PWDN, 0); /* Speaker amplifier */

            /* INIT AAI */
            init_aai(aai_clock_mode);
            
            // I2C :
            p6_i2cm0_device.dev.platform_data = &i2cm_platform_js_i2c0;
            p6i_set_pads_i2c(0);
            p6i_set_i2c_drive_strength(0, 0x0);
	        
            printk("Initializing Ethernet JTAG ...\n");
            p6i_eth_on_jtag_init();
            
            platform_add_devices( p6i_jpsumo_A2plus_devices, ARRAY_SIZE(p6i_jpsumo_A2plus_devices));
            
            break;
        
        case JPSUMO_HW_A3:
        case JPSUMO_HW_B1:
        case JPSUMO_HW_B2:
        case JPSUMO_HW_07:
        default:
            switch ( jpsumo_hw_version )
            {
                case JPSUMO_HW_A3:
                    printk("=> JPSUMO_HW_A3 (HW3)\n");
                    break;
                case JPSUMO_HW_B1:
                    printk("=> JPSUMO_HW_B1 (HW4)\n");
                    break;
                case JPSUMO_HW_B2:
                    printk("=> JPSUMO_HW_B2 (HW5/HW6)\n");
                    break;
                case JPSUMO_HW_07:
                    printk("=> JPSUMO_HW_07\n");
                    break;
                default:
                    printk("=> WARNING : Unknow HW (0x%X), do as if it was last known HW...\n", jpsumo_hw_version);
                    break;
            }
            parrot_init_pin(pins_init_p6i_jpsumo_A3);

            // Pull Up on UART1 RX:
            p6i_set_pads_uart1_pullup();

	        // Turn Off Jump H-Bridge
            gpio_direction_input(JS_HSIS_A3__GPIO_JUMP_CTRL_1);
        	gpio_direction_input(JS_HSIS_A3__GPIO_JUMP_CTRL_2);
            
            // JS : Turn Off RED LEDs
            gpio_direction_output(JS_HSIS_A3__GPIO_LEFT_EYE_LED, 1);
            gpio_direction_output(JS_HSIS_A3__GPIO_RIGHT_EYE_LED, 1);
            
            // JS : Turn Off PWM Generator
            gpio_direction_output(JS_HSIS_A3__GPIO_PWMGEN_nOE, 1);
            
            // Turn Off LED IR:
            gpio_direction_output(JS_HSIS_A3__GPIO_LED_IR, 0);
            
            gpio_direction_input(JS_HSIS_A3__GPIO_CHARGE_STATUS);
            
            gpio_direction_output(JS_HSIS_A3__GPIO_nSLEEP_WHEEL, 0);
            gpio_direction_input(JS_HSIS_A3__GPIO_nFAULT_WHEEL);
            
            /* Interrupt pin for pal_gpio_irq */
            gpio_direction_input(JS_HSIS_A3__GPIO_INT_INERT);
            gpio_interrupt_register(JS_HSIS_A3__GPIO_INT_INERT, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
            
            gpio_direction_output(JS_HSIS_A3__GPIO_nAMP_PWDN, 0); /* Speaker amplifier */
			
            /* INIT AAI */
            init_aai(aai_clock_mode);
            
            // I2C :
            p6_i2cm0_device.dev.platform_data = &i2cm_platform_js_i2c0;
            p6i_set_pads_i2c(0);
            p6i_set_i2c_drive_strength(0, 0x0);
	        
            printk("Initializing Ethernet JTAG ...\n");
            p6i_eth_on_jtag_init();
            
            platform_add_devices(p6i_jpsumo_A2plus_devices, ARRAY_SIZE(p6i_jpsumo_A2plus_devices));
            
            break;
        
    }
    
}

MACHINE_START(PARROT_JPSUMO, "Jumping sumo sip6 board")
    /* Maintainer: Parrot S.A. */
    .phys_io    = PARROT6_UART0,
    .io_pg_offst    = (PARROT6_VA_UART0 >> 18) & 0xfffc,
    .boot_params    = PARROT6_DDR_BASE+0x100,
    .map_io     = p6i_map_io,
    .init_irq   = p6_init_irq,
    .timer      = &p6_timer,
    .init_machine   = p6idev_init,
MACHINE_END
