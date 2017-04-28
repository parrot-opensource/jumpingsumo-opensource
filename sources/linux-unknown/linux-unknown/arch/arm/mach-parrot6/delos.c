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
#include <linux/clk.h>

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

#include <media/soc_camera.h>
#include <media/soc_camera_platform.h>

#include "timer.h"
#include "devs.h"
#include "pm.h"
#include "parrot6.h"
#include "parrot6i.h"

#include <mach/i2c.h>

#include <mach/delos.h>


#include <mach/regs-rtc-p6i.h>

static delos_hw_version_t delos_hw_version = -1;


static unsigned int pins_init_p6i[] = {
    /* uart */
    P6I_UART1_DEFAULT,
    P6I_I2CM0_DEFAULT,
    P6I_GPIO_005, /* activate bluetooth chip */
    DELOS_HSIS__GPIOPIN_HRESET_CAM, /* Enable (1) or Disable (0) USB Camera */
    DELOS_HSIS__GPIOPIN_US_POWER, /* When US_POWER is set, amplitude of ultrasound "pwm" moves from 3.3V to 0.8V */
    DELOS_HSIS__GPIOPIN_INT_MAGNETO, /* Input data ready signal of magnetometer ak8963 */
    P6I_SPI1_MOSI, /* Init MOSI spi pin for ultrasound driver */
    P6I_USB_PWR_ON, /* usb pwr on */
    /* PWM motors */
    P6I_PWM_00a,
    P6I_PWM_01a,
    P6I_PWM_02b,
    P6I_PWM_03a,
    0,
};

static unsigned int pins_init_p6i_hw00[] = {
    DELOS_HSIS__GPIOPIN_OFF, /* When OFF is set, board is off */
    DELOS_HSIS__GPIOPIN_INT_INERT, /* Input data ready signal of sensor mpu6050 */
    DELOS_HSIS__GPIOPIN_BUTTON_HW00, /* Input Button to monitor : 0 when this button is released; 1 when someone presses it */
    DELOS_HSIS__GPIOPIN_RED_LED_HW00, /* Red led gpio */
    DELOS_HSIS__GPIOPIN_BLUE_LED, /* Green led gpio */
    DELOS_HSIS__GPIOPIN_MOTOR_FAULT, /* Motors are "blocked" when input MOTOR_FAULT is set */
    DELOS_HSIS__GPIOPIN_MOTOR_ENABLE, /* When MOTOR_ENABLE is cleared, MOTOR_FAULT is cleared */
    DELOS_HSIS__GPIOPIN_VBUS, /*When delos is connected with usb interface, vbus is set*/
    0,
};

static unsigned int pins_init_p6i_hw01[] = {
    DELOS_HSIS__GPIOPIN_OFF, /* When OFF is set, board is off */
    DELOS_HSIS__GPIOPIN_INT_INERT, /* Input data ready signal of sensor mpu6050 */
    DELOS_HSIS__GPIOPIN_BLUE_LED2_HW01, /* Red led gpio */
    DELOS_HSIS__GPIOPIN_BUTTON_HW01, /* Input Button to monitor : 0 when this button is released; 1 when someone presses it */
    DELOS_HSIS__GPIOPIN_BLUE_LED, /* Green led gpio */
    DELOS_HSIS__GPIOPIN_RESET_MOTOR, /* Enable cut detection, before setting RESET_MOTOR, MOTOR_FAULT must be cleared (by clearing MOTOR_ENABLE) */
    DELOS_HSIS__GPIOPIN_MOTOR_FAULT, /* Motors are "blocked" when input MOTOR_FAULT is set */
    DELOS_HSIS__GPIOPIN_MOTOR_ENABLE, /* When MOTOR_ENABLE is cleared, MOTOR_FAULT is cleared */
    DELOS_HSIS__GPIOPIN_VBUS, /*When delos is connected with usb interface, vbus is set*/
    0,
};

static unsigned int pins_init_p6i_hw02[] = {
    DELOS_HSIS__GPIOPIN_OFF, /* When OFF is set, board is off */
    DELOS_HSIS__GPIOPIN_INT_INERT, /* Input data ready signal of sensor mpu6050 */
    DELOS_HSIS__GPIOPIN_BUTTON_HW01, /* Input Button to monitor : 0 when this button is released; 1 when someone presses it */
    DELOS_HSIS__GPIOPIN_RESET_MOTOR, /* Enable cut detection, before setting RESET_MOTOR, MOTOR_FAULT must be cleared (by clearing MOTOR_ENABLE) */
    DELOS_HSIS__GPIOPIN_MOTOR_FAULT, /* Motors are "blocked" when input MOTOR_FAULT is set */
    DELOS_HSIS__GPIOPIN_MOTOR_ENABLE, /* When MOTOR_ENABLE is cleared, MOTOR_FAULT is cleared */
    DELOS_HSIS__GPIOPIN_RED_LED_L_HW02,  /* Red left led gpio */
    DELOS_HSIS__GPIOPIN_GREEN_LED_L_HW02, /* Green left led gpio */
    DELOS_HSIS__GPIOPIN_GREEN_LED_R_HW02, /* Green right led gpio */
    DELOS_HSIS__GPIOPIN_RED_LED_R_HW02, /* Red right led gpio */
    DELOS_HSIS__GPIOPIN_VBUS, /*When delos is connected with usb interface, vbus is set*/
    0,
};


static unsigned int pins_init_p6i_hw03[] = {
    DELOS_HSIS__GPIOPIN_ONOFF_HW03, /* Control of power device */
    DELOS_HSIS__GPIOPIN_BUTTON_HW03, /* Input Button to monitor : 0 when this button is released; 1 when someone presses it */
    DELOS_HSIS__GPIOPIN_MOTOR_FAULT_HW03, /* Motors are "blocked" when output MOTOR_FAULT is set */
    DELOS_HSIS__GPIOPIN_RED_LED_L_HW03, /* Red left led gpio */
    DELOS_HSIS__GPIOPIN_GREEN_LED_R_HW03, /* Green right led gpio */
    DELOS_HSIS__GPIOPIN_GREEN_LED_L_HW03, /* Green left led gpio */
    DELOS_HSIS__GPIOPIN_RED_LED_R_HW03, /* Red right led gpio */
    DELOS_HSIS__GPIOPIN_INT_INERT_HW03, /* Input data ready signal of sensor mpu6050 */
    DELOS_HSIS__GPIOPIN_VBUS_HW03, /*When delos is connected with usb interface, vbus is set*/
    0,
};

static unsigned int pins_init_p6i_hw05[] = {
    DELOS_HSIS__GPIOPIN_ONOFF_HW05, /*Control of power device */
    DELOS_HSIS__GPIOPIN_BUTTON_HW03, /* Input Button to monitor : 0 when this button is released; 1 when someone presses it */
    DELOS_HSIS__GPIOPIN_MOTOR_FAULT_HW03, /* Motors are "blocked" when output MOTOR_FAULT is set */
    DELOS_HSIS__GPIOPIN_RED_LED_L_HW03, /* Red left led gpio */
    DELOS_HSIS__GPIOPIN_GREEN_LED_R_HW03, /* Green right led gpio */
    DELOS_HSIS__GPIOPIN_GREEN_LED_L_HW03, /* Green left led gpio */
    DELOS_HSIS__GPIOPIN_RED_LED_R_HW03, /* Red right led gpio */
    DELOS_HSIS__GPIOPIN_INT_INERT_HW03, /* Input data ready signal of sensor mpu6050 */
    DELOS_HSIS__GPIOPIN_VBUS_HW05, /*When delos is connected with usb interface, vbus is set*/
    0,
};

static unsigned int pins_init_p6i_hw07[] = {
    DELOS_HSIS__GPIOPIN_ONOFF_HW07, /*Control of power device */
    DELOS_HSIS__GPIOPIN_BUTTON_HW07, /* Input Button to monitor : 0 when this button is released; 1 when someone presses it */
    DELOS_HSIS__GPIOPIN_MOTOR_FAULT_HW07, /* Motors are "blocked" when output MOTOR_FAULT is set */
    DELOS_HSIS__GPIOPIN_RED_LED_L_HW07, /* Red left led gpio */
    DELOS_HSIS__GPIOPIN_GREEN_LED_R_HW07, /* Green right led gpio */
    DELOS_HSIS__GPIOPIN_GREEN_LED_L_HW07, /* Green left led gpio */
    DELOS_HSIS__GPIOPIN_RED_LED_R_HW07, /* Red right led gpio */
    DELOS_HSIS__GPIOPIN_INT_INERT_HW07, /* Input data ready signal of sensor mpu6050 */
    DELOS_HSIS__GPIOPIN_VBUS_HW07, /*When delos is connected with usb interface, vbus is set*/
    0,
};


static unsigned int pins_reboot_p6i[] = {
    P6I_RTC_WKUP_1,
    P6I_RTC_WKUP_2,
    P6I_RTC_OUT_1,
    0,
};

static int delos_rst_notify_sys(struct notifier_block *this,
        unsigned long code, void *unused)
{
    parrot_init_pin(pins_reboot_p6i);
    return 0;
}

static struct notifier_block delos_rst_notifier = {
    .notifier_call = delos_rst_notify_sys,
};

static struct platform_device *p6i_devices[] __initdata = {
    &p6_uart0_device,/* BT*/
    &p6_uart1_device, /* uart console */
    &p6_nand_device,
    &p6_i2cm0_device, /* i2c for reading sensor datas and for configuring camera */
    &p6i_usb0_device,
    &p6_aai_device,
    &p6_us_device,
    &p6_dmac_device,
    &p6_gpio,
};


typedef struct { unsigned int val[4]; } __attribute__ ((packed)) pwm_delos_quadruplet;
void delos_set_raw_ctrl_reg(unsigned int reg);
void delos_set_raw_ratios(pwm_delos_quadruplet* ratios);


static struct parrot5_i2cm_platform i2cm_platform_delos = {
    /*i2c frequency at 400kHz*/
    .bus_freq	= 400*1000,
    .retries        = 1,
};

/*
 * Disable motors using MOTOR_FAULT signal
 * This function doesn't reset pwm registers
 * Parameter gpio_status must be passed in order to know after execution of this function if motors are already disabled (gpio_status = 1)
 * @param gpio_status : address of variable gpio_status
 */
void delos_motor_disable(unsigned int* gpio_status)
{
	switch(delos_hw_version)
	{
        case DELOS_HW_02 :
        case DELOS_HW_01 :

		*gpio_status = gpio_get_value(DELOS_HSIS__GPIO_MOTOR_FAULT);
		gpio_set_value(DELOS_HSIS__GPIO_MOTOR_ENABLE, 1); /* CLR# */
		gpio_set_value(DELOS_HSIS__GPIO_RESET_MOTOR, 0); /* PR# */

		break;

        case DELOS_HW_00 :
        case DELOS_HW_03 :
        case DELOS_HW_04 :
        case DELOS_HW_05 :
        case DELOS_HW_07 :

		*gpio_status = gpio_get_value(DELOS_HSIS__GPIO_MOTOR_FAULT_HW03);
		gpio_set_value(DELOS_HSIS__GPIO_MOTOR_FAULT_HW03, 1); /* CLR# */

		break;
	}
}
EXPORT_SYMBOL(delos_motor_disable);


/*
 * Enable Delos motors if gpio_status is cleared
 * @param gpio_status : parameter gpio_status
 */
void delos_motor_enable(unsigned int gpio_status)
{
	switch(delos_hw_version)
	{
        case DELOS_HW_02 :
        case DELOS_HW_01 :

		/* If gpio_status is cleared, i.e. motors are not disabled
		 * before using delos_motor_disable(), we have to "reload" them */
		if(!gpio_status)
		{
			/* After this line, MOTOR_FAULT is cleared*/
			gpio_set_value(DELOS_HSIS__GPIO_MOTOR_ENABLE, 0); /* CLR# */
		}
		break;

        case DELOS_HW_00 :
        case DELOS_HW_03 :
        case DELOS_HW_04 :
        case DELOS_HW_05 :
        case DELOS_HW_07 :

		/* If gpio_status is cleared, i.e. motors are not disabled before
		 * using delos_motor_disable(), we have to "reload" them */
		if(!gpio_status)
		{
			/* After this line, MOTOR_FAULT is cleared*/
			gpio_set_value(DELOS_HSIS__GPIO_MOTOR_FAULT_HW03, 0); /* CLR# */
		}
		break;
	}
}
EXPORT_SYMBOL(delos_motor_enable);


/*
 * This function reset the P6_PWM_RATIOxx registers
 */
static void delos_reset_pwms(void)
{
	char __iomem *pwm_regbase;
	const int pwm_ctrl_enable_all = 0x0f;
	unsigned int ntimer;

	struct clk *pwm_clk;
	pwm_clk = clk_get(NULL, "pwm");
	clk_enable(pwm_clk);

	pwm_regbase = ioremap(PARROT6_PWM, 256);

	/* Force all ratios to zero */
	for (ntimer=0;ntimer<4;ntimer++)
	{
		__raw_writel(0, pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
	}
	/* Activate the PWM output to generate the 0 output */
	__raw_writel(pwm_ctrl_enable_all, pwm_regbase + P6_PWM_CTL);

	clk_disable(pwm_clk);

	iounmap(pwm_clk);
}




static void __init p6idev_init(void)
{

	u32 tmp;
	/*
	 * MOTOR_FAULT must be set before enabling pins on pwm mode
	 */
	gpio_direction_output(DELOS_HSIS__GPIO_MOTOR_ENABLE, 1);
	/*
	 *  Identical to :
	 *  gpio_direction_output(DELOS_HSIS__GPIO_MOTOR_FAULT_HW03, 1);
	 *  for hardware 03 and superior versions
	 */


	sip6_init(1);
	p6_i2cm0_device.dev.platform_data = &i2cm_platform_delos;


	switch(parrot_board_rev())
	{
        case 0x0 :
		printk("hardware version 00 detected\n");
		delos_hw_version = DELOS_HW_00;
		break;

        case 0x4 :
		printk("hardware version 01 detected\n");
		delos_hw_version = DELOS_HW_01;
		break;

        case 0x2 :
		printk("hardware version 02 detected\n");
		delos_hw_version = DELOS_HW_02;
		break;

        case 0x6 :
		printk("hardware version 03 detected\n");
		delos_hw_version = DELOS_HW_03;
		break;

        case 0x1 :
		printk("hardware version 04 detected\n");
		delos_hw_version = DELOS_HW_04;
		break;

        default :
		printk("WARNING : unknown and/or not supported board version\n");
		printk("value of pcb version : %x\n", parrot_board_rev());
		printk("We're assuming that hardware version of this board is similar to the last hardware version managed by this bsp\n");
		/* There is no break, it's normal... */
        case 0x05:
		printk("hardware version 05 detected\n");
		delos_hw_version = DELOS_HW_05;
		break;

        case 0x07:
		printk("hardware version 07 detected\n");
		delos_hw_version = DELOS_HW_07;
		break;

	}

	/* Reset pwms before using pins as pwms */
	delos_reset_pwms();

	parrot_init_pin(pins_init_p6i);

	/* pads init */
	p6i_set_pads_i2c(0);

	p6i_eth_on_jtag_init();
	register_reboot_notifier(&delos_rst_notifier);


	printk("Initializing USB activation pin\n");
	/* USB Camera is disabled */
	gpio_direction_output(DELOS_HSIS__GPIO_HRESET_CAM, 0);




	switch(delos_hw_version)
	{
        case DELOS_HW_00:
		parrot_init_pin(pins_init_p6i_hw00);

		/* leds */
		/* Red light on */
		gpio_direction_output(DELOS_HSIS__GPIO_RED_LED_HW00, 1);
		/* Blue light off */
		gpio_direction_output(DELOS_HSIS__GPIO_BLUE_LED, 0);

		/* We have to monitor the interrupt pin of mpu6050 which raises each 5 ms */
		gpio_direction_input(DELOS_HSIS__GPIO_INT_INERT);
		gpio_interrupt_register(DELOS_HSIS__GPIO_INT_INERT, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

		/* Detection of plug/unplug usb */
		gpio_direction_input(DELOS_HSIS__GPIO_VBUS);
		gpio_interrupt_register(DELOS_HSIS__GPIO_VBUS, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

		break;

        case DELOS_HW_01:
		parrot_init_pin(pins_init_p6i_hw01);

		/* Initialisation for detection of press button */
		gpio_direction_input(DELOS_HSIS__GPIO_BUTTON_HW01);
		gpio_interrupt_register(DELOS_HSIS__GPIO_BUTTON_HW01, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

		/* leds */
		/* Blue light right on */
		gpio_direction_output(DELOS_HSIS__GPIO_BLUE_LED2_HW01, 1);
		/* Blue light left off */
		gpio_direction_output(DELOS_HSIS__GPIO_BLUE_LED, 0);

		/* Reset MOTOR_FAULT */
		gpio_direction_output(DELOS_HSIS__GPIO_RESET_MOTOR, 0);

		/* cut-out detection */
		gpio_direction_input(DELOS_HSIS__GPIO_MOTOR_FAULT);

		/* We have to monitor the interrupt pin of mpu6050 which raises each 5 ms */
		gpio_direction_input(DELOS_HSIS__GPIO_INT_INERT);
		gpio_interrupt_register(DELOS_HSIS__GPIO_INT_INERT, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

		/* Detection of plug/unplug usb */
		gpio_direction_input(DELOS_HSIS__GPIO_VBUS);
		gpio_interrupt_register(DELOS_HSIS__GPIO_VBUS, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

		break;

        case DELOS_HW_02:
		parrot_init_pin(pins_init_p6i_hw02);

		/* Initialisation for detection of press button */
		gpio_direction_input(DELOS_HSIS__GPIO_BUTTON_HW01);
		gpio_interrupt_register(DELOS_HSIS__GPIO_BUTTON_HW01, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

		/* leds */
		/* "Eyes" of Delos are red at boot */
		gpio_direction_output(DELOS_HSIS__GPIO_RED_LED_L_HW02, 1);
		gpio_direction_output(DELOS_HSIS__GPIO_GREEN_LED_L_HW02, 0);
		gpio_direction_output(DELOS_HSIS__GPIO_GREEN_LED_R_HW02, 0);
		gpio_direction_output(DELOS_HSIS__GPIO_RED_LED_R_HW02, 1);

		/* Reset MOTOR_FAULT */
		gpio_direction_output(DELOS_HSIS__GPIO_RESET_MOTOR, 0);

		/* cut-out detection */
		gpio_direction_input(DELOS_HSIS__GPIO_MOTOR_FAULT);

		/* We have to monitor the interrupt pin of mpu6050 which raises each 5 ms */
		gpio_direction_input(DELOS_HSIS__GPIO_INT_INERT);
		gpio_interrupt_register(DELOS_HSIS__GPIO_INT_INERT, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

		/* Detection of plug/unplug usb */
		gpio_direction_input(DELOS_HSIS__GPIO_VBUS);
		gpio_interrupt_register(DELOS_HSIS__GPIO_VBUS, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

		break;

        case DELOS_HW_03:
		parrot_init_pin(pins_init_p6i_hw03);

		/* Set gpio2 (button) in high impendance */
		tmp = __raw_readl(PARROT6I_VA_RTC+_RTC_PULLS) & ~(0xff07);
		__raw_writel(tmp, PARROT6I_VA_RTC+_RTC_PULLS);

		/* Takes alive power device by setting this gpio */
		gpio_direction_output(DELOS_HSIS__GPIO_ONOFF_HW03, 1);

		/* Detection of press button */
		gpio_direction_input(DELOS_HSIS__GPIO_BUTTON_HW03);
		gpio_interrupt_register(DELOS_HSIS__GPIO_BUTTON_HW03, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

		/* leds */
		/* "Eyes" of Delos are red at boot */
		gpio_direction_output(DELOS_HSIS__GPIO_RED_LED_L_HW03, 1);
		gpio_direction_output(DELOS_HSIS__GPIO_GREEN_LED_L_HW03, 0);
		gpio_direction_output(DELOS_HSIS__GPIO_GREEN_LED_R_HW03, 0);
		gpio_direction_output(DELOS_HSIS__GPIO_RED_LED_R_HW03, 1);

		/* We have to monitor the interrupt pin of mpu6050 which raises each 5 ms */
		gpio_direction_input(DELOS_HSIS__GPIO_INT_INERT_HW03);
		gpio_interrupt_register(DELOS_HSIS__GPIO_INT_INERT_HW03, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

		/* Detection of plug/unplug usb */
		gpio_direction_input(DELOS_HSIS__GPIO_VBUS_HW03);
		gpio_interrupt_register(DELOS_HSIS__GPIO_VBUS_HW03, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
		break;

        case DELOS_HW_04:
		parrot_init_pin(pins_init_p6i_hw03);

		/* Set gpio2 (button) in high impendance */
		tmp = __raw_readl(PARROT6I_VA_RTC+_RTC_PULLS) & ~(0xff07);
		__raw_writel(tmp, PARROT6I_VA_RTC+_RTC_PULLS);

		/* Takes alive power device by REsetting this gpio */
		gpio_direction_output(DELOS_HSIS__GPIO_ONOFF_HW03, 0);

		/* Detection of press button */
		gpio_direction_input(DELOS_HSIS__GPIO_BUTTON_HW03);
		gpio_interrupt_register(DELOS_HSIS__GPIO_BUTTON_HW03, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

		/* leds */
		/* "Eyes" of Delos are red at boot */
		gpio_direction_output(DELOS_HSIS__GPIO_RED_LED_L_HW03, 1);
		gpio_direction_output(DELOS_HSIS__GPIO_GREEN_LED_L_HW03, 0);
		gpio_direction_output(DELOS_HSIS__GPIO_GREEN_LED_R_HW03, 0);
		gpio_direction_output(DELOS_HSIS__GPIO_RED_LED_R_HW03, 1);

		/* We have to monitor the interrupt pin of mpu6050 which raises each 5 ms */
		gpio_direction_input(DELOS_HSIS__GPIO_INT_INERT_HW03);
		gpio_interrupt_register(DELOS_HSIS__GPIO_INT_INERT_HW03, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

		/* Detection of plug/unplug usb */
		gpio_direction_input(DELOS_HSIS__GPIO_VBUS_HW03);
		gpio_interrupt_register(DELOS_HSIS__GPIO_VBUS_HW03, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
		break;

        case DELOS_HW_05:
		parrot_init_pin(pins_init_p6i_hw05);

		/* Set gpio2 (button) in high impendance */
		tmp = __raw_readl(PARROT6I_VA_RTC+_RTC_PULLS) & ~(0xff07);
		__raw_writel(tmp, PARROT6I_VA_RTC+_RTC_PULLS);

		/* Takes alive power device by REsetting this gpio */
		gpio_direction_output(DELOS_HSIS__GPIO_ONOFF_HW05, 0); //Takes alive power device by resetting this gpio

		/* Detection of press button */
		gpio_direction_input(DELOS_HSIS__GPIO_BUTTON_HW03);
		gpio_interrupt_register(DELOS_HSIS__GPIO_BUTTON_HW03, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

		/* leds */
		/* "Eyes" of Delos are red at boot */
		gpio_direction_output(DELOS_HSIS__GPIO_RED_LED_L_HW03, 1);
		gpio_direction_output(DELOS_HSIS__GPIO_GREEN_LED_L_HW03, 0);
		gpio_direction_output(DELOS_HSIS__GPIO_GREEN_LED_R_HW03, 0);
		gpio_direction_output(DELOS_HSIS__GPIO_RED_LED_R_HW03, 1);

		/* We have to monitor the interrupt pin of mpu6050 which raises each 5 ms */
		gpio_direction_input(DELOS_HSIS__GPIO_INT_INERT_HW03);
		gpio_interrupt_register(DELOS_HSIS__GPIO_INT_INERT_HW03, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

		/* Detection of plug/unplug usb */
		gpio_direction_input(DELOS_HSIS__GPIO_VBUS_HW05);
		gpio_interrupt_register(DELOS_HSIS__GPIO_VBUS_HW05, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
		break;

        case DELOS_HW_07:
		parrot_init_pin(pins_init_p6i_hw07);

		/* Set gpio2 (button) in high impendance */
		tmp = __raw_readl(PARROT6I_VA_RTC+_RTC_PULLS) & ~(0xff07);
		__raw_writel(tmp, PARROT6I_VA_RTC+_RTC_PULLS);

		/* Takes alive power device by REsetting this gpio */
		gpio_direction_output(DELOS_HSIS__GPIO_ONOFF_HW07, 0); //Takes alive power device by resetting this gpio

		/* Detection of press button */
		gpio_direction_input(DELOS_HSIS__GPIO_BUTTON_HW07);
		gpio_interrupt_register(DELOS_HSIS__GPIO_BUTTON_HW07, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

		/* leds */
		/* "Eyes" of Delos are red at boot */
		gpio_direction_output(DELOS_HSIS__GPIO_RED_LED_L_HW07, 1);
		gpio_direction_output(DELOS_HSIS__GPIO_GREEN_LED_L_HW07, 0);
		gpio_direction_output(DELOS_HSIS__GPIO_GREEN_LED_R_HW07, 0);
		gpio_direction_output(DELOS_HSIS__GPIO_RED_LED_R_HW07, 1);

		/* We have to monitor the interrupt pin of mpu6050 which raises each 5 ms */
		gpio_direction_input(DELOS_HSIS__GPIO_INT_INERT_HW07);
		gpio_interrupt_register(DELOS_HSIS__GPIO_INT_INERT_HW07, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

		/* Detection of plug/unplug usb */
		gpio_direction_input(DELOS_HSIS__GPIO_VBUS_HW07);
		gpio_interrupt_register(DELOS_HSIS__GPIO_VBUS_HW07, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
		break;
	}

	platform_add_devices(p6i_devices, ARRAY_SIZE(p6i_devices));
}

MACHINE_START(PARROT_DELOS, "Delos sip6 board")
	/* Maintainer: Parrot S.A. */
	.phys_io    = PARROT6_UART0,
	.io_pg_offst    = (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params    = PARROT6_DDR_BASE+0x100,
	.map_io     = p6i_map_io,
	.init_irq   = p6_init_irq,
	.timer      = &p6_timer,
	.init_machine   = p6idev_init,
MACHINE_END

