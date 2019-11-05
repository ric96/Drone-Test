#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include <console/console.h>


#define PORTD	"GPIOD"
#define PORTE   "GPIOE"
#define ESC1_PWR    2  //PD2
#define ESC2_PWR    15 //PD15
#define ESC3_PWR    14 //PD14
#define ESC4_PWR    15 //PE15

#define PERIOD (USEC_PER_SEC / 50U)

/* all in micro second */
#define PULSEWIDTH 2000


/* 1000 msec = 1 sec */
#define SLEEP_TIME	1000

/*
OLED & MPU: I2C2
SDA PC12
SCL PB10

BT UART: UART3
PD8 TX
PD9 RX
*/

void main(void)
{
	console_init();

	u32_t cnt = 0;
	struct device *devd;
	struct device *deve;

	struct device *pwm_dev3;
	struct device *pwm_dev9;

	u32_t pulse_width = PULSEWIDTH;
	u8_t dir = 0U;

	devd = device_get_binding(PORTD);
	deve = device_get_binding(PORTE);

	pwm_dev3 = device_get_binding("PWM_3");
    pwm_dev9 = device_get_binding("PWM_9");

	/* Set LED pin as output */
	gpio_pin_configure(devd, ESC1_PWR, GPIO_DIR_OUT);
	gpio_pin_configure(devd, ESC2_PWR, GPIO_DIR_OUT);
	gpio_pin_configure(devd, ESC3_PWR, GPIO_DIR_OUT);
	gpio_pin_configure(deve, ESC4_PWR, GPIO_DIR_OUT);
	u8_t c;

/*
	gpio_pin_write(devd, ESC1_PWR, 0);
	gpio_pin_write(devd, ESC2_PWR, 0);
	gpio_pin_write(devd, ESC3_PWR, 0);
	gpio_pin_write(deve, ESC4_PWR, 0);

	printk("Start Cal?");

	u8_t c = console_getchar();
	console_putchar(c);
	while (c != '\r');

	printk("Starting Cal: \n");

	pwm_pin_set_usec(pwm_dev3, 1, PERIOD, 2000);
	pwm_pin_set_usec(pwm_dev3, 3, PERIOD, 2000);
	pwm_pin_set_usec(pwm_dev9, 1, PERIOD, 2000);
	pwm_pin_set_usec(pwm_dev9, 2, PERIOD, 2000);

	printk("All PWM Values set to 2ms \n");
	
	printk("Calibrate Rotor 1? \n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	printk("Powering ON Rotor 1 and waiting for tone \n");
	gpio_pin_write(devd, ESC1_PWR, 1);
	printk("High tone detected? \n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	printk("Setting pwm1 to 1ms \n");
	pwm_pin_set_usec(pwm_dev3, 1, PERIOD, 1000);
	printk("Power down rotor 1? \n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	gpio_pin_write(devd, ESC1_PWR, 0);
	printk("rotor 1 caliberated and off. \n");



	printk("Calibrate Rotor 2? \n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	printk("Powering ON Rotor 2 and waiting for tone");
	gpio_pin_write(devd, ESC2_PWR, 1);
	printk("High tone detected? \n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	printk("Setting pwm2 to 1ms \n");
	pwm_pin_set_usec(pwm_dev3, 3, PERIOD, 1000);
	printk("Power down rotor 2 \n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	gpio_pin_write(devd, ESC2_PWR, 0);
	printk("rotor 2 caliberated and off. \n");



	printk("Calibrate Rotor 3? \n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	printk("Powering ON Rotor 3 and waiting for tone");
	gpio_pin_write(devd, ESC3_PWR, 1);
	printk("High tone detected? \n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	printk("Setting pwm3 to 1ms \n");
	pwm_pin_set_usec(pwm_dev9, 1, PERIOD, 1000);
	printk("Power down rotor 3? \n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	gpio_pin_write(devd, ESC3_PWR, 0);
	printk("rotor 3 caliberated and off. \n");



	printk("Calibrate Rotor 4? \n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	printk("Powering ON Rotor 4 and waiting for tone");
	gpio_pin_write(deve, ESC4_PWR, 1);
	printk("High tone detected? \n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	printk("Setting pwm4 to 1ms \n");
	pwm_pin_set_usec(pwm_dev9, 2, PERIOD, 1000);
	printk("Power down rotor 4? \n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	gpio_pin_write(deve, ESC4_PWR, 0);
	printk("rotor 4 caliberated and off. \n");

*/

	printk("start test spinup ?\n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	pwm_pin_set_usec(pwm_dev3, 1, PERIOD, 1000);
	pwm_pin_set_usec(pwm_dev3, 3, PERIOD, 1000);
	pwm_pin_set_usec(pwm_dev9, 1, PERIOD, 1000);
	pwm_pin_set_usec(pwm_dev9, 2, PERIOD, 1000);
	gpio_pin_write(devd, ESC1_PWR, 1);
	gpio_pin_write(devd, ESC2_PWR, 1);
	gpio_pin_write(devd, ESC3_PWR, 1);
	gpio_pin_write(deve, ESC4_PWR, 1);
	printk("All ESCs ON, strart spin? L1\n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	pwm_pin_set_usec(pwm_dev3, 1, PERIOD, 1100);
	pwm_pin_set_usec(pwm_dev3, 3, PERIOD, 1100);
	pwm_pin_set_usec(pwm_dev9, 1, PERIOD, 1100);
	pwm_pin_set_usec(pwm_dev9, 2, PERIOD, 1100);
	printk("All ESCs ON, strart spin? L2\n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	pwm_pin_set_usec(pwm_dev3, 1, PERIOD, 1200);
	pwm_pin_set_usec(pwm_dev3, 3, PERIOD, 1200);
	pwm_pin_set_usec(pwm_dev9, 1, PERIOD, 1200);
	pwm_pin_set_usec(pwm_dev9, 2, PERIOD, 1200);
	printk("All ESCs ON, strart spin? L3\n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	pwm_pin_set_usec(pwm_dev3, 1, PERIOD, 1500);
	pwm_pin_set_usec(pwm_dev3, 3, PERIOD, 1500);
	pwm_pin_set_usec(pwm_dev9, 1, PERIOD, 1500);
	pwm_pin_set_usec(pwm_dev9, 2, PERIOD, 1500);
	printk("All ESCs ON, strart spin? L4\n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	pwm_pin_set_usec(pwm_dev3, 1, PERIOD, 2000);
	pwm_pin_set_usec(pwm_dev3, 3, PERIOD, 2000);
	pwm_pin_set_usec(pwm_dev9, 1, PERIOD, 2000);
	pwm_pin_set_usec(pwm_dev9, 2, PERIOD, 2000);
	printk("Spin OK, shut down? \n");
	c = console_getchar();
	console_putchar(c);
	while (c != '\r');
	pwm_pin_set_usec(pwm_dev3, 1, PERIOD, 1000);
	pwm_pin_set_usec(pwm_dev3, 3, PERIOD, 1000);
	pwm_pin_set_usec(pwm_dev9, 1, PERIOD, 1000);
	pwm_pin_set_usec(pwm_dev9, 2, PERIOD, 1000);


}
