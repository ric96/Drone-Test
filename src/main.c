#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include <console/console.h>
#include <string.h>

#define PORTD	"GPIOD"
#define PORTE   "GPIOE"

#define ESC1_PWR    2  //PD2
#define ESC2_PWR    15 //PD15
#define ESC3_PWR    14 //PD14
#define ESC4_PWR    15 //PE15

#define NUM_ESC 4

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

struct esc {
	struct device *pwr_dev;
	struct device *pwm_dev;
	char pwr_port[5];
	uint8_t pwr_pin;
	char pwm_port[5];
	uint8_t pwm_channel
	uint8_t pwm_width = 0;
}

int esc_init(const struct esc* esc_c)
{
  esc_c.pwr_dev=device_get_binding(esc_c.pwr_port);
  gpio_pin_configure(esc_c.pwr_port, esc_c.pwr_pin, GPIO_DIR_OUT);
  esc_c.pwm_dev=device_get_binding(esc_c.pwm_port);
  gpio_pin_write(esc_c.pwr_dev, esc_c.pwr_pin, 0);
  return 0;
}

void rotor_pwm_thread(const struct esc esc_arr)
{
	while(1)
	{
		for(uint8_t i=0; i<NUM_ESC; i++)
		{
			pwm_pin_set_usec(esc_arr[i].pwm_dev, esc_arr[i].pwm_channel, PERIOD, esc_arr[i].pwm_width);
		}
	}
}


char controller_input()
{
	u8_t c;
	c = console_getchar();
	console_putchar(c);
	return c;
}

void esc_startup_thread(const struct esc esc_arr)
{
	printk("Ready to initialize drone? \n");
	while(controller_input() != 'y')
	for(uint8_t i=0; i<NUM_ESC; i++)
	{
		printk("Powering On ESC %d \n", i+1);
		esc_arr[i].pwm_width=1000;
		gpio_pin_write(esc_c.pwr_dev, esc_c.pwr_pin, 1);
		printk("ESC %d tone OK? \n", i+1);
		while(controller_input() != 'y')
	}
}

void main(void)
{
	console_init();

	struct esc *esc_s[3];
	esc_s[0] = {.pwr_pin = ESC1_PWR, .pwr_port = PORTD, .pwm_port = "PWM3", .pwm_channel = 1};
	esc_s[1] = {.pwr_pin = ESC2_PWR, .pwr_port = PORTD, .pwm_port = "PWM3", .pwm_channel = 3};
	esc_s[2] = {.pwr_pin = ESC3_PWR, .pwr_port = PORTD, .pwm_port = "PWM9", .pwm_channel = 1};
	esc_s[3] = {.pwr_pin = ESC4_PWR, .pwr_port = PORTE, .pwm_port = "PWM9", .pwm_channel = 2};

	for(uint8_t i=0; i<NUM_ESC; i++)
	{
		esc_init(&esc_s[i]);
	}
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
