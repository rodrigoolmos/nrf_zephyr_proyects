#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define SLEEP_TIME_MS   100

/* The devicetree node identifier for the "led0" alias. */
#define LED4_NODE DT_ALIAS(led4)
#define LED5_NODE DT_ALIAS(led5)
#define BTN0_NODE DT_ALIAS(sw0)
#define BTN5_NODE DT_ALIAS(btn5)

#define ON 1
#define OFF 0

static const struct gpio_dt_spec led4 = GPIO_DT_SPEC_GET(LED4_NODE, gpios);
static const struct gpio_dt_spec led5 = GPIO_DT_SPEC_GET(LED5_NODE, gpios);
static const struct gpio_dt_spec btn0 = GPIO_DT_SPEC_GET(BTN0_NODE, gpios);
static const struct gpio_dt_spec btn4 = GPIO_DT_SPEC_GET(BTN0_NODE, gpios);
static const struct gpio_dt_spec btn5 = GPIO_DT_SPEC_GET(BTN5_NODE, gpios);

static struct gpio_callback btn0_cb;
static struct gpio_callback btn5_cb;

void button5_callback(const struct device *gpio, struct gpio_callback *cb, gpio_port_pins_t pins){
	printk("Boton 5 pressed\n");
}

void button0_callback(const struct device *gpio, struct gpio_callback *cb, gpio_port_pins_t pins){
	printk("Boton 0 pressed\n");
}

uint32_t init_gpios(){

	int ret;

	if (!gpio_is_ready_dt(&led4))
		return 1;

	if (!gpio_is_ready_dt(&led5))
		return 1;

	if (!gpio_is_ready_dt(&btn0))
		return 1;

	if (!gpio_is_ready_dt(&btn5))
		return 1;

	ret = gpio_pin_configure_dt(&led4, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
		return 1;
	
	ret = gpio_pin_configure_dt(&led5, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
		return 1;
	
	ret = gpio_pin_configure_dt(&btn0, GPIO_INPUT);
	if (ret < 0)
		return 1;

	ret = gpio_pin_configure_dt(&btn5, GPIO_INPUT);
	if (ret < 0)
		return 1;

	return 0;
}

uint32_t init_callback(){

	volatile int32_t status;

	gpio_init_callback(&btn0_cb, button0_callback, BIT(btn0.pin));
	gpio_init_callback(&btn5_cb, button5_callback, BIT(btn5.pin));
	status = gpio_add_callback(btn0.port, &btn0_cb);
	status = gpio_add_callback(btn5.port, &btn5_cb);
	status = gpio_pin_interrupt_configure_dt(&btn0, GPIO_INT_EDGE_TO_ACTIVE);
	status = gpio_pin_interrupt_configure_dt(&btn5, GPIO_INT_EDGE_TO_ACTIVE);

	return 0;
}

int main(void)
{
	volatile uint32_t status;

	status = init_gpios();

	init_callback();

	while (1) {

		status = gpio_pin_set_dt(&led4, ON);

		status = gpio_pin_set_dt(&led5, OFF);

		k_msleep(SLEEP_TIME_MS);

		status = gpio_pin_set_dt(&led4, OFF);

		status = gpio_pin_set_dt(&led5, ON);
		
		k_msleep(SLEEP_TIME_MS);

	}
	return 0;
}