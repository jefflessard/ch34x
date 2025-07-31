/*
 * CH341a USB GPIO chip implementation
 */

#include "ch341-core.h"
#include <linux/gpio/driver.h>


/* Output-only pins */
#define CH341_GPIO_OUT_B0   GENMASK(7, 0)
#define CH341_GPIO_OUT_B1   (GENMASK(7, 0) & ~GENMASK(5, 4))
#define CH341_GPIO_OUT_B2   GENMASK(3, 0)
#define CH341_GPIO_OUT_MASK (CH341_GPIO_OUT_B0 | (CH341_GPIO_OUT_B1 << 8) | (CH341_GPIO_OUT_B2 << 16))

/* Input-only pins */
#define CH341_GPIO_IN_B0   GENMASK(7, 0)
#define CH341_GPIO_IN_B1   (GENMASK(7, 0) & ~BIT(4))
#define CH341_GPIO_IN_B2   0
#define CH341_GPIO_IN_MASK (CH341_GPIO_IN_B0 | (CH341_GPIO_IN_B1 << 8) | (CH341_GPIO_IN_B2 << 16))

/* Available pins */
#define CH341_GPIO_B0   (CH341_GPIO_OUT_B0 | CH341_GPIO_IN_B0)
#define CH341_GPIO_B1   (CH341_GPIO_OUT_B1 | CH341_GPIO_IN_B1)
#define CH341_GPIO_B2   (CH341_GPIO_OUT_B2 | CH341_GPIO_IN_B2)
#define CH341_GPIO_MASK (CH341_GPIO_OUT_MASK | CH341_GPIO_IN_MASK)

static const char *ch341_pin_names[] = {
	"CS0",
	"CS1",
	"CS2",
	"DCK",
	"DOUT2",
	"DOUT",
	"DIN2",
	"DIN",
	"TXD",
	"RXD",
	"INT",
	"IN3",
	NULL,
	"TEN",
	"ROV",
	"IN7",
	"RST",
	"RDY",
	"SDL",
	"SDA",
};

/* valid mask helper */
static inline u32 ch341_gpio_valid_mask(struct ch341_device *ch341)
{
	return CH341_GPIO_MASK & ~ch341->spi_mask & ~ch341->i2c_mask;
}

/* CH341a GPIO commands */
/* Get current status of all pins */
static int ch341_gpio_get_status(struct ch341_device *ch341, u32 *status)
{
	struct urb *tx = ch341_alloc_urb(ch341, NULL, 1);
	struct urb *rx = ch341_alloc_urb(ch341, NULL, 6);
	u8 *cmd = tx->transfer_buffer;
	u8 *buf = rx->transfer_buffer;
	int ret;

	cmd[0] = CH341_CMD_GET_STATUS;
	ret = ch341_usb_transfer_wait(ch341, tx, rx);
	if (ret < 0) {
		ch341_free_urb(tx);
		ch341_free_urb(rx);
		return ret;
	}

	*status = ((buf[2] & CH341_GPIO_B2) << 16) |
		  ((buf[1] & CH341_GPIO_B1) << 8) |
		   (buf[0] & CH341_GPIO_B0);

	ch341_free_urb(tx);
	ch341_free_urb(rx);

	return 0;
}

/* Write output pin values */
static int ch341_gpio_write_outputs(struct ch341_device *ch341, u32 mask, u32 data)
{
	struct urb *tx = ch341_alloc_urb(ch341, NULL, 11);
	u8 *cmd = tx->transfer_buffer;

	dev_dbg(CH341_DEV, "%s %x %x\n", __func__, mask, data);
	data = data & mask;

	cmd[0] = CH341_CMD_SET_OUTPUT;
	cmd[1] = 0x6a;

	/* output control requires byte 2 bits 1-0 | byte mask */
	cmd[2] = GENMASK(4, 0);  /* Enable all bit ranges */

	/* enabled by byte 2 bit 2 */
	cmd[3] = (data >> 8) & CH341_GPIO_OUT_B1;
	cmd[4] = (mask >> 8) & CH341_GPIO_OUT_B1;

	/* enabled by byte 2 bit 3 */
	cmd[5] = data & CH341_GPIO_OUT_B0;
	cmd[6] = mask & CH341_GPIO_OUT_B0;

	/* enabled by byte 2 bit 4 */
	cmd[7] = (data >> 16) & CH341_GPIO_OUT_B2;

	cmd[8] = 0;
	cmd[9] = 0;
	cmd[10] = 0;

	return ch341_usb_transfer(ch341, tx, NULL, ch341_complete, NULL);
}

/* GPIO chip operations */
static int ch341_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct ch341_device *ch341 = gpiochip_get_data(chip);
	u32 pin_mask = BIT(offset);

	dev_dbg(CH341_DEV, "%s: %u %x\n", __func__, offset, !!(pin_mask & ch341->gpio_mask));

	if (pin_mask & ~ch341_gpio_valid_mask(ch341))
		return -EPERM;

	if (pin_mask & ch341->gpio_mask)
		return GPIO_LINE_DIRECTION_OUT;
	else
		return GPIO_LINE_DIRECTION_IN;
}

static int ch341_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	struct ch341_device *ch341 = gpiochip_get_data(chip);
	u32 pin_mask = BIT(offset);
	u32 old_mask;
	int ret = 0;

	dev_dbg(CH341_DEV, "%s: %u\n", __func__, offset);

	if (pin_mask & ~ch341_gpio_valid_mask(ch341))
		return -EPERM;

	old_mask = set_mask_bits(&ch341->gpio_mask, pin_mask, 0);

	/* if pin was output */
	if (pin_mask & old_mask)
		ret = ch341_gpio_write_outputs(ch341, ch341->gpio_mask, ch341->gpio_data);

	return ret;
}

static int ch341_gpio_direction_output(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ch341_device *ch341 = gpiochip_get_data(chip);
	u32 pin_mask = BIT(offset);
	u32 pin_bits = value ? pin_mask : 0;
	u32 old_mask, old_data;
	int ret = 0;

	dev_dbg(CH341_DEV, "%s: %u, %u\n", __func__, offset, value);

	if (pin_mask & ~ch341_gpio_valid_mask(ch341))
		return -EPERM;

	old_mask = set_mask_bits(&ch341->gpio_mask, pin_mask, pin_mask);
	old_data = set_mask_bits(&ch341->gpio_data, pin_mask, pin_bits);

	/* if either pin was input or data changed */
	if (pin_mask & ~old_mask || (old_data & pin_mask) != pin_bits)
		ret = ch341_gpio_write_outputs(ch341, ch341->gpio_mask, ch341->gpio_data);

	return ret;
}

static int ch341_gpio_get_multiple(struct gpio_chip *chip, unsigned long *mask, unsigned long *bits)
{
	struct ch341_device *ch341 = gpiochip_get_data(chip);
	u32 status;
	int ret;

	if (*mask & ~ch341_gpio_valid_mask(ch341))
		return -EPERM;

	if (*mask & ch341->gpio_mask)
		return -EINVAL;

	ret = ch341_gpio_get_status(ch341, &status);
	if (ret < 0)
		return ret;

	ch341->gpio_data = status;

	/* Return requested bits */
	*bits = status & *mask;

	dev_dbg(CH341_DEV, "%s: %ph, %ph", __func__, mask, bits);

	return 0;
}

static int ch341_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	long unsigned int pin_mask = BIT(offset);

	return !!ch341_gpio_get_multiple(chip, &pin_mask, &pin_mask);
}

static int ch341_gpio_set_multiple_rv(struct gpio_chip *chip, unsigned long *mask, unsigned long *bits)
{
	struct ch341_device *ch341 = gpiochip_get_data(chip);
	u32 output_mask = READ_ONCE(ch341->gpio_mask);
	u32 old_data;
	int ret = 0;

	dev_dbg(CH341_DEV, "%s: %ph, %ph", __func__, mask, bits);

	if (*mask & ~ch341_gpio_valid_mask(ch341))
		return -EPERM;

	if (*mask & ~output_mask)
		return -EINVAL;

	old_data = set_mask_bits(&ch341->gpio_data, *mask, *bits);

	/* if data changed */
	if ((old_data & *mask) != (*bits & *mask))
		ret = ch341_gpio_write_outputs(ch341, ch341->gpio_mask, ch341->gpio_data);

	return ret;
}

static int ch341_gpio_set_rv(struct gpio_chip *chip, unsigned int offset, int value)
{
	long unsigned int pin_mask = BIT(offset),
			  pin_bits = value ? pin_mask : 0;

	return ch341_gpio_set_multiple_rv(chip, &pin_mask, &pin_bits);
}

static void ch341_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	ch341_gpio_set_rv(chip, offset, value);
}

static void ch341_gpio_set_multiple(struct gpio_chip *chip, unsigned long *mask, unsigned long *bits)
{
	ch341_gpio_set_multiple_rv(chip, mask, bits);
}

static int ch341_gpio_init_valid_mask(struct gpio_chip *chip,
				      unsigned long *valid_mask,
				      unsigned int ngpios)
{
	struct ch341_device *ch341 = gpiochip_get_data(chip);

	dev_dbg(CH341_DEV, "%s %lx %d\n", __func__, *valid_mask, ngpios);

	if (ngpios != fls(CH341_GPIO_MASK))
		return -EINVAL;

	*valid_mask = ch341_gpio_valid_mask(ch341);

	return 0;
}

/* GPIO chip registration */
int ch341_gpio_probe(struct ch341_device *ch341)
{
	struct gpio_chip *gpio_chip;
	struct fwnode_handle *fwnode = NULL;
	int ret;

	if (CH341_DEV->fwnode) {
		fwnode = ch341_get_compatible_fwnode(ch341, "wch-ic,ch341-gpio");
		if (!fwnode) {
			dev_info(CH341_DEV, "GPIO controller disabled (no DT node found)\n");
			return 0;
		}
	}

	gpio_chip = devm_kzalloc(CH341_DEV, sizeof(struct gpio_chip), GFP_KERNEL);
	if (!gpio_chip)
		return -ENOMEM;

	/* Initialize GPIO chip structure */
	gpio_chip->label = "ch341-gpio";
	gpio_chip->parent = CH341_DEV;
	gpio_chip->owner = THIS_MODULE;
	gpio_chip->fwnode = fwnode;
	gpio_chip->base = -1;  /* Dynamic allocation */
	gpio_chip->ngpio = fls(CH341_GPIO_MASK);
	gpio_chip->names = ch341_pin_names;
	gpio_chip->can_sleep = true;  /* USB operations can sleep */

	/* Set GPIO operations */
	gpio_chip->init_valid_mask = ch341_gpio_init_valid_mask;
	gpio_chip->get_direction = ch341_gpio_get_direction;
	gpio_chip->direction_input = ch341_gpio_direction_input;
	gpio_chip->direction_output = ch341_gpio_direction_output;
	gpio_chip->get = ch341_gpio_get;
	gpio_chip->get_multiple = ch341_gpio_get_multiple;
#if 1
	gpio_chip->set = ch341_gpio_set;
	gpio_chip->set_multiple = ch341_gpio_set_multiple;
#else
	gpio_chip->set_rv = ch341_gpio_set_rv;
	gpio_chip->set_multiple_rv = ch341_gpio_set_multiple_rv;
#endif

	/* Sync GPIO state */
	ret = ch341_gpio_write_outputs(ch341, ch341->gpio_mask, ch341->gpio_data);
	if (ret < 0) {
		dev_err(CH341_DEV, "Failed to set default state: %d\n", ret);
		goto err_free_gpio;
	}
	ret = ch341_gpio_get_status(ch341, &ch341->gpio_data);
	if (ret < 0) {
		dev_err(CH341_DEV, "Failed to read state: %d\n", ret);
		goto err_free_gpio;
	}

	/* Register GPIO chip */
	ret = gpiochip_add_data(gpio_chip, ch341);
	if (ret) {
		dev_err(CH341_DEV, "Failed to register GPIO chip: %d\n", ret);
		goto err_free_gpio;
	}

	ch341->gpio_chip = gpio_chip;

	return 0;

err_free_gpio:
	if (fwnode) fwnode_handle_put(fwnode);
	devm_kfree(CH341_DEV, gpio_chip);
	ch341->gpio_chip = NULL;
	return ret;
}

void ch341_gpio_remove(struct ch341_device *ch341)
{
	struct fwnode_handle *fwnode;

	if (!ch341->gpio_chip)
		return;

	gpiochip_remove(ch341->gpio_chip);

	fwnode = ch341->gpio_chip->fwnode;
	if (fwnode) fwnode_handle_put(fwnode);

	/* let devm clean up memory later
	 * devm_kfree(CH341_DEV, ch341->gpio_chip); */

	ch341->gpio_chip = NULL;
}
