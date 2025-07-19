/*
 * CH341a USB GPIO chip implementation
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/gpio/driver.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#define DRIVER_NAME "ch341-gpio"
#define CH341_DEFAULT_TIMEOUT	1000
#define CH341_RX_BUFFER_LEN	32

/* Control Commands */
#define CH341_CTRL_VERSION	0x5F
#define CH341_CTRL_PARA_INIT	0xB1
#define CH341_CTRL_DEBUG_WRITE	0x9A

/* CH341 Commands */
#define CH341_CMD_GET_STATUS    0xa0
#define CH341_CMD_SET_OUTPUT    0xa1
#define CH341_CMD_UIO_STREAM    0xab
#define CH341_CMD_UIO_STM_IN    0x00
#define CH341_CMD_UIO_STM_END   0x20
#define CH341_CMD_UIO_STM_DIR   0x40
#define CH341_CMD_UIO_STM_OUT   0x80

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

#define CH341_GPIO_SPI_MASK  (BIT(3) | BIT(5) | BIT(7))
#define CH341_GPIO_I2C_MASK  (BIT(18) | BIT(19))

#define CH341_DEV (&ch341->intf->dev)

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

struct ch341_device {
	struct usb_device *udev;
	struct usb_interface *intf;
	struct gpio_chip gpio_chip;

	/* USB transfer pipes */
	unsigned int tx_pipe;
	unsigned int rx_pipe;

	/* GPIO state tracking */
	u32 gpio_mask;  /* Direction: 1=output, 0=input */
	u32 gpio_data;  /* Current pin values */

	/* IC version for compatibility */
	u16 ic_version;
};

/* USB transfer helper functions */
static void ch341_usb_tx_complete(struct urb *urb)
{
	struct ch341_device *ch341 = urb->context;

	if (urb->status) {
		dev_err(CH341_DEV, "tx completion failed: %d\n", urb->status);
	} else if (urb->transfer_buffer_length != urb->actual_length) {
		dev_err(CH341_DEV, "usb_send length mismatch: %d, %d\n", urb->transfer_buffer_length, urb->actual_length);
	} else {
		dev_dbg(CH341_DEV, "tx completed\n");
	}

	usb_free_coherent(urb->dev, urb->transfer_buffer_length, urb->transfer_buffer, urb->transfer_dma);
	usb_free_urb(urb);
}

static int ch341_submit_tx_urb(struct ch341_device *ch341, void *data, int len)
{
	struct urb *urb;
	void *buf;
	int ret;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		return -ENOMEM;

	buf = usb_alloc_coherent(ch341->udev, len, GFP_KERNEL, &urb->transfer_dma);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	memcpy(buf, data, len);

	usb_fill_bulk_urb(urb, ch341->udev, ch341->tx_pipe, buf, len,
			  ch341_usb_tx_complete, ch341);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	ret = usb_submit_urb(urb, GFP_KERNEL);

	if (ret) {
		dev_err(CH341_DEV, "usb_send urb failed, %d\n", ret);
		usb_free_coherent(urb->dev, urb->transfer_buffer_length, urb->transfer_buffer, urb->transfer_dma);
		usb_free_urb(urb);
		return ret;
	}

	return 0;
}

static int ch341_bulk_msg(struct ch341_device *ch341, unsigned int pipe, void *data, int len)
{
	int buf_len, actual_len, ret;
	void *buf;

	if (pipe == ch341->tx_pipe) {
		buf_len = len;
	} else {
		buf_len = CH341_RX_BUFFER_LEN;
	}

	buf = kmalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (pipe == ch341->tx_pipe)
		memcpy(buf, data, len);

	ret = usb_bulk_msg(ch341->udev, pipe, buf, buf_len, &actual_len, CH341_DEFAULT_TIMEOUT);
	if (ret < 0) {
		dev_err(CH341_DEV, "usb_bulk_msg %x failed: %d\n", pipe, ret);
		goto exit;
	}

	ret = actual_len;
	if (actual_len != len) {
		dev_warn(CH341_DEV, "usb_bulk_msg %x length mismatch: %d, %d\n", pipe, len, actual_len);
		ret = -EIO;
		goto exit;
	}

	if (pipe == ch341->rx_pipe) {
		memcpy(data, buf, len);
		dev_dbg(CH341_DEV, "usb_bulk_msg %x received: %*ph\n", pipe, actual_len, data);
	}

exit:
	kfree(buf);
	return ret;
}

static int ch341_usb_transfer(struct ch341_device *ch341, void *tx_buf, u32 tx_len, void *rx_buf, u32 rx_len)
{
	int ret = 0;

	/* Send command */
	if (tx_buf && tx_len) {

		dev_dbg(CH341_DEV, "usb_send: %*ph\n", tx_len, tx_buf);

		if (!rx_buf || !rx_len) {
			ret = ch341_submit_tx_urb(ch341, tx_buf, tx_len);
		} else {
			ret = ch341_bulk_msg(ch341, ch341->tx_pipe, tx_buf, tx_len);
		}
		if (ret < 0)
			return ret;
	}

	/* Receive response */
	if (rx_buf && rx_len) {
		ret = ch341_bulk_msg(ch341, ch341->rx_pipe, rx_buf, rx_len);
		if (ret < 0)
			return ret;
	}

	return ret;
}

/* USB control transfer wrappers */
static int ch341_control_read(struct ch341_device *ch341, u8 request,
			      u16 value, u16 index, void *data, u16 size)
{
	int ret;
	
	ret = usb_control_msg(ch341->udev,
			      usb_rcvctrlpipe(ch341->udev, 0), request,
			      USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			      value, index, data, size, CH341_DEFAULT_TIMEOUT);
	
	if (ret < 0)
		dev_err(CH341_DEV, "Control read failed: %d\n", ret);
	
	return ret;
}

static int __maybe_unused ch341_control_write(struct ch341_device *ch341, u8 request,
			      u16 value, u16 index, void *data, u16 size)
{
	int ret;
	
	ret = usb_control_msg(ch341->udev,
			      usb_sndctrlpipe(ch341->udev, 0), request,
			      USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			      value, index, data, size, CH341_DEFAULT_TIMEOUT);
	
	if (ret < 0)
		dev_err(CH341_DEV, "Control write failed: %d\n", ret);
	
	return ret;
}

static int ch341_init_device(struct ch341_device *ch341)
{
	int ret;

#if 0
	u8 mode = 2;
	u16 mode_cmd = (mode << 8) | mode;
	u16 init_cmd = (mode << 8) | 0x02;

	/* Set mode */
	ret = ch341_control_write(ch341, CH341_CTRL_DEBUG_WRITE, 0x2525, mode_cmd, NULL, 0);
	if (ret < 0) {
		dev_err(CH341_DEV, "Failed to write mode\n");
		return ret;
	}

	/* Initialize mode */
	ret = ch341_control_write(ch341, CH341_CTRL_PARA_INIT, init_cmd, 0, NULL, 0);
	if (ret < 0) {
		dev_err(CH341_DEV, "Device initialization failed\n");
		return ret;
	}
#endif

	/* Get device version */
	ret = ch341_control_read(ch341, CH341_CTRL_VERSION, 0, 0,
				 &ch341->ic_version, sizeof(ch341->ic_version));
	if (ret < 0) {
		dev_warn(CH341_DEV, "Failed to get version\n");
		ch341->ic_version = 0;
	}
	
	dev_info(CH341_DEV,
		 "CH34x device initialized (version: 0x%04x)\n",
		 ch341->ic_version);

	return 0;
}

/* CH341a GPIO commands */
/* Get current status of all pins */
static int ch341_gpio_get_status(struct ch341_device *ch341, u32 *status)
{
	u8 cmd[1];
	u8 buf[6];
	int ret;

#if 0
	cmd[0] = CH341_CMD_UIO_STREAM;
	cmd[1] = CH341_CMD_UIO_STM_IN;
	cmd[2] = CH341_CMD_UIO_STM_END;
#endif

	cmd[0] = CH341_CMD_GET_STATUS;
	ret = ch341_usb_transfer(ch341, cmd, sizeof(cmd), buf, sizeof(buf));
	if (ret < 0)
		return ret;

	*status = ((buf[2] & CH341_GPIO_B2) << 16) |
		  ((buf[1] & CH341_GPIO_B1) << 8) |
		   (buf[0] & CH341_GPIO_B0);

	return 0;
}

/* Write output pin values */
static int ch341_gpio_write_outputs(struct ch341_device *ch341, u32 mask, u32 data)
{
	u8 cmd[11];

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

	return ch341_usb_transfer(ch341, cmd, sizeof(cmd), NULL, 0);
}

/* GPIO chip operations */
static int ch341_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct ch341_device *ch341 = gpiochip_get_data(chip);
	u32 pin_mask = BIT(offset);

	dev_dbg(CH341_DEV, "%s: %u\n", __func__, offset);

	if (pin_mask & ~CH341_GPIO_MASK)
		return -EINVAL;

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

	if (pin_mask & ~CH341_GPIO_IN_MASK)
		return -EINVAL;

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

	if (pin_mask & ~CH341_GPIO_OUT_MASK)
		return -EINVAL;

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

	dev_dbg(CH341_DEV, "%s: %ph, %ph", __func__, mask, bits);

	if (*mask & ch341->gpio_mask)
		return -EINVAL;

	ret = ch341_gpio_get_status(ch341, &status);
	if (ret < 0)
		return ret;

	ch341->gpio_data = status;

	/* Return requested bits */
	*bits = status & *mask;

	return 0;
}

static int ch341_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	long unsigned int pin_mask = BIT(offset);

	return ch341_gpio_get_multiple(chip, &pin_mask, &pin_mask) ? 1 : 0;
}

static int ch341_gpio_set_multiple_rv(struct gpio_chip *chip, unsigned long *mask, unsigned long *bits)
{
	struct ch341_device *ch341 = gpiochip_get_data(chip);
	u32 output_mask = READ_ONCE(ch341->gpio_mask);
	u32 old_data;
	int ret = 0;

	dev_dbg(CH341_DEV, "%s: %ph, %ph", __func__, mask, bits);

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
	*valid_mask = CH341_GPIO_MASK;
	return 0;
}

/* GPIO chip registration */
static struct fwnode_handle *ch341_find_fwnode(struct usb_device *udev)
{
    struct usb_device *parent = udev->parent;
    struct fwnode_handle *usb_controller_fwnode;
    struct fwnode_handle *child_fwnode;
    const char *node_compatible;
    char *expected_compatible;
    int ret;

    if (!parent)
        return NULL;

    // Get the USB controller's fwnode
    usb_controller_fwnode = dev_fwnode(&parent->dev);
    if (!usb_controller_fwnode)
        return NULL;

    expected_compatible = kasprintf(GFP_KERNEL, "usb%04x,%04x",
                                   le16_to_cpu(udev->descriptor.idVendor),
                                   le16_to_cpu(udev->descriptor.idProduct));
    if (!expected_compatible)
        return NULL;

    // Look for child nodes with matching compatible string
    fwnode_for_each_child_node(usb_controller_fwnode, child_fwnode) {
	dev_info(&udev->dev, "checking %s", fwnode_get_name(child_fwnode));
	ret = fwnode_device_is_compatible(child_fwnode, expected_compatible);
        if (ret) {
            kfree(expected_compatible);
            return fwnode_handle_get(child_fwnode);
        }
    }

    kfree(expected_compatible);
    return NULL;
}

int ch341_gpio_probe(struct ch341_device *ch341)
{
	struct gpio_chip *gpio_chip = &ch341->gpio_chip;
	struct fwnode_handle *fwnode;
	int ret;

	/* Initialize GPIO chip structure */
	gpio_chip->label = "ch341-gpio";
	gpio_chip->parent = CH341_DEV;
	gpio_chip->owner = THIS_MODULE;
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

	/* Initialize GPIO state */
	ch341->gpio_mask = CH341_GPIO_OUT_MASK;
	ch341->gpio_data = 0;   /* All pins default to low */
	ret = ch341_gpio_write_outputs(ch341, ch341->gpio_mask, ch341->gpio_data);
	if (ret < 0) {
		dev_err(CH341_DEV, "Failed to set default state: %d\n", ret);
		return ret;
	}
	ret = ch341_gpio_get_status(ch341, &ch341->gpio_data);
	if (ret < 0) {
		dev_err(CH341_DEV, "Failed to read state: %d\n", ret);
		return ret;
	}

	/* Look for USB device tree node */
	fwnode = ch341_find_fwnode(ch341->udev);
	if (fwnode) {
		gpio_chip->fwnode = fwnode;
		dev_info(CH341_DEV, "Found USB device tree configuration\n");
	} else {
		dev_info(CH341_DEV, "No USB device tree configuration found\n");
	}

	/* Register GPIO chip */
	ret = devm_gpiochip_add_data(CH341_DEV, gpio_chip, ch341);
	if (ret) {
		dev_err(CH341_DEV, "Failed to register GPIO chip: %d\n", ret);
		if (fwnode)
			fwnode_handle_put(fwnode);
		return ret;
	}

	dev_info(CH341_DEV, "CH341 GPIO chip registered with %d pins\n",
			gpio_chip->ngpio);

	if (fwnode) {
		// Populate DT children (e.g. spi-gpio)
		ret = of_platform_populate(to_of_node(fwnode), NULL, NULL, CH341_DEV);
		if (ret)
			dev_warn(CH341_DEV, "Failed to populate child devices: %d\n", ret);
	}

	return 0;
}

void ch341_gpio_remove(struct ch341_device *ch341)
{
	struct fwnode_handle *fwnode = ch341->gpio_chip.fwnode;

	dev_info(CH341_DEV, "Unregistering gpio chip");

	if (fwnode) {
		dev_info(CH341_DEV, "Unregistering child devices");
		// Remove child devices created by of_platform_populate
		of_platform_depopulate(CH341_DEV);

		fwnode_handle_put(fwnode);
	}
}

static int ch341_probe(struct usb_interface *interface,
		const struct usb_device_id *id)
{
	struct device *dev = &interface->dev;
	struct usb_device *udev = interface_to_usbdev(interface);
	struct usb_endpoint_descriptor *bulk_in;
	struct usb_endpoint_descriptor *bulk_out;
	struct ch341_device *ch341;
	int ret;

	ch341 = devm_kzalloc(dev, sizeof(*ch341), GFP_KERNEL);
	if (!ch341)
		return -ENOMEM;

	ch341->udev = usb_get_dev(udev);
	ch341->intf = interface;

	ret = usb_find_common_endpoints(interface->cur_altsetting, &bulk_in, &bulk_out, NULL, NULL);
	if (ret) {
		dev_err(dev, "Required endpoints not found\n");
		goto err_put_dev;
	}

	ch341->tx_pipe = usb_sndbulkpipe(udev, usb_endpoint_num(bulk_out));
	ch341->rx_pipe = usb_rcvbulkpipe(udev, usb_endpoint_num(bulk_in));

	usb_set_intfdata(interface, ch341);

	ch341_init_device(ch341);

	ret = ch341_gpio_probe(ch341);
	if (ret) {
		dev_err(dev, "Failed to initialize gpio: %d\n", ret);
		goto err_put_dev;
	}

	return 0;

err_put_dev:
	usb_put_dev(ch341->udev);
	return ret;
}

static void ch341_disconnect(struct usb_interface *interface)
{
	struct ch341_device *ch341 = usb_get_intfdata(interface);

	if (!ch341)
		return;

	ch341_gpio_remove(ch341);
	usb_set_intfdata(interface, NULL);
}

static const struct usb_device_id ch341_table[] = {
	{ USB_DEVICE(0x1A86, 0x5512) },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(usb, ch341_table);

static struct usb_driver ch341_driver = {
	.name = DRIVER_NAME,
	.probe = ch341_probe,
	.disconnect = ch341_disconnect,
	.id_table = ch341_table,
};

module_usb_driver(ch341_driver);


MODULE_AUTHOR("Jean-François Lessard");
MODULE_DESCRIPTION("CH341 USB-GPIO Driver");
MODULE_LICENSE("GPL v2");
