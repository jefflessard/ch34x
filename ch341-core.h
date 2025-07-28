/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __CH341_CORE_H
#define __CH341_CORE_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/gpio/driver.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>

#define DRIVER_NAME "ch341-bridge"
#define CH341_TIMEOUT_MS	100
#define CH341_BUFFER_LENGTH	0x1000

/* Control Commands */
#define CH341_CTRL_VERSION	0x5F
#define CH341_CTRL_PARA_INIT	0xB1
#define CH341_CTRL_DEBUG_WRITE	0x9A

/* CH341 Commands */
#define CH341_CMD_GET_STATUS    0xA0
#define CH341_CMD_SET_OUTPUT    0xA1

#define CH341_CMD_SPI_STREAM	0xA8

#define CH341_CMD_I2C_STREAM	0xAA
#define CH341_I2C_STM_US	0x40
#define CH341_I2C_STM_MS	0x50
#define CH341_I2C_STM_SET	0x60
#define CH341_I2C_STM_STA	0x74
#define CH341_I2C_STM_STO	0x75
#define CH341_I2C_STM_OUT	0x80
#define CH341_I2C_STM_IN	0xC0
#define CH341_I2C_STM_END	0x00

#define CH341_CMD_UIO_STREAM    0xAB
#define CH341_UIO_STM_IN	0x00
#define CH341_UIO_STM_END	0x20
#define CH341_UIO_STM_DIR	0x40
#define CH341_UIO_STM_OUT	0x80
#define CH341_UIO_STM_US	0xc0

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

#define CH341_DEV (&ch341->intf->dev)

struct ch341_device {
	struct usb_device *udev;
	struct usb_interface *intf;
	struct gpio_chip *gpio_chip;
	struct spi_controller *spi_ctlr;
	struct i2c_adapter *i2c;
	struct usb_anchor anchor;

	/* USB transfer pipes */
	unsigned int tx_pipe;
	unsigned int rx_pipe;

	/* GPIO state tracking */
	u32 gpio_mask;  /* Direction: 1=output, 0=input */
	u32 gpio_data;  /* Current pin values */
	u32 spi_mask; /* Reserved SPI pins */
	u32 i2c_mask; /* Reserved I2C pins */

	/* IC version for compatibility */
	u16 ic_version;
};

struct ch341_transfer {
	struct ch341_device *dev;
	struct urb *tx_urb;
	struct urb *rx_urb;

	/* Completion tracking */
	atomic_t pending_urbs;
	int status;

	/* User callback */
	void (*complete)(struct ch341_transfer *xfer);
	void *context;
};

/* subdevices */
int ch341_spi_probe(struct ch341_device *ch341);
void ch341_spi_remove(struct ch341_device *ch341);

int ch341_gpio_probe(struct ch341_device *ch341);
void ch341_gpio_remove(struct ch341_device *ch341);

int ch341_i2c_probe(struct ch341_device *ch341);
void ch341_i2c_remove(struct ch341_device *ch341);

/* fwnode helper functions */
struct fwnode_handle* ch341_get_compatible_fwnode(struct ch341_device *ch341, const char *compatible);

/* URB and USB wrappers */
struct urb *ch341_alloc_urb(struct ch341_device *ch341, void *buf, int len);
void ch341_free_urb(struct urb *urb);

int ch341_usb_transfer(struct ch341_device *dev,
		struct urb *tx_urb, struct urb *rx_urb,
		void (*complete)(struct ch341_transfer *xfer),
		void *context);

static inline void ch341_complete(struct ch341_transfer *xfer)
{
	if (xfer->tx_urb) ch341_free_urb(xfer->tx_urb);
	if (xfer->rx_urb) ch341_free_urb(xfer->rx_urb);

	xfer->tx_urb = xfer->rx_urb = NULL;
}

int ch341_usb_transfer_wait(struct ch341_device *dev,
		struct urb *tx_urb, struct urb *rx_urb);

#endif /* __CH341_CORE_H */
