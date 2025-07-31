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
#define CH341_PKT_LEN      	32

/* CH341 pins */
#define CH341_PIN_CS0		0  /* D0 */
#define CH341_PIN_CS1		1  /* D1 */
#define CH341_PIN_CS2		2  /* D2 */
#define CH341_PIN_DCK		3  /* D3 */
#define CH341_PIN_DOUT2		4  /* D4 */
#define CH341_PIN_DOUT		5  /* D5 */
#define CH341_PIN_DIN2		6  /* D6 */
#define CH341_PIN_DIN		7  /* D7 */
#define CH341_PIN_TXD		8  /* TXD/ERR# */
#define CH341_PIN_RXD		9  /* RXD/PEMP */
#define CH341_PIN_INT		10 /* INT# */
#define CH341_PIN_IN3		11 /* IN3/SLCT */
/*				12    resrved */
#define CH341_PIN_TEN		13 /* TEN/BUSY/WAIT# */
#define CH341_PIN_ROV		14 /* ROV/AUTOFD#/DATAS#/READ# */
#define CH341_PIN_IN7		15 /* SLCTIN#/ADDRS#/ALE */
#define CH341_PIN_RST		16 /* RST# */
#define CH341_PIN_RDY		17 /* WRITE# */
#define CH341_PIN_SDL		18 /* SDL */
#define CH341_PIN_SDA		19 /* SDA */

/* In parallel port mode:
 * - default direction is 0x000FC000
 * - all input pins are low by default */
#define CH341_PINS_DIR_DEFAULT	GENMASK(CH341_PIN_SDA, CH341_PIN_ROV)
#define CH341_PINS_VAL_DEFAULT	0 

/* Control Commands */
#define CH341_CTRL_VERSION	0x5F
#define CH341_CTRL_PARA_INIT	0xB1
#define CH341_PARA_MODE_EPP17	(0 << 8)
#define CH341_PARA_MODE_EPP19	(1 << 8)
#define CH341_PARA_MODE_MEM	(2 << 8)
#define CH341_PARA_MODE_ECP	(3 << 8)
#define CH341_PARA_MODE_KEEP	0
#define CH341_PARA_MODE_SET	2
	/* Initialize parallel port mode:
	 *  - reset / clear buffer
	 *  - RST# outputs a low level pulse
	 * high byte:
	 * - 0: EPP mode/EPP mode V1.7 (default)
	 * - 1: EPP mode V1.9
	 * - 2: MEM mode
	 * - 3: ECP mode
	 * low byte:
	 * - 0: keeps the current mode
	 * - 2: configures specified mode */

#define CH341_CTRL_DEBUG_WRITE	0x9A

/* CH341 Commands */
#define CH341_CMD_GET_STATUS    0xA0
#define CH341_CMD_SET_OUTPUT    0xA1

#define CH341_CMD_SPI_STREAM	0xA8

#define CH341_CMD_I2C_STREAM	0xAA
#define CH341_I2C_STM_US	0x40
#define CH341_I2C_STM_MS	0x50
#define CH341_I2C_STM_SET	0x60
#define CH341_I2C_SPEED_MASK	GENMASK(1, 0)
#define CH341_SPI_DUAL_MASK	BIT(2)
	/* Bits 1-0: I2C speed/SCL frequency:
	 * - 0: low speed 20KHz
	 * - 1: standard 100KHz (default)
	 * - 2: fast 400KHz
	 * - 3: high speed 750KHz
	 * Bit 2: SPI I/O number/IO pin:
	 * - 0: 4-wire - single input/output (default)
	 * - 1: 5-wire - dual input/output */

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

#define CH341_DEV (&ch341->intf->dev)

struct ch341_device {
	struct usb_device *udev;
	struct usb_interface *intf;
	struct gpio_chip *gpio_chip;
	struct spi_controller *spi;
	struct i2c_adapter *i2c;
	struct usb_anchor anchor;
	spinlock_t lock; /* protect against interweaved write-read usb transfers */

	/* USB transfer pipes */
	unsigned int tx_pipe;
	unsigned int rx_pipe;

	/* pins state tracking */
	u32 gpio_mask;  /* Direction: 1=output, 0=input */
	u32 gpio_data;  /* Current pin values */
	u32 spi_mask; /* Reserved SPI pins */
	u32 i2c_mask; /* Reserved I2C pins */

	/* I2C/SPI stream config */
	u8 stream_config;

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

/* shared functions */
int ch341_stream_config(struct ch341_device *ch341, u8 mask, u8 bits);

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
