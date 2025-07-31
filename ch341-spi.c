#include "ch341-core.h"
#include <linux/bitrev.h>

#define CH341_NUM_CHIPSELECT	3
#define CH341_OUT_MASK		GENMASK(5, 0)

static inline void ch341_spi_buf_bitrev8(u8 *buf, size_t len) {
    for (u8 *p = buf; p < buf + len; p++) {
        *p = bitrev8(*p);
    }
}

static int ch341_spi_update_pins(struct ch341_device *ch341, u32 gpio_mask, u32 gpio_data)
{
	struct urb *tx_urb;
	u8 *cmd;
	int ret;

	gpio_mask &= ch341->spi_mask & CH341_OUT_MASK;
	gpio_data &= ch341->spi_mask & CH341_OUT_MASK;

	tx_urb = ch341_alloc_urb(ch341, NULL, 4);
	if (!tx_urb) return -ENOMEM;
	cmd = tx_urb->transfer_buffer;

	cmd[0] = CH341_CMD_UIO_STREAM;
	cmd[1] = CH341_UIO_STM_DIR | gpio_mask;
	cmd[2] = CH341_UIO_STM_OUT | ch341->gpio_data;
	cmd[3] = CH341_UIO_STM_END;

	ret = ch341_usb_transfer(ch341, tx_urb, NULL, ch341_complete, NULL);
	if (ret < 0)
		dev_err(CH341_DEV, "Failed to update SPI pins: %d\n", ret);

	return 0;
}

static int ch341_spi_enable_pins(struct ch341_device *ch341, u32 mask, u32 bits)
{
	u32 old_mask;

	mask &= ch341->spi_mask;
	bits &= mask;

	old_mask = set_mask_bits(&ch341->gpio_mask, mask, bits);

	if ((old_mask & mask) != bits)
		return ch341_spi_update_pins(ch341, ch341->gpio_mask, ch341->gpio_data);

	return 0;
}

static int ch341_spi_set_pins(struct ch341_device *ch341, u32 mask, u32 bits)
{
	u32 old_data;

	mask &= ch341->spi_mask;
	bits &= mask;

	old_data = set_mask_bits(&ch341->gpio_data, mask, bits);

	if ((old_data & mask) != bits)
		return ch341_spi_update_pins(ch341, ch341->gpio_mask, ch341->gpio_data);

	return 0;
}

static void ch341_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct ch341_device *ch341 = spi_controller_get_devdata(spi->controller);
	u32 cs_bit, cs_data;

	dev_dbg(CH341_DEV, "%s %d %d\n", __func__, spi_get_chipselect(spi, 0), enable);

	if (!(spi->mode & SPI_NO_CS)) {
		cs_bit = BIT(spi_get_chipselect(spi, 0));

		if (spi->mode & SPI_CS_HIGH)
			enable = !enable;

		if (enable)
			cs_data = cs_bit;

		ch341_spi_set_pins(ch341, cs_bit, cs_data);
	}
}

static size_t ch341_spi_max_transfer_size(struct spi_device *spi)
{
	struct ch341_device *ch341 = spi_controller_get_devdata(spi->controller);

	return ch341->max_pkt_len - 1;
}

static int ch341_spi_prepare_message(struct spi_controller *ctlr, struct spi_message *msg)
{
	struct ch341_device *ch341 = spi_controller_get_devdata(msg->spi->controller);
	u32 dck_data;

	dev_dbg(CH341_DEV, "%s %x\n", __func__, msg->spi->mode);

	/* cpol=0: start with clock low
	   cpol=1: start with clock high */
	dck_data = msg->spi->mode & SPI_CPOL ? BIT(CH341_PIN_DCK) : 0;

	return ch341_spi_set_pins(ch341, BIT(CH341_PIN_DCK), dck_data);
}

static int ch341_spi_unprepare_message(struct spi_controller *ctlr, struct spi_message *msg)
{
	struct ch341_device *ch341 = spi_controller_get_devdata(msg->spi->controller);

	dev_dbg(CH341_DEV, "%s\n", __func__);

	return 0;
}


static int ch341_spi_transfer_one(struct spi_controller *ctlr,
				  struct spi_device *spi,
				  struct spi_transfer *xfer)
{
	struct ch341_device *ch341 = spi_controller_get_devdata(ctlr);
	struct urb *tx_urb, *rx_urb;
	u32 dout_mask;
	u8 *cmd;
	int ret;

	dev_dbg(CH341_DEV, "%s %d\n", __func__, xfer->len);

	/* disable DOUT when in 3-wire RX */
	dout_mask = spi->mode & SPI_3WIRE && xfer->rx_buf ? 0 : BIT(CH341_PIN_DOUT);
	ret = ch341_spi_enable_pins(ch341, BIT(CH341_PIN_DOUT), dout_mask);
	if (ret < 0)
		dev_err(CH341_DEV, "Failed to set SPI DOUT mask: %d\n", ret);

	/* tx bounce buffer required to prepend CH341 SPI stream command */
	tx_urb = ch341_alloc_urb(ch341, NULL, xfer->len + 1);
	if (!tx_urb) return -ENOMEM;
	cmd = tx_urb->transfer_buffer;

	cmd[0] = CH341_CMD_SPI_STREAM;
	if (xfer->tx_buf) {
		/* copy to tx bounce buffer */
		memcpy(&cmd[1], xfer->tx_buf, xfer->len);

		/* ch341 sends LSb first */
		if (!(spi->mode & SPI_LSB_FIRST))
			ch341_spi_buf_bitrev8(&cmd[1], xfer->len);
	} else {
		/* zeroes dummy tx buffer */
		memset(&cmd[1], 0, xfer->len);
	}

	if (!xfer->rx_buf) {
		/* dummy rx buffer required */
		rx_urb = ch341_alloc_urb(ch341, NULL, xfer->len);
	} else if (object_is_on_stack(xfer->rx_buf)) {
		/* rx bounce buffer required */
		rx_urb = ch341_alloc_urb(ch341, NULL, xfer->len);
	} else {
		/* wrap rx buffer (zero-copy) */
		rx_urb = ch341_alloc_urb(ch341, xfer->rx_buf, xfer->len);
	}

	if (!rx_urb) {
		ch341_free_urb(tx_urb);
		return -ENOMEM;
	}

	ret = ch341_usb_transfer_wait(ch341, tx_urb, rx_urb);

	if (!ret && xfer->rx_buf) {
		/* copy back if rx bounce buffer */
		if (object_is_on_stack(xfer->rx_buf))
			memcpy(xfer->rx_buf, rx_urb->transfer_buffer, rx_urb->actual_length);

		/* ch341 sends LSb first */
		if (!(spi->mode & SPI_LSB_FIRST))
			ch341_spi_buf_bitrev8(xfer->rx_buf, rx_urb->actual_length);
	}

	ch341_free_urb(tx_urb);
	ch341_free_urb(rx_urb);

	return ret;
}

#if IS_ENABLED(CONFIG_SPI_SPIDEV)
static int ch341_spi_create_spidev(struct ch341_device *ch341)
{
	struct spi_controller *ctlr = ch341->spi;
	struct spi_device *spidevs[CH341_NUM_CHIPSELECT];
	int i, ret;

	struct spi_board_info info = {
		.modalias = "m53cpld",
		.max_speed_hz = 1000000,
		.bus_num = ctlr->bus_num,
		.chip_select = 0,
		.mode = SPI_MODE_0,
	};

	for (i = 0; i < ARRAY_SIZE(spidevs); i++) {
		info.chip_select = i;
		spidevs[i] = spi_new_device(ctlr, &info);
		if (!spidevs[i]) {
			dev_warn(CH341_DEV, "Failed to create spidev\n");
			ret = -ENODEV;
			goto cleanup;
		}
	}

	return 0;

cleanup:
	while(--i) {
		spi_unregister_device(spidevs[i]);
                spidevs[i] = NULL;
	}
	return ret;
}
#else
static int ch341_spi_create_spidev(struct ch341_device *ch341)
{
	return 0;
}
#endif

int ch341_spi_probe(struct ch341_device *ch341)
{
	struct spi_controller *ctlr;
	struct fwnode_handle *fwnode = NULL;
	u32 num_chipselect = CH341_NUM_CHIPSELECT;
	int ret;

	/* child DT node required when using parent DT */
	if (CH341_DEV->fwnode) {
		fwnode = ch341_get_compatible_fwnode(ch341, "wch-ic,ch341-spi");
		if (!fwnode) {
			dev_info(CH341_DEV, "SPI controller disabled (no DT node found)\n");
			return 0;
		}

		ret = fwnode_property_read_u32(fwnode, "num-cs", &num_chipselect);
		if (ret)
			dev_warn(CH341_DEV, "Invalid num-cs: %d\n", ret);

		if (num_chipselect == 0) {
			dev_info(CH341_DEV, "SPI controller disabled (num-cs=0)\n");
			return 0;
		}
	}
	
	ctlr = devm_spi_alloc_host(CH341_DEV, 0);
	if (!ctlr)
		return -ENOMEM;

	ctlr->dev.fwnode = fwnode;
	if (fwnode && is_of_node(fwnode))
		ctlr->dev.of_node = to_of_node(fwnode);

	// TODO bitbanged streaming protocol: SPI_CPHA ?
	// TODO hardware supported: SPI_TX_DUAL | SPI_RX_DUAL;
	ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_3WIRE | SPI_LSB_FIRST | SPI_NO_CS | SPI_CS_HIGH;
	/*
	 CH341 requires full duplex RX/TX but this is managed by this driver
	 to support SPI_3WIRE which doesn't allocate dummy buffers
	 this also allows to save useless memcy on dummy buffers
	 so this is not required:
	ctlr->flags = SPI_CONTROLLER_MUST_RX | SPI_CONTROLLER_MUST_TX;
	*/
#if 0
	ctlr->flags |= SPI_CONTROLLER_MULTI_CS;
#endif
	ctlr->bits_per_word_mask = SPI_BPW_MASK(8);
	ctlr->min_speed_hz = 400;
	ctlr->max_speed_hz = 1e6;
	ctlr->num_chipselect = num_chipselect;
	ctlr->set_cs = ch341_spi_set_cs;
	ctlr->max_transfer_size = ch341_spi_max_transfer_size;
	ctlr->prepare_message = ch341_spi_prepare_message;
	ctlr->unprepare_message = ch341_spi_unprepare_message;
	ctlr->transfer_one = ch341_spi_transfer_one;

	spi_controller_set_devdata(ctlr, ch341);

	/* reserve SPI pins, except DOUT2 which isn't implemented (dual TX) */
	ch341->spi_mask = BIT(CH341_PIN_DIN) | BIT(CH341_PIN_DCK) | BIT(CH341_PIN_DOUT) |
			  GENMASK(ctlr->num_chipselect - 1, 0);
	set_mask_bits(&ch341->gpio_mask, ch341->spi_mask, ch341->spi_mask & CH341_OUT_MASK);
	/* all CS HIGH CS by default */
	set_mask_bits(&ch341->gpio_data, ch341->spi_mask, GENMASK(ctlr->num_chipselect - 1, 0));

	ch341->spi = ctlr;

	ret = spi_register_controller(ctlr);
	if (ret) {
		dev_err(CH341_DEV, "Failed to register SPI controller: %d\n", ret);
		goto err_free_spi;
	}

	/* only create spidev when there is no DT (assumed hot-plugged)
	 * developer can add wanted spidev in DT */
	if (!fwnode) ch341_spi_create_spidev(ch341);

	return 0;

err_free_spi:
	spi_controller_set_devdata(ctlr, NULL);
	if (fwnode) fwnode_handle_put(fwnode);
	spi_controller_put(ctlr);
	ch341->spi = NULL;
	return ret;
}

void ch341_spi_remove(struct ch341_device *ch341)
{
	struct fwnode_handle *fwnode;

	if (!ch341->spi)
		return;

	spi_unregister_controller(ch341->spi);
	spi_controller_set_devdata(ch341->spi, NULL);

	fwnode = ch341->spi->dev.fwnode;
	if (fwnode) fwnode_handle_put(fwnode);

	/* let devm clean up memory later
	 * spi_controller_put(ctlr); */

	ch341->spi = NULL;
}
