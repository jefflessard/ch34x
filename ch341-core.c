#include "ch341-core.h"

inline int ch341_i2c_probe(struct ch341_device *ch341){return 0;}
inline void ch341_i2c_remove(struct ch341_device *ch341){}

/* usb wrappers */
struct urb *ch341_alloc_urb(struct ch341_device *ch341, void *buf, int len) {
	struct urb *urb;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		return NULL;

	/* allocate new buffer if none
	 * otherwise, wrap existing buffer */
	if (!buf) {
		buf = usb_alloc_coherent(ch341->udev, len, GFP_KERNEL, &urb->transfer_dma);
		if (!buf) {
			usb_free_urb(urb);
			return NULL;
		}

		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	}

	urb->dev = ch341->udev;
	urb->transfer_buffer = buf;
	urb->transfer_buffer_length = len;

	return urb;
}

void ch341_free_urb(struct urb *urb) {
	/* only free allocated buffers
	 * wraped buffers are managed by caller */
	if (urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP) {
		usb_free_coherent(urb->dev, urb->transfer_buffer_length, urb->transfer_buffer, urb->transfer_dma);
	}

	usb_free_urb(urb);
}

static void ch341_urb_complete(struct urb *urb)
{
	struct ch341_transfer *xfer = urb->context;
	struct ch341_device *ch341 = xfer->dev;
	int remaining;

	/* Decrement pending URB count */
	remaining = atomic_dec_return(&xfer->pending_urbs);

	/* check results */
	if (urb->status) {
		dev_err(CH341_DEV, "usb_submit_urb failed: %d\n", urb->status);
		xfer->status = urb->status;
	} else if (urb == xfer->tx_urb && urb->transfer_buffer_length != urb->actual_length) {
		dev_err(CH341_DEV, "usb_submit_urb tx length mismatch: %d, %d\n", urb->transfer_buffer_length, urb->actual_length);
		xfer->status = -ENXIO;
	} else {
		dev_dbg(CH341_DEV, "usb_submit_urb completed\n");

		if (urb == xfer->rx_urb) {
			dev_dbg(CH341_DEV, "rx: %*ph\n", urb->actual_length, urb->transfer_buffer);
		}
	}

	dev_dbg(CH341_DEV, "transfer remaining %d\n", remaining);

	/* Call user callback only when ALL URBs complete */
	if (remaining == 0) {
		if (xfer->complete)
			xfer->complete(xfer);

		/* clean up xfer */
		if (xfer->tx_urb)
			xfer->tx_urb->context = NULL;
		if (xfer->rx_urb)
			xfer->rx_urb->context = NULL;
		kfree(xfer);
	}
}

/* Full-duplex transfer */
int ch341_usb_transfer(struct ch341_device *ch341,
		struct urb *tx_urb, struct urb *rx_urb,
		void (*complete)(struct ch341_transfer *xfer),
		void *context)
{
	struct ch341_transfer *xfer;
	int ret;

	dev_dbg(CH341_DEV, "%s: tx %*ph\n", __func__, tx_urb->transfer_buffer_length, tx_urb->transfer_buffer);

	xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
	if (!xfer)
		return -ENOMEM;

	/* Setup TX URB */
	if (tx_urb) {
		tx_urb->pipe = ch341->tx_pipe;
		tx_urb->complete = ch341_urb_complete;
		tx_urb->context = xfer;
		atomic_inc(&xfer->pending_urbs);
	}

	/* Setup TX URB */
	if (rx_urb) {
		rx_urb->pipe = ch341->rx_pipe;
		rx_urb->complete = ch341_urb_complete;
		rx_urb->context = xfer;
		atomic_inc(&xfer->pending_urbs);
	}

	/* Setup transfer */
	xfer->dev = ch341;
	xfer->tx_urb = tx_urb;
	xfer->rx_urb = rx_urb;
	xfer->complete = complete;
	xfer->context = context;
	if (WARN_ON(!xfer->complete)) {
		xfer->complete = ch341_complete;
	}

	/* Submit both URBs simultaneously for full-duplex */
	if (tx_urb) {
		ret = usb_submit_urb(tx_urb, GFP_KERNEL);
		if (ret) {
			tx_urb->context = NULL;
			if (rx_urb) rx_urb->context = NULL;
			kfree(xfer);
			return ret;
		}
	}

	if (rx_urb) {
		ret = usb_submit_urb(rx_urb, GFP_KERNEL);
		if (ret) {
			if (tx_urb) {
				usb_kill_urb(tx_urb);
				tx_urb->context = NULL;
			}
			rx_urb->context = NULL;
			kfree(xfer);
			return ret;
		}
	}

	return 0;
}

/* sync/wait completion */
static void ch341_sync_complete(struct ch341_transfer *xfer)
{
    struct completion *done = xfer->context;
    complete(done);
}

int ch341_usb_transfer_wait(struct ch341_device *dev,
		struct urb *tx_urb, struct urb *rx_urb)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int ret;
	
	ret = ch341_usb_transfer(dev, tx_urb, rx_urb, ch341_sync_complete, &done);
	if (ret) return ret;

	if (!wait_for_completion_timeout(&done, msecs_to_jiffies(CH341_TIMEOUT_MS))) {
		if (tx_urb) usb_kill_urb(tx_urb);
		if(rx_urb) usb_kill_urb(rx_urb);
		if ((tx_urb && tx_urb->status == -ENOENT) ||
		    (rx_urb && rx_urb->status == -ENOENT))
			return -ETIMEDOUT;
	}

	if (tx_urb && tx_urb->status)
		return tx_urb->status;

	if (rx_urb && rx_urb->status)
		return rx_urb->status;

	return 0;
}

/* fwnode helper functions */
struct fwnode_handle* ch341_get_compatible_fwnode(struct ch341_device *ch341, const char *compatible)
{
	struct fwnode_handle *child, *result = NULL;

	if (!CH341_DEV->fwnode)
		return NULL;

	fwnode_for_each_available_child_node(CH341_DEV->fwnode, child) {
		if (fwnode_device_is_compatible(child, compatible)) {
			result = child;
		}
	}

	if (!result)
		return NULL;

	return fwnode_handle_get(result);
}

/* USB control transfer wrappers */
static int ch341_control_read(struct ch341_device *ch341, u8 request,
			      u16 value, u16 index, void *data, u16 size)
{
	int ret;

	ret = usb_control_msg(ch341->udev,
			      usb_rcvctrlpipe(ch341->udev, 0), request,
			      USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			      value, index, data, size, CH341_TIMEOUT_MS);

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
			      value, index, data, size, CH341_TIMEOUT_MS);

	if (ret < 0)
		dev_err(CH341_DEV, "Control write failed: %d\n", ret);

	return ret;
}

static int ch341_init_device(struct ch341_device *ch341)
{
	int ret;

	/* Get device version */
	ret = ch341_control_read(ch341, CH341_CTRL_VERSION, 0, 0,
				 &ch341->ic_version, sizeof(ch341->ic_version));
	if (ret < 0) {
		dev_warn(CH341_DEV, "Failed to get version\n");
		ch341->ic_version = 0;
	}

	dev_info(CH341_DEV, "CH341 version 0x%04x\n", ch341->ic_version);

	return 0;
}

/* USB topology matching */
static bool ch341_usb_instance_matches(struct usb_device *udev, struct fwnode_handle *fwnode)
{
	u32 dt_port, dt_hub_tier;
	int actual_hub_tier;

	/* Check port number */
	if (!fwnode_property_read_u32(fwnode, "usb-port", &dt_port)) {
		if (dt_port != udev->portnum)
			return false;
	}

	/* Check hub tier (0 = root hub, 1 = first tier, etc.) */
	if (!fwnode_property_read_u32(fwnode, "usb-hub-tier", &dt_hub_tier)) {
		/* Calculate actual hub tier */
		actual_hub_tier = 0;
		struct usb_device *parent = udev->parent;
		while (parent && parent != udev->bus->root_hub) {
			actual_hub_tier++;
			parent = parent->parent;
		}

		if (dt_hub_tier != actual_hub_tier)
			return false;
	}

	return true;
}

/* Find DT/ACPI node for this USB device */
static struct fwnode_handle *ch341_find_usb_fwnode(struct usb_device *udev)
{
	struct usb_device *parent = udev->parent;
	struct fwnode_handle *usb_fwnode, *child;
	u8 compatible[32];

	if (!parent)
		return NULL;

	usb_fwnode = dev_fwnode(&parent->dev);
	if (!usb_fwnode)
		return NULL;

	snprintf(compatible, sizeof(compatible), "usb%04x,%04x",
			le16_to_cpu(udev->descriptor.idVendor),
			le16_to_cpu(udev->descriptor.idProduct));

	fwnode_for_each_available_child_node(usb_fwnode, child) {
		if (!fwnode_device_is_compatible(child, compatible))
			continue;

		/* Match by USB topology */
		if (ch341_usb_instance_matches(udev, child))
			/* caller must put fwnode handle */
			return child;
	}

	return NULL;
}

static int ch341_probe(struct usb_interface *interface,
		const struct usb_device_id *id)
{
	struct device *dev = &interface->dev;
	struct usb_device *udev = interface_to_usbdev(interface);
	struct usb_endpoint_descriptor *bulk_in;
	struct usb_endpoint_descriptor *bulk_out;
	struct fwnode_handle *fwnode;
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

	fwnode = ch341_find_usb_fwnode(ch341->udev);
	if (fwnode) {
		dev_info(CH341_DEV, "Found USB DT node\n");
		device_set_node(CH341_DEV, fwnode);
	}

	ch341_init_device(ch341);

	/* Initialize GPIO state */
	ch341->gpio_mask = CH341_GPIO_OUT_MASK;
	ch341->gpio_data = 0;   /* All pins default to low */

	ret = ch341_spi_probe(ch341);
	if (ret) {
		dev_err(dev, "Failed to initialize spi: %d\n", ret);
		goto err_put_fwnode;
	}

	ret = ch341_gpio_probe(ch341);
	if (ret) {
		dev_err(dev, "Failed to initialize gpio: %d\n", ret);
		goto err_spi_remove;
	}

	return 0;

err_spi_remove:
	ch341_spi_remove(ch341);
err_put_fwnode:
	if (CH341_DEV->fwnode)
		fwnode_handle_put(CH341_DEV->fwnode);
err_put_dev:
	usb_put_dev(ch341->udev);
	return ret;
}

static void ch341_disconnect(struct usb_interface *interface)
{
	struct ch341_device *ch341 = usb_get_intfdata(interface);

	if (!ch341)
		return;

	ch341_spi_remove(ch341);
	ch341_gpio_remove(ch341);

	if (CH341_DEV->fwnode)
		fwnode_handle_put(CH341_DEV->fwnode);

	usb_set_intfdata(interface, NULL);
	usb_put_dev(ch341->udev);
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
MODULE_DESCRIPTION("CH341 USB Bridge");
MODULE_LICENSE("GPL v2");
