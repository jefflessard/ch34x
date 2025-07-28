/* ==========================
 * I2C adapter implementation
 * ========================== */

#include "ch341-core.h"

#if 1
static inline u8 i2c_10bit_addr_hi_from_msg(const struct i2c_msg *msg)
{
		return 0xf0 | ((msg->addr & GENMASK(9, 8)) >> 7) | (msg->flags & I2C_M_RD);
}

static inline u8 i2c_10bit_addr_lo_from_msg(const struct i2c_msg *msg)
{
		return msg->addr & GENMASK(7, 0);
}
#endif

#define CH341_PKT_LEN      32
#define CH341_I2C_20KHZ     0
#define CH341_I2C_100KHZ    1
#define CH341_I2C_400KHZ    2
#define CH341_I2C_750KHZ    3

#define WRITE_BYTE(val) \
	do { \
		ch341_i2c_append_byte(cmd, pos, val); \
		ch341_i2c_align_packet(cmd, pos); \
	} while (0)

#define COPY_BYTES(src, len) \
	ch341_i2c_append_bytes(cmd, pos, (src), (len))

#define PAD_PACKET() \
	ch341_i2c_pad_packet(cmd, pos)

struct ch341_i2c_msg {
	struct completion *done;
	struct i2c_msg *msg;
	atomic_t *pending;
	int status;
};

static inline void ch341_i2c_append_byte(u8 *cmd, unsigned int *pos, u8 val)
{
	if (cmd) cmd[*pos] = (val) & 0xFF;
	(*pos)++;
}

static inline void ch341_i2c_align_packet(u8 *cmd, unsigned int *pos)
{
	if (*pos % CH341_PKT_LEN == CH341_PKT_LEN - 1) {
		ch341_i2c_append_byte(cmd, pos, CH341_I2C_STM_END);
		ch341_i2c_append_byte(cmd, pos, CH341_CMD_I2C_STREAM);
	}
}

static inline void ch341_i2c_pad_packet(u8 *cmd, unsigned int *pos)
{
	u16 len = CH341_PKT_LEN - *pos % CH341_PKT_LEN;

	if (len) {
		if (cmd) memset(&cmd[*pos], CH341_I2C_STM_END, len);
		*pos += (len);
		ch341_i2c_append_byte(cmd, pos, CH341_CMD_I2C_STREAM);
	}
}

static inline void ch341_i2c_append_bytes(u8 *cmd, unsigned int *pos, void *src, int len)
{
	if (cmd) memcpy(&cmd[*pos], src, len);
	*pos += (len);
	ch341_i2c_align_packet(cmd, pos);
}

static int ch341_i2c_build_cmd(u8 *cmd, struct i2c_msg *msg, struct i2c_msg *prev, struct i2c_msg *next)
{
	unsigned int len = 0;
	unsigned int *pos = &len;

	WRITE_BYTE(CH341_CMD_I2C_STREAM);

	if (!prev || !(msg->flags & I2C_M_NOSTART))
		WRITE_BYTE(CH341_I2C_STM_STA);

	if (!(msg->flags & I2C_M_NOSTART)) {
		if (msg->flags & I2C_M_TEN) {
			WRITE_BYTE(CH341_I2C_STM_OUT | 2);
			WRITE_BYTE(i2c_10bit_addr_hi_from_msg(msg));
			WRITE_BYTE(i2c_10bit_addr_lo_from_msg(msg));
		} else {
			WRITE_BYTE(CH341_I2C_STM_OUT | 1);
			WRITE_BYTE(i2c_8bit_addr_from_msg(msg));
		}
	}

	if (msg->len) {
		if (msg->flags & I2C_M_RD) {
			u16 len = msg->len;
			while (len > 0) {
				u16 qty = umin(CH341_PKT_LEN, len);
				WRITE_BYTE(CH341_I2C_STM_IN | qty);
				len -= qty;

				/* tx packet should be aligned
				 * on corresponding rx packet */
				if (len > 0) PAD_PACKET();
			}
		} else {
			u16 len = msg->len;
			u8 *buf = msg->buf;
			while (len > 0) {
				u16 qty = umin(CH341_PKT_LEN - *pos % CH341_PKT_LEN - 2, len); /* reserve out + end bytes */
				WRITE_BYTE(CH341_I2C_STM_OUT | qty);
				COPY_BYTES(buf, qty);
				len -= qty;
				buf += qty;
			}
		}
	}

	if (!next || !(next->flags & I2C_M_NOSTART))
		WRITE_BYTE(CH341_I2C_STM_STO);

	WRITE_BYTE(CH341_I2C_STM_END);

	return len;
}

#ifdef CH341_I2C_NOWAIT
static void ch341_i2c_msg_complete(struct ch341_transfer *xfer)
{
	struct ch341_device *ch341 = xfer->dev;
	struct ch341_i2c_msg *ctx = xfer->context;

	dev_dbg(CH341_DEV, "%s: %d", __func__, xfer->status);

	ctx->status = xfer->status;
	if (!ctx->status && xfer->rx_urb) {
		/* I2C_M_RD implied */
		if (xfer->rx_urb->actual_length != ctx->msg->len) {
			dev_err(CH341_DEV, "read length mismatch: %d, %d\n", ctx->msg->len, xfer->rx_urb->actual_length);
			ctx->status = -ENXIO;
		} else {
			memcpy(ctx->msg->buf, xfer->rx_urb->transfer_buffer, ctx->msg->len);
		}
	}

	/* signal completion on last item */
	if (atomic_dec_and_test(ctx->pending))
		complete(ctx->done);

	/* free urbs */
	ch341_complete(xfer);
}
#endif

static int ch341_i2c_process_msg(struct ch341_device *ch341, struct i2c_msg *msg, struct i2c_msg *prev, struct i2c_msg *next) {
#ifdef CH341_I2C_NOWAIT
	struct ch341_i2c_msg *item = &items[i];
#endif
	struct urb *tx_urb = NULL, *rx_urb = NULL;
	unsigned int tx_len = 0, rx_len = 0;
	int ret;

	/* dry run to know buffer length */
	tx_len = ch341_i2c_build_cmd(NULL, msg, prev, next);

	if (tx_len == 0) {
		dev_err(CH341_DEV, "empty command/no content\n");
		return  -ENOENT;
	}

#if 0
	if (cmd_len > CH341_BUFFER_LENGTH) {
		dev_err(CH341_DEV, "transfer exceed capacity: %d\n", cmd_len);
		return -EINVAL;
	}
#endif

	/* allocate tx buffer */
	tx_urb = ch341_alloc_urb(ch341, NULL, tx_len);
	if (!tx_urb) return -ENOMEM;

	/* build command */
	ch341_i2c_build_cmd(tx_urb->transfer_buffer, msg, prev, next);

	/* allocate rx buffer */
	if ((msg->flags & I2C_M_RD) && msg->len) {
		/* multiple of max usb packet length required
		 * for mutli packet responses */
		rx_len = ALIGN(msg->len, CH341_PKT_LEN);
		rx_urb = ch341_alloc_urb(ch341, NULL, rx_len);
		if (!rx_urb) {
			ret = -ENOMEM;
			goto free_urbs;
		}
	}

	/* send command */
#ifdef CH341_I2C_NOWAIT
	item->msg = msg;
	item->done = &done;
	item->pending = &pending;
	ret = ch341_usb_transfer(ch341, tx_urb, rx_urb, ch341_i2c_msg_complete, item);
#else
	ret = ch341_usb_transfer_wait(ch341, tx_urb, rx_urb);
#endif
	if (ret) goto free_urbs;

#ifndef CH341_I2C_NOWAIT
	if (rx_len) {
		if (rx_urb->actual_length != msg->len) {
			dev_err(CH341_DEV, "received length mismatch: %d, %d\n", msg->len, rx_urb->actual_length);
			ret = -ENXIO;
			goto free_urbs;
		} else {
			memcpy(msg->buf, rx_urb->transfer_buffer, msg->len);
		}
	}
#endif

free_urbs:
	if (tx_urb) ch341_free_urb(tx_urb);
	if (rx_urb) ch341_free_urb(rx_urb);

	return ret;
}

static int ch341_i2c_xfer(struct i2c_adapter *adapter,
				 struct i2c_msg *msgs, int num)
{
	struct ch341_device *ch341 = i2c_get_adapdata(adapter);
#ifdef CH341_I2C_NOWAIT
	struct ch341_i2c_msg *items;
	DECLARE_COMPLETION_ONSTACK(done);
	atomic_t pending;
#endif
	struct i2c_msg *msg, *prev, *next;
	int i, ret;

	if (!num) return -ENOENT;

#ifdef CH341_I2C_NOWAIT
	items = kcalloc(num, sizeof(*items), GFP_KERNEL);
        if (!items) return -ENOMEM;

	atomic_set(&pending, num);
	// TODO replace pending and done with USB anchor to kill all pending urbs when there is an error
#endif

	for (i = 0; i < num; i++) {
		prev = i - 1 >= 0 ? &msgs[i - 1] : NULL;
		next = i + 1 < num ? &msgs[i + 1] : NULL;
		msg = &msgs[i];

		ret = ch341_i2c_process_msg(ch341, msg, prev, next);
		if (ret) {
#ifdef CH341_I2C_NOWAIT
			atomic_dec(&pending);
#endif
			break;
		}
	}

#ifdef CH341_I2C_NOWAIT
	if (atomic_read(&pending)) {
		if (!wait_for_completion_timeout(&done, msecs_to_jiffies(2*CH341_TIMEOUT_MS))) {
			dev_err(CH341_DEV, "%s timeout\n", __func__);
			// TODO kill any pending urb
			ret = -ETIMEDOUT;
		}
	}

	if (!ret) {
		for (i = 0; i < num; i++) {
			if (items[i].status) {
				ret = items[i].status;
				goto free_items;
			}
		}
	}

	kfree(items);
#endif

	return ret ? ret : num;
}

int ch341_i2c_set_speed(struct ch341_device *ch341, u8 speed)
{
	struct urb *tx_urb;
	u8 *cmd;
	int ret;

	tx_urb = ch341_alloc_urb(ch341, NULL, 3);
	if (!tx_urb) return -ENOMEM;
	cmd = tx_urb->transfer_buffer;

	cmd[0] = CH341_CMD_I2C_STREAM;
	cmd[1] = CH341_I2C_STM_SET | (speed & 0x03);
	cmd[2] = CH341_I2C_STM_END;

	ret = ch341_usb_transfer(ch341, tx_urb, NULL, ch341_complete, NULL);
	if (ret < 0)
		dev_err(CH341_DEV, "Failed to set I2C speed: %d\n", ret);

	return 0;
}

static u32 ch341_i2c_func(struct i2c_adapter *adap)
{
	/* TODO check potential support for:
	 * I2C_FUNC_PROTOCOL_MANGLING:
	 *   - I2C_M_NO_RD_ACK: master ACK/NACK bit is skipped for read msgs
	 *   - I2C_M_IGNORE_NAK: treat NACK from client as ACK
	 *   - I2C_M_REV_DIR_ADDR: toggles the Rd/Wr bit
	 *   - I2C_M_STOP: force a STOP condition after the message
	 * I2C_FUNC_SMBUS_READ_BLOCK_DATA:
	 *   - I2C_M_RECV_LEN: message length will be first received byte */
	return I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR | I2C_FUNC_NOSTART| I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm ch341_i2c_algo = {
#if 1
	.master_xfer = ch341_i2c_xfer,
#else
	.xfer = ch341_i2c_xfer,
#endif
	.functionality = ch341_i2c_func,
};

int ch341_i2c_probe(struct ch341_device *ch341)
{
	struct i2c_adapter *i2c;
	struct fwnode_handle *fwnode;
	int ret;

	fwnode = ch341_get_compatible_fwnode(ch341, "wch,ch341-i2c");
#if 0
	if (!fwnode) {
		dev_info(CH341_DEV, "I2C adapter disabled\n");
		return 0;
	}
#endif

	i2c = devm_kzalloc(CH341_DEV, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->dev.fwnode = fwnode;
	if (fwnode && is_of_node(fwnode)) {
		i2c->dev.of_node = to_of_node(fwnode);
	}

	i2c->owner = THIS_MODULE;
	i2c->algo = &ch341_i2c_algo;
	i2c->dev.parent = CH341_DEV;
	snprintf(i2c->name, sizeof(i2c->name), "ch341-i2c");

	i2c_set_adapdata(i2c, ch341);
	ch341->i2c = i2c;

	/* reserve I2C pins */
	ch341->i2c_mask = GENMASK(19, 18);

	ch341_i2c_set_speed(ch341, CH341_I2C_100KHZ);

	ret = devm_i2c_add_adapter(CH341_DEV, ch341->i2c);
	if (ret) {
		dev_err(CH341_DEV, "Failed to register I2C adapter: %d\n", ret);
		return ret;
	}

	dev_info(CH341_DEV, "I2C adapter registered\n");

	return 0;
}

void ch341_i2c_remove(struct ch341_device *ch341) {
	struct fwnode_handle *fwnode;

	if (!ch341->i2c)
		return;

	i2c_set_adapdata(ch341->i2c, NULL);

	fwnode = ch341->i2c->dev.fwnode;
	if (fwnode)
		fwnode_handle_put(fwnode);

	/* not required since using devm_*:
	 * i2c_del_adapter(ch341->i2c);
	 * kfree(ch341->i2c); */

	ch341->i2c = NULL;
}
