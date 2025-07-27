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

#define CH341_PKT_LEN 32

#define _WRITE_BYTE(val) \
	do { \
		if (cmd) cmd[*pos] = (val); \
		(*pos)++; \
	} while (0)

#define WRITE_BYTE(val) \
	do { \
		_WRITE_BYTE(val); \
		ch341_i2c_align_packet(cmd, pos); \
	} while (0)

#define COPY_BYTES(src, len) \
	do { \
		if (cmd) memcpy(&cmd[*pos], (src), (len)); \
		*pos += (len); \
		ch341_i2c_align_packet(cmd, pos); \
	} while (0)


static inline void ch341_i2c_align_packet(u8 *cmd, int *pos)
{
	if (*pos % CH341_PKT_LEN == CH341_PKT_LEN - 1) {
		_WRITE_BYTE(CH341_I2C_STM_END);
		_WRITE_BYTE(CH341_CMD_I2C_STREAM);
	}
}

static int ch341_i2c_build_cmd(u8 *cmd, unsigned int *pos, struct i2c_msg *msg, struct i2c_msg *prev, struct i2c_msg *next)
{
	if (!prev)
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
	
	if (!next)
		WRITE_BYTE(CH341_I2C_STM_END);

	return *pos;
}

static int ch341_i2c_xfer(struct i2c_adapter *adap,
				 struct i2c_msg *msgs, int num)
{
	struct ch341_device *ch341 = i2c_get_adapdata(adap);
	struct i2c_msg *prev, *next, *msg;
	struct urb *tx_urb = NULL, *rx_urb = NULL;
	int i, cmd_len, rx_len, ret;
	u8 *cmd = NULL, *resp;

	/* dry run to know buffer length */
	rx_len = cmd_len = 0;
	for (i = 0; i < num; i++) {
		prev = i - 1 >= 0 ? &msgs[i - 1] : NULL;
		next = i + 1 < num ? &msgs[i + 1] : NULL;
		msg = &msgs[i];

		ch341_i2c_build_cmd(NULL, &cmd_len, msg, prev, next);

		if (msg->flags & I2C_M_RD) {
			rx_len += msg->len;
		}
	}

	if (cmd_len == 0 && rx_len == 0)
		return -EINVAL;

	if (cmd_len > CH341_BUFFER_LENGTH) {
		dev_err(CH341_DEV, "transfer exceed capacity: %d\n", cmd_len);
		return -EINVAL;
	}

	/* allocate rx/tx buffers */
	tx_urb = ch341_alloc_urb(ch341, NULL, cmd_len);
	if (!tx_urb) return -ENOMEM;
	cmd = tx_urb->transfer_buffer;

	if (rx_len) {
		rx_urb = ch341_alloc_urb(ch341, NULL, rx_len);
		if (!rx_urb) {
			ch341_free_urb(tx_urb);
			return -ENOMEM;
		}
	}

	/* build command */
	cmd_len = 0;
	for (i = 0; i < num; i++) {
		prev = i - 1 >= 0 ? &msgs[i - 1] : NULL;
		next = i + 1 < num ? &msgs[i + 1] : NULL;
		msg = &msgs[i];

		ch341_i2c_build_cmd(cmd, &cmd_len, msg, prev, next);
	}

	ret = ch341_usb_transfer_wait(ch341, tx_urb, rx_urb);


	/* populate msgs responses */
	if (ret && rx_urb) {
		resp = rx_urb->transfer_buffer;

		if (rx_urb->actual_length != rx_len) {
			dev_warn(CH341_DEV, "read length mismatch: avail %d expected %d\n", rx_urb->actual_length, rx_len);
			rx_len = rx_urb->actual_length;
		}

		for (i = 0; i < num; i++) {
			msg = &msgs[i];

			if (msg->flags & I2C_M_RD && msg->len) {
				if (rx_len < msg->len) {
					dev_err(CH341_DEV, "can't read msg %d: avail %d expected %d\n", i, rx_len, msg->len);
					rx_len = 0;
					if (!ret) ret = i;
					continue;
				}

				memcpy(resp, msg->buf, msg->len);
				resp += msg->len;
				rx_len -= msg->len;
			}
		}
	}

	ch341_free_urb(tx_urb);
	if (rx_urb) ch341_free_urb(rx_urb);
 
	return ret ? ret : num;
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

	/* reserve I2C pins */
	ch341->i2c_mask = GENMASK(19, 18);

	ch341->i2c = i2c;
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
