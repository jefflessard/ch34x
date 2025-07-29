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

static int ch341_i2c_build_cmd(u8 *cmd, struct i2c_msg *msgs, int num)
{
	unsigned int cmd_len = 0, read_len = 0, msg_len, msg_qty;
	unsigned int *pos = &cmd_len;
	struct i2c_msg *msg;
	u8 *msg_buf;

	WRITE_BYTE(CH341_CMD_I2C_STREAM);

	for (msg = msgs; msg < msgs + num; msg++) {
		bool first = msg == msgs;
		bool last = msg == msgs + num - 1;
		u8 rev_dir_addr = !!(msg->flags & I2C_M_REV_DIR_ADDR);

		if (first || !(msg->flags & I2C_M_NOSTART))
			WRITE_BYTE(CH341_I2C_STM_STA);

		if (!(msg->flags & I2C_M_NOSTART)) {
			if (msg->flags & I2C_M_TEN) {
				WRITE_BYTE(CH341_I2C_STM_OUT | 2);
				WRITE_BYTE(i2c_10bit_addr_hi_from_msg(msg) ^ rev_dir_addr);
				WRITE_BYTE(i2c_10bit_addr_lo_from_msg(msg));
			} else {
				WRITE_BYTE(CH341_I2C_STM_OUT | 1);
				WRITE_BYTE(i2c_8bit_addr_from_msg(msg) ^ rev_dir_addr);
			}
		}

		if (msg->len) {
			if (msg->flags & I2C_M_RD) {
				msg_len = msg->len;
				while (msg_len > 0) {
					/* tx packet being executed must have
					 * sufficent room for reading length
					 * in its corresponding rx packet */
					if (read_len > 0 &&
					    read_len % CH341_PKT_LEN == 0)
						PAD_PACKET();

					msg_qty = umin(CH341_PKT_LEN -
						       read_len % CH341_PKT_LEN,
						       msg_len);
					WRITE_BYTE(CH341_I2C_STM_IN | msg_qty);
					msg_len -= msg_qty;
					read_len += msg_qty;
				}
			} else {
				msg_len = msg->len;
				msg_buf = msg->buf;
				while (msg_len > 0) {
					/* reserve out + end bytes */
					msg_qty = umin(CH341_PKT_LEN -
						       *pos % CH341_PKT_LEN - 2,
						       msg_len);
					WRITE_BYTE(CH341_I2C_STM_OUT | msg_qty);
					COPY_BYTES(msg_buf, msg_qty);
					msg_len -= msg_qty;
					msg_buf += msg_qty;
				}
			}
		}

		if (last || msg->flags & I2C_M_STOP)
			WRITE_BYTE(CH341_I2C_STM_STO);
	}

	WRITE_BYTE(CH341_I2C_STM_END);

	return cmd_len;
}

static int ch341_i2c_read_len(struct i2c_msg *msgs, int num)
{
	unsigned int len = 0;
	struct i2c_msg *msg;

	for (msg = msgs; msg < msgs + num; msg++) {
		if (msg->flags & I2C_M_RD)
			len += msg->len;
	}

	return len;
}

static void ch341_i2c_fill_read(u8 *buf, struct i2c_msg *msgs, int num)
{
	struct i2c_msg *msg;

	for (msg = msgs; msg < msgs + num; msg++) {
		if (msg->flags & I2C_M_RD && msg->len) {
			memcpy(msg->buf, buf, msg->len);
			buf += msg->len;
		}
	}
}

/* generates a single usb transfer for the entire
 * i2c transfer to avoid breaking its atomicity
 * that could be caused by usb scheduling */
static int ch341_i2c_xfer(struct i2c_adapter *adapter,
				 struct i2c_msg *msgs, int num)
{
	struct ch341_device *ch341 = i2c_get_adapdata(adapter);
	struct urb *tx_urb = NULL, *rx_urb = NULL;
	unsigned int tx_len = 0, rx_len = 0, read_len = 0;
	int ret;

	if (!num) return -ENOENT;

	/* dry run to know buffer length */
	tx_len = ch341_i2c_build_cmd(NULL, msgs, num);

	if (tx_len == 0) {
		dev_err(CH341_DEV, "empty command/no content\n");
		return  -ENOENT;
	}

	/* allocate tx buffer */
	tx_urb = ch341_alloc_urb(ch341, NULL, tx_len);
	if (!tx_urb) return -ENOMEM;

	/* build command */
	ch341_i2c_build_cmd(tx_urb->transfer_buffer, msgs, num);

	/* allocate rx buffer */
	read_len = ch341_i2c_read_len(msgs, num);
	if (read_len > 0) {
		/* multiple of max usb packet length required
		 * for mutli packet responses */
		rx_len = ALIGN(read_len, CH341_PKT_LEN);
		rx_urb = ch341_alloc_urb(ch341, NULL, rx_len);
		if (!rx_urb) {
			ret = -ENOMEM;
			goto free_urbs;
		}
	}

	/* send command */
	ret = ch341_usb_transfer_wait(ch341, tx_urb, rx_urb);
	if (ret) goto free_urbs;

	if (rx_urb) {
		if (rx_urb->actual_length != read_len) {
			dev_err(CH341_DEV, "read length mismatch: %d expected, %d received\n", read_len, rx_urb->actual_length);
			ret = -ENXIO;
			goto free_urbs;
		}

		ch341_i2c_fill_read(rx_urb->transfer_buffer, msgs, num);
	}

free_urbs:
	if (tx_urb) ch341_free_urb(tx_urb);
	if (rx_urb) ch341_free_urb(rx_urb);

	return ret ? ret : num;
}

static u32 ch341_i2c_func(struct i2c_adapter *adap)
{
	/* Supported:
	 * I2C_FUNC_PROTOCOL_MANGLING:
	 *   - I2C_M_STOP: force a STOP condition after the message
	 *   - I2C_M_REV_DIR_ADDR: toggles the Rd/Wr bit
	 *
	 * Not supported (ignored):
	 * I2C_FUNC_PROTOCOL_MANGLING:
	 *   - I2C_M_NO_RD_ACK: master ACK/NACK bit is skipped for read msgs
	 *   - I2C_M_IGNORE_NAK: treat NACK from client as ACK
	 *
	 * Not supported:
	 * I2C_FUNC_SMBUS_READ_BLOCK_DATA:
	 *   - I2C_M_RECV_LEN: message length will be first received byte */
	return I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR | I2C_FUNC_NOSTART |
		I2C_FUNC_PROTOCOL_MANGLING | I2C_FUNC_SMBUS_EMUL;
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
	struct i2c_timings timings;
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

	/* configure I2C bus speed */
	i2c_parse_fw_timings(&i2c->dev, &timings, true);
	if (timings.bus_freq_hz < 100000)
		ch341_stream_config(ch341, CH341_I2C_SPEED_MASK,
				    CH341_I2C_20KHZ);
	else if (timings.bus_freq_hz < 400000)
		ch341_stream_config(ch341, CH341_I2C_SPEED_MASK,
				    CH341_I2C_100KHZ);
	else if (timings.bus_freq_hz < 750000)
		ch341_stream_config(ch341, CH341_I2C_SPEED_MASK,
				    CH341_I2C_400KHZ);
	else ch341_stream_config(ch341, CH341_I2C_SPEED_MASK, CH341_I2C_750KHZ);

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
