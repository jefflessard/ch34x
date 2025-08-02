#include "ch341-core.h"
#include <linux/spinlock.h>
#include <linux/irqdomain.h>


#define CH341_INT_IN_BUF_LEN	7


extern int ch341_gpio_get_status(struct ch341_device *ch341, u32 *status);


static unsigned int state_poll = 100;
module_param(state_poll, uint, 0644);
MODULE_PARM_DESC(state_poll, "GPIO state polling refresh delay in ms (default: 100, 0 to disable)");

static void ch341_state_poll_state(struct work_struct *work)
{
	struct ch341_device *ch341 = container_of(work, struct ch341_device, state_poll_work.work);

	ch341_gpio_get_status(ch341, &ch341->pins_state);

	if (state_poll > 0) {
		schedule_delayed_work(&ch341->state_poll_work,
				      msecs_to_jiffies(state_poll));
	}
}

static void ch341_irq_handle(struct ch341_device *ch341, u32 current_state)
{
	unsigned long changed_pins;
	raw_spinlock_t lock = __RAW_SPIN_LOCK_UNLOCKED(lock);
	unsigned int pin;

	/* Find changed pins */
	changed_pins = current_state ^ ch341->pins_state;

	dev_dbg(CH341_DEV, "%s: prev=%x, new=%x, changed=%lx\n", __func__, ch341->pins_state, current_state, changed_pins);

	/* Update GPIO state */
	ch341->pins_state = current_state;

	/* Always trigger irq on the interrupt pin */
	changed_pins |= BIT(CH341_PIN_INT);

	/* "fake" raw spinlock workaround */
	scoped_guard(raw_spinlock_irq, &lock) {
		/* Handle interrupts for each changed pin */
		for_each_set_bit(pin, &changed_pins, CH341_PIN_END) {
			dev_dbg(CH341_DEV, "Triggering virtual IRQ for pin %u\n", pin);

			generic_handle_domain_irq(ch341->gpio_chip->irq.domain, pin);
		}
	}
}

static void ch341_irq_complete_interrupt(struct urb *urb)
{
	struct ch341_device *ch341 = urb->context;
	u32 state;
	u8 *buf;

	dev_dbg(CH341_DEV, "%s: status=%d, len=%d data=%*ph\n", __func__, urb->status, urb->actual_length, urb->actual_length, urb->transfer_buffer);

	buf = urb->transfer_buffer;

	state = (buf[3] << 16) |
		 (buf[1] << 8) |
		 (buf[2]);

	ch341_irq_handle(ch341, state);
	
	/* Schedule delayed work to update state after GPIOs settle */
	if (state_poll > 0) {
		schedule_delayed_work(&ch341->state_poll_work,
				      msecs_to_jiffies(state_poll));
	}

	usb_submit_urb(urb, GFP_ATOMIC);
}

static void ch341_irq_mask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct ch341_device *ch341 = gpiochip_get_data(gc);

	dev_dbg(CH341_DEV, "IRQ %u (pin %lu) masked\n", data->irq, data->hwirq);

	gpiochip_disable_irq(gc, data->hwirq);
}

static void ch341_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct ch341_device *ch341 = gpiochip_get_data(gc);

	gpiochip_enable_irq(gc, data->hwirq);

	dev_dbg(CH341_DEV, "IRQ %u (pin %lu) unmasked\n", data->irq, data->hwirq);
}

static const struct irq_chip ch341_irq_chip = {
	.name = DRIVER_NAME,
        .irq_ack = NULL, /* not needed, source won't double trigger */
	.irq_mask = ch341_irq_mask,
	.irq_unmask = ch341_irq_unmask,
        .irq_set_type = NULL, /* Only IRQ_TYPE_NONE is supported*/
	.flags = IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

int ch341_irq_probe(struct ch341_device *ch341)
{
	struct usb_endpoint_descriptor *int_in;
	struct gpio_irq_chip *girq;
	unsigned int int_in_pipe;
	int int_in_num, ret;

	ret = usb_find_int_in_endpoint(ch341->intf->cur_altsetting, &int_in);
	if (ret) {
		dev_err(CH341_DEV, "interrupt input USB endpoint not found\n");
		return ret;
	}

	int_in_num = usb_endpoint_num(int_in);
	int_in_pipe = usb_rcvintpipe(ch341->udev, int_in_num);

	ch341->int_in_urb = ch341_alloc_urb(ch341, NULL, CH341_INT_IN_BUF_LEN);
	if (!ch341->int_in_urb) {
		return -ENOMEM;
	}

	usb_fill_int_urb(ch341->int_in_urb, ch341->udev, int_in_pipe,
			 ch341->int_in_urb->transfer_buffer,
			 ch341->int_in_urb->transfer_buffer_length,
			 ch341_irq_complete_interrupt, ch341,
			 int_in->bInterval);

	/* Configure the GPIO IRQ chip */
	girq = &ch341->gpio_chip->irq;
	gpio_irq_chip_set_chip(girq, &ch341_irq_chip);
	girq->parent_handler = NULL;  /* We handle muxing ourselves */
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_simple_irq;

	INIT_DELAYED_WORK(&ch341->state_poll_work, ch341_state_poll_state);

	ret = usb_submit_urb(ch341->int_in_urb, GFP_KERNEL);
	if (ret) {
		dev_err(CH341_DEV, "Failed to submit interrupt urb: %d\n", ret);
		goto err_free_urb;
	}

	return 0;

err_free_urb:
	ch341_free_urb(ch341->int_in_urb);
	ch341->int_in_urb = NULL;

	return ret;
}

void ch341_irq_remove(struct ch341_device *ch341)
{
	if (!ch341->int_in_urb)
		return;

	cancel_delayed_work_sync(&ch341->state_poll_work);

	usb_kill_urb(ch341->int_in_urb);
	ch341_free_urb(ch341->int_in_urb);
	ch341->int_in_urb = NULL;
}
