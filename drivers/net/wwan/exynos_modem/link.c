// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos integrated modem: SIPC5 over shared memory
 *
 * The application processor and the modem share a region of RAM with two
 * pairs of ring buffers: one for control messages ("FMT") and one for
 * everything else ("RAW": packet data, the remote file system, boot and dump
 * traffic). A mailbox interrupt with a message word tells the other side what
 * happened to the rings, or carries a link command.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include "modem.h"

/* Layout of the IPC region */
#define MAP_MAGIC		0x0
#define MAP_ACCESS		0x4
#define MAP_FMT_TX_HEAD		0x8
#define MAP_FMT_TX_TAIL		0xc
#define MAP_FMT_RX_HEAD		0x10
#define MAP_FMT_RX_TAIL		0x14
#define MAP_RAW_TX_HEAD		0x18
#define MAP_RAW_TX_TAIL		0x1c
#define MAP_RAW_RX_HEAD		0x20
#define MAP_RAW_RX_TAIL		0x24
#define MAP_CLK_TABLE		0x28
#define MAP_FMT_TX_BUF		0x1000
#define MAP_FMT_RX_BUF		0x2000
#define MAP_RAW_TX_BUF		0x3000
#define MAP_RAW_RX_BUF		0x200000
#define MAP_SIZE		0x400000

/* Message word that accompanies the message interrupt */
#define MSG_VALID		BIT(7)
#define MSG_CMD_VALID		BIT(6)
#define MSG_CMD			GENMASK(5, 0)
#define MSG_REQ_ACK_FMT		BIT(5)
#define MSG_REQ_ACK_RAW		BIT(4)
#define MSG_RES_ACK_FMT		BIT(3)
#define MSG_RES_ACK_RAW		BIT(2)
#define MSG_SEND_FMT		BIT(1)
#define MSG_SEND_RAW		BIT(0)

#define CMD_INIT_START		0x1
#define CMD_INIT_END		0x2
#define CMD_CRASH_RESET		0x7
#define CMD_PHONE_START		0x8
#define CMD_CRASH_EXIT		0x9
#define CMD_PIF_INIT_DONE	0xd

/* Status word of the modem */
#define STATUS_TX_SUSPEND	BIT(4)

#define TXQ_DEPTH		1024
#define TX_RETRY_NS		NSEC_PER_MSEC
#define TX_REQ_ACK_PERIOD	16	/* retries between two ack requests */
#define UDL_TX_RETRIES		20
#define UDL_TX_RETRY_MS		50
#define CRASH_ACK_TIMEOUT	(10 * HZ)

static u32 link_magic(struct exmodem *md)
{
	return le32_to_cpu(READ_ONCE(*md->magic));
}

static void link_set_magic(struct exmodem *md, u32 magic)
{
	WRITE_ONCE(*md->magic, cpu_to_le32(magic));
}

static u32 link_access(struct exmodem *md)
{
	return le32_to_cpu(READ_ONCE(*md->access));
}

static void link_set_access(struct exmodem *md, u32 access)
{
	WRITE_ONCE(*md->access, cpu_to_le32(access));
}

static void link_send_irq(struct exmodem *md, u16 msg)
{
	unsigned long flags;

	spin_lock_irqsave(&md->msg_lock, flags);
	exmodem_issr_write(md, EXMODEM_ISSR_AP2CP_MSG, MSG_VALID | msg);
	exmodem_mbox_ring(md, EXMODEM_MBOX_MSG);
	spin_unlock_irqrestore(&md->msg_lock, flags);
}

static void link_send_cmd(struct exmodem *md, u16 cmd)
{
	link_send_irq(md, MSG_CMD_VALID | cmd);
}

static bool link_ipc_active(struct exmodem *md)
{
	if (exmodem_get_state(md) != EXMODEM_ONLINE)
		return false;

	if (link_magic(md) != EXMODEM_MAGIC_IPC || link_access(md) != 1)
		return false;

	return !atomic_read(&md->forced_crash);
}

/* Called with the tx_lock of the RAW rings held */
static void link_update_flow(struct exmodem *md)
{
	const struct exmodem_frontend *frontend = READ_ONCE(md->frontend);
	bool stop = md->tx_suspended || md->ipc[EXMODEM_IPC_RAW].stopped;

	if (stop == md->tx_flow_stopped)
		return;

	md->tx_flow_stopped = stop;
	if (frontend && frontend->tx_flow)
		frontend->tx_flow(md, stop);
}

static void link_purge_txq(struct exmodem *md)
{
	unsigned long flags;
	int i;

	for (i = 0; i < EXMODEM_IPC_COUNT; i++) {
		struct exmodem_ipc_dev *dev = &md->ipc[i];

		spin_lock_irqsave(&dev->tx_lock, flags);
		__skb_queue_purge(&dev->skb_txq);
		dev->stopped = false;
		dev->busy_count = 0;
		spin_unlock_irqrestore(&dev->tx_lock, flags);
	}
}

static void link_reset_rings(struct exmodem *md)
{
	int i;

	for (i = 0; i < EXMODEM_IPC_COUNT; i++) {
		sipc_ring_reset(&md->ipc[i].txq);
		sipc_ring_reset(&md->ipc[i].rxq);
	}
}

/*
 * Tell the modem which frequencies it can ask for. Its performance requests
 * are 1-based indices into these tables.
 */
static void link_write_clk_table(struct exmodem *md)
{
	static const char names[EXMODEM_CLK_COUNT][4] = {
		"CL0", "CL1", "MIF", "INT",
	};
	const struct exmodem_soc_data *soc = md->soc;
	__le32 *p = md->ipc_base + MAP_CLK_TABLE;
	__le32 *count = p + 1;
	u32 tables = 0;
	int i, j;

	memcpy(p, "CT0", 4);
	p += 2;

	for (i = 0; i < EXMODEM_CLK_COUNT; i++) {
		if (!soc->clk_table_len[i])
			continue;

		memcpy(p++, names[i], 4);
		*p++ = cpu_to_le32(soc->clk_table_len[i]);
		tables++;
	}

	for (i = 0; i < EXMODEM_CLK_COUNT; i++)
		for (j = 0; j < soc->clk_table_len[i]; j++)
			*p++ = cpu_to_le32(soc->clk_tables[i][j]);

	*count = cpu_to_le32(tables);
}

/* Called with link_lock held */
static void link_send_init_end(struct exmodem *md)
{
	link_write_clk_table(md);
	link_send_cmd(md, CMD_INIT_END);
	md->init_end_sent = true;
}

/**
 * exmodem_link_prepare() - set up what the modem reads when it starts
 * @md: the modem, powered off
 */
void exmodem_link_prepare(struct exmodem *md)
{
	link_write_clk_table(md);
}

/*
 * Transmit
 */

/* Called with dev->tx_lock held. Returns -EIO if the ring is corrupted. */
static int link_tx_flush(struct exmodem *md, struct exmodem_ipc_dev *dev)
{
	struct sk_buff *skb;
	bool sent = false;
	int ret = 0;

	while (!dev->stopped && (skb = skb_peek(&dev->skb_txq))) {
		ret = sipc_ring_write(&dev->txq, skb->data, skb->len);
		if (ret)
			break;

		__skb_unlink(skb, &dev->skb_txq);
		consume_skb(skb);
		sent = true;
	}

	if (ret == -ENOSPC) {
		ret = 0;
		if (dev->flow_ctrl) {
			/* Ask the modem to tell us when it has caught up */
			dev->stopped = true;
			dev->busy_count = 0;
			link_update_flow(md);
			link_send_irq(md, dev->req_ack_mask |
				      (sent ? dev->send_mask : 0));
			sent = false;
		} else {
			/* Make the modem look at the ring */
			sent = true;
		}
		hrtimer_start(&md->tx_timer, ns_to_ktime(TX_RETRY_NS),
			      HRTIMER_MODE_REL);
	}

	if (sent)
		link_send_irq(md, dev->send_mask);

	return ret;
}

static void link_tx_restart(struct exmodem *md, struct exmodem_ipc_dev *dev)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&dev->tx_lock, flags);
	if (dev->stopped) {
		dev->stopped = false;
		link_update_flow(md);
	}
	ret = link_tx_flush(md, dev);
	spin_unlock_irqrestore(&dev->tx_lock, flags);

	if (ret)
		exmodem_link_force_crash(md, EXMODEM_CRASH_BY_AP,
					 "transmit ring corrupted");
}

static enum hrtimer_restart link_tx_timer(struct hrtimer *timer)
{
	struct exmodem *md = container_of(timer, struct exmodem, tx_timer);
	bool again = false;
	unsigned long flags;
	int ret = 0;
	int i;

	if (!link_ipc_active(md))
		return HRTIMER_NORESTART;

	for (i = 0; i < EXMODEM_IPC_COUNT && !ret; i++) {
		struct exmodem_ipc_dev *dev = &md->ipc[i];

		spin_lock_irqsave(&dev->tx_lock, flags);
		if (dev->stopped) {
			if (sipc_ring_empty(&dev->txq)) {
				dev->stopped = false;
				link_update_flow(md);
			} else if (!(++dev->busy_count % TX_REQ_ACK_PERIOD)) {
				link_send_irq(md, dev->req_ack_mask);
			}
		}
		ret = link_tx_flush(md, dev);
		if (dev->stopped || !skb_queue_empty(&dev->skb_txq))
			again = true;
		spin_unlock_irqrestore(&dev->tx_lock, flags);
	}

	if (ret) {
		exmodem_link_force_crash(md, EXMODEM_CRASH_BY_AP,
					 "transmit ring corrupted");
		return HRTIMER_NORESTART;
	}

	/* link_tx_flush() may have started the timer already */
	if (!again || hrtimer_is_queued(timer))
		return HRTIMER_NORESTART;

	hrtimer_forward_now(timer, ns_to_ktime(TX_RETRY_NS));
	return HRTIMER_RESTART;
}

/* Boot and dump frames: the modem's loader polls the ring */
static int link_send_udl(struct exmodem *md, struct sk_buff *skb)
{
	struct exmodem_ipc_dev *dev = &md->ipc[EXMODEM_IPC_RAW];
	unsigned long flags;
	int tries;
	int ret;

	might_sleep();

	for (tries = 0; tries < UDL_TX_RETRIES; tries++) {
		spin_lock_irqsave(&dev->tx_lock, flags);
		ret = sipc_ring_write(&dev->txq, skb->data, skb->len);
		spin_unlock_irqrestore(&dev->tx_lock, flags);
		if (ret != -ENOSPC)
			break;

		msleep(UDL_TX_RETRY_MS);
	}

	if (ret)
		kfree_skb(skb);
	else
		consume_skb(skb);

	return ret;
}

/**
 * exmodem_link_send() - send one payload to the modem
 * @md: the modem
 * @ch: SIPC channel
 * @skb: the payload; always consumed. Callers that want to avoid a copy leave
 *	 EXMODEM_TX_HEADROOM and EXMODEM_TX_TAILROOM.
 *
 * Can be called from atomic context, except for boot and dump channels.
 *
 * Return: 0, -EIO if the link is down, -EBUSY if too much is queued already,
 * -EMSGSIZE if the payload can never fit, -ENOMEM, or -ENOSPC for a boot or
 * dump frame the modem did not take.
 */
int exmodem_link_send(struct exmodem *md, u8 ch, struct sk_buff *skb)
{
	bool padded = READ_ONCE(md->padded);
	unsigned int payload_len = skb->len;
	unsigned int hdr_len = sipc5_tx_hdr_len(payload_len);
	unsigned int pad = padded ? sipc5_pad_len(hdr_len + payload_len) : 0;
	struct exmodem_ipc_dev *dev;
	unsigned long flags;
	int ret;

	dev = &md->ipc[sipc5_ch_is_fmt(ch) ? EXMODEM_IPC_FMT : EXMODEM_IPC_RAW];
	if (hdr_len + payload_len + pad >= dev->txq.size) {
		kfree_skb(skb);
		return -EMSGSIZE;
	}

	if (skb_headroom(skb) < hdr_len || skb_tailroom(skb) < pad ||
	    skb_cloned(skb) || skb_is_nonlinear(skb)) {
		struct sk_buff *nskb;

		nskb = skb_copy_expand(skb, hdr_len, pad, GFP_ATOMIC);
		kfree_skb(skb);
		if (!nskb)
			return -ENOMEM;
		skb = nskb;
	}

	sipc5_build_hdr(skb_push(skb, hdr_len), ch, payload_len, padded);
	skb_put_zero(skb, pad);

	if (sipc5_ch_is_udl(ch))
		return link_send_udl(md, skb);

	if (!link_ipc_active(md)) {
		kfree_skb(skb);
		return -EIO;
	}

	spin_lock_irqsave(&dev->tx_lock, flags);
	if (skb_queue_len(&dev->skb_txq) >= TXQ_DEPTH) {
		spin_unlock_irqrestore(&dev->tx_lock, flags);
		kfree_skb(skb);
		return -EBUSY;
	}
	__skb_queue_tail(&dev->skb_txq, skb);
	ret = link_tx_flush(md, dev);
	spin_unlock_irqrestore(&dev->tx_lock, flags);

	if (ret) {
		exmodem_link_force_crash(md, EXMODEM_CRASH_BY_AP,
					 "transmit ring corrupted");
		return -EIO;
	}

	return 0;
}

/*
 * Modem crashes
 */

static void link_handle_crash(struct exmodem *md, enum exmodem_state state)
{
	enum exmodem_state cur = exmodem_get_state(md);
	const struct exmodem_frontend *frontend = READ_ONCE(md->frontend);

	link_set_magic(md, EXMODEM_MAGIC_CRASH);
	link_set_access(md, 0);

	if (frontend && frontend->tx_flow)
		frontend->tx_flow(md, true);
	link_purge_txq(md);

	if (cur == EXMODEM_ONLINE || cur == EXMODEM_BOOTING)
		exmodem_set_state(md, state);

	atomic_set(&md->forced_crash, 0);
}

static void link_crash_ack_timeout(struct timer_list *t)
{
	struct exmodem *md = timer_container_of(md, t, crash_ack_timer);
	enum exmodem_state state = exmodem_get_state(md);

	if (state == EXMODEM_CRASH_EXIT || state == EXMODEM_CRASH_WATCHDOG)
		return;

	dev_err(md->dev, "the modem did not acknowledge the crash request\n");
	link_handle_crash(md, EXMODEM_CRASH_EXIT);
}

/**
 * exmodem_link_force_crash() - make the modem stop and prepare a dump
 * @md: the modem
 * @owner: EXMODEM_CRASH_BY_*
 * @reason: text for the crash report
 *
 * The modem answers with a crash command of its own, which moves the state
 * to EXMODEM_CRASH_EXIT. Can be called from atomic context.
 */
void exmodem_link_force_crash(struct exmodem *md, u32 owner, const char *reason)
{
	const struct exmodem_frontend *frontend = READ_ONCE(md->frontend);
	enum exmodem_state state = exmodem_get_state(md);

	link_set_magic(md, EXMODEM_MAGIC_CRASH);
	link_set_access(md, 0);

	if (atomic_inc_return(&md->forced_crash) > 1)
		return;

	if (state != EXMODEM_ONLINE && state != EXMODEM_BOOTING)
		return;

	dev_err(md->dev, "forcing a modem crash: %s\n", reason);

	md->crash_reason.owner = owner;
	strscpy(md->crash_reason.string, reason,
		sizeof(md->crash_reason.string));

	if (frontend && frontend->tx_flow)
		frontend->tx_flow(md, true);

	mod_timer(&md->crash_ack_timer, jiffies + CRASH_ACK_TIMEOUT);
	link_send_cmd(md, CMD_CRASH_EXIT);
}

/*
 * Receive
 */

static struct exmodem_port *link_find_port(struct exmodem *md, u8 ch)
{
	struct exmodem_port *port;

	list_for_each_entry(port, &md->ports, node)
		if (port->ch == ch)
			return port;

	return NULL;
}

/*
 * Limits for multi-frame messages: the largest joined message and the
 * fragments waiting for their last one
 */
#define LINK_MSG_MAX	SZ_512K
#define LINK_FRAGS_MAX	512

static void link_rx_dispatch(struct exmodem *md, const u8 *hdr,
			     struct sk_buff *skb)
{
	struct exmodem_port *port;
	struct sk_buff *whole;
	struct sk_buff *tmp;
	unsigned int len;
	u8 id;

	mutex_lock(&md->port_lock);

	port = link_find_port(md, hdr[1]);
	if (!port) {
		dev_dbg_ratelimited(md->dev, "no listener on channel %u\n",
				    hdr[1]);
		kfree_skb(skb);
		goto unlock;
	}

	if (!sipc5_is_multi(hdr)) {
		port->rx(port, skb);
		goto unlock;
	}

	/*
	 * Join the fragments of a multi-frame message once the last one is
	 * in, so that a read returns the whole message. Long messages without
	 * the multi-frame flag (RFS NV data in 2 KiB frames) still come in
	 * pieces: readers join those by the length in the IPC header
	 */
	id = hdr[4] & SIPC5_CTL_ID;
	skb->cb[0] = id;
	if (skb_queue_len(&port->frags) >= LINK_FRAGS_MAX) {
		dev_err_ratelimited(md->dev,
				    "channel %u: unfinished messages dropped\n",
				    hdr[1]);
		__skb_queue_purge(&port->frags);
	}
	__skb_queue_tail(&port->frags, skb);
	if (hdr[4] & SIPC5_CTL_MORE)
		goto unlock;

	len = 0;
	skb_queue_walk(&port->frags, tmp)
		if (tmp->cb[0] == id)
			len += tmp->len;

	whole = len <= LINK_MSG_MAX ? alloc_skb(len, GFP_KERNEL) : NULL;
	if (!whole)
		dev_err_ratelimited(md->dev,
				    "channel %u: message of %u bytes dropped\n",
				    hdr[1], len);

	skb_queue_walk_safe(&port->frags, skb, tmp) {
		if (skb->cb[0] != id)
			continue;

		__skb_unlink(skb, &port->frags);
		if (whole)
			skb_put_data(whole, skb->data, skb->len);
		kfree_skb(skb);
	}

	if (whole)
		port->rx(port, whole);

unlock:
	mutex_unlock(&md->port_lock);
}

/* Returns a reason if the modem sent something that cannot be right */
static const char *link_rx_frames(struct exmodem *md,
				  struct exmodem_ipc_dev *dev)
{
	struct sipc_ring *ring = &dev->rxq;
	u32 head = sipc_ring_head(ring);
	u32 tail = sipc_ring_tail(ring);
	u32 avail;

	if (!sipc_ring_valid(ring, head, tail))
		return "receive ring indices out of range";

	avail = sipc_ring_used(ring, head, tail);
	while (avail) {
		u8 hdr[SIPC5_HDR_MAX] = {};
		unsigned int hdr_len, frame_len, total;
		struct sk_buff *skb;
		u32 payload;

		sipc_ring_peek(ring, tail, hdr, min_t(u32, avail, sizeof(hdr)));
		hdr_len = sipc5_hdr_len(hdr);
		if (avail < SIPC5_HDR_MIN || !sipc5_start_valid(hdr) ||
		    avail < hdr_len)
			goto bad;

		frame_len = sipc5_frame_len(hdr);
		total = sipc5_total_len(hdr);
		if (frame_len < hdr_len || total > avail)
			goto bad;

		skb = alloc_skb(frame_len - hdr_len, GFP_KERNEL);
		if (skb) {
			payload = tail + hdr_len;
			if (payload >= ring->size)
				payload -= ring->size;
			sipc_ring_peek(ring, payload,
				       skb_put(skb, frame_len - hdr_len),
				       frame_len - hdr_len);
		} else {
			dev_err_ratelimited(md->dev, "%s: frame dropped\n",
					    dev->name);
		}

		sipc_ring_consume(ring, tail, total);
		tail += total;
		if (tail >= ring->size)
			tail -= ring->size;
		avail -= total;

		if (skb)
			link_rx_dispatch(md, hdr, skb);
	}

	return NULL;

bad:
	/* Nothing in the ring can be trusted any more */
	sipc_ring_consume(ring, tail, avail);
	return "bad frame from the modem";
}

static void link_phone_start(struct exmodem *md)
{
	mutex_lock(&md->link_lock);

	if (md->link_state == EXMODEM_LINK_IPC) {
		/*
		 * The modem repeats the command until it gets the end of the
		 * initialization, which waits for userspace.
		 */
		if (md->ipc_ready)
			link_send_init_end(md);
		goto unlock;
	}

	link_set_access(md, 0);
	link_set_magic(md, 0);
	link_reset_rings(md);
	link_purge_txq(md);
	link_set_magic(md, EXMODEM_MAGIC_IPC);
	link_set_access(md, 1);
	if (link_magic(md) != EXMODEM_MAGIC_IPC || link_access(md) != 1) {
		dev_err(md->dev, "the IPC region does not keep its magic\n");
		goto unlock;
	}

	if (md->ipc_ready)
		link_send_init_end(md);

	md->link_state = EXMODEM_LINK_IPC;
	complete_all(&md->init_done);

unlock:
	mutex_unlock(&md->link_lock);
}

static void link_handle_cmd(struct exmodem *md, u16 cmd)
{
	switch (cmd) {
	case CMD_PHONE_START:
		link_phone_start(md);
		break;
	case CMD_CRASH_RESET:
		mutex_lock(&md->link_lock);
		md->link_state = EXMODEM_LINK_OFFLINE;
		mutex_unlock(&md->link_lock);
		dev_err(md->dev, "the modem asks for a reset\n");
		link_handle_crash(md, EXMODEM_CRASH_RESET);
		break;
	case CMD_CRASH_EXIT:
		mutex_lock(&md->link_lock);
		md->link_state = EXMODEM_LINK_CRASH;
		mutex_unlock(&md->link_lock);
		timer_delete(&md->crash_ack_timer);
		if (!atomic_read(&md->forced_crash)) {
			md->crash_reason.owner = EXMODEM_CRASH_BY_CP;
			md->crash_reason.string[0] = '\0';
			dev_err(md->dev, "the modem crashed\n");
		}
		link_handle_crash(md, EXMODEM_CRASH_EXIT);
		break;
	default:
		dev_warn(md->dev, "unknown link command %#x\n", cmd);
		break;
	}
}

static void link_rx_work(struct work_struct *work)
{
	struct exmodem *md = container_of(work, struct exmodem, rx_work);
	u16 msg;
	int i;

	while (kfifo_get(&md->rx_fifo, &msg)) {
		enum exmodem_state state = exmodem_get_state(md);

		if (!(msg & MSG_VALID)) {
			dev_warn_ratelimited(md->dev,
					     "invalid message word %#x\n", msg);
			continue;
		}

		if (state != EXMODEM_ONLINE && state != EXMODEM_BOOTING &&
		    state != EXMODEM_CRASH_EXIT &&
		    state != EXMODEM_CRASH_WATCHDOG)
			continue;

		if (msg & MSG_CMD_VALID) {
			/* The other bits are the command, not ring events */
			link_handle_cmd(md, msg & MSG_CMD);
			msg = 0;
		}

		for (i = 0; i < EXMODEM_IPC_COUNT; i++) {
			struct exmodem_ipc_dev *dev = &md->ipc[i];
			const char *err;

			err = link_rx_frames(md, dev);
			if (err) {
				exmodem_link_force_crash(md,
							 EXMODEM_CRASH_BY_AP,
							 err);
				break;
			}

			if (msg & dev->req_ack_mask)
				link_send_irq(md, dev->res_ack_mask);
			if (msg & dev->res_ack_mask)
				link_tx_restart(md, dev);
		}
	}
}

/**
 * exmodem_link_irq() - the modem raised the message interrupt
 * @md: the modem
 *
 * Called in interrupt context. The message word has to be sampled right
 * away: the modem's next message overwrites it.
 */
void exmodem_link_irq(struct exmodem *md)
{
	u16 msg = exmodem_issr_read(md, EXMODEM_ISSR_CP2AP_MSG);

	if (!kfifo_put(&md->rx_fifo, msg))
		dev_err_ratelimited(md->dev, "message word %#x lost\n", msg);

	queue_work(md->rx_wq, &md->rx_work);
}

/**
 * exmodem_link_status_irq() - the modem changed its status word
 * @md: the modem
 *
 * Called in interrupt context. The modem uses the status word to stop and
 * restart the transmission of packet data.
 */
void exmodem_link_status_irq(struct exmodem *md)
{
	struct exmodem_ipc_dev *dev = &md->ipc[EXMODEM_IPC_RAW];
	u16 status = exmodem_issr_read(md, EXMODEM_ISSR_CP2AP_STATUS);
	unsigned long flags;

	spin_lock_irqsave(&dev->tx_lock, flags);
	if (status != md->last_status) {
		md->last_status = status;
		md->tx_suspended = status & STATUS_TX_SUSPEND;
		link_update_flow(md);
	}
	spin_unlock_irqrestore(&dev->tx_lock, flags);
}

/**
 * exmodem_link_boot_on() - forget the link state before the modem boots
 * @md: the modem
 */
void exmodem_link_boot_on(struct exmodem *md)
{
	mutex_lock(&md->link_lock);
	md->link_state = EXMODEM_LINK_OFFLINE;
	md->init_end_sent = false;
	mutex_unlock(&md->link_lock);

	hrtimer_cancel(&md->tx_timer);
	link_purge_txq(md);
}

/**
 * exmodem_link_start() - prepare the rings for the modem's loader
 * @md: the modem
 * @magic: EXMODEM_MAGIC_BOOT or EXMODEM_MAGIC_DUMP
 *
 * Return: 0 or -EIO if the IPC region does not behave like memory.
 */
int exmodem_link_start(struct exmodem *md, u32 magic)
{
	link_reset_rings(md);

	/* Boot and dump frames are padded, and so is everything after them */
	WRITE_ONCE(md->padded, true);

	link_set_magic(md, magic);
	if (link_magic(md) != magic)
		return -EIO;

	return 0;
}

/**
 * exmodem_link_set_ready() - userspace started or stopped listening
 * @md: the modem
 * @ready: the control and file system channels both have a listener
 *
 * The modem only finishes its start-up once it is told that the other side
 * is ready for its messages.
 */
void exmodem_link_set_ready(struct exmodem *md, bool ready)
{
	mutex_lock(&md->link_lock);
	md->ipc_ready = ready;
	if (ready && !md->init_end_sent && md->link_state == EXMODEM_LINK_IPC)
		link_send_init_end(md);
	mutex_unlock(&md->link_lock);
}

/**
 * exmodem_port_register() - start listening on a channel
 * @md: the modem
 * @port: the port with @ch and @rx set
 *
 * Return: 0 or -EBUSY if the channel has a listener.
 */
int exmodem_port_register(struct exmodem *md, struct exmodem_port *port)
{
	int ret = 0;

	skb_queue_head_init(&port->frags);

	mutex_lock(&md->port_lock);
	if (link_find_port(md, port->ch))
		ret = -EBUSY;
	else
		list_add_tail(&port->node, &md->ports);
	mutex_unlock(&md->port_lock);

	return ret;
}

/**
 * exmodem_port_unregister() - stop listening on a channel
 * @md: the modem
 * @port: a registered port; its @rx is not called any more on return
 */
void exmodem_port_unregister(struct exmodem *md, struct exmodem_port *port)
{
	mutex_lock(&md->port_lock);
	list_del(&port->node);
	mutex_unlock(&md->port_lock);

	__skb_queue_purge(&port->frags);
}

static void link_init_dev(struct exmodem *md, struct exmodem_ipc_dev *dev,
			  const char *name, unsigned int tx_head,
			  unsigned int rx_head, unsigned int tx_buf,
			  unsigned int rx_buf, unsigned int rx_end)
{
	void *base = md->ipc_base;

	dev->name = name;
	dev->txq.head = base + tx_head;
	dev->txq.tail = base + tx_head + 4;
	dev->txq.buf = base + tx_buf;
	dev->txq.size = rx_buf - tx_buf;
	dev->rxq.head = base + rx_head;
	dev->rxq.tail = base + rx_head + 4;
	dev->rxq.buf = base + rx_buf;
	dev->rxq.size = rx_end - rx_buf;
	spin_lock_init(&dev->tx_lock);
	__skb_queue_head_init(&dev->skb_txq);
}

/**
 * exmodem_link_init() - set up the link on the mapped IPC region
 * @md: the modem
 *
 * Return: 0 or a negative error code.
 */
int exmodem_link_init(struct exmodem *md)
{
	struct exmodem_ipc_dev *fmt = &md->ipc[EXMODEM_IPC_FMT];
	struct exmodem_ipc_dev *raw = &md->ipc[EXMODEM_IPC_RAW];

	if (md->ipc_size != MAP_SIZE)
		return dev_err_probe(md->dev, -EINVAL,
				     "unsupported IPC region size %#zx\n",
				     md->ipc_size);

	md->magic = md->ipc_base + MAP_MAGIC;
	md->access = md->ipc_base + MAP_ACCESS;

	/*
	 * The region is plain RAM to everything that ran before, and the
	 * modem's firmware reads the header as soon as it starts.
	 */
	memset(md->ipc_base, 0, MAP_FMT_TX_BUF);

	link_init_dev(md, fmt, "FMT", MAP_FMT_TX_HEAD, MAP_FMT_RX_HEAD,
		      MAP_FMT_TX_BUF, MAP_FMT_RX_BUF, MAP_RAW_TX_BUF);
	fmt->send_mask = MSG_SEND_FMT;
	fmt->req_ack_mask = MSG_REQ_ACK_FMT;
	fmt->res_ack_mask = MSG_RES_ACK_FMT;

	link_init_dev(md, raw, "RAW", MAP_RAW_TX_HEAD, MAP_RAW_RX_HEAD,
		      MAP_RAW_TX_BUF, MAP_RAW_RX_BUF, MAP_SIZE);
	raw->send_mask = MSG_SEND_RAW;
	raw->req_ack_mask = MSG_REQ_ACK_RAW;
	raw->res_ack_mask = MSG_RES_ACK_RAW;
	raw->flow_ctrl = true;

	spin_lock_init(&md->msg_lock);
	mutex_init(&md->link_lock);
	mutex_init(&md->port_lock);
	INIT_LIST_HEAD(&md->ports);
	INIT_KFIFO(md->rx_fifo);
	INIT_WORK(&md->rx_work, link_rx_work);
	hrtimer_setup(&md->tx_timer, link_tx_timer, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL);
	timer_setup(&md->crash_ack_timer, link_crash_ack_timeout, 0);

	md->rx_wq = alloc_ordered_workqueue("exmodem_rx", WQ_HIGHPRI);
	if (!md->rx_wq)
		return -ENOMEM;

	return 0;
}

/**
 * exmodem_link_exit() - stop all link activity
 * @md: the modem, with its mailbox channels released
 */
void exmodem_link_exit(struct exmodem *md)
{
	hrtimer_cancel(&md->tx_timer);
	timer_delete_sync(&md->crash_ack_timer);
	cancel_work_sync(&md->rx_work);
	destroy_workqueue(md->rx_wq);
	link_purge_txq(md);
}
