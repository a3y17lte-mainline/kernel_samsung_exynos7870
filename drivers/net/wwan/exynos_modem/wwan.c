// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos integrated modem: WWAN ports and network interfaces
 *
 * Two control ports: Samsung IPC (the modem's control protocol) and the
 * remote file system channel, on which the modem asks for its non-volatile
 * data. A read returns one message, a write sends one. The first open of the
 * IPC port loads and starts the modem; it keeps running when the ports are
 * closed. When the modem crashes, both ports are removed and added again, so
 * that their users see the end of the session; the next open starts the
 * modem again.
 *
 * Packet data contexts are WWAN links: link N is packet data channel N.
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/rcupdate.h>
#include <linux/rtnetlink.h>
#include <linux/skbuff.h>
#include <linux/wwan.h>

#include "modem.h"

#define EXMODEM_NUM_LINKS	(SIPC5_CH_PDP_LAST - SIPC5_CH_PDP_0 + 1)

enum {
	EXMODEM_WWAN_IPC,
	EXMODEM_WWAN_RFS,
	EXMODEM_WWAN_PORTS
};

struct exmodem_wwan;

struct exmodem_wwan_port {
	struct exmodem_wwan *ww;
	struct exmodem_port port;
	struct wwan_port *wport;
	struct wwan_port *rx_wport;
	enum wwan_port_type type;
	bool started;
};

struct exmodem_wwan {
	struct exmodem *md;
	struct exmodem_wwan_port ports[EXMODEM_WWAN_PORTS];
	struct mutex lock;		/* started, starting the modem */
	struct mutex ports_lock;	/* wport */
	struct work_struct crash_work;
	struct net_device __rcu *links[EXMODEM_NUM_LINKS];
};

static const struct {
	u8 ch;
	enum wwan_port_type type;
} exmodem_wwan_port_info[EXMODEM_WWAN_PORTS] = {
	[EXMODEM_WWAN_IPC] = { SIPC5_CH_FMT_0, WWAN_PORT_SIPC },
	[EXMODEM_WWAN_RFS] = { SIPC5_CH_RFS_0, WWAN_PORT_SIPC_RFS },
};

/* The modem finishes its start-up once both ports are open */
static void exmodem_wwan_update_ready(struct exmodem_wwan *ww)
{
	exmodem_link_set_ready(ww->md,
			       ww->ports[EXMODEM_WWAN_IPC].started &&
			       ww->ports[EXMODEM_WWAN_RFS].started);
}

static void exmodem_wwan_rx(struct exmodem_port *port, struct sk_buff *skb)
{
	struct exmodem_wwan_port *p = container_of(port,
						   struct exmodem_wwan_port,
						   port);

	wwan_port_rx(READ_ONCE(p->rx_wport), skb);
}

static int exmodem_wwan_start(struct wwan_port *wport)
{
	struct exmodem_wwan_port *p = wwan_port_get_drvdata(wport);
	struct exmodem_wwan *ww = p->ww;
	struct exmodem *md = ww->md;
	int ret;

	guard(mutex)(&ww->lock);

	/* p->wport may not be set yet, the port can be opened right away */
	WRITE_ONCE(p->rx_wport, wport);
	ret = exmodem_port_register(md, &p->port);
	if (ret)
		return ret;

	if (p->type == WWAN_PORT_SIPC &&
	    exmodem_get_state(md) != EXMODEM_ONLINE) {
		ret = exmodem_boot(md);
		if (ret) {
			exmodem_port_unregister(md, &p->port);
			return ret;
		}
	}

	p->started = true;
	exmodem_wwan_update_ready(ww);

	return 0;
}

static void exmodem_wwan_stop(struct wwan_port *wport)
{
	struct exmodem_wwan_port *p = wwan_port_get_drvdata(wport);
	struct exmodem_wwan *ww = p->ww;

	guard(mutex)(&ww->lock);

	p->started = false;
	exmodem_wwan_update_ready(ww);
	exmodem_port_unregister(ww->md, &p->port);
}

static int exmodem_wwan_tx(struct wwan_port *wport, struct sk_buff *skb)
{
	struct exmodem_wwan_port *p = wwan_port_get_drvdata(wport);
	int ret;

	/* The WWAN core frees the message itself if sending fails */
	skb_get(skb);
	ret = exmodem_link_send(p->ww->md, p->port.ch, skb);
	if (!ret)
		consume_skb(skb);

	return ret;
}

static const struct wwan_port_ops exmodem_wwan_port_ops = {
	.start = exmodem_wwan_start,
	.stop = exmodem_wwan_stop,
	.tx = exmodem_wwan_tx,
};

static int exmodem_wwan_add_ports(struct exmodem_wwan *ww)
{
	struct wwan_port_caps caps = {
		.frag_len = SIZE_MAX,
		.headroom_len = EXMODEM_TX_HEADROOM,
	};
	int i;

	guard(mutex)(&ww->ports_lock);

	for (i = 0; i < EXMODEM_WWAN_PORTS; i++) {
		struct exmodem_wwan_port *p = &ww->ports[i];
		struct wwan_port *wport;

		wport = wwan_create_port(ww->md->dev, p->type,
					 &exmodem_wwan_port_ops, &caps, p);
		if (IS_ERR(wport))
			return PTR_ERR(wport);

		p->wport = wport;
	}

	return 0;
}

static void exmodem_wwan_remove_ports(struct exmodem_wwan *ww)
{
	int i;

	guard(mutex)(&ww->ports_lock);

	for (i = 0; i < EXMODEM_WWAN_PORTS; i++) {
		if (!ww->ports[i].wport)
			continue;

		wwan_remove_port(ww->ports[i].wport);
		ww->ports[i].wport = NULL;
	}
}

static void exmodem_wwan_crash_work(struct work_struct *work)
{
	struct exmodem_wwan *ww = container_of(work, struct exmodem_wwan,
					       crash_work);
	int ret;

	exmodem_wwan_remove_ports(ww);

	ret = exmodem_wwan_add_ports(ww);
	if (ret)
		dev_err(ww->md->dev, "cannot add the control ports: %d\n", ret);
}

/*
 * Packet data
 */

static void exmodem_wwan_link_setup(struct net_device *ndev)
{
	exmodem_net_setup(ndev);
	ndev->needs_free_netdev = true;
}

static int exmodem_wwan_newlink(void *ctxt, struct net_device *ndev,
				u32 if_id, struct netlink_ext_ack *extack)
{
	struct exmodem_wwan *ww = ctxt;
	int ret;

	if (if_id >= EXMODEM_NUM_LINKS) {
		NL_SET_ERR_MSG_MOD(extack, "link id out of range");
		return -EINVAL;
	}

	if (rtnl_dereference(ww->links[if_id])) {
		NL_SET_ERR_MSG_MOD(extack, "link id in use");
		return -EBUSY;
	}

	exmodem_net_init(wwan_netdev_drvpriv(ndev), ww->md, ndev,
			 SIPC5_CH_PDP_0 + if_id);

	ret = register_netdevice(ndev);
	if (ret)
		return ret;

	rcu_assign_pointer(ww->links[if_id], ndev);

	return 0;
}

static void exmodem_wwan_dellink(void *ctxt, struct net_device *ndev,
				 struct list_head *head)
{
	struct exmodem_wwan *ww = ctxt;
	struct exmodem_net *net = wwan_netdev_drvpriv(ndev);

	RCU_INIT_POINTER(ww->links[net->port.ch - SIPC5_CH_PDP_0], NULL);
	unregister_netdevice_queue(ndev, head);
}

static const struct wwan_ops exmodem_wwan_ops = {
	.priv_size = sizeof(struct exmodem_net),
	.setup = exmodem_wwan_link_setup,
	.newlink = exmodem_wwan_newlink,
	.dellink = exmodem_wwan_dellink,
};

/* Called in atomic context */
static void exmodem_wwan_tx_flow(struct exmodem *md, bool stop)
{
	struct exmodem_wwan *ww = md->frontend_data;
	int i;

	rcu_read_lock();
	for (i = 0; i < EXMODEM_NUM_LINKS; i++) {
		struct net_device *ndev = rcu_dereference(ww->links[i]);

		if (ndev)
			exmodem_net_tx_flow(ndev, stop);
	}
	rcu_read_unlock();
}

/* Called in atomic context */
static void exmodem_wwan_state_changed(struct exmodem *md,
				       enum exmodem_state state)
{
	struct exmodem_wwan *ww = md->frontend_data;

	switch (state) {
	case EXMODEM_CRASH_RESET:
	case EXMODEM_CRASH_EXIT:
	case EXMODEM_CRASH_WATCHDOG:
		schedule_work(&ww->crash_work);
		break;
	default:
		break;
	}
}

static const struct exmodem_frontend exmodem_wwan_frontend = {
	.state_changed = exmodem_wwan_state_changed,
	.tx_flow = exmodem_wwan_tx_flow,
};

int exmodem_wwan_init(struct exmodem *md)
{
	struct exmodem_wwan *ww;
	int ret;
	int i;

	ww = devm_kzalloc(md->dev, sizeof(*ww), GFP_KERNEL);
	if (!ww)
		return -ENOMEM;

	ww->md = md;
	mutex_init(&ww->lock);
	mutex_init(&ww->ports_lock);
	INIT_WORK(&ww->crash_work, exmodem_wwan_crash_work);

	for (i = 0; i < EXMODEM_WWAN_PORTS; i++) {
		struct exmodem_wwan_port *p = &ww->ports[i];

		p->ww = ww;
		p->type = exmodem_wwan_port_info[i].type;
		p->port.ch = exmodem_wwan_port_info[i].ch;
		p->port.rx = exmodem_wwan_rx;
	}

	md->frontend_data = ww;
	exmodem_set_frontend(md, &exmodem_wwan_frontend);

	ret = exmodem_wwan_add_ports(ww);
	if (ret)
		goto err;

	ret = wwan_register_ops(md->dev, &exmodem_wwan_ops, ww,
				WWAN_NO_DEFAULT_LINK);
	if (ret)
		goto err;

	return 0;

err:
	exmodem_set_frontend(md, NULL);
	cancel_work_sync(&ww->crash_work);
	exmodem_wwan_remove_ports(ww);
	return ret;
}

void exmodem_wwan_exit(struct exmodem *md)
{
	struct exmodem_wwan *ww = md->frontend_data;

	exmodem_set_frontend(md, NULL);
	cancel_work_sync(&ww->crash_work);
	wwan_unregister_ops(md->dev);
	exmodem_wwan_remove_ports(ww);
}
