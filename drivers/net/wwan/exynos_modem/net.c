// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos integrated modem: packet data channels as network interfaces
 *
 * Each packet data channel carries raw IP packets, one per frame, without a
 * link layer header.
 */

#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/netdevice.h>

#include "modem.h"

static void exmodem_net_rx(struct exmodem_port *port, struct sk_buff *skb)
{
	struct exmodem_net *net = container_of(port, struct exmodem_net, port);
	struct net_device *ndev = net->ndev;

	if (!skb->len) {
		kfree_skb(skb);
		return;
	}

	skb->dev = ndev;
	switch (skb->data[0] & 0xf0) {
	case 0x40:
		skb->protocol = htons(ETH_P_IP);
		break;
	case 0x60:
		skb->protocol = htons(ETH_P_IPV6);
		break;
	default:
		dev_core_stats_rx_dropped_inc(ndev);
		kfree_skb(skb);
		return;
	}
	skb_reset_mac_header(skb);
	skb_reset_network_header(skb);

	dev_sw_netstats_rx_add(ndev, skb->len);
	netif_rx(skb);
}

static int exmodem_net_open(struct net_device *ndev)
{
	struct exmodem_net *net = ndev->ml_priv;
	int ret;

	ret = exmodem_port_register(net->md, &net->port);
	if (ret)
		return ret;

	netif_start_queue(ndev);

	return 0;
}

static int exmodem_net_stop(struct net_device *ndev)
{
	struct exmodem_net *net = ndev->ml_priv;

	netif_stop_queue(ndev);
	exmodem_port_unregister(net->md, &net->port);

	return 0;
}

static netdev_tx_t exmodem_net_xmit(struct sk_buff *skb,
				    struct net_device *ndev)
{
	struct exmodem_net *net = ndev->ml_priv;
	unsigned int len = skb->len;

	if (exmodem_link_send(net->md, net->port.ch, skb))
		dev_core_stats_tx_dropped_inc(ndev);
	else
		dev_sw_netstats_tx_add(ndev, 1, len);

	return NETDEV_TX_OK;
}

static const struct net_device_ops exmodem_net_ops = {
	.ndo_open = exmodem_net_open,
	.ndo_stop = exmodem_net_stop,
	.ndo_start_xmit = exmodem_net_xmit,
};

/**
 * exmodem_net_setup() - set up a raw IP interface for a packet data channel
 * @ndev: the interface
 */
void exmodem_net_setup(struct net_device *ndev)
{
	ndev->netdev_ops = &exmodem_net_ops;
	ndev->type = ARPHRD_RAWIP;
	ndev->flags = IFF_POINTOPOINT | IFF_NOARP;
	ndev->hard_header_len = 0;
	ndev->addr_len = 0;
	ndev->needed_headroom = EXMODEM_TX_HEADROOM;
	ndev->needed_tailroom = EXMODEM_TX_TAILROOM;
	ndev->mtu = ETH_DATA_LEN;
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = ETH_DATA_LEN;
	ndev->tx_queue_len = 1000;
	ndev->pcpu_stat_type = NETDEV_PCPU_STAT_TSTATS;
}

/**
 * exmodem_net_init() - tie an interface to a packet data channel
 * @net: the channel's state, owned by the caller
 * @md: the modem
 * @ndev: the interface, set up with exmodem_net_setup()
 * @ch: the channel
 */
void exmodem_net_init(struct exmodem_net *net, struct exmodem *md,
		      struct net_device *ndev, u8 ch)
{
	net->md = md;
	net->ndev = ndev;
	net->port.ch = ch;
	net->port.rx = exmodem_net_rx;
	ndev->ml_priv = net;
}

/**
 * exmodem_net_tx_flow() - stop or restart an interface's transmit queue
 * @ndev: the interface
 * @stop: stop if true, restart otherwise
 *
 * Can be called in atomic context.
 */
void exmodem_net_tx_flow(struct net_device *ndev, bool stop)
{
	if (!netif_running(ndev))
		return;

	if (stop)
		netif_stop_queue(ndev);
	else
		netif_wake_queue(ndev);
}
