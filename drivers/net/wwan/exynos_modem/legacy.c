// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos integrated modem: the vendor kernel's userspace interface
 *
 * Samsung's modem software (the boot daemon "cbd" and the radio interface
 * library) and libsamsung-ipc talk to the vendor kernel driver through one
 * character device per SIPC channel, with ioctls on the boot device for each
 * step of the modem's start-up, and through one network interface per packet
 * data channel. This file provides the same interface on top of the link.
 */

#include <linux/compat.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include "modem.h"

#define IOCTL_MODEM_ON			_IO('o', 0x19)
#define IOCTL_MODEM_OFF			_IO('o', 0x20)
#define IOCTL_MODEM_RESET		_IO('o', 0x21)
#define IOCTL_MODEM_BOOT_ON		_IO('o', 0x22)
#define IOCTL_MODEM_BOOT_OFF		_IO('o', 0x23)
#define IOCTL_MODEM_BOOT_DONE		_IO('o', 0x24)
#define IOCTL_MODEM_PROTOCOL_SUSPEND	_IO('o', 0x25)
#define IOCTL_MODEM_PROTOCOL_RESUME	_IO('o', 0x26)
#define IOCTL_MODEM_STATUS		_IO('o', 0x27)
#define IOCTL_MODEM_DL_START		_IO('o', 0x28)
#define IOCTL_MODEM_DUMP_START		_IO('o', 0x32)
#define IOCTL_MODEM_FORCE_CRASH_EXIT	_IO('o', 0x34)
#define IOCTL_MODEM_SET_TX_LINK		_IO('o', 0x37)
#define IOCTL_MODEM_XMIT_BOOT		_IO('o', 0x40)
#define IOCTL_MODEM_GET_SHMEM_SRINFO	_IO('o', 0x45)
#define IOCTL_MODEM_SET_SHMEM_SRINFO	_IO('o', 0x46)
#define IOCTL_SECURITY_REQ		_IO('o', 0x53)
#define IOCTL_MODEM_CRASH_REASON	_IO('o', 0x55)

struct modem_firmware {
	__u64 binary;
	__u32 size;
	__u32 m_offset;
	__u32 b_offset;
	__u32 mode;
	__u32 len;
} __packed;

struct modem_sec_req {
	__u32 mode;
	__u32 size_boot;
	__u32 size_main;
	__u32 dummy;
} __packed;

static bool legacy_abi;
module_param(legacy_abi, bool, 0444);
MODULE_PARM_DESC(legacy_abi,
		 "use the vendor kernel's interface instead of WWAN ports and links");

bool exmodem_legacy_enabled(void)
{
	return legacy_abi;
}

#define LEGACY_RXQ_MAX		2048
#define LEGACY_READ_WAIT_MS	100
#define LEGACY_XMIT_CHUNK	SZ_64K
#define LEGACY_NUM_NETDEVS	8

enum legacy_format {
	LEGACY_FMT,
	LEGACY_RAW,
	LEGACY_RFS,
	LEGACY_BOOT,
	LEGACY_DUMP,
};

struct legacy_dev_info {
	const char *name;
	u8 ch;
	enum legacy_format format;
};

static const struct legacy_dev_info legacy_devs[] = {
	{ "umts_boot0", SIPC5_CH_BOOT_0, LEGACY_BOOT },
	{ "umts_ramdump0", SIPC5_CH_DUMP_0, LEGACY_DUMP },
	{ "umts_ipc0", SIPC5_CH_FMT_0, LEGACY_FMT },
	{ "umts_ipc1", SIPC5_CH_FMT_0 + 1, LEGACY_FMT },
	{ "umts_rfs0", SIPC5_CH_RFS_0, LEGACY_RFS },
	{ "umts_csd", 1, LEGACY_RAW },
	{ "umts_router", 25, LEGACY_RAW },
	{ "umts_dm0", 28, LEGACY_RAW },
};

struct exmodem_legacy;

struct legacy_dev {
	struct exmodem_legacy *lg;
	const struct legacy_dev_info *info;
	struct miscdevice misc;
	struct exmodem_port port;
	struct sk_buff_head rxq;
	wait_queue_head_t wq;
	struct mutex open_lock;		/* opened */
	unsigned int opened;
};

struct exmodem_legacy {
	struct exmodem *md;
	struct legacy_dev devs[ARRAY_SIZE(legacy_devs)];
	struct net_device *ndevs[LEGACY_NUM_NETDEVS];
	struct miscdevice multipdp;
	unsigned int waketime;
	struct mutex ready_lock;	/* readiness of the IPC channels */
};

/* The modem finishes its start-up once both of these are open */
static struct legacy_dev *legacy_find_dev(struct exmodem_legacy *lg, u8 ch)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lg->devs); i++)
		if (lg->devs[i].info->ch == ch)
			return &lg->devs[i];

	return NULL;
}

static void legacy_update_ready(struct exmodem_legacy *lg)
{
	struct legacy_dev *fmt = legacy_find_dev(lg, SIPC5_CH_FMT_0);
	struct legacy_dev *rfs = legacy_find_dev(lg, SIPC5_CH_RFS_0);

	mutex_lock(&lg->ready_lock);
	exmodem_link_set_ready(lg->md,
			       READ_ONCE(fmt->opened) && READ_ONCE(rfs->opened));
	mutex_unlock(&lg->ready_lock);
}

static void legacy_dev_rx(struct exmodem_port *port, struct sk_buff *skb)
{
	struct legacy_dev *d = container_of(port, struct legacy_dev, port);

	if (d->info->format != LEGACY_DUMP &&
	    skb_queue_len(&d->rxq) > LEGACY_RXQ_MAX) {
		dev_err_ratelimited(d->misc.this_device,
				    "nobody reads, message dropped\n");
		kfree_skb(skb);
	} else {
		skb_queue_tail(&d->rxq, skb);
	}

	wake_up(&d->wq);
}

static int legacy_open(struct inode *inode, struct file *filp)
{
	struct legacy_dev *d = container_of(filp->private_data,
					    struct legacy_dev, misc);
	int ret = 0;

	filp->private_data = d;

	mutex_lock(&d->open_lock);
	if (!d->opened)
		ret = exmodem_port_register(d->lg->md, &d->port);
	if (!ret)
		d->opened++;
	mutex_unlock(&d->open_lock);
	if (ret)
		return ret;

	legacy_update_ready(d->lg);

	return 0;
}

static int legacy_release(struct inode *inode, struct file *filp)
{
	struct legacy_dev *d = filp->private_data;

	mutex_lock(&d->open_lock);
	if (!--d->opened) {
		exmodem_port_unregister(d->lg->md, &d->port);
		skb_queue_purge(&d->rxq);
	}
	mutex_unlock(&d->open_lock);

	legacy_update_ready(d->lg);

	return 0;
}

static __poll_t legacy_poll(struct file *filp, poll_table *wait)
{
	struct legacy_dev *d = filp->private_data;
	__poll_t in = skb_queue_empty(&d->rxq) ? 0 : EPOLLIN | EPOLLRDNORM;

	poll_wait(filp, &d->wq, wait);

	switch (exmodem_get_state(d->lg->md)) {
	case EXMODEM_BOOTING:
	case EXMODEM_ONLINE:
		return in;
	case EXMODEM_CRASH_EXIT:
	case EXMODEM_CRASH_RESET:
	case EXMODEM_NV_REBUILDING:
	case EXMODEM_CRASH_WATCHDOG:
		/* This is how the modem software learns about a crash */
		if (d->info->format == LEGACY_FMT ||
		    d->info->format == LEGACY_BOOT)
			return EPOLLHUP;
		if (d->info->format == LEGACY_DUMP)
			return in;
		return 0;
	default:
		return 0;
	}
}

static ssize_t legacy_read(struct file *filp, char __user *buf, size_t count,
			   loff_t *ppos)
{
	struct legacy_dev *d = filp->private_data;
	struct sk_buff *skb;
	size_t copied;

	/* An empty read after a short wait is part of the interface */
	if (skb_queue_empty(&d->rxq))
		wait_event_timeout(d->wq, !skb_queue_empty(&d->rxq),
				   msecs_to_jiffies(LEGACY_READ_WAIT_MS));

	skb = skb_dequeue(&d->rxq);
	if (!skb)
		return 0;

	copied = min(count, skb->len);
	if (copy_to_user(buf, skb->data, copied)) {
		kfree_skb(skb);
		return -EFAULT;
	}

	if (skb->len > copied) {
		skb_pull(skb, copied);
		skb_queue_head(&d->rxq, skb);
	} else {
		consume_skb(skb);
	}

	return copied;
}

static ssize_t legacy_write(struct file *filp, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct legacy_dev *d = filp->private_data;
	struct exmodem *md = d->lg->md;
	struct sk_buff *skb;
	int ret;

	if (!sipc5_ch_is_udl(d->info->ch) &&
	    exmodem_get_state(md) != EXMODEM_ONLINE)
		return -EPERM;

	if (!count || count > SZ_1M)
		return -EMSGSIZE;

	skb = alloc_skb(EXMODEM_TX_HEADROOM + count + EXMODEM_TX_TAILROOM,
			GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	skb_reserve(skb, EXMODEM_TX_HEADROOM);
	if (copy_from_user(skb_put(skb, count), buf, count)) {
		kfree_skb(skb);
		return -EFAULT;
	}

	ret = exmodem_link_send(md, d->info->ch, skb);
	if (ret)
		return ret;

	return count;
}

static int legacy_xmit_boot(struct exmodem *md, void __user *arg)
{
	const void __user *src;
	struct modem_firmware mf;
	void *buf;
	int ret = 0;

	if (copy_from_user(&mf, arg, sizeof(mf)))
		return -EFAULT;

	src = u64_to_user_ptr(mf.binary);

	buf = kvmalloc(LEGACY_XMIT_CHUNK, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	while (mf.len) {
		u32 n = min(mf.len, LEGACY_XMIT_CHUNK);

		if (copy_from_user(buf, src, n)) {
			ret = -EFAULT;
			break;
		}

		ret = exmodem_fw_write(md, mf.mode, mf.m_offset, buf, n);
		if (ret)
			break;

		src += n;
		mf.m_offset += n;
		mf.len -= n;
	}

	kvfree(buf);

	return ret;
}

static int legacy_srinfo(struct exmodem *md, void __user *arg, bool set)
{
	void *srinfo = md->ipc_base + EXMODEM_SRINFO_OFFSET;
	u32 __user *usize = arg;
	void __user *ubuf = arg + sizeof(u32);
	u32 size;

	if (get_user(size, usize))
		return -EFAULT;

	size = min(size, EXMODEM_SRINFO_SIZE);

	if (set)
		return copy_from_user(srinfo, ubuf, size) ? -EFAULT : 0;

	if (put_user(size, usize) || copy_to_user(ubuf, srinfo, size))
		return -EFAULT;

	return 0;
}

static void legacy_tx_flow(struct exmodem *md, bool stop);

static long legacy_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct legacy_dev *d = filp->private_data;
	struct exmodem *md = d->lg->md;
	void __user *uarg = (void __user *)arg;
	struct modem_sec_req msr;
	enum exmodem_state state;

	switch (cmd) {
	case IOCTL_MODEM_ON:
		return exmodem_power_on(md);
	case IOCTL_MODEM_OFF:
		return exmodem_power_off(md);
	case IOCTL_MODEM_RESET:
		return exmodem_reset(md);
	case IOCTL_MODEM_BOOT_ON:
		return exmodem_boot_on(md);
	case IOCTL_MODEM_BOOT_OFF:
		return exmodem_boot_off(md);
	case IOCTL_MODEM_BOOT_DONE:
		return 0;
	case IOCTL_MODEM_STATUS:
		state = exmodem_get_state(md);
		if (state == EXMODEM_NV_REBUILDING)
			exmodem_set_state(md, EXMODEM_ONLINE);
		return state;
	case IOCTL_MODEM_DL_START:
		return exmodem_dl_start(md);
	case IOCTL_MODEM_DUMP_START:
		return exmodem_dump_start(md);
	case IOCTL_MODEM_FORCE_CRASH_EXIT:
		exmodem_link_force_crash(md, EXMODEM_CRASH_BY_AP,
					 "requested by userspace");
		return 0;
	case IOCTL_MODEM_XMIT_BOOT:
		return legacy_xmit_boot(md, uarg);
	case IOCTL_SECURITY_REQ:
		if (copy_from_user(&msr, uarg, sizeof(msr)))
			return -EFAULT;
		return exmodem_security(md, msr.mode, msr.size_boot,
					msr.size_main);
	case IOCTL_MODEM_CRASH_REASON:
		if (copy_to_user(uarg, &md->crash_reason,
				 sizeof(md->crash_reason)))
			return -EFAULT;
		return 0;
	case IOCTL_MODEM_GET_SHMEM_SRINFO:
		return legacy_srinfo(md, uarg, false);
	case IOCTL_MODEM_SET_SHMEM_SRINFO:
		return legacy_srinfo(md, uarg, true);
	case IOCTL_MODEM_PROTOCOL_SUSPEND:
		legacy_tx_flow(md, true);
		return 0;
	case IOCTL_MODEM_PROTOCOL_RESUME:
		legacy_tx_flow(md, false);
		return 0;
	case IOCTL_MODEM_SET_TX_LINK:
		/* There is only the shared memory link */
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct file_operations legacy_fops = {
	.owner = THIS_MODULE,
	.open = legacy_open,
	.release = legacy_release,
	.poll = legacy_poll,
	.read = legacy_read,
	.write = legacy_write,
	.unlocked_ioctl = legacy_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

/*
 * Packet data: one raw IP interface per channel
 */

/* Called in atomic context */
static void legacy_tx_flow(struct exmodem *md, bool stop)
{
	struct exmodem_legacy *lg = md->frontend_data;
	int i;

	for (i = 0; i < LEGACY_NUM_NETDEVS; i++)
		if (lg->ndevs[i])
			exmodem_net_tx_flow(lg->ndevs[i], stop);
}

/* Called in atomic context */
static void legacy_state_changed(struct exmodem *md, enum exmodem_state state)
{
	struct exmodem_legacy *lg = md->frontend_data;
	int i;

	for (i = 0; i < ARRAY_SIZE(lg->devs); i++)
		wake_up(&lg->devs[i].wq);
}

static const struct exmodem_frontend legacy_frontend = {
	.state_changed = legacy_state_changed,
	.tx_flow = legacy_tx_flow,
};

/*
 * "multipdp" carries no data. The radio interface library sets a wake time
 * through it, which only meant something for the vendor's wake locks.
 */

static ssize_t waketime_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct miscdevice *misc = dev_get_drvdata(dev);
	struct exmodem_legacy *lg = container_of(misc, struct exmodem_legacy,
						 multipdp);

	return sysfs_emit(buf, "raw waketime : %ums\n", lg->waketime);
}

static ssize_t waketime_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct miscdevice *misc = dev_get_drvdata(dev);
	struct exmodem_legacy *lg = container_of(misc, struct exmodem_legacy,
						 multipdp);
	int ret;

	ret = kstrtouint(buf, 10, &lg->waketime);
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(waketime);

static struct attribute *legacy_multipdp_attrs[] = {
	&dev_attr_waketime.attr,
	NULL
};
ATTRIBUTE_GROUPS(legacy_multipdp);

static long legacy_multipdp_ioctl(struct file *filp, unsigned int cmd,
				  unsigned long arg)
{
	struct exmodem_legacy *lg = container_of(filp->private_data,
						 struct exmodem_legacy,
						 multipdp);

	switch (cmd) {
	case IOCTL_MODEM_PROTOCOL_SUSPEND:
		legacy_tx_flow(lg->md, true);
		return 0;
	case IOCTL_MODEM_PROTOCOL_RESUME:
		legacy_tx_flow(lg->md, false);
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct file_operations legacy_multipdp_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = legacy_multipdp_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.llseek = noop_llseek,
};

static void legacy_unregister(struct exmodem_legacy *lg)
{
	int i;

	for (i = 0; i < LEGACY_NUM_NETDEVS; i++) {
		if (!lg->ndevs[i])
			continue;

		unregister_netdev(lg->ndevs[i]);
		free_netdev(lg->ndevs[i]);
		lg->ndevs[i] = NULL;
	}

	for (i = 0; i < ARRAY_SIZE(lg->devs); i++)
		if (lg->devs[i].misc.this_device)
			misc_deregister(&lg->devs[i].misc);

	if (lg->multipdp.this_device)
		misc_deregister(&lg->multipdp);
}

int exmodem_legacy_init(struct exmodem *md)
{
	struct exmodem_legacy *lg;
	int ret;
	int i;

	lg = devm_kzalloc(md->dev, sizeof(*lg), GFP_KERNEL);
	if (!lg)
		return -ENOMEM;

	lg->md = md;
	mutex_init(&lg->ready_lock);
	md->frontend_data = lg;

	for (i = 0; i < ARRAY_SIZE(lg->devs); i++) {
		struct legacy_dev *d = &lg->devs[i];

		d->lg = lg;
		d->info = &legacy_devs[i];
		d->port.ch = d->info->ch;
		d->port.rx = legacy_dev_rx;
		skb_queue_head_init(&d->rxq);
		init_waitqueue_head(&d->wq);
		mutex_init(&d->open_lock);
	}

	exmodem_set_frontend(md, &legacy_frontend);

	for (i = 0; i < ARRAY_SIZE(lg->devs); i++) {
		struct legacy_dev *d = &lg->devs[i];

		d->misc.minor = MISC_DYNAMIC_MINOR;
		d->misc.name = d->info->name;
		d->misc.fops = &legacy_fops;
		d->misc.parent = md->dev;
		ret = misc_register(&d->misc);
		if (ret) {
			d->misc.this_device = NULL;
			goto err;
		}
	}

	lg->multipdp.minor = MISC_DYNAMIC_MINOR;
	lg->multipdp.name = "multipdp";
	lg->multipdp.fops = &legacy_multipdp_fops;
	lg->multipdp.groups = legacy_multipdp_groups;
	ret = misc_register(&lg->multipdp);
	if (ret) {
		lg->multipdp.this_device = NULL;
		goto err;
	}

	for (i = 0; i < LEGACY_NUM_NETDEVS; i++) {
		struct net_device *ndev;
		char name[IFNAMSIZ];

		snprintf(name, sizeof(name), "rmnet%d", i);
		ndev = alloc_netdev(sizeof(struct exmodem_net), name,
				    NET_NAME_PREDICTABLE, exmodem_net_setup);
		if (!ndev) {
			ret = -ENOMEM;
			goto err;
		}

		exmodem_net_init(netdev_priv(ndev), md, ndev,
				 SIPC5_CH_PDP_0 + i);
		SET_NETDEV_DEV(ndev, md->dev);

		ret = register_netdev(ndev);
		if (ret) {
			free_netdev(ndev);
			goto err;
		}
		lg->ndevs[i] = ndev;
	}

	return 0;

err:
	exmodem_set_frontend(md, NULL);
	legacy_unregister(lg);
	return ret;
}

void exmodem_legacy_exit(struct exmodem *md)
{
	struct exmodem_legacy *lg = md->frontend_data;

	legacy_unregister(lg);
	exmodem_set_frontend(md, NULL);
}
