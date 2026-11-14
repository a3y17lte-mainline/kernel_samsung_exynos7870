// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Raw NCI character device for the Samsung S3FWRN5 family
 *
 * Android's NFC stack is an NCI host in userspace and needs the packets
 * themselves, which the kernel's NCI core does not hand out. With
 * CONFIG_NFC_S3FWRN5_RAW the chip is not registered with the NCI core;
 * /dev/s3fwrn5 carries its NCI packets instead. The driver keeps the bus
 * framing, the interrupt, the power sequence and the clock. Only the NCI
 * mode can be entered: the bootloader (firmware download) mode is out of
 * reach of userspace.
 */

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/skbuff.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <uapi/linux/s3fwrn5-raw.h>

#include "s3fwrn5.h"

#define S3FWRN5_RAW_MAX_PACKET	(NCI_CTRL_HDR_SIZE + 255)
#define S3FWRN5_RAW_MAX_QUEUE	64

struct s3fwrn5_raw {
	struct miscdevice misc;
	struct s3fwrn5_info *info;
	struct sk_buff_head rx;
	wait_queue_head_t wait;
	unsigned long opened;
};

static struct s3fwrn5_raw *s3fwrn5_raw_from_file(struct file *file)
{
	/* misc_open() leaves the miscdevice in private_data */
	return container_of(file->private_data, struct s3fwrn5_raw, misc);
}

static void s3fwrn5_raw_power(struct s3fwrn5_raw *raw, bool on)
{
	struct s3fwrn5_info *info = raw->info;

	mutex_lock(&info->mutex);

	s3fwrn5_set_wake(info, false);
	s3fwrn5_set_mode(info, S3FWRN5_MODE_COLD);
	skb_queue_purge(&raw->rx);

	if (on) {
		s3fwrn5_set_mode(info, S3FWRN5_MODE_NCI);
		s3fwrn5_set_wake(info, true);
	}

	mutex_unlock(&info->mutex);
}

static int s3fwrn5_raw_open(struct inode *inode, struct file *file)
{
	struct s3fwrn5_raw *raw = s3fwrn5_raw_from_file(file);

	if (test_and_set_bit(0, &raw->opened))
		return -EBUSY;

	return nonseekable_open(inode, file);
}

static int s3fwrn5_raw_release(struct inode *inode, struct file *file)
{
	struct s3fwrn5_raw *raw = s3fwrn5_raw_from_file(file);

	s3fwrn5_raw_power(raw, false);
	clear_bit(0, &raw->opened);

	return 0;
}

static ssize_t s3fwrn5_raw_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct s3fwrn5_raw *raw = s3fwrn5_raw_from_file(file);
	struct sk_buff *skb;
	ssize_t ret;

	while (!(skb = skb_dequeue(&raw->rx))) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(raw->wait,
					       !skb_queue_empty(&raw->rx));
		if (ret)
			return ret;
	}

	if (count < skb->len) {
		skb_queue_head(&raw->rx, skb);
		return -EMSGSIZE;
	}

	ret = skb->len;
	if (copy_to_user(buf, skb->data, skb->len))
		ret = -EFAULT;
	consume_skb(skb);

	return ret;
}

static ssize_t s3fwrn5_raw_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct s3fwrn5_raw *raw = s3fwrn5_raw_from_file(file);
	struct s3fwrn5_info *info = raw->info;
	struct sk_buff *skb;
	int ret;

	if (count < NCI_CTRL_HDR_SIZE || count > S3FWRN5_RAW_MAX_PACKET)
		return -EINVAL;

	skb = alloc_skb(count, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	if (copy_from_user(skb_put(skb, count), buf, count)) {
		kfree_skb(skb);
		return -EFAULT;
	}

	mutex_lock(&info->mutex);
	if (s3fwrn5_get_mode(info) == S3FWRN5_MODE_NCI)
		ret = s3fwrn5_write(info, skb);
	else
		ret = -ENODEV;
	mutex_unlock(&info->mutex);

	kfree_skb(skb);

	return ret ? ret : count;
}

static __poll_t s3fwrn5_raw_poll(struct file *file, poll_table *wait)
{
	struct s3fwrn5_raw *raw = s3fwrn5_raw_from_file(file);
	__poll_t mask = EPOLLOUT | EPOLLWRNORM;

	poll_wait(file, &raw->wait, wait);
	if (!skb_queue_empty(&raw->rx))
		mask |= EPOLLIN | EPOLLRDNORM;

	return mask;
}

static long s3fwrn5_raw_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	struct s3fwrn5_raw *raw = s3fwrn5_raw_from_file(file);
	u32 on;

	switch (cmd) {
	case S3FWRN5_RAW_IOC_POWER:
		if (get_user(on, (u32 __user *)arg))
			return -EFAULT;
		if (on > 1)
			return -EINVAL;
		s3fwrn5_raw_power(raw, on);
		return 0;
	default:
		return -ENOTTY;
	}
}

static const struct file_operations s3fwrn5_raw_fops = {
	.owner = THIS_MODULE,
	.open = s3fwrn5_raw_open,
	.release = s3fwrn5_raw_release,
	.read = s3fwrn5_raw_read,
	.write = s3fwrn5_raw_write,
	.poll = s3fwrn5_raw_poll,
	.unlocked_ioctl = s3fwrn5_raw_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

/* A packet from the chip, called by the bus driver's interrupt thread */
int s3fwrn5_raw_recv_frame(struct s3fwrn5_info *info, struct sk_buff *skb)
{
	struct s3fwrn5_raw *raw = info->raw;

	/* Nobody reads: keep the newest packets */
	if (skb_queue_len(&raw->rx) >= S3FWRN5_RAW_MAX_QUEUE)
		kfree_skb(skb_dequeue(&raw->rx));

	skb_queue_tail(&raw->rx, skb);
	wake_up_interruptible(&raw->wait);

	return 0;
}

int s3fwrn5_raw_register(struct s3fwrn5_info *info)
{
	struct s3fwrn5_raw *raw;

	raw = devm_kzalloc(info->pdev, sizeof(*raw), GFP_KERNEL);
	if (!raw)
		return -ENOMEM;

	raw->info = info;
	skb_queue_head_init(&raw->rx);
	init_waitqueue_head(&raw->wait);
	raw->misc.minor = MISC_DYNAMIC_MINOR;
	raw->misc.name = "s3fwrn5";
	raw->misc.fops = &s3fwrn5_raw_fops;
	raw->misc.parent = info->pdev;
	info->raw = raw;

	return misc_register(&raw->misc);
}

void s3fwrn5_raw_unregister(struct s3fwrn5_info *info)
{
	misc_deregister(&info->raw->misc);
	skb_queue_purge(&info->raw->rx);
}
