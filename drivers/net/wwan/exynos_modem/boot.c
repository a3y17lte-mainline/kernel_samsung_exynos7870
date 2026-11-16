// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos integrated modem: loading and starting the firmware
 *
 * The modem runs from a region of the system RAM. The application processor
 * copies the loader, the firmware and the modem's non-volatile data there,
 * asks the secure monitor to verify and lock the region, releases the modem
 * and then greets the loader on the boot channel. The loader answers and
 * starts the firmware, which reports in with PHONE_START.
 */

#include <linux/firmware.h>
#include <linux/unaligned.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/wait.h>

#include "fw.h"
#include "modem.h"

/* Greetings of the boot channel handshake and the loader's answers */
#define BOOT_HELLO		0x900d
#define BOOT_HELLO_ACK		0xa00d
#define BOOT_START		0x9f00
#define BOOT_START_ACK		0xaf00
#define BOOT_ACK_TIMEOUT	(2 * HZ)

struct exmodem_boot_port {
	struct exmodem_port port;
	struct sk_buff_head rxq;
	wait_queue_head_t wq;
};

static void boot_port_rx(struct exmodem_port *port, struct sk_buff *skb)
{
	struct exmodem_boot_port *bp = container_of(port, struct exmodem_boot_port,
						    port);

	skb_queue_tail(&bp->rxq, skb);
	wake_up(&bp->wq);
}

static int boot_exchange(struct exmodem *md, struct exmodem_boot_port *bp,
			 u32 hello, u32 ack)
{
	struct sk_buff *skb;
	u32 answer;
	int ret;

	skb = alloc_skb(EXMODEM_TX_HEADROOM + sizeof(__le32) +
			EXMODEM_TX_TAILROOM, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	skb_reserve(skb, EXMODEM_TX_HEADROOM);
	put_unaligned_le32(hello, skb_put(skb, sizeof(__le32)));

	ret = exmodem_link_send(md, SIPC5_CH_BOOT_0, skb);
	if (ret)
		return ret;

	if (!wait_event_timeout(bp->wq, !skb_queue_empty(&bp->rxq),
				BOOT_ACK_TIMEOUT))
		return -ETIMEDOUT;

	skb = skb_dequeue(&bp->rxq);
	if (skb->len < sizeof(__le32)) {
		kfree_skb(skb);
		return -EPROTO;
	}

	answer = get_unaligned_le32(skb->data);
	kfree_skb(skb);

	if (answer != ack) {
		dev_err(md->dev, "loader answered %#x to %#x\n", answer, hello);
		return -EPROTO;
	}

	return 0;
}

static int boot_handshake(struct exmodem *md)
{
	struct exmodem_boot_port bp = {
		.port = { .ch = SIPC5_CH_BOOT_0, .rx = boot_port_rx },
	};
	int ret;

	skb_queue_head_init(&bp.rxq);
	init_waitqueue_head(&bp.wq);

	ret = exmodem_port_register(md, &bp.port);
	if (ret)
		return ret;

	ret = boot_exchange(md, &bp, BOOT_HELLO, BOOT_HELLO_ACK);
	if (!ret)
		ret = boot_exchange(md, &bp, BOOT_START, BOOT_START_ACK);

	exmodem_port_unregister(md, &bp.port);
	skb_queue_purge(&bp.rxq);

	return ret;
}

/*
 * Load a part of a file directly into the modem's memory. Partial loads also
 * keep the files out of the firmware cache: the image is big and the NV data
 * changes while the modem runs.
 */
static int boot_load_part(struct exmodem *md, const char *name, u32 offset,
			  u32 len, u32 dest)
{
	const struct firmware *fw;
	void *buf;
	int ret;

	buf = exmodem_fw_buf(md, dest, len);
	if (!buf) {
		dev_err(md->dev, "%s: %#x bytes at %#x do not fit\n", name, len,
			dest);
		return -EINVAL;
	}

	ret = request_partial_firmware_into_buf(&fw, name, md->dev, buf, len,
						offset);
	if (ret)
		return ret;

	if (fw->size != len) {
		dev_err(md->dev, "%s is truncated\n", name);
		ret = -EINVAL;
	}
	release_firmware(fw);

	return ret;
}

static int boot_read_toc(struct exmodem *md, const char *name,
			 struct exmodem_toc_entry *boot,
			 struct exmodem_toc_entry *main,
			 struct exmodem_toc_entry *nv)
{
	const struct firmware *fw;
	u8 *toc;
	int ret;

	toc = kmalloc(EXMODEM_TOC_SIZE, GFP_KERNEL);
	if (!toc)
		return -ENOMEM;

	ret = request_partial_firmware_into_buf(&fw, name, md->dev, toc,
						EXMODEM_TOC_SIZE, 0);
	if (ret)
		goto out;

	ret = exmodem_toc_find(toc, fw->size, "BOOT", boot);
	if (!ret)
		ret = exmodem_toc_find(toc, fw->size, "MAIN", main);
	if (!ret)
		ret = exmodem_toc_find(toc, fw->size, "NV", nv);
	if (!ret && (!boot->size || !main->size || !nv->size ||
		     main->load_addr < boot->load_addr ||
		     nv->load_addr < boot->load_addr))
		ret = -EINVAL;
	if (ret)
		dev_err(md->dev, "%s: no usable table of contents\n", name);

	release_firmware(fw);
out:
	kfree(toc);
	return ret;
}

static int boot_load(struct exmodem *md, const char *image, const char *nv)
{
	struct exmodem_toc_entry boot, main, nv_part;
	u32 base;
	int ret;

	ret = boot_read_toc(md, image, &boot, &main, &nv_part);
	if (ret)
		return ret;

	/* An answer other than an error is information only */
	ret = exmodem_security(md, EXMODEM_SEC_UNLOCK, 0, 0);
	if (ret < 0)
		return ret;

	base = boot.load_addr;
	ret = boot_load_part(md, image, boot.offset, boot.size, 0);
	if (!ret)
		ret = boot_load_part(md, image, main.offset, main.size,
				     main.load_addr - base);
	if (!ret)
		ret = boot_load_part(md, nv, 0, nv_part.size,
				     nv_part.load_addr - base);
	if (!ret)
		ret = exmodem_fw_check(md);
	if (ret)
		return ret;

	/* The firmware region belongs to the modem from here on */
	ret = exmodem_security(md, EXMODEM_SEC_VERIFY, boot.size, main.size);
	if (ret) {
		dev_err(md->dev, "firmware verification failed: %d\n", ret);
		return ret < 0 ? ret : -EPERM;
	}

	return 0;
}

/**
 * exmodem_boot() - load the firmware and start the modem
 * @md: the modem
 *
 * Loads the two files named by the "firmware-name" property: the firmware
 * image and the modem's non-volatile data. The NV data is read anew on every
 * start because the modem changes it while it runs.
 *
 * Return: 0 once the firmware has reported in, or a negative error code.
 */
int exmodem_boot(struct exmodem *md)
{
	const char *names[2];
	int ret;

	ret = device_property_read_string_array(md->dev, "firmware-name",
						names, ARRAY_SIZE(names));
	if (ret < 0)
		return ret;
	if (ret < ARRAY_SIZE(names))
		return -EINVAL;

	ret = exmodem_reset(md);
	if (!ret)
		ret = boot_load(md, names[0], names[1]);
	if (!ret)
		ret = exmodem_power_on(md);
	if (!ret)
		ret = exmodem_boot_on(md);
	if (!ret)
		ret = exmodem_dl_start(md);
	if (!ret)
		ret = boot_handshake(md);
	if (!ret)
		ret = exmodem_boot_off(md);

	if (ret) {
		dev_err(md->dev, "modem start failed: %d\n", ret);
		exmodem_stop(md);
	}

	return ret;
}
