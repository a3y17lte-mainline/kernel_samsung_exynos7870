// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 Samsung Electronics Co., Ltd.
 * Copyright 2020 Google LLC.
 * Copyright 2024 Linaro Ltd.
 */

#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox/exynos-message.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define EXYNOS_MBOX_INTMR0		0x28	/* Interrupt Mask Register 0 */
#define EXYNOS_MBOX_INTGR1		0x40	/* Interrupt Generation Register 1 */

#define EXYNOS_MBOX_INTMR0_MASK		GENMASK(15, 0)
#define EXYNOS_MBOX_INTGR1_MASK		GENMASK(15, 0)

#define EXYNOS_MBOX_CHAN_COUNT		HWEIGHT32(EXYNOS_MBOX_INTGR1_MASK)

#define EXYNOS850_MBOX_INTGR0		0x8	/* Interrupt Generation Register 0	*/
#define EXYNOS850_MBOX_INTMR1		0x24	/* Interrupt Mask Register 1		*/

#define EXYNOS850_MBOX_INTMR1_MASK	GENMASK(15, 0)
#define EXYNOS850_MBOX_INTGR0_MASK	GENMASK(31, 16)

#define EXYNOS850_MBOX_CHAN_COUNT	HWEIGHT32(EXYNOS850_MBOX_INTGR0_MASK)

/*
 * Exynos7870 modem (CP) mailbox: the AP owns interrupt group 0 and rings
 * group 1. The 64 shared registers carry status words in both directions.
 */
#define EXYNOS7870_MBOX_INTCR0		0xc	/* Interrupt Clear Register 0		*/
#define EXYNOS7870_MBOX_INTSR0		0x14	/* Interrupt Status Register 0		*/
#define EXYNOS7870_MBOX_INTGR1		0x1c	/* Interrupt Generation Register 1	*/
#define EXYNOS7870_MBOX_INTCR1		0x20	/* Interrupt Clear Register 1		*/
#define EXYNOS7870_MBOX_ISSR0		0x80	/* Shared Register 0			*/

#define EXYNOS7870_MBOX_INTSR0_MASK	GENMASK(31, 16)
#define EXYNOS7870_MBOX_INTGR1_MASK	GENMASK(15, 0)
#define EXYNOS7870_MBOX_NUM_ISSR	64

#define EXYNOS7870_MBOX_CHAN_COUNT	HWEIGHT32(EXYNOS7870_MBOX_INTGR1_MASK)

/**
 * struct exynos_mbox_driver_data - platform-specific mailbox configuration.
 * @intgr:		offset to the IRQ generation register, doorbell
 *			to APM co-processor.
 * @intgr_shift:	shift to apply to the value written to IRQ generation
 *			register.
 * @intmr:		offset to the IRQ mask register.
 * @intmr_mask:		value to write to the mask register to mask out all
 *			interrupts.
 * @num_chans:		number of channels the mailbox can support (hardware
 *			capability).
 * @intsr:		offset to the IRQ status register of received
 *			interrupts, for controllers with one channel per
 *			interrupt bit in both directions.
 * @intsr_mask:		the receive interrupt bits in the status register;
 *			zero if the controller is a doorbell only.
 * @intcr:		offset to the IRQ clear register of received interrupts.
 * @tx_intcr:		offset to the IRQ clear register of sent interrupts.
 * @issr:		offset to the first shared register.
 * @num_issr:		number of shared registers exposed as a regmap.
 */
struct exynos_mbox_driver_data {
	u32 intgr;
	u32 intgr_shift;
	u32 intmr;
	u32 intmr_mask;
	int num_chans;
	u32 intsr;
	u32 intsr_mask;
	u32 intcr;
	u32 tx_intcr;
	u32 issr;
	u32 num_issr;
};

/**
 * struct exynos_mbox - driver's private data.
 * @regs:	mailbox registers base address.
 * @mbox:	pointer to the mailbox controller.
 * @data:	pointer to driver platform-specific data.
 */
struct exynos_mbox {
	void __iomem *regs;
	struct mbox_controller *mbox;
	const struct exynos_mbox_driver_data *data;
};

static const struct exynos_mbox_driver_data exynos850_mbox_data = {
	.intgr = EXYNOS850_MBOX_INTGR0,
	.intgr_shift = 16,
	.intmr = EXYNOS850_MBOX_INTMR1,
	.intmr_mask = EXYNOS850_MBOX_INTMR1_MASK,
	.num_chans = EXYNOS850_MBOX_CHAN_COUNT,
};

static const struct exynos_mbox_driver_data exynos7870_mbox_data = {
	.intgr = EXYNOS7870_MBOX_INTGR1,
	.intgr_shift = 0,
	.num_chans = EXYNOS7870_MBOX_CHAN_COUNT,
	.intsr = EXYNOS7870_MBOX_INTSR0,
	.intsr_mask = EXYNOS7870_MBOX_INTSR0_MASK,
	.intcr = EXYNOS7870_MBOX_INTCR0,
	.tx_intcr = EXYNOS7870_MBOX_INTCR1,
	.issr = EXYNOS7870_MBOX_ISSR0,
	.num_issr = EXYNOS7870_MBOX_NUM_ISSR,
};

static const struct exynos_mbox_driver_data exynos_gs101_mbox_data = {
	.intgr = EXYNOS_MBOX_INTGR1,
	.intgr_shift = 0,
	.intmr = EXYNOS_MBOX_INTMR0,
	.intmr_mask = EXYNOS_MBOX_INTMR0_MASK,
	.num_chans = EXYNOS_MBOX_CHAN_COUNT,
};

static int exynos_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct device *dev = chan->mbox->dev;
	struct exynos_mbox *exynos_mbox = dev_get_drvdata(dev);
	struct exynos_mbox_msg *msg = data;

	if (msg->chan_id >= exynos_mbox->mbox->num_chans) {
		dev_err(dev, "Invalid channel ID %d\n", msg->chan_id);
		return -EINVAL;
	}

	if (msg->chan_type != EXYNOS_MBOX_CHAN_TYPE_DOORBELL) {
		dev_err(dev, "Unsupported channel type [%d]\n", msg->chan_type);
		return -EINVAL;
	}

	/* Ring the doorbell */
	writel(BIT(msg->chan_id) << exynos_mbox->data->intgr_shift,
	       exynos_mbox->regs + exynos_mbox->data->intgr);

	return 0;
}

static const struct mbox_chan_ops exynos_mbox_chan_ops = {
	.send_data = exynos_mbox_send_data,
};

/* One channel per interrupt bit: the message is the interrupt itself. */
static int exynos_mbox_ring(struct mbox_chan *chan, void *data)
{
	struct exynos_mbox *exynos_mbox = dev_get_drvdata(chan->mbox->dev);
	unsigned int id = chan - chan->mbox->chans;

	writel(BIT(id) << exynos_mbox->data->intgr_shift,
	       exynos_mbox->regs + exynos_mbox->data->intgr);

	return 0;
}

static const struct mbox_chan_ops exynos_mbox_irq_chan_ops = {
	.send_data = exynos_mbox_ring,
};

static irqreturn_t exynos_mbox_irq(int irq, void *dev_id)
{
	struct exynos_mbox *exynos_mbox = dev_id;
	const struct exynos_mbox_driver_data *data = exynos_mbox->data;
	struct mbox_controller *mbox = exynos_mbox->mbox;
	unsigned long status;
	unsigned int bit;

	status = readl(exynos_mbox->regs + data->intsr) & data->intsr_mask;
	if (!status)
		return IRQ_NONE;

	writel(status, exynos_mbox->regs + data->intcr);

	status >>= __ffs(data->intsr_mask);
	for_each_set_bit(bit, &status, mbox->num_chans) {
		struct mbox_chan *chan = &mbox->chans[bit];

		if (chan->cl)
			mbox_chan_received_data(chan, NULL);
		else
			dev_warn_ratelimited(mbox->dev,
					     "interrupt %u without a client\n",
					     bit);
	}

	return IRQ_HANDLED;
}

static struct mbox_chan *exynos_mbox_irq_of_xlate(struct mbox_controller *mbox,
						  const struct of_phandle_args *sp)
{
	if (sp->args_count != 1 || sp->args[0] >= mbox->num_chans)
		return ERR_PTR(-EINVAL);

	return &mbox->chans[sp->args[0]];
}

static int exynos_mbox_irq_init(struct platform_device *pdev,
				struct exynos_mbox *exynos_mbox)
{
	const struct exynos_mbox_driver_data *data = exynos_mbox->data;
	const struct regmap_config config = {
		.reg_bits = 32,
		.reg_stride = 4,
		.val_bits = 32,
		.max_register = (data->num_issr - 1) * 4,
	};
	struct device *dev = &pdev->dev;
	struct regmap *regmap;
	int irq, ret;

	regmap = devm_regmap_init_mmio(dev, exynos_mbox->regs + data->issr,
				       &config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* Nothing may be pending in either direction before the peer starts. */
	writel(data->intsr_mask, exynos_mbox->regs + data->intcr);
	writel(GENMASK(data->num_chans - 1, 0) << data->intgr_shift,
	       exynos_mbox->regs + data->tx_intcr);

	ret = devm_request_irq(dev, irq, exynos_mbox_irq, 0, dev_name(dev),
			       exynos_mbox);
	if (ret)
		return ret;

	/* A message from the peer, e.g. an incoming call, can wake the system */
	if (device_property_read_bool(dev, "wakeup-source")) {
		device_init_wakeup(dev, true);
		ret = devm_pm_set_wake_irq(dev, irq);
	}

	return ret;
}

static struct mbox_chan *exynos_mbox_of_xlate(struct mbox_controller *mbox,
					      const struct of_phandle_args *sp)
{
	int i;

	if (sp->args_count != 0)
		return ERR_PTR(-EINVAL);

	/*
	 * Return the first available channel. When we don't pass the
	 * channel ID from device tree, each channel populated by the driver is
	 * just a software construct or a virtual channel. We use 'void *data'
	 * in send_data() to pass the channel identifiers.
	 */
	for (i = 0; i < mbox->num_chans; i++)
		if (mbox->chans[i].cl == NULL)
			return &mbox->chans[i];
	return ERR_PTR(-EINVAL);
}

static const struct of_device_id exynos_mbox_match[] = {
	{
		.compatible = "google,gs101-mbox",
		.data = &exynos_gs101_mbox_data
	},
	{
		.compatible = "samsung,exynos7870-mbox",
		.data = &exynos7870_mbox_data
	},
	{
		.compatible = "samsung,exynos850-mbox",
		.data = &exynos850_mbox_data
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_mbox_match);

static int exynos_mbox_probe(struct platform_device *pdev)
{
	const struct exynos_mbox_driver_data *data;
	struct device *dev = &pdev->dev;
	struct exynos_mbox *exynos_mbox;
	struct mbox_controller *mbox;
	struct mbox_chan *chans;
	struct clk *pclk;

	data = device_get_match_data(&pdev->dev);
	if (!data)
		return -ENODEV;

	exynos_mbox = devm_kzalloc(dev, sizeof(*exynos_mbox), GFP_KERNEL);
	if (!exynos_mbox)
		return -ENOMEM;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	chans = devm_kcalloc(dev, data->num_chans, sizeof(*chans), GFP_KERNEL);
	if (!chans)
		return -ENOMEM;

	exynos_mbox->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(exynos_mbox->regs))
		return PTR_ERR(exynos_mbox->regs);

	pclk = devm_clk_get_enabled(dev, "pclk");
	if (IS_ERR(pclk))
		return dev_err_probe(dev, PTR_ERR(pclk),
				     "Failed to enable clock.\n");

	exynos_mbox->data = data;
	mbox->num_chans = data->num_chans;
	mbox->chans = chans;
	mbox->dev = dev;
	mbox->ops = &exynos_mbox_chan_ops;
	mbox->of_xlate = exynos_mbox_of_xlate;

	exynos_mbox->mbox = mbox;

	platform_set_drvdata(pdev, exynos_mbox);

	if (data->intsr_mask) {
		int ret;

		mbox->ops = &exynos_mbox_irq_chan_ops;
		mbox->of_xlate = exynos_mbox_irq_of_xlate;

		ret = exynos_mbox_irq_init(pdev, exynos_mbox);
		if (ret)
			return ret;
	} else {
		/*
		 * Mask out all interrupts. The ACPM interface only polls its
		 * channels.
		 */
		writel(data->intmr_mask, exynos_mbox->regs + data->intmr);
	}

	return devm_mbox_controller_register(dev, mbox);
}

static struct platform_driver exynos_mbox_driver = {
	.probe	= exynos_mbox_probe,
	.driver	= {
		.name = "exynos-acpm-mbox",
		.of_match_table	= exynos_mbox_match,
	},
};
module_platform_driver(exynos_mbox_driver);

MODULE_AUTHOR("Tudor Ambarus <tudor.ambarus@linaro.org>");
MODULE_DESCRIPTION("Samsung Exynos mailbox driver");
MODULE_LICENSE("GPL");
