// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos integrated modem: power, boot and performance requests
 *
 * The modem ("CP") is a second processor inside the SoC with its own radio
 * firmware. It runs from a region of the system RAM that the bootloader
 * assigned to it. The application processor copies the firmware there, has
 * the secure monitor verify it, and starts the modem through the PMU. From
 * then on the two talk through a mailbox and a shared memory region, see
 * link.c.
 */

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/soc/samsung/exynos-pmu.h>

#include "modem.h"

/* PMU registers */
#define PMU_CENTRAL_SEQ_CP_CONFIGURATION	0x0280
#define PMU_RESET_AHEAD_CP_SYS_PWR_REG	0x1170
#define PMU_CLEANY_BUS_SYS_PWR_REG	0x11cc
#define PMU_LOGIC_RESET_CP_SYS_PWR_REG	0x11d0
#define PMU_TCXO_GATE_SYS_PWR_REG	0x11d4
#define PMU_RESET_ASB_CP_SYS_PWR_REG	0x11d8

#define CP_PWRON			BIT(1)
#define CP_RESET_SET			BIT(2)
#define CP_START			BIT(3)
#define CP_ACTIVE_REQ_CLR		BIT(6)
#define CP_RESET_REQ_CLR		BIT(8)

/*
 * Secure monitor calls. The PMU registers that power and start the modem are
 * not accessible from the normal world: the monitor reads and writes them.
 * The other requests of the same call are those of enum exmodem_sec_mode.
 */
#define SMC_CP_BOOT			0x82000700
#define SMC_CP_BOOT_READ_CTRL		3
#define SMC_CP_BOOT_WRITE_CTRL		4
#define SMC_CP_CTRL_S			0
#define SMC_CP_CTRL_NS			1
#define SMC_CP_CTRL_ERROR		GENMASK(15, 0)
#define SMC_CP_CTRL_VALUE		GENMASK(31, 16)
#define SMC_SSS_CLK			0x82001011
#define SMC_SSS_CLK_ENABLE		0
#define SMC_SSS_CLK_DISABLE		1

/* The bootloader's data for the modem goes to the start of this buffer */
#define BOOTINFO_IPC_OFFSET		0x1000
#define BOOTINFO_SIZE			0x800

/* A debugger attached to the modem sets this to keep it from being reset */
#define NO_RESET_IPC_OFFSET		0xf80
#define NO_RESET_MAGIC			0xdeb

/* The firmware states the size of the regions it was built for */
#define FW_MEM_INFO_OFFSET		0x10000
#define FW_MEM_INFO_CP_SIZE		17
#define FW_MEM_INFO_IPC_SIZE		18
#define FW_MEM_INFO_WORDS		19

#define INFO_DUAL_SIM			BIT(8)

#define PERF_CLUSTER0			BIT(31)
#define PERF_LEVEL			GENMASK(7, 0)

#define BOOT_STATUS_POLLS		100
#define INIT_TIMEOUT			(15 * HZ)
#define OFF_TIMEOUT			(3 * HZ)

enum {
	BUS_MIF,
	BUS_INT,
};

static const char * const exmodem_mbox_names[EXMODEM_MBOX_COUNT] = {
	[EXMODEM_MBOX_MSG] = "msg",
	[EXMODEM_MBOX_STATUS] = "status",
	[EXMODEM_MBOX_PERF] = "perf",
	[EXMODEM_MBOX_ACTIVE] = "active",
	[EXMODEM_MBOX_PERF_CPU] = "perf-cpu",
	[EXMODEM_MBOX_PERF_MIF] = "perf-mif",
	[EXMODEM_MBOX_PERF_INT] = "perf-int",
};

enum exmodem_state exmodem_get_state(struct exmodem *md)
{
	return READ_ONCE(md->state);
}

/**
 * exmodem_set_state() - change the modem state and tell the front end
 * @md: the modem
 * @state: the new state
 *
 * Can be called from atomic context.
 */
void exmodem_set_state(struct exmodem *md, enum exmodem_state state)
{
	const struct exmodem_frontend *frontend = READ_ONCE(md->frontend);
	unsigned long flags;
	bool changed;

	spin_lock_irqsave(&md->state_lock, flags);
	changed = md->state != state;
	WRITE_ONCE(md->state, state);
	spin_unlock_irqrestore(&md->state_lock, flags);

	if (changed)
		dev_info(md->dev, "state %d\n", state);

	if (frontend && frontend->state_changed)
		frontend->state_changed(md, state);
}

void exmodem_set_frontend(struct exmodem *md,
			  const struct exmodem_frontend *frontend)
{
	WRITE_ONCE(md->frontend, frontend);
}

int exmodem_issr_read(struct exmodem *md, unsigned int reg)
{
	unsigned int val = 0;

	regmap_read(md->issr, reg * 4, &val);

	return val;
}

void exmodem_issr_write(struct exmodem *md, unsigned int reg, u32 val)
{
	regmap_write(md->issr, reg * 4, val);
}

/* Raise an interrupt at the modem */
void exmodem_mbox_ring(struct exmodem *md, enum exmodem_mbox id)
{
	struct mbox_chan *chan = md->mbox[id].chan;

	/* The interrupt is the message, but the mailbox core wants a pointer */
	if (mbox_send_message(chan, md) >= 0)
		mbox_client_txdone(chan, 0);
}

static void exmodem_irq_set(struct exmodem *md, int irq, bool *enabled,
			    bool enable)
{
	unsigned long flags;

	spin_lock_irqsave(&md->state_lock, flags);
	if (*enabled != enable) {
		*enabled = enable;
		if (enable)
			enable_irq(irq);
		else
			disable_irq_nosync(irq);
	}
	spin_unlock_irqrestore(&md->state_lock, flags);
}

static int exmodem_ctrl_read(struct exmodem *md, unsigned int reg, u32 *val)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_CP_BOOT, SMC_CP_BOOT_READ_CTRL, 0, reg, 0, 0, 0, 0,
		      &res);
	/* The monitor refuses until it has verified a modem image */
	if (res.a0 & SMC_CP_CTRL_ERROR) {
		dev_dbg(md->dev, "reading modem control %u failed: %lu\n", reg,
			res.a0 & SMC_CP_CTRL_ERROR);
		return -EIO;
	}

	*val = FIELD_GET(SMC_CP_CTRL_VALUE, res.a0);

	return 0;
}

static int exmodem_ctrl_update(struct exmodem *md, unsigned int reg, u32 mask,
			       u32 val)
{
	struct arm_smccc_res res;
	u32 ctrl;
	int ret;

	ret = exmodem_ctrl_read(md, reg, &ctrl);
	if (ret)
		return ret;

	ctrl = (ctrl & ~mask) | val;
	dev_dbg(md->dev, "modem control %u: %#x\n", reg, ctrl);
	arm_smccc_smc(SMC_CP_BOOT, SMC_CP_BOOT_WRITE_CTRL, ctrl, reg, 0, 0, 0, 0,
		      &res);
	if ((s32)res.a0 > 0) {
		dev_err(md->dev, "writing modem control %u failed: %d\n", reg,
			(s32)res.a0);
		return -EIO;
	}

	return 0;
}

static bool exmodem_powered(struct exmodem *md)
{
	u32 val = 0;

	exmodem_ctrl_read(md, SMC_CP_CTRL_NS, &val);

	return val & CP_PWRON;
}

/*
 * Boot and power sequences. They follow what the vendor's boot daemon
 * requests one step at a time: reset, unlock, firmware upload, verify, power
 * on, boot on, download start, (loader handshake over the boot channel),
 * boot off.
 */

/**
 * exmodem_reset() - put a running modem into reset
 * @md: the modem
 *
 * Return: 0 or a negative error code.
 */
int exmodem_reset(struct exmodem *md)
{
	static const unsigned int seq_regs[] = {
		PMU_CENTRAL_SEQ_CP_CONFIGURATION,
		PMU_RESET_AHEAD_CP_SYS_PWR_REG,
		PMU_LOGIC_RESET_CP_SYS_PWR_REG,
		PMU_RESET_ASB_CP_SYS_PWR_REG,
		PMU_TCXO_GATE_SYS_PWR_REG,
		PMU_CLEANY_BUS_SYS_PWR_REG,
	};
	__le32 *no_reset = md->ipc_base + NO_RESET_IPC_OFFSET;
	int ret = 0;
	int i;

	guard(mutex)(&md->lock);

	if (le32_to_cpu(READ_ONCE(*no_reset)) == NO_RESET_MAGIC)
		return 0;

	if (!exmodem_powered(md))
		return 0;

	for (i = 0; i < ARRAY_SIZE(seq_regs) && !ret; i++)
		ret = regmap_write(md->pmu, seq_regs[i], 0);
	if (ret)
		return ret;

	ret = exmodem_ctrl_update(md, SMC_CP_CTRL_NS, CP_RESET_SET,
				  CP_RESET_SET);
	usleep_range(80, 100);

	return ret;
}

struct exmodem_smc {
	unsigned long mode;
	unsigned long arg2;
	unsigned long arg3;
};

static long exmodem_smc_fn(void *data)
{
	struct exmodem_smc *smc = data;
	struct arm_smccc_res res;
	long ret;

	/* The verification runs on the crypto engine */
	arm_smccc_smc(SMC_SSS_CLK, SMC_SSS_CLK_ENABLE, 0, 0, 0, 0, 0, 0, &res);
	arm_smccc_smc(SMC_CP_BOOT, smc->mode, smc->arg2, smc->arg3, 0, 0, 0, 0,
		      &res);
	ret = res.a0;
	arm_smccc_smc(SMC_SSS_CLK, SMC_SSS_CLK_DISABLE, 0, 0, 0, 0, 0, 0, &res);

	return ret;
}

/**
 * exmodem_security() - secure monitor request for the firmware region
 * @md: the modem
 * @mode: what to do
 * @size_boot: size of the loader image, for EXMODEM_SEC_VERIFY and _DUMP
 * @size_main: size of the main image, for EXMODEM_SEC_VERIFY
 *
 * Return: the secure monitor's answer, 0 for success, or a negative error
 * code.
 */
int exmodem_security(struct exmodem *md, enum exmodem_sec_mode mode,
		     u32 size_boot, u32 size_main)
{
	struct exmodem_smc smc = { .mode = mode };
	unsigned int cpu;
	long ret;

	switch (mode) {
	case EXMODEM_SEC_VERIFY:
		smc.arg2 = size_boot;
		smc.arg3 = size_main;
		break;
	case EXMODEM_SEC_DUMP:
		smc.arg2 = size_boot;
		smc.arg3 = md->ipc_phys;
		break;
	case EXMODEM_SEC_UNLOCK:
		break;
	default:
		return -EINVAL;
	}

	guard(mutex)(&md->lock);

	/* The vendor software keeps this request away from the boot CPU */
	cpus_read_lock();
	cpu = cpumask_any_but(cpu_online_mask, 0);
	if (cpu >= nr_cpu_ids)
		cpu = 0;
	ret = work_on_cpu(cpu, exmodem_smc_fn, &smc);
	cpus_read_unlock();

	dev_info(md->dev, "security request %d: %ld\n", mode, ret);

	return ret;
}

/**
 * exmodem_fw_write() - copy a part of an image into the modem's memory
 * @md: the modem
 * @ipc_region: write to the IPC region instead of the firmware region
 * @offset: offset in the region
 * @data: the bytes
 * @len: number of bytes
 *
 * Return: 0, -EINVAL if the part does not fit, or -EFAULT if the firmware
 * was built for other region sizes.
 */
int exmodem_fw_write(struct exmodem *md, bool ipc_region, u32 offset,
		     const void *data, u32 len)
{
	size_t size = ipc_region ? md->ipc_size : md->fw_size;
	void *base = ipc_region ? md->ipc_base : md->fw_base;

	if (len > size || offset > size - len)
		return -EINVAL;

	if (!ipc_region && offset == FW_MEM_INFO_OFFSET &&
	    len >= FW_MEM_INFO_WORDS * sizeof(__le32)) {
		const __le32 *info = data;
		u32 cp_size = get_unaligned_le32(&info[FW_MEM_INFO_CP_SIZE]);
		u32 ipc_size = get_unaligned_le32(&info[FW_MEM_INFO_IPC_SIZE]);

		if (cp_size != md->fw_size || ipc_size != md->ipc_size) {
			dev_err(md->dev,
				"firmware wants %#x + %#x bytes of memory\n",
				cp_size, ipc_size);
			return -EFAULT;
		}
	}

	guard(mutex)(&md->lock);
	memcpy(base + offset, data, len);

	return 0;
}

/**
 * exmodem_power_on() - power the modem up and release it from reset
 * @md: the modem, with a verified firmware in its memory
 *
 * Return: 0 or a negative error code.
 */
int exmodem_power_on(struct exmodem *md)
{
	int ret;
	int i;

	guard(mutex)(&md->lock);

	exmodem_set_state(md, EXMODEM_OFFLINE);

	for (i = 0; i < EXMODEM_ISSR_COUNT; i++)
		exmodem_issr_write(md, i, 0);
	exmodem_issr_write(md, EXMODEM_ISSR_INFO,
			   md->board_rev | (md->dual_sim ? INFO_DUAL_SIM : 0));

	exmodem_link_prepare(md);
	memcpy(md->ipc_base + BOOTINFO_IPC_OFFSET, md->bootinfo, BOOTINFO_SIZE);

	exmodem_issr_write(md, EXMODEM_ISSR_AP2CP_ACTIVE, 1);

	if (!exmodem_powered(md)) {
		/* A modem that was powered off needs a new start request */
		ret = exmodem_ctrl_update(md, SMC_CP_CTRL_S, CP_START, 0);
		if (ret)
			return ret;

		ret = exmodem_ctrl_update(md, SMC_CP_CTRL_NS, CP_PWRON,
					  CP_PWRON);
		if (ret)
			return ret;
	}

	ret = exmodem_ctrl_update(md, SMC_CP_CTRL_S, CP_START, CP_START);
	if (ret)
		return ret;

	msleep(300);

	return 0;
}

/**
 * exmodem_power_off() - cut the modem's power
 * @md: the modem
 *
 * Return: 0 or a negative error code.
 */
int exmodem_power_off(struct exmodem *md)
{
	guard(mutex)(&md->lock);

	return exmodem_ctrl_update(md, SMC_CP_CTRL_NS, CP_PWRON, 0);
}

/**
 * exmodem_boot_on() - wait for the modem's loader to come up
 * @md: the modem, just powered on
 *
 * Return: 0 or -EACCES if the loader does not report in.
 */
int exmodem_boot_on(struct exmodem *md)
{
	int i;

	guard(mutex)(&md->lock);

	exmodem_link_boot_on(md);
	reinit_completion(&md->init_done);
	reinit_completion(&md->off_done);
	exmodem_set_state(md, EXMODEM_BOOTING);

	for (i = 0; i < BOOT_STATUS_POLLS; i++) {
		if (exmodem_issr_read(md, EXMODEM_ISSR_CP2AP_STATUS))
			break;
		usleep_range(10000, 20000);
	}
	if (i == BOOT_STATUS_POLLS)
		return -EACCES;

	exmodem_irq_set(md, md->irq_wdt, &md->irq_wdt_enabled, false);
	exmodem_irq_set(md, md->irq_fail, &md->irq_fail_enabled, true);

	return 0;
}

/**
 * exmodem_dl_start() - open the boot channel to the loader
 * @md: the modem
 *
 * Return: 0 or a negative error code.
 */
int exmodem_dl_start(struct exmodem *md)
{
	int ret;

	guard(mutex)(&md->lock);

	ret = exmodem_link_start(md, EXMODEM_MAGIC_BOOT);
	exmodem_issr_write(md, EXMODEM_ISSR_AP2CP_STATUS, 1);

	return ret;
}

/**
 * exmodem_boot_off() - wait for the modem's firmware to start
 * @md: the modem, after the handshake with its loader
 *
 * Return: 0 or -EAGAIN if the firmware did not start in time.
 */
int exmodem_boot_off(struct exmodem *md)
{
	int ret = 0;

	guard(mutex)(&md->lock);

	if (wait_for_completion_timeout(&md->init_done, INIT_TIMEOUT)) {
		exmodem_irq_set(md, md->irq_wdt, &md->irq_wdt_enabled, true);
		exmodem_set_state(md, EXMODEM_ONLINE);
	} else {
		ret = -EAGAIN;
	}

	exmodem_irq_set(md, md->irq_fail, &md->irq_fail_enabled, false);

	return ret;
}

/**
 * exmodem_dump_start() - let the crashed modem's loader send a memory dump
 * @md: the modem
 *
 * Return: 0 or a negative error code.
 */
int exmodem_dump_start(struct exmodem *md)
{
	int ret;

	guard(mutex)(&md->lock);

	ret = exmodem_link_start(md, EXMODEM_MAGIC_DUMP);
	if (ret)
		return ret;

	ret = exmodem_ctrl_update(md, SMC_CP_CTRL_S, CP_START, CP_START);
	exmodem_issr_write(md, EXMODEM_ISSR_AP2CP_STATUS, 1);

	return ret;
}

static irqreturn_t exmodem_fail_irq(int irq, void *data)
{
	struct exmodem *md = data;

	exmodem_irq_set(md, md->irq_fail, &md->irq_fail_enabled, false);
	dev_err(md->dev, "the modem failed to boot\n");

	exmodem_ctrl_update(md, SMC_CP_CTRL_NS, CP_ACTIVE_REQ_CLR,
			    CP_ACTIVE_REQ_CLR);
	exmodem_set_state(md, EXMODEM_CRASH_RESET);

	return IRQ_HANDLED;
}

static irqreturn_t exmodem_wdt_irq(int irq, void *data)
{
	struct exmodem *md = data;

	exmodem_irq_set(md, md->irq_wdt, &md->irq_wdt_enabled, false);
	dev_err(md->dev, "modem watchdog reset\n");

	exmodem_ctrl_update(md, SMC_CP_CTRL_NS, CP_RESET_REQ_CLR,
			    CP_RESET_REQ_CLR);
	exmodem_set_state(md, EXMODEM_CRASH_WATCHDOG);

	return IRQ_HANDLED;
}

/* The modem changed its "active" word: it is going down by itself */
static void exmodem_active_work(struct work_struct *work)
{
	struct exmodem *md = container_of(work, struct exmodem, active_work);

	if (exmodem_issr_read(md, EXMODEM_ISSR_CP2AP_ACTIVE))
		return;

	if (!exmodem_powered(md))
		return;

	exmodem_set_state(md, EXMODEM_OFFLINE);
	complete_all(&md->off_done);
}

/*
 * Performance requests: the modem asks for a minimum frequency of a CPU
 * cluster, the memory interface or the internal bus by level number.
 */

static u32 exmodem_perf_freq(struct exmodem *md, enum exmodem_clk_table table,
			     u32 request)
{
	u32 level = FIELD_GET(PERF_LEVEL, request);

	if (!level || level > md->soc->clk_table_len[table])
		return 0;

	return md->soc->clk_tables[table][level - 1];
}

static void exmodem_perf_cpu_work(struct work_struct *work)
{
	struct exmodem *md = container_of(work, struct exmodem,
					  perf_work[EXMODEM_PERF_CPU]);
	u32 request = exmodem_issr_read(md, EXMODEM_ISSR_PERF_CPU);
	bool big = !(request & PERF_CLUSTER0) &&
		   md->soc->clk_table_len[EXMODEM_CLK_CL1];
	unsigned int cluster = big ? 1 : 0;
	u32 freq = exmodem_perf_freq(md, big ? EXMODEM_CLK_CL1 :
				     EXMODEM_CLK_CL0, request);
	struct cpufreq_policy *policy = md->cpu_policy[cluster];

	if (!policy) {
		policy = cpufreq_cpu_get(md->soc->cluster_cpu[cluster]);
		if (!policy)
			return;

		if (freq_qos_add_request(&policy->constraints,
					 &md->cpu_qos[cluster], FREQ_QOS_MIN,
					 FREQ_QOS_MIN_DEFAULT_VALUE) < 0) {
			cpufreq_cpu_put(policy);
			return;
		}
		md->cpu_policy[cluster] = policy;
	}

	dev_dbg(md->dev, "cluster %u minimum %u kHz\n", cluster, freq);
	freq_qos_update_request(&md->cpu_qos[cluster],
				freq ?: FREQ_QOS_MIN_DEFAULT_VALUE);
}

static void exmodem_perf_bus(struct exmodem *md, unsigned int bus,
			     enum exmodem_clk_table table, unsigned int reg)
{
	u32 freq = exmodem_perf_freq(md, table, exmodem_issr_read(md, reg));

	if (!md->bus[bus])
		return;

	dev_dbg(md->dev, "bus %u minimum %u kHz\n", bus, freq);
	dev_pm_qos_update_request(&md->bus_qos[bus],
				  freq ?: PM_QOS_MIN_FREQUENCY_DEFAULT_VALUE);
}

static void exmodem_perf_mif_work(struct work_struct *work)
{
	struct exmodem *md = container_of(work, struct exmodem,
					  perf_work[EXMODEM_PERF_MIF]);

	exmodem_perf_bus(md, BUS_MIF, EXMODEM_CLK_MIF, EXMODEM_ISSR_PERF_MIF);
}

static void exmodem_perf_int_work(struct work_struct *work)
{
	struct exmodem *md = container_of(work, struct exmodem,
					  perf_work[EXMODEM_PERF_INT]);

	exmodem_perf_bus(md, BUS_INT, EXMODEM_CLK_INT, EXMODEM_ISSR_PERF_INT);
}

/* The modem follows the memory interface frequency: it shares the DRAM */
static int exmodem_mif_notify(struct notifier_block *nb, unsigned long event,
			      void *data)
{
	struct exmodem *md = container_of(nb, struct exmodem, mif_nb);
	const u32 *table = md->soc->clk_tables[EXMODEM_CLK_MIF];
	unsigned int len = md->soc->clk_table_len[EXMODEM_CLK_MIF];
	struct devfreq_freqs *freqs = data;
	u32 khz, best;
	int i;

	if (event != DEVFREQ_POSTCHANGE)
		return NOTIFY_DONE;

	khz = freqs->new / 1000;
	best = khz;

	/* Report the level the modem knows from its table */
	for (i = 0; i < len; i++)
		if (!i || abs_diff(table[i], khz) < abs_diff(best, khz))
			best = table[i];

	exmodem_issr_write(md, EXMODEM_ISSR_MIF_FREQ, best);

	return NOTIFY_OK;
}

static int exmodem_perf_init(struct exmodem *md)
{
	static const char * const props[] = {
		[BUS_MIF] = "samsung,mif-devfreq",
		[BUS_INT] = "samsung,int-devfreq",
	};
	struct device *dev = md->dev;
	int ret;
	int i;

	INIT_WORK(&md->perf_work[EXMODEM_PERF_CPU], exmodem_perf_cpu_work);
	INIT_WORK(&md->perf_work[EXMODEM_PERF_MIF], exmodem_perf_mif_work);
	INIT_WORK(&md->perf_work[EXMODEM_PERF_INT], exmodem_perf_int_work);

	for (i = 0; i < ARRAY_SIZE(props); i++) {
		struct devfreq *devfreq;

		if (!device_property_present(dev, props[i]))
			continue;

		devfreq = devfreq_get_devfreq_by_phandle(dev, props[i], 0);
		if (IS_ERR(devfreq))
			return dev_err_probe(dev, PTR_ERR(devfreq),
					     "no devfreq device for %s\n",
					     props[i]);

		ret = dev_pm_qos_add_request(devfreq->dev.parent,
					     &md->bus_qos[i],
					     DEV_PM_QOS_MIN_FREQUENCY,
					     PM_QOS_MIN_FREQUENCY_DEFAULT_VALUE);
		if (ret < 0)
			return ret;

		md->bus[i] = devfreq;
	}

	if (md->bus[BUS_MIF]) {
		md->mif_nb.notifier_call = exmodem_mif_notify;
		ret = devm_devfreq_register_notifier(dev, md->bus[BUS_MIF],
						     &md->mif_nb,
						     DEVFREQ_TRANSITION_NOTIFIER);
		if (ret)
			return ret;
	}

	return 0;
}

static void exmodem_perf_exit(struct exmodem *md)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(md->perf_work); i++)
		cancel_work_sync(&md->perf_work[i]);

	for (i = 0; i < ARRAY_SIZE(md->bus); i++)
		if (md->bus[i])
			dev_pm_qos_remove_request(&md->bus_qos[i]);

	for (i = 0; i < ARRAY_SIZE(md->cpu_policy); i++) {
		if (!md->cpu_policy[i])
			continue;

		freq_qos_remove_request(&md->cpu_qos[i]);
		cpufreq_cpu_put(md->cpu_policy[i]);
	}
}

static void exmodem_mbox_rx(struct mbox_client *cl, void *data)
{
	struct exmodem_mbox_chan *mc = container_of(cl, struct exmodem_mbox_chan,
						    cl);
	struct exmodem *md = mc->md;

	switch (mc->id) {
	case EXMODEM_MBOX_MSG:
		exmodem_link_irq(md);
		break;
	case EXMODEM_MBOX_STATUS:
		exmodem_link_status_irq(md);
		break;
	case EXMODEM_MBOX_ACTIVE:
		schedule_work(&md->active_work);
		break;
	case EXMODEM_MBOX_PERF_CPU:
		schedule_work(&md->perf_work[EXMODEM_PERF_CPU]);
		break;
	case EXMODEM_MBOX_PERF_MIF:
		schedule_work(&md->perf_work[EXMODEM_PERF_MIF]);
		break;
	case EXMODEM_MBOX_PERF_INT:
		schedule_work(&md->perf_work[EXMODEM_PERF_INT]);
		break;
	default:
		/* The combined request of older firmware is not used */
		break;
	}
}

static void exmodem_mbox_free(struct exmodem *md)
{
	int i;

	for (i = 0; i < EXMODEM_MBOX_COUNT; i++)
		if (!IS_ERR_OR_NULL(md->mbox[i].chan))
			mbox_free_channel(md->mbox[i].chan);
}

static int exmodem_mbox_init(struct exmodem *md)
{
	struct device *dev = md->dev;
	int i;

	for (i = 0; i < EXMODEM_MBOX_COUNT; i++) {
		struct exmodem_mbox_chan *mc = &md->mbox[i];

		mc->md = md;
		mc->id = i;
		mc->cl.dev = dev;
		mc->cl.rx_callback = exmodem_mbox_rx;
		mc->cl.knows_txdone = true;

		mc->chan = mbox_request_channel_byname(&mc->cl,
						       exmodem_mbox_names[i]);
		if (IS_ERR(mc->chan)) {
			int ret = PTR_ERR(mc->chan);

			exmodem_mbox_free(md);
			return dev_err_probe(dev, ret, "no mailbox \"%s\"\n",
					     exmodem_mbox_names[i]);
		}
	}

	return 0;
}

/* The shared registers belong to the mailbox controller */
static int exmodem_issr_init(struct exmodem *md)
{
	struct device *dev = md->dev;
	struct platform_device *mbox_pdev;
	struct device_node *np;

	np = of_parse_phandle(dev->of_node, "mboxes", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "no mailbox\n");

	mbox_pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!mbox_pdev)
		return -EPROBE_DEFER;

	md->issr = dev_get_regmap(&mbox_pdev->dev, NULL);
	if (md->issr &&
	    !device_link_add(dev, &mbox_pdev->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		md->issr = NULL;
	put_device(&mbox_pdev->dev);
	if (!md->issr)
		return -EPROBE_DEFER;

	return 0;
}

static void *exmodem_map_region(struct exmodem *md, const char *name,
				phys_addr_t *phys, size_t *size)
{
	struct device *dev = md->dev;
	struct resource res;
	void *base;
	int ret;

	ret = of_reserved_mem_region_to_resource_byname(dev->of_node, name,
							&res);
	if (ret)
		return ERR_PTR(dev_err_probe(dev, ret,
					     "no memory region \"%s\"\n",
					     name));

	base = devm_memremap(dev, res.start, resource_size(&res), MEMREMAP_WC);
	if (IS_ERR(base))
		return ERR_PTR(dev_err_probe(dev, PTR_ERR(base),
					     "cannot map \"%s\"\n", name));

	if (phys)
		*phys = res.start;
	*size = resource_size(&res);

	return base;
}

static int exmodem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exmodem *md;
	int ret;

	md = devm_kzalloc(dev, sizeof(*md), GFP_KERNEL);
	if (!md)
		return -ENOMEM;

	md->dev = dev;
	md->soc = device_get_match_data(dev);
	mutex_init(&md->lock);
	spin_lock_init(&md->state_lock);
	init_completion(&md->init_done);
	init_completion(&md->off_done);
	INIT_WORK(&md->active_work, exmodem_active_work);
	platform_set_drvdata(pdev, md);

	device_property_read_u32(dev, "samsung,board-revision", &md->board_rev);
	md->dual_sim = device_property_read_bool(dev, "samsung,dual-sim");

	md->pmu = exynos_get_pmu_regmap_by_phandle(dev->of_node,
						   "samsung,syscon-phandle");
	if (IS_ERR(md->pmu))
		return dev_err_probe(dev, PTR_ERR(md->pmu), "no PMU\n");

	ret = exmodem_issr_init(md);
	if (ret)
		return ret;

	md->fw_base = exmodem_map_region(md, "firmware", NULL, &md->fw_size);
	if (IS_ERR(md->fw_base))
		return PTR_ERR(md->fw_base);

	md->ipc_base = exmodem_map_region(md, "ipc", &md->ipc_phys,
					  &md->ipc_size);
	if (IS_ERR(md->ipc_base))
		return PTR_ERR(md->ipc_base);

	md->bootinfo = exmodem_map_region(md, "bootinfo", NULL,
					  &md->bootinfo_size);
	if (IS_ERR(md->bootinfo))
		return PTR_ERR(md->bootinfo);
	if (md->bootinfo_size < BOOTINFO_SIZE)
		return dev_err_probe(dev, -EINVAL, "bootinfo too small\n");

	md->irq_fail = platform_get_irq_byname(pdev, "fail");
	if (md->irq_fail < 0)
		return md->irq_fail;

	md->irq_wdt = platform_get_irq_byname(pdev, "watchdog");
	if (md->irq_wdt < 0)
		return md->irq_wdt;

	/* Only meaningful during, respectively after, the modem's boot */
	ret = devm_request_threaded_irq(dev, md->irq_fail, NULL,
					exmodem_fail_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"modem-fail", md);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, md->irq_wdt, NULL,
					exmodem_wdt_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"modem-watchdog", md);
	if (ret)
		return ret;

	ret = exmodem_perf_init(md);
	if (ret)
		goto err_perf;

	ret = exmodem_link_init(md);
	if (ret)
		goto err_perf;

	ret = exmodem_mbox_init(md);
	if (ret)
		goto err_link;

	return 0;

err_link:
	exmodem_link_exit(md);
err_perf:
	exmodem_perf_exit(md);
	return ret;
}

/* Give the modem the time to shut down that its software asked for */
static void exmodem_shutdown(struct platform_device *pdev)
{
	struct exmodem *md = platform_get_drvdata(pdev);

	if (exmodem_get_state(md) != EXMODEM_OFFLINE && exmodem_powered(md) &&
	    !wait_for_completion_timeout(&md->off_done, OFF_TIMEOUT))
		dev_warn(md->dev, "the modem did not shut down\n");

	exmodem_irq_set(md, md->irq_wdt, &md->irq_wdt_enabled, false);
	exmodem_irq_set(md, md->irq_fail, &md->irq_fail_enabled, false);
	exmodem_power_off(md);
	exmodem_set_state(md, EXMODEM_OFFLINE);
}

static void exmodem_remove(struct platform_device *pdev)
{
	struct exmodem *md = platform_get_drvdata(pdev);

	exmodem_shutdown(pdev);
	exmodem_mbox_free(md);
	cancel_work_sync(&md->active_work);
	exmodem_link_exit(md);
	exmodem_perf_exit(md);
}

/* The modem keeps running while the application processor sleeps */
static int exmodem_suspend(struct device *dev)
{
	struct exmodem *md = dev_get_drvdata(dev);

	if (exmodem_get_state(md) == EXMODEM_OFFLINE)
		return 0;

	exmodem_issr_write(md, EXMODEM_ISSR_AP2CP_ACTIVE, 0);
	exmodem_mbox_ring(md, EXMODEM_MBOX_PERF);

	return 0;
}

static int exmodem_resume(struct device *dev)
{
	struct exmodem *md = dev_get_drvdata(dev);

	if (exmodem_get_state(md) == EXMODEM_OFFLINE)
		return 0;

	exmodem_issr_write(md, EXMODEM_ISSR_AP2CP_ACTIVE, 1);
	exmodem_mbox_ring(md, EXMODEM_MBOX_PERF);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(exmodem_pm_ops, exmodem_suspend,
				exmodem_resume);

/* The DVFS levels of the bootloader's tables, which the modem indexes */
static const u32 exynos7870_cpu_freqs[] = {
	343000, 449000, 546000, 676000, 757000, 839000, 902000, 1014000,
	1144000, 1248000, 1352000, 1482000, 1586000, 1690000, 1794000,
	1924000, 2002000,
};

static const u32 exynos7870_mif_freqs[] = {
	275000, 451000, 546000, 676000, 728000, 836000, 902000,
};

static const u32 exynos7870_int_freqs[] = {
	134000, 200000, 334000, 400000,
};

static const struct exmodem_soc_data exynos7870_modem_data = {
	.clk_tables = {
		[EXMODEM_CLK_CL0] = exynos7870_cpu_freqs,
		[EXMODEM_CLK_CL1] = exynos7870_cpu_freqs,
		[EXMODEM_CLK_MIF] = exynos7870_mif_freqs,
		[EXMODEM_CLK_INT] = exynos7870_int_freqs,
	},
	.clk_table_len = {
		[EXMODEM_CLK_CL0] = ARRAY_SIZE(exynos7870_cpu_freqs),
		[EXMODEM_CLK_CL1] = ARRAY_SIZE(exynos7870_cpu_freqs),
		[EXMODEM_CLK_MIF] = ARRAY_SIZE(exynos7870_mif_freqs),
		[EXMODEM_CLK_INT] = ARRAY_SIZE(exynos7870_int_freqs),
	},
	.cluster_cpu = { 0, 4 },
};

static const struct of_device_id exmodem_of_match[] = {
	{
		.compatible = "samsung,exynos7870-modem",
		.data = &exynos7870_modem_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, exmodem_of_match);

static struct platform_driver exmodem_driver = {
	.probe = exmodem_probe,
	.remove = exmodem_remove,
	.shutdown = exmodem_shutdown,
	.driver = {
		.name = "exynos-modem",
		.of_match_table = exmodem_of_match,
		.suppress_bind_attrs = true,
		.pm = pm_sleep_ptr(&exmodem_pm_ops),
	},
};
module_platform_driver(exmodem_driver);

MODULE_DESCRIPTION("Samsung Exynos integrated modem interface");
MODULE_LICENSE("GPL");
