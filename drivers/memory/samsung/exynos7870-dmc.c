// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos7870 DRAM controller (DREX) frequency scaling
 *
 * The DREX keeps two timing parameter sets. To change the memory frequency,
 * the PHY clock moves from MEM_PLL to a switching clock (MEDIA_PLL, 666 MHz
 * at the PHY), MEM_PLL is relocked and the PHY clock moves back. With the
 * pause enabled in CMU_MIF, each move of the PHY clock mux pauses the
 * controller, which then switches to the timing set CMU_MIF names. The
 * switching clock uses set 1, the target frequency set 0, as in the vendor
 * kernel.
 *
 * The bootloader trains the memory and programs set 0 for the top level.
 * The DRAM timings are the same at every level; only the tick granularity
 * and the PHY DVFS settings change, so the timings are copied from set 0.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

/* DREX */
#define DREX_MEMORY_CONTROL0		0x0010
#define  DREX_MEMCTRL0_DSREF_EN		BIT(4)
#define DREX_POWER_DOWN_CONFIG		0x0014
#define  DREX_PWRDN_DSREF_CYC		GENMASK(31, 16)
#define DREX_CG_CONTROL			0x0018
#define  DREX_CG_PHY_CG_EN		BIT(24)
#define DREX_TIMING_SET_SW		0x0020
#define  DREX_TIMING_SET_SW_CMU		0	/* the CMU picks the set */
#define DREX_PAUSE_MRS(n)		(0x0080 + 4 * (n))
#define DREX_NUM_PAUSE_MRS		5

/* Per timing set, from DREX_SET_BASE(set) */
#define DREX_SET_BASE(set)		(0x0100 + 0x80 * (set))
#define DREX_SET_TICK_GRANULARITY	0x00

static const u16 drex_set_timing_regs[] = {
	0x08,	/* TEMP_SENSING */
	0x10,	/* DQS_OSC_CON1 */
	0x14,	/* TERMINATION_CONTROL */
	0x18,	/* WINCONFIG_WRITE_ODT */
	0x40,	/* TIMING_ROW0 */
	0x44,	/* TIMING_ROW1 */
	0x48,	/* TIMING_DATA_ACLK */
	0x4c,	/* TIMING_DATA_MCLK */
	0x50,	/* TIMING_POWER0 */
	0x54,	/* TIMING_POWER1 */
	0x5c,	/* TIMING_ETC1 */
};

/* DREX performance filter */
#define DREX_PF_TICK_GRANULARITY(set)	(0x10 + 4 * (set))

/* PHY */
#define PHY_DVFS_CON0			0x00b8
#define  PHY_DVFS_CON0_MODE		GENMASK(25, 24)
#define  PHY_DVFS_MODE_NORMAL		0
#define  PHY_DVFS_MODE_SET0		1
#define  PHY_DVFS_MODE_SET1		2
#define PHY_DVFS_CON2			0x00bc
#define PHY_DVFS_CON3			0x00c0
#define PHY_DVFS_CON1			0x00e0

/* CMU_MIF pause control */
#define CMU_DMC_TIMING_SET		0x4
#define CMU_PAUSE			0x8
#define  CMU_PAUSE_EN			BIT(0)
#define  CMU_PAUSE_STATUS		GENMASK(17, 16)

#define DMC_NUM_SETS			2
#define DMC_SWITCH_SET			1
#define DMC_TARGET_SET			0

/* Mux inputs */
enum {
	DMC_BUSD_BUS_PLL_DIV2,
	DMC_BUSD_MEDIA_PLL_DIV2,
	DMC_BUSD_MEM_PLL_DIV2,
};

enum {
	DMC_CCI_BUS_PLL_DIV2,
	DMC_CCI_MEDIA_PLL_DIV2,
	DMC_CCI_BUS_PLL,
};

/* PHY DVFS register update: value per timing set and the bits it owns */
struct dmc_phy_field {
	u16 reg;
	u32 val[DMC_NUM_SETS];
	u32 mask[DMC_NUM_SETS];
};

/* Fields that do not depend on the level (CON0 is per level) */
static const struct dmc_phy_field dmc_phy_fields[] = {
	{ PHY_DVFS_CON1, { 0x00000000, 0x00000000 },
			 { 0x00000000, 0x00ffffff } },
	{ PHY_DVFS_CON2, { 0x000e0021, 0x0e002100 },
			 { 0x401f00ff, 0x9f00ff00 } },
	{ PHY_DVFS_CON3, { 0x90700000, 0x63800000 },
			 { 0x907100ff, 0x6382ff00 } },
};

static const u32 dmc_phy_con0_mask[DMC_NUM_SETS] = { 0x50ffffff, 0xa0000000 };

/* Mode register writes the controller issues during a pause */
static const u32 dmc_pause_mrs[DREX_NUM_PAUSE_MRS] = {
	0x84008301, 0x84001c02, 0x84000103, 0x8400000b, 0x00000000,
};

struct dmc_level {
	unsigned long rate;		/* memory clock, Hz */
	unsigned long mem_pll;		/* Hz */
	u8 busd_mux, busd_div;		/* divider field values */
	u8 cci_mux, cci_div;
	u8 tick;
	u32 phy_con0[DMC_NUM_SETS];
};

/* From the DVFS and timing tables of the vendor bootloader */
static const struct dmc_level dmc_levels[] = {
	{ 902200000, 1804400000, DMC_BUSD_MEM_PLL_DIV2, 1, DMC_CCI_BUS_PLL, 2,
	  9, { 0x48000000, 0x8a000000 } },
	{ 835250000, 1670500000, DMC_BUSD_MEM_PLL_DIV2, 1, DMC_CCI_BUS_PLL, 2,
	  8, { 0x49000000, 0x8a000000 } },
	{ 728000000, 1456000000, DMC_BUSD_BUS_PLL_DIV2, 1,
	  DMC_CCI_BUS_PLL_DIV2, 1, 7, { 0x01000000, 0x02000000 } },
	{ 676000000, 1352000000, DMC_BUSD_MEDIA_PLL_DIV2, 1,
	  DMC_CCI_MEDIA_PLL_DIV2, 1, 7, { 0x01000000, 0x02000000 } },
	{ 546000000, 1092000000, DMC_BUSD_MEDIA_PLL_DIV2, 1,
	  DMC_CCI_MEDIA_PLL_DIV2, 1, 6, { 0x49000000, 0x8a000000 } },
	{ 451100000, 902200000, DMC_BUSD_MEDIA_PLL_DIV2, 2,
	  DMC_CCI_MEDIA_PLL_DIV2, 2, 4, { 0x49000000, 0x8a000000 } },
	{ 274625000, 549250000, DMC_BUSD_MEDIA_PLL_DIV2, 4,
	  DMC_CCI_MEDIA_PLL_DIV2, 4, 3, { 0x01000000, 0x8a000000 } },
};

/*
 * While on the switching clock, the buses run as at 546 MHz and the
 * controller uses the 676 MHz timings, the lowest level above the 666 MHz
 * switching clock.
 */
#define DMC_SWITCH_BUS_LEVEL		4
#define DMC_SWITCH_TIMING_LEVEL		3

enum dmc_clk_id {
	DMC_CLK_MEM_PLL,
	DMC_CLK_MEM_PLL_DIV2,
	DMC_CLK_BUS_PLL,
	DMC_CLK_BUS_PLL_DIV2,
	DMC_CLK_MEDIA_PLL_DIV2,
	DMC_CLK_PHY_CLK2X,
	DMC_CLK_PHY_MEM,
	DMC_CLK_PHY_SWITCH,
	DMC_CLK_BUSD_MUX,
	DMC_CLK_BUSD,
	DMC_CLK_CCI_MUX,
	DMC_CLK_CCI,
	DMC_NUM_CLKS,
};

static const char * const dmc_clk_names[DMC_NUM_CLKS] = {
	"mem_pll", "mem_pll_div2", "bus_pll", "bus_pll_div2", "media_pll_div2",
	"phy_clk2x", "phy_mem", "phy_switch", "busd_mux", "busd", "cci_mux",
	"cci",
};

struct exynos7870_dmc {
	struct device *dev;
	void __iomem *drex;
	void __iomem *pf;
	void __iomem *phy;
	void __iomem *pause;
	struct clk *clks[DMC_NUM_CLKS];
	struct regulator *vdd;
	struct devfreq *devfreq;
	struct notifier_block opp_nb;
	struct mutex lock;		/* serialises level changes */
	u32 timing[ARRAY_SIZE(drex_set_timing_regs)];
	unsigned int cur;		/* index in dmc_levels[] */
	unsigned long cur_volt;
	unsigned long volt_max;
	bool suspended;
};

static int dmc_level_of(unsigned long rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dmc_levels); i++)
		if (dmc_levels[i].rate == rate)
			return i;

	return -EINVAL;
}

static struct clk *dmc_busd_parent(struct exynos7870_dmc *dmc, u8 mux)
{
	static const enum dmc_clk_id parents[] = {
		[DMC_BUSD_BUS_PLL_DIV2] = DMC_CLK_BUS_PLL_DIV2,
		[DMC_BUSD_MEDIA_PLL_DIV2] = DMC_CLK_MEDIA_PLL_DIV2,
		[DMC_BUSD_MEM_PLL_DIV2] = DMC_CLK_MEM_PLL_DIV2,
	};

	return dmc->clks[parents[mux]];
}

static struct clk *dmc_cci_parent(struct exynos7870_dmc *dmc, u8 mux)
{
	static const enum dmc_clk_id parents[] = {
		[DMC_CCI_BUS_PLL_DIV2] = DMC_CLK_BUS_PLL_DIV2,
		[DMC_CCI_MEDIA_PLL_DIV2] = DMC_CLK_MEDIA_PLL_DIV2,
		[DMC_CCI_BUS_PLL] = DMC_CLK_BUS_PLL,
	};

	return dmc->clks[parents[mux]];
}

/* Set a divider field to @div, whatever its current parent rate */
static int dmc_set_div(struct exynos7870_dmc *dmc, enum dmc_clk_id id, u8 div)
{
	struct clk *clk = dmc->clks[id];

	return clk_set_rate(clk, clk_get_rate(clk_get_parent(clk)) / (div + 1));
}

/*
 * The bus dividers: raise the ones that grow before the muxes move and lower
 * the others after, so no bus ever runs faster than at either level.
 */
static int dmc_set_divs(struct exynos7870_dmc *dmc, const struct dmc_level *from,
			const struct dmc_level *to, bool raise)
{
	int ret = 0;

	if (raise ? to->busd_div > from->busd_div : to->busd_div < from->busd_div)
		ret = dmc_set_div(dmc, DMC_CLK_BUSD, to->busd_div);
	if (!ret &&
	    (raise ? to->cci_div > from->cci_div : to->cci_div < from->cci_div))
		ret = dmc_set_div(dmc, DMC_CLK_CCI, to->cci_div);

	return ret;
}

static int dmc_set_muxes(struct exynos7870_dmc *dmc,
			 const struct dmc_level *from,
			 const struct dmc_level *to)
{
	int ret = 0;

	if (to->busd_mux != from->busd_mux)
		ret = clk_set_parent(dmc->clks[DMC_CLK_BUSD_MUX],
				     dmc_busd_parent(dmc, to->busd_mux));
	if (!ret && to->cci_mux != from->cci_mux)
		ret = clk_set_parent(dmc->clks[DMC_CLK_CCI_MUX],
				     dmc_cci_parent(dmc, to->cci_mux));

	return ret;
}

static void dmc_phy_update(struct exynos7870_dmc *dmc, u16 reg, u32 mask,
			   u32 val)
{
	u32 tmp = readl(dmc->phy + reg);

	writel((tmp & ~mask) | val, dmc->phy + reg);
}

/* Program timing set @set for @lvl and make CMU_MIF switch to it */
static void dmc_program_set(struct exynos7870_dmc *dmc, unsigned int set,
			    unsigned int lvl)
{
	const struct dmc_level *l = &dmc_levels[lvl];
	void __iomem *base = dmc->drex + DREX_SET_BASE(set);
	unsigned int mode;
	int i;

	if (set == DMC_SWITCH_SET)
		mode = PHY_DVFS_MODE_SET1;
	else
		mode = lvl ? PHY_DVFS_MODE_SET0 : PHY_DVFS_MODE_NORMAL;
	dmc_phy_update(dmc, PHY_DVFS_CON0, PHY_DVFS_CON0_MODE,
		       FIELD_PREP(PHY_DVFS_CON0_MODE, mode));

	writel(DREX_TIMING_SET_SW_CMU, dmc->drex + DREX_TIMING_SET_SW);
	writel(set, dmc->pause + CMU_DMC_TIMING_SET);

	writel(l->tick, base + DREX_SET_TICK_GRANULARITY);
	writel(l->tick, dmc->pf + DREX_PF_TICK_GRANULARITY(set));
	for (i = 0; i < ARRAY_SIZE(drex_set_timing_regs); i++)
		writel(dmc->timing[i], base + drex_set_timing_regs[i]);

	dmc_phy_update(dmc, PHY_DVFS_CON0, dmc_phy_con0_mask[set],
		       l->phy_con0[set]);
	for (i = 0; i < ARRAY_SIZE(dmc_phy_fields); i++)
		dmc_phy_update(dmc, dmc_phy_fields[i].reg,
			       dmc_phy_fields[i].mask[set],
			       dmc_phy_fields[i].val[set]);

	for (i = 0; i < DREX_NUM_PAUSE_MRS; i++)
		writel(dmc_pause_mrs[i], dmc->drex + DREX_PAUSE_MRS(i));
}

/* Move the PHY clock and wait for the controller to leave the pause */
static int dmc_switch_phy_clock(struct exynos7870_dmc *dmc,
				enum dmc_clk_id parent)
{
	u32 val;
	int ret;

	ret = clk_set_parent(dmc->clks[DMC_CLK_PHY_CLK2X], dmc->clks[parent]);
	if (ret)
		return ret;

	ret = readl_poll_timeout(dmc->pause + CMU_PAUSE, val,
				 !(val & CMU_PAUSE_STATUS), 1, 10000);
	if (ret)
		dev_err(dmc->dev, "pause did not end: %#x\n", val);

	return ret;
}

static void dmc_set_dsref_cycle(struct exynos7870_dmc *dmc, unsigned long rate)
{
	u32 cycle, val;

	if (rate > 800 * HZ_PER_MHZ)
		cycle = 0x3ff;
	else if (rate > 400 * HZ_PER_MHZ)
		cycle = 0x1ff;
	else
		cycle = 0x90;

	val = readl(dmc->drex + DREX_MEMORY_CONTROL0);
	writel(val & ~DREX_MEMCTRL0_DSREF_EN, dmc->drex + DREX_MEMORY_CONTROL0);

	val = readl(dmc->drex + DREX_POWER_DOWN_CONFIG);
	val &= ~DREX_PWRDN_DSREF_CYC;
	val |= FIELD_PREP(DREX_PWRDN_DSREF_CYC, cycle);
	writel(val, dmc->drex + DREX_POWER_DOWN_CONFIG);

	val = readl(dmc->drex + DREX_MEMORY_CONTROL0);
	writel(val | DREX_MEMCTRL0_DSREF_EN, dmc->drex + DREX_MEMORY_CONTROL0);
}

static int dmc_change_level(struct exynos7870_dmc *dmc, unsigned int to)
{
	const struct dmc_level *from = &dmc_levels[dmc->cur];
	const struct dmc_level *sw = &dmc_levels[DMC_SWITCH_BUS_LEVEL];
	const struct dmc_level *target = &dmc_levels[to];
	u32 val;
	int ret;

	/* PHY clock gating off, pause on */
	val = readl(dmc->drex + DREX_CG_CONTROL);
	writel(val & ~DREX_CG_PHY_CG_EN, dmc->drex + DREX_CG_CONTROL);
	writel(CMU_PAUSE_EN, dmc->pause + CMU_PAUSE);

	/* To the switching clock, on timing set 1 */
	ret = dmc_set_divs(dmc, from, sw, true);
	if (!ret)
		ret = dmc_set_muxes(dmc, from, sw);
	if (ret)
		goto err;
	dmc_program_set(dmc, DMC_SWITCH_SET, DMC_SWITCH_TIMING_LEVEL);
	ret = dmc_switch_phy_clock(dmc, DMC_CLK_PHY_SWITCH);
	if (ret)
		goto err;
	ret = dmc_set_divs(dmc, from, sw, false);
	if (ret)
		goto err;

	/* Relock MEM_PLL and back to it, on timing set 0 */
	ret = clk_set_rate(dmc->clks[DMC_CLK_MEM_PLL], target->mem_pll);
	if (!ret)
		ret = dmc_set_divs(dmc, sw, target, true);
	if (!ret)
		ret = dmc_set_muxes(dmc, sw, target);
	if (ret)
		goto err;
	dmc_program_set(dmc, DMC_TARGET_SET, to);
	ret = dmc_switch_phy_clock(dmc, DMC_CLK_PHY_MEM);
	if (ret)
		goto err;
	ret = dmc_set_divs(dmc, sw, target, false);
	if (ret)
		goto err;

	val = readl(dmc->drex + DREX_CG_CONTROL);
	writel(val | DREX_CG_PHY_CG_EN, dmc->drex + DREX_CG_CONTROL);
	dmc_set_dsref_cycle(dmc, target->rate);

	dmc->cur = to;
	return 0;

err:
	dev_err(dmc->dev, "failed to change from %lu to %lu Hz: %d\n",
		from->rate, target->rate, ret);
	return ret;
}

static int dmc_set_voltage(struct exynos7870_dmc *dmc, unsigned long volt)
{
	int ret;

	ret = regulator_set_voltage(dmc->vdd, volt, dmc->volt_max);
	if (!ret)
		dmc->cur_volt = volt;

	return ret;
}

static unsigned long dmc_opp_voltage(struct device *dev, unsigned long rate)
{
	struct dev_pm_opp *opp;
	unsigned long volt;

	opp = dev_pm_opp_find_freq_exact(dev, rate, true);
	if (IS_ERR(opp))
		return 0;

	volt = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);

	return volt;
}

static int exynos7870_dmc_target(struct device *dev, unsigned long *freq,
				 u32 flags)
{
	struct exynos7870_dmc *dmc = dev_get_drvdata(dev);
	unsigned long volt, sw_volt;
	struct dev_pm_opp *opp;
	int to, ret;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	volt = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);

	to = dmc_level_of(*freq);
	if (to < 0)
		return to;

	guard(mutex)(&dmc->lock);

	if (to == dmc->cur)
		return 0;

	/*
	 * Hold the suspend level until resume, whatever the CPUs do. Report
	 * it rather than failing, so the governor keeps tracking the CPUs.
	 */
	if (dmc->suspended) {
		*freq = dmc_levels[dmc->cur].rate;
		return 0;
	}

	/* The switching clock needs the voltage of the level above it */
	sw_volt = dmc_opp_voltage(dev, dmc_levels[DMC_SWITCH_TIMING_LEVEL].rate);
	ret = dmc_set_voltage(dmc, max3(dmc->cur_volt, volt, sw_volt));
	if (ret)
		return ret;

	ret = dmc_change_level(dmc, to);
	if (ret)
		return ret;

	return dmc_set_voltage(dmc, volt);
}

static int exynos7870_dmc_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct exynos7870_dmc *dmc = dev_get_drvdata(dev);

	guard(mutex)(&dmc->lock);
	*freq = dmc_levels[dmc->cur].rate;

	return 0;
}

/* Apply a new voltage of the current level, e.g. from the ASV tables */
static int exynos7870_dmc_opp_notify(struct notifier_block *nb,
				     unsigned long event, void *data)
{
	struct exynos7870_dmc *dmc = container_of(nb, struct exynos7870_dmc,
						  opp_nb);
	struct dev_pm_opp *opp = data;
	unsigned long volt;

	if (event != OPP_EVENT_ADJUST_VOLTAGE)
		return NOTIFY_DONE;

	guard(mutex)(&dmc->lock);

	if (dev_pm_opp_get_freq(opp) != dmc_levels[dmc->cur].rate)
		return NOTIFY_DONE;

	volt = dev_pm_opp_get_voltage(opp);
	if (dmc_set_voltage(dmc, volt))
		dev_warn(dmc->dev, "failed to apply %lu uV\n", volt);

	return NOTIFY_OK;
}

static void exynos7870_dmc_unregister_opp_nb(void *data)
{
	struct exynos7870_dmc *dmc = data;

	dev_pm_opp_unregister_notifier(dmc->dev, &dmc->opp_nb);
}

/* Only start from what the bootloader leaves: the top level on set 0 */
static int exynos7870_dmc_check_boot_state(struct exynos7870_dmc *dmc)
{
	struct clk *phy_parent = clk_get_parent(dmc->clks[DMC_CLK_PHY_CLK2X]);
	unsigned long pll = clk_get_rate(dmc->clks[DMC_CLK_MEM_PLL]);
	u32 set = readl(dmc->pause + CMU_DMC_TIMING_SET);
	u32 sw = readl(dmc->drex + DREX_TIMING_SET_SW);

	if (!clk_is_match(phy_parent, dmc->clks[DMC_CLK_PHY_MEM]) ||
	    pll != dmc_levels[0].mem_pll || set != DMC_TARGET_SET ||
	    sw != DREX_TIMING_SET_SW_CMU) {
		dev_err(dmc->dev,
			"unexpected boot state: MEM_PLL %lu Hz, set %u, set_sw %#x\n",
			pll, set, sw);
		return -ENODEV;
	}

	dmc->cur = 0;
	return 0;
}

static int exynos7870_dmc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct devfreq_passive_data *passive;
	struct devfreq_dev_profile *profile;
	struct dev_pm_opp_supply supply;
	struct exynos7870_dmc *dmc;
	struct dev_pm_opp *opp;
	unsigned long freq;
	int i, ret;

	dmc = devm_kzalloc(dev, sizeof(*dmc), GFP_KERNEL);
	if (!dmc)
		return -ENOMEM;

	dmc->dev = dev;
	mutex_init(&dmc->lock);
	platform_set_drvdata(pdev, dmc);

	dmc->drex = devm_platform_ioremap_resource_byname(pdev, "drex");
	if (IS_ERR(dmc->drex))
		return PTR_ERR(dmc->drex);
	dmc->pf = devm_platform_ioremap_resource_byname(pdev, "pf");
	if (IS_ERR(dmc->pf))
		return PTR_ERR(dmc->pf);
	dmc->phy = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(dmc->phy))
		return PTR_ERR(dmc->phy);
	dmc->pause = devm_platform_ioremap_resource_byname(pdev, "pause");
	if (IS_ERR(dmc->pause))
		return PTR_ERR(dmc->pause);

	for (i = 0; i < DMC_NUM_CLKS; i++) {
		dmc->clks[i] = devm_clk_get(dev, dmc_clk_names[i]);
		if (IS_ERR(dmc->clks[i]))
			return dev_err_probe(dev, PTR_ERR(dmc->clks[i]),
					     "failed to get clock %s\n",
					     dmc_clk_names[i]);
	}

	dmc->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(dmc->vdd))
		return dev_err_probe(dev, PTR_ERR(dmc->vdd),
				     "failed to get vdd supply\n");

	ret = exynos7870_dmc_check_boot_state(dmc);
	if (ret)
		return ret;

	/* The bootloader's timings, the same for every level */
	for (i = 0; i < ARRAY_SIZE(drex_set_timing_regs); i++)
		dmc->timing[i] = readl(dmc->drex + DREX_SET_BASE(DMC_TARGET_SET) +
				       drex_set_timing_regs[i]);

	ret = devm_pm_opp_of_add_table(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add the OPP table\n");

	for (i = 0; i < ARRAY_SIZE(dmc_levels); i++) {
		freq = dmc_levels[i].rate;
		opp = dev_pm_opp_find_freq_exact(dev, freq, true);
		if (IS_ERR(opp))
			return dev_err_probe(dev, PTR_ERR(opp),
					     "no OPP for %lu Hz\n", freq);
		dev_pm_opp_put(opp);
	}

	/* Start voting for the voltage of the boot level */
	opp = dev_pm_opp_find_freq_exact(dev, dmc_levels[0].rate, true);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	ret = dev_pm_opp_get_supplies(opp, &supply);
	dev_pm_opp_put(opp);
	if (ret)
		return ret;

	dmc->volt_max = supply.u_volt_max;
	ret = dmc_set_voltage(dmc, supply.u_volt);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set the voltage\n");

	dmc->opp_nb.notifier_call = exynos7870_dmc_opp_notify;
	ret = dev_pm_opp_register_notifier(dev, &dmc->opp_nb);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, exynos7870_dmc_unregister_opp_nb,
				       dmc);
	if (ret)
		return ret;

	profile = devm_kzalloc(dev, sizeof(*profile), GFP_KERNEL);
	passive = devm_kzalloc(dev, sizeof(*passive), GFP_KERNEL);
	if (!profile || !passive)
		return -ENOMEM;

	profile->initial_freq = dmc_levels[0].rate;
	profile->target = exynos7870_dmc_target;
	profile->get_cur_freq = exynos7870_dmc_get_cur_freq;

	/* The memory follows the CPUs, through their OPPs' required-opps */
	passive->parent_type = CPUFREQ_PARENT_DEV;

	dmc->devfreq = devm_devfreq_add_device(dev, profile, DEVFREQ_GOV_PASSIVE,
					       passive);
	if (IS_ERR(dmc->devfreq))
		return dev_err_probe(dev, PTR_ERR(dmc->devfreq),
				     "failed to add devfreq device\n");

	return 0;
}

/*
 * The devfreq core has already moved to the opp-suspend level, the one the
 * bootloader trains the memory for, before the devices suspend. The passive
 * governor still follows CPU frequency changes, so hold that level.
 */
static int exynos7870_dmc_suspend(struct device *dev)
{
	struct exynos7870_dmc *dmc = dev_get_drvdata(dev);

	guard(mutex)(&dmc->lock);
	dmc->suspended = true;

	return 0;
}

static int exynos7870_dmc_resume(struct device *dev)
{
	struct exynos7870_dmc *dmc = dev_get_drvdata(dev);

	guard(mutex)(&dmc->lock);
	dmc->suspended = false;

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(exynos7870_dmc_pm, exynos7870_dmc_suspend,
				exynos7870_dmc_resume);

static const struct of_device_id exynos7870_dmc_of_match[] = {
	{ .compatible = "samsung,exynos7870-dmc" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos7870_dmc_of_match);

static struct platform_driver exynos7870_dmc_driver = {
	.probe = exynos7870_dmc_probe,
	.driver = {
		.name = "exynos7870-dmc",
		.of_match_table = exynos7870_dmc_of_match,
		.pm = pm_sleep_ptr(&exynos7870_dmc_pm),
		/* The memory frequency must not change behind its back */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(exynos7870_dmc_driver);

MODULE_DESCRIPTION("Exynos7870 DRAM controller frequency scaling");
MODULE_LICENSE("GPL");
