/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Samsung Exynos integrated modem ("CP") interface
 */

#ifndef __EXYNOS_MODEM_H__
#define __EXYNOS_MODEM_H__

#include <linux/completion.h>
#include <linux/hrtimer.h>
#include <linux/kfifo.h>
#include <linux/mailbox_client.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/pm_qos.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "sipc5.h"

struct cpufreq_policy;
struct net_device;
struct devfreq;
struct device;
struct regmap;

/*
 * Modem states. The values are what the vendor interface reports to
 * userspace.
 */
enum exmodem_state {
	EXMODEM_OFFLINE,
	EXMODEM_CRASH_RESET,
	EXMODEM_CRASH_EXIT,
	EXMODEM_BOOTING,
	EXMODEM_ONLINE,
	EXMODEM_NV_REBUILDING,
	EXMODEM_LOADER_DONE,
	EXMODEM_SIM_ATTACH,
	EXMODEM_SIM_DETACH,
	EXMODEM_CRASH_WATCHDOG,
};

enum exmodem_link_state {
	EXMODEM_LINK_OFFLINE,
	EXMODEM_LINK_IPC,
	EXMODEM_LINK_CRASH,
};

/* Secure monitor request modes for the modem image */
enum exmodem_sec_mode {
	EXMODEM_SEC_VERIFY,	/* lock and verify the uploaded image */
	EXMODEM_SEC_DUMP,	/* prepare a memory dump */
	EXMODEM_SEC_UNLOCK,	/* let the AP write the firmware region */
};

/* Mailbox channels: one interrupt in each direction per channel */
enum exmodem_mbox {
	EXMODEM_MBOX_MSG,
	EXMODEM_MBOX_STATUS,
	EXMODEM_MBOX_PERF,		/* CP: performance request; AP: active */
	EXMODEM_MBOX_ACTIVE,
	EXMODEM_MBOX_PERF_CPU,
	EXMODEM_MBOX_PERF_MIF,
	EXMODEM_MBOX_PERF_INT,
	EXMODEM_MBOX_COUNT
};

enum exmodem_clk_table {
	EXMODEM_CLK_CL0,
	EXMODEM_CLK_CL1,
	EXMODEM_CLK_MIF,
	EXMODEM_CLK_INT,
	EXMODEM_CLK_COUNT
};

enum exmodem_perf {
	EXMODEM_PERF_CPU,
	EXMODEM_PERF_MIF,
	EXMODEM_PERF_INT,
	EXMODEM_PERF_COUNT
};

#define EXMODEM_CRASH_REASON_SIZE	512

/* Who asked for a modem crash dump */
#define EXMODEM_CRASH_BY_CP	0
#define EXMODEM_CRASH_BY_AP	1
#define EXMODEM_CRASH_BY_RIL	2

/* Silent reset information kept in the IPC region across modem resets */
#define EXMODEM_SRINFO_OFFSET	0x800
#define EXMODEM_SRINFO_SIZE	0x800

struct exmodem;

/**
 * struct exmodem_port - user of one SIPC channel
 * @ch: channel number
 * @rx: called from the link's receive work with each payload; takes
 *	ownership of @skb. The fragments of a multi-frame message arrive in
 *	order once the last one has been received.
 * @node: entry in the modem's port list
 * @frags: fragments of unfinished multi-frame messages
 */
struct exmodem_port {
	u8 ch;
	void (*rx)(struct exmodem_port *port, struct sk_buff *skb);
	struct list_head node;
	struct sk_buff_head frags;
};

/**
 * struct exmodem_frontend - the userspace interface on top of the link
 * @state_changed: the modem state changed, may be called in atomic context
 * @tx_flow: stop or restart the transmit queues of the packet data channels,
 *	     called in atomic context
 */
struct exmodem_frontend {
	void (*state_changed)(struct exmodem *md, enum exmodem_state state);
	void (*tx_flow)(struct exmodem *md, bool stop);
};

/* One pair of ring buffers of the shared memory map */
struct exmodem_ipc_dev {
	const char *name;
	struct sipc_ring txq;
	struct sipc_ring rxq;
	u16 send_mask;
	u16 req_ack_mask;
	u16 res_ack_mask;
	bool flow_ctrl;		/* stops the packet data queues when full */

	spinlock_t tx_lock;	/* txq, skb_txq order, stopped, busy_count */
	struct sk_buff_head skb_txq;
	bool stopped;
	unsigned int busy_count;
};

enum {
	EXMODEM_IPC_FMT,
	EXMODEM_IPC_RAW,
	EXMODEM_IPC_COUNT
};

struct exmodem_crash_reason {
	u32 owner;
	char string[EXMODEM_CRASH_REASON_SIZE];
};

/**
 * struct exmodem_soc_data - SoC specific parts of the interface
 * @clk_tables: frequency levels in kHz, lowest first, that the modem selects
 *		its performance requests from
 * @clk_table_len: number of levels in each table
 * @cluster_cpu: one CPU of each cluster
 */
struct exmodem_soc_data {
	const u32 *clk_tables[EXMODEM_CLK_COUNT];
	unsigned int clk_table_len[EXMODEM_CLK_COUNT];
	unsigned int cluster_cpu[2];
};

struct exmodem_mbox_chan {
	struct mbox_client cl;
	struct mbox_chan *chan;
	struct exmodem *md;
	unsigned int id;
};

struct exmodem {
	struct device *dev;
	const struct exmodem_soc_data *soc;
	struct regmap *pmu;
	struct regmap *issr;		/* shared mailbox registers */
	struct exmodem_mbox_chan mbox[EXMODEM_MBOX_COUNT];
	int irq_fail;
	int irq_wdt;
	bool irq_fail_enabled;
	bool irq_wdt_enabled;
	u32 board_rev;
	bool dual_sim;

	/* memory */
	void *fw_base;			/* firmware region, modem private */
	size_t fw_size;
	void *ipc_base;			/* shared IPC region */
	phys_addr_t ipc_phys;
	size_t ipc_size;
	void *bootinfo;			/* left by the bootloader for the modem */
	size_t bootinfo_size;

	/* control */
	struct mutex lock;		/* boot and power sequences */
	spinlock_t state_lock;		/* state */
	enum exmodem_state state;
	enum exmodem_link_state link_state;
	const struct exmodem_frontend *frontend;
	void *frontend_data;
	struct completion init_done;	/* modem sent PHONE_START */
	struct completion off_done;	/* modem reported inactive */
	struct work_struct active_work;

	/* link */
	struct mutex link_lock;		/* link_state, ipc_ready, init_end_sent */
	spinlock_t msg_lock;		/* message word and its interrupt */
	__le32 *magic;
	__le32 *access;
	struct exmodem_ipc_dev ipc[EXMODEM_IPC_COUNT];
	DECLARE_KFIFO(rx_fifo, u16, 64);
	struct workqueue_struct *rx_wq;
	struct work_struct rx_work;
	struct hrtimer tx_timer;
	struct mutex port_lock;		/* ports */
	struct list_head ports;
	bool padded;			/* frames carry padding */
	bool ipc_ready;			/* userspace listens on FMT and RFS */
	bool init_end_sent;
	bool tx_suspended;		/* by the modem's status word */
	bool tx_flow_stopped;
	u16 last_status;
	atomic_t forced_crash;
	struct timer_list crash_ack_timer;
	struct exmodem_crash_reason crash_reason;

	/* performance requests */
	struct work_struct perf_work[EXMODEM_PERF_COUNT];
	struct freq_qos_request cpu_qos[2];
	struct cpufreq_policy *cpu_policy[2];
	struct dev_pm_qos_request bus_qos[2];	/* MIF, INT */
	struct devfreq *bus[2];
	struct notifier_block mif_nb;
};

/* ctrl.c */
enum exmodem_state exmodem_get_state(struct exmodem *md);
void exmodem_set_state(struct exmodem *md, enum exmodem_state state);
int exmodem_issr_read(struct exmodem *md, unsigned int reg);
void exmodem_issr_write(struct exmodem *md, unsigned int reg, u32 val);
void exmodem_mbox_ring(struct exmodem *md, enum exmodem_mbox id);
int exmodem_reset(struct exmodem *md);
int exmodem_security(struct exmodem *md, enum exmodem_sec_mode mode,
		     u32 size_boot, u32 size_main);
int exmodem_fw_write(struct exmodem *md, bool ipc_region, u32 offset,
		     const void *data, u32 len);
void *exmodem_fw_buf(struct exmodem *md, u32 offset, u32 len);
int exmodem_fw_check(struct exmodem *md);
int exmodem_power_on(struct exmodem *md);
int exmodem_power_off(struct exmodem *md);
int exmodem_boot_on(struct exmodem *md);
int exmodem_dl_start(struct exmodem *md);
int exmodem_boot_off(struct exmodem *md);
int exmodem_dump_start(struct exmodem *md);
void exmodem_stop(struct exmodem *md);
void exmodem_set_frontend(struct exmodem *md,
			  const struct exmodem_frontend *frontend);

/* boot.c */
int exmodem_boot(struct exmodem *md);

/* net.c */

/**
 * struct exmodem_net - a packet data channel as a raw IP interface
 * @md: the modem
 * @port: the channel
 * @ndev: the interface; its ml_priv points back here
 */
struct exmodem_net {
	struct exmodem *md;
	struct exmodem_port port;
	struct net_device *ndev;
};

void exmodem_net_setup(struct net_device *ndev);
void exmodem_net_init(struct exmodem_net *net, struct exmodem *md,
		      struct net_device *ndev, u8 ch);
void exmodem_net_tx_flow(struct net_device *ndev, bool stop);

/* wwan.c */
int exmodem_wwan_init(struct exmodem *md);
void exmodem_wwan_exit(struct exmodem *md);

/* link.c */
int exmodem_link_init(struct exmodem *md);
void exmodem_link_exit(struct exmodem *md);
void exmodem_link_irq(struct exmodem *md);
void exmodem_link_status_irq(struct exmodem *md);
void exmodem_link_prepare(struct exmodem *md);
void exmodem_link_boot_on(struct exmodem *md);
int exmodem_link_start(struct exmodem *md, u32 magic);
void exmodem_link_set_ready(struct exmodem *md, bool ready);
int exmodem_link_send(struct exmodem *md, u8 ch, struct sk_buff *skb);
void exmodem_link_force_crash(struct exmodem *md, u32 owner,
			      const char *reason);
int exmodem_port_register(struct exmodem *md, struct exmodem_port *port);
void exmodem_port_unregister(struct exmodem *md, struct exmodem_port *port);

/* Headroom and tailroom a payload skb needs for exmodem_link_send() */
#define EXMODEM_TX_HEADROOM	SIPC5_HDR_MAX
#define EXMODEM_TX_TAILROOM	(SIPC5_PAD_ALIGN - 1)

/* Shared mailbox registers */
#define EXMODEM_ISSR_AP2CP_MSG		0
#define EXMODEM_ISSR_CP2AP_MSG		1
#define EXMODEM_ISSR_AP2CP_STATUS	4
#define EXMODEM_ISSR_CP2AP_STATUS	5
#define EXMODEM_ISSR_AP2CP_ACTIVE	6
#define EXMODEM_ISSR_CP2AP_ACTIVE	9
#define EXMODEM_ISSR_PERF_CPU		10
#define EXMODEM_ISSR_PERF_MIF		11
#define EXMODEM_ISSR_PERF_INT		12
#define EXMODEM_ISSR_MIF_FREQ		13
#define EXMODEM_ISSR_INFO		0x21
#define EXMODEM_ISSR_COUNT		64

/* Magic values at the start of the IPC region */
#define EXMODEM_MAGIC_IPC	0x000000aa
#define EXMODEM_MAGIC_BOOT	0x424f4f54
#define EXMODEM_MAGIC_DUMP	0x44554d50
#define EXMODEM_MAGIC_CRASH	0xdeaddead

#ifdef CONFIG_EXYNOS_MODEM_LEGACY_ABI
bool exmodem_legacy_enabled(void);
int exmodem_legacy_init(struct exmodem *md);
void exmodem_legacy_exit(struct exmodem *md);
#else
static inline bool exmodem_legacy_enabled(void) { return false; }
static inline int exmodem_legacy_init(struct exmodem *md) { return 0; }
static inline void exmodem_legacy_exit(struct exmodem *md) { }
#endif

#endif /* __EXYNOS_MODEM_H__ */
