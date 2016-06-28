/*
 * MEN 16Z192 CAN Controller driver
 *
 * Copyright (C) 2016 MEN Mikroelektronik GmbH (www.men.de)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2 of the License.
 */

#include <linux/netdevice.h>
#include <linux/can/error.h>
#include <linux/can/dev.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/mcb.h>
#include <linux/can.h>
#include <linux/io.h>

#define DRV_NAME	"z192_can"

#define MEN_Z192_NAPI_WEIGHT	64
#define MEN_Z192_MODE_TOUT_US	40

/* CTL/BTR Register Bits */
#define MEN_Z192_CTL0_INITRQ	BIT(0)
#define MEN_Z192_CTL0_SLPRQ	BIT(1)
#define MEN_Z192_CTL1_INITAK	BIT(8)
#define MEN_Z192_CTL1_SLPAK	BIT(9)
#define MEN_Z192_CTL1_LISTEN	BIT(12)
#define MEN_Z192_CTL1_LOOPB	BIT(13)
#define MEN_Z192_CTL1_CANE	BIT(15)
#define MEN_Z192_BTR0_BRP(x)	(((x) & 0x3f) << 16)
#define MEN_Z192_BTR0_SJW(x)	(((x) & 0x03) << 22)
#define MEN_Z192_BTR1_TSEG1(x)	(((x) & 0x0f) << 24)
#define MEN_Z192_BTR1_TSEG2(x)	(((x) & 0x07) << 28)
#define MEN_Z192_BTR1_SAMP	BIT(31)

/* IER Interrupt Enable Register bits */
#define MEN_Z192_RXIE		BIT(0)
#define MEN_Z192_OVRIE		BIT(1)
#define MEN_Z192_CSCIE		BIT(6)
#define MEN_Z192_TOUTE		BIT(7)
#define MEN_Z192_TXIE		BIT(16)
#define MEN_Z192_ERRIE		BIT(17)

#define MEN_Z192_IRQ_ALL				\
		(MEN_Z192_RXIE | MEN_Z192_OVRIE |	\
		 MEN_Z192_CSCIE | MEN_Z192_TOUTE |	\
		 MEN_Z192_TXIE)

#define MEN_Z192_IRQ_NAPI	(MEN_Z192_RXIE | MEN_Z192_TOUTE)

/* RX_TX_STAT RX/TX Status status register bits */
#define MEN_Z192_RX_BUF_CNT(x)	((x) & 0xff)
#define MEN_Z192_TX_BUF_CNT(x)	(((x) & 0xff00) >> 8)
#define	MEN_Z192_RFLG_RXIF	BIT(16)
#define	MEN_Z192_RFLG_OVRF	BIT(17)
#define	MEN_Z192_RFLG_TSTATE	GENMASK(19, 18)
#define	MEN_Z192_RFLG_RSTATE	GENMASK(21, 20)
#define	MEN_Z192_RFLG_CSCIF	BIT(22)
#define	MEN_Z192_RFLG_TOUTF	BIT(23)
#define MEN_Z192_TFLG_TXIF	BIT(24)

#define MEN_Z192_GET_TSTATE(x)	(((x) & MEN_Z192_RFLG_TSTATE) >> 18)
#define MEN_Z192_GET_RSTATE(x)	(((x) & MEN_Z192_RFLG_RSTATE) >> 20)

#define MEN_Z192_IRQ_FLAGS_ALL					\
		(MEN_Z192_RFLG_RXIF | MEN_Z192_RFLG_OVRF |	\
		 MEN_Z192_RFLG_TSTATE | MEN_Z192_RFLG_RSTATE |	\
		 MEN_Z192_RFLG_CSCIF | MEN_Z192_RFLG_TOUTF |	\
		 MEN_Z192_TFLG_TXIF)

/* RX/TX Error counter bits */
#define MEN_Z192_GET_RX_ERR_CNT(x)	((x) & 0xff)
#define MEN_Z192_GET_TX_ERR_CNT(x)	(((x) & 0x00ff0000) >> 16)

/* Buffer level register bits */
#define MEN_Z192_RX_BUF_LVL	GENMASK(15, 0)
#define MEN_Z192_TX_BUF_LVL	GENMASK(31, 16)

/* RX/TX Buffer register bits */
#define MEN_Z192_CFBUF_LEN	GENMASK(3, 0)
#define MEN_Z192_CFBUF_ID1	GENMASK(31, 21)
#define MEN_Z192_CFBUF_ID2	GENMASK(18, 1)
#define MEN_Z192_CFBUF_TS	GENMASK(31, 8)
#define MEN_Z192_CFBUF_E_RTR	BIT(0)
#define MEN_Z192_CFBUF_IDE	BIT(19)
#define MEN_Z192_CFBUF_SRR	BIT(20)
#define MEN_Z192_CFBUF_S_RTR	BIT(20)
#define MEN_Z192_CFBUF_ID2_SHIFT	1
#define MEN_Z192_CFBUF_ID1_SHIFT	21

/* Global register offsets */
#define MEN_Z192_RX_BUF_START	0x0000
#define MEN_Z192_TX_BUF_START	0x1000
#define MEN_Z192_REGS_OFFS	0x2000

/* Buffer level control values */
#define MEN_Z192_MIN_BUF_LVL	0
#define MEN_Z192_MAX_BUF_LVL	254
#define MEN_Z192_RX_BUF_LVL_DEF	5
#define MEN_Z192_TX_BUF_LVL_DEF	5
#define MEN_Z192_RX_TOUT_MIN	0
#define MEN_Z192_RX_TOUT_MAX	65535
#define MEN_Z192_RX_TOUT_DEF	1000

static int txlvl = MEN_Z192_TX_BUF_LVL_DEF;
module_param(txlvl, int, S_IRUGO);
MODULE_PARM_DESC(txlvl, "TX IRQ trigger level (in frames) 0-254, default="
		 __MODULE_STRING(MEN_Z192_TX_BUF_LVL_DEF) ")");

static int rxlvl = MEN_Z192_RX_BUF_LVL_DEF;
module_param(rxlvl, int, S_IRUGO);
MODULE_PARM_DESC(rxlvl, "RX IRQ trigger level (in frames) 0-254, default="
		 __MODULE_STRING(MEN_Z192_RX_BUF_LVL_DEF) ")");

static int rx_timeout = MEN_Z192_RX_TOUT_DEF;
module_param(rx_timeout, int, S_IRUGO);
MODULE_PARM_DESC(rx_timeout, "RX IRQ timeout (in 100usec steps), default="
		 __MODULE_STRING(MEN_Z192_RX_TOUT_DEF) ")");

struct men_z192_regs {
	u32 ctl_btr;		/* Control and bus timing register */
	u32 ier;                /* Interrupt enable register */
	u32 buf_lvl;            /* Buffer level register */
	u32 rxa;                /* RX Data acknowledge register */
	u32 txa;                /* TX data acknowledge register */
	u32 rx_tx_sts;          /* RX/TX flags and buffer level */
	u32 ovr_ecc_sts;        /* Overrun/ECC status register */
	u32 idac_ver;           /* ID acceptance control / version */
	u32 rx_tx_err;          /* RX/TX error counter register */
	u32 idar_0_to_3;        /* ID acceptance register 0...3 */
	u32 idar_4_to_7;        /* ID acceptance register 4...7 */
	u32 idmr_0_to_3;        /* ID mask register 0...3 */
	u32 idmr_4_to_7;        /* ID mask register 4...7 */
	u32 rx_timeout;		/* receive timeout */
	u32 timebase;		/* Base frequency for baudrate calculation */
};

struct men_z192 {
	struct can_priv can;
	struct napi_struct napi;
	struct net_device *ndev;
	struct device *dev;

	/* Lock for CTL_BTR register access.
	 * This register combines bittiming bits
	 * and the operation mode bits.
	 * It is also used for bit r/m/w access
	 * to all registers.
	 */
	spinlock_t lock;
	struct resource *mem;
	struct men_z192_regs __iomem *regs;
	void __iomem *dev_base;
};

struct men_z192_cf_buf {
	u32 can_id;
	u32 data[2];
	u32 length;
};

enum men_z192_int_state {
	MEN_Z192_CAN_DIS = 0,
	MEN_Z192_CAN_EN,
	MEN_Z192_CAN_NAPI_DIS,
	MEN_Z192_CAN_NAPI_EN,
};

static enum can_state bus_state_map[] = {
	CAN_STATE_ERROR_ACTIVE,
	CAN_STATE_ERROR_WARNING,
	CAN_STATE_ERROR_PASSIVE,
	CAN_STATE_BUS_OFF
};

static const struct can_bittiming_const men_z192_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 4,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 2,
	.brp_max = 64,
	.brp_inc = 1,
};

static inline void men_z192_bit_clr(struct men_z192 *priv, void __iomem *addr,
				    u32 mask)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&priv->lock, flags);

	val = readl(addr);
	val &= ~mask;
	writel(val, addr);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static inline void men_z192_bit_set(struct men_z192 *priv, void __iomem *addr,
				    u32 mask)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&priv->lock, flags);

	val = readl(addr);
	val |= mask;
	writel(val, addr);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static inline void men_z192_ack_rx_pkg(struct men_z192 *priv,
				       unsigned int count)
{
	writel(count, &priv->regs->rxa);
}

static inline void men_z192_ack_tx_pkg(struct men_z192 *priv,
				       unsigned int count)
{
	writel(count, &priv->regs->txa);
}

static void men_z192_set_int(struct men_z192 *priv,
			     enum men_z192_int_state state)
{
	struct men_z192_regs __iomem *regs = priv->regs;

	switch (state) {
	case MEN_Z192_CAN_DIS:
		men_z192_bit_clr(priv, &regs->ier, MEN_Z192_IRQ_ALL);
		break;

	case MEN_Z192_CAN_EN:
		men_z192_bit_set(priv, &regs->ier, MEN_Z192_IRQ_ALL);
		break;

	case MEN_Z192_CAN_NAPI_DIS:
		men_z192_bit_clr(priv, &regs->ier, MEN_Z192_IRQ_NAPI);
		break;

	case MEN_Z192_CAN_NAPI_EN:
		men_z192_bit_set(priv, &regs->ier, MEN_Z192_IRQ_NAPI);
		break;

	default:
		netdev_err(priv->ndev, "invalid interrupt state\n");
		break;
	}
}

static int men_z192_get_berr_counter(const struct net_device *ndev,
				     struct can_berr_counter *bec)
{
	struct men_z192 *priv = netdev_priv(ndev);
	struct men_z192_regs __iomem *regs = priv->regs;
	u32 err_cnt;

	err_cnt = readl(&regs->rx_tx_err);

	bec->txerr = MEN_Z192_GET_TX_ERR_CNT(err_cnt);
	bec->rxerr = MEN_Z192_GET_RX_ERR_CNT(err_cnt);

	return 0;
}

static int men_z192_req_run_mode(struct men_z192 *priv)
{
	unsigned int timeout = MEN_Z192_MODE_TOUT_US / 10;
	struct men_z192_regs __iomem *regs = priv->regs;
	u32 val;

	men_z192_bit_clr(priv, &regs->ctl_btr, MEN_Z192_CTL0_INITRQ);

	while (timeout--) {
		val = readl(&regs->ctl_btr);
		if (!(val & MEN_Z192_CTL1_INITAK))
			break;

		udelay(10);
	}

	if (val & MEN_Z192_CTL1_INITAK)
		return -ETIMEDOUT;

	return 0;
}

static int men_z192_req_init_mode(struct men_z192 *priv)
{
	unsigned int timeout = MEN_Z192_MODE_TOUT_US / 10;
	struct men_z192_regs __iomem *regs = priv->regs;
	u32 val;

	men_z192_bit_set(priv, &regs->ctl_btr, MEN_Z192_CTL0_INITRQ);

	while (timeout--) {
		val = readl(&regs->ctl_btr);
		if (val & MEN_Z192_CTL1_INITAK)
			break;

		udelay(10);
	}

	if (!(val & MEN_Z192_CTL1_INITAK))
		return -ETIMEDOUT;

	return 0;
}

static int men_z192_read_frame(struct net_device *ndev, unsigned int frame_nr)
{
	struct net_device_stats *stats = &ndev->stats;
	struct men_z192 *priv = netdev_priv(ndev);
	struct men_z192_cf_buf __iomem *cf_buf;
	struct can_frame *cf;
	struct sk_buff *skb;
	u32 cf_offset;
	u32 length;
	u32 data;
	u32 id;

	skb = alloc_can_skb(ndev, &cf);
	if (unlikely(!skb)) {
		stats->rx_dropped++;
		return 0;
	}

	cf_offset = sizeof(struct men_z192_cf_buf) * frame_nr;

	cf_buf = priv->dev_base + MEN_Z192_RX_BUF_START + cf_offset;
	length = readl(&cf_buf->length) & MEN_Z192_CFBUF_LEN;
	id = readl(&cf_buf->can_id);

	if (id & MEN_Z192_CFBUF_IDE) {
		/* Extended frame */
		cf->can_id = (id & MEN_Z192_CFBUF_ID1) >> 3;
		cf->can_id |= (id & MEN_Z192_CFBUF_ID2) >>
				MEN_Z192_CFBUF_ID2_SHIFT;

		cf->can_id |= CAN_EFF_FLAG;

		if (id & MEN_Z192_CFBUF_SRR)
			cf->can_id |= CAN_RTR_FLAG;
	} else {
		/* Standard frame */
		cf->can_id = (id & MEN_Z192_CFBUF_ID1) >>
				MEN_Z192_CFBUF_ID1_SHIFT;

		if (id & MEN_Z192_CFBUF_S_RTR)
			cf->can_id |= CAN_RTR_FLAG;
	}

	cf->can_dlc = get_can_dlc(length);

	/* remote transmission request frame
	 * contains no data field even if the
	 * data length is set to a value > 0
	 */
	if (!(cf->can_id & CAN_RTR_FLAG)) {
		if (cf->can_dlc > 0) {
			data = readl(&cf_buf->data[0]);
			*(__be32 *)cf->data = cpu_to_be32(data);
		}
		if (cf->can_dlc > 4) {
			data = readl(&cf_buf->data[1]);
			*(__be32 *)(cf->data + 4) = cpu_to_be32(data);
		}
	}

	stats->rx_bytes += cf->can_dlc;
	stats->rx_packets++;
	netif_receive_skb(skb);

	return 1;
}

static int men_z192_poll(struct napi_struct *napi, int quota)
{
	struct net_device *ndev = napi->dev;
	struct men_z192 *priv = netdev_priv(ndev);
	struct men_z192_regs __iomem *regs = priv->regs;
	int work_done = 0;
	u32 frame_cnt;
	u32 status;

	status = readl(&regs->rx_tx_sts);

	frame_cnt = MEN_Z192_RX_BUF_CNT(status);

	while (frame_cnt-- && (work_done < quota)) {
		work_done += men_z192_read_frame(ndev, 0);
		men_z192_ack_rx_pkg(priv, 1);
	}

	if (work_done < quota) {
		napi_complete(napi);
		men_z192_set_int(priv, MEN_Z192_CAN_NAPI_EN);
	}

	return work_done;
}

static int men_z192_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct can_frame *cf = (struct can_frame *)skb->data;
	struct men_z192 *priv = netdev_priv(ndev);
	struct men_z192_regs __iomem *regs = priv->regs;
	struct net_device_stats *stats = &ndev->stats;
	struct men_z192_cf_buf __iomem *cf_buf;
	u32 data[2] = {0, 0};
	int status;
	u32 id;

	if (can_dropped_invalid_skb(ndev, skb))
		return NETDEV_TX_OK;

	status = readl(&regs->rx_tx_sts);

	if (MEN_Z192_TX_BUF_CNT(status) >= 255) {
		netif_stop_queue(ndev);
		netdev_err(ndev, "not enough space in TX buffer\n");

		return NETDEV_TX_BUSY;
	}

	cf_buf = priv->dev_base + MEN_Z192_TX_BUF_START;

	if (cf->can_id & CAN_EFF_FLAG) {
		/* Extended frame */
		id = ((cf->can_id & CAN_EFF_MASK) <<
			MEN_Z192_CFBUF_ID2_SHIFT) & MEN_Z192_CFBUF_ID2;

		id |= (((cf->can_id & CAN_EFF_MASK) >>
			(CAN_EFF_ID_BITS - CAN_SFF_ID_BITS)) <<
			 MEN_Z192_CFBUF_ID1_SHIFT) & MEN_Z192_CFBUF_ID1;

		id |= MEN_Z192_CFBUF_IDE;

		if (cf->can_id & CAN_RTR_FLAG) {
			id |= MEN_Z192_CFBUF_SRR;
			id |= MEN_Z192_CFBUF_E_RTR;
		}
	} else {
		/* Standard frame */
		id = ((cf->can_id & CAN_SFF_MASK) <<
		       MEN_Z192_CFBUF_ID1_SHIFT) & MEN_Z192_CFBUF_ID1;

		if (cf->can_id & CAN_RTR_FLAG)
			id |= MEN_Z192_CFBUF_S_RTR;
	}

	if (cf->can_dlc > 0)
		data[0] = be32_to_cpup((__be32 *)(cf->data));
	if (cf->can_dlc > 3)
		data[1] = be32_to_cpup((__be32 *)(cf->data + 4));

	writel(id, &cf_buf->can_id);
	writel(cf->can_dlc, &cf_buf->length);

	if (!(cf->can_id & CAN_RTR_FLAG)) {
		writel(data[0], &cf_buf->data[0]);
		writel(data[1], &cf_buf->data[1]);

		stats->tx_bytes += cf->can_dlc;
	}

	/* be sure everything is written to the
	 * device before acknowledge the data.
	 */
	mmiowb();

	/* trigger the transmission */
	men_z192_ack_tx_pkg(priv, 1);

	stats->tx_packets++;

	kfree_skb(skb);

	return NETDEV_TX_OK;
}

static void men_z192_err_interrupt(struct net_device *ndev, u32 status)
{
	struct net_device_stats *stats = &ndev->stats;
	struct men_z192 *priv = netdev_priv(ndev);
	struct can_berr_counter bec;
	struct can_frame *cf;
	struct sk_buff *skb;
	enum can_state rx_state = 0, tx_state = 0;

	skb = alloc_can_err_skb(ndev, &cf);
	if (unlikely(!skb))
		return;

	/* put the rx/tx error counter to
	 * the additional controller specific
	 * section of the error frame.
	 */
	men_z192_get_berr_counter(ndev, &bec);
	cf->data[6] = bec.txerr;
	cf->data[7] = bec.rxerr;

	/* overrun interrupt */
	if (status & MEN_Z192_RFLG_OVRF) {
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
		stats->rx_over_errors++;
		stats->rx_errors++;
	}

	/* bus change interrupt */
	if (status & MEN_Z192_RFLG_CSCIF) {
		rx_state = bus_state_map[MEN_Z192_GET_RSTATE(status)];
		tx_state = bus_state_map[MEN_Z192_GET_TSTATE(status)];
		can_change_state(ndev, cf, tx_state, rx_state);

		if (priv->can.state == CAN_STATE_BUS_OFF)
			can_bus_off(ndev);
	}

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_receive_skb(skb);
}

static irqreturn_t men_z192_isr(int irq, void *dev_id)
{
	struct net_device *ndev = dev_id;
	struct men_z192 *priv = netdev_priv(ndev);
	struct men_z192_regs __iomem *regs = priv->regs;
	bool handled = false;
	u32 irq_flags;
	u32 status;

	status = readl(&regs->rx_tx_sts);

	irq_flags = status & MEN_Z192_IRQ_FLAGS_ALL;
	if (!irq_flags)
		goto out;

	/* It is save to write to RX_TS_STS[15:0] */
	writel(irq_flags, &regs->rx_tx_sts);

	if (irq_flags & MEN_Z192_TFLG_TXIF) {
		netif_wake_queue(ndev);
		handled = true;
	}

	/* handle errors */
	if ((irq_flags & MEN_Z192_RFLG_OVRF) ||
	    (irq_flags & MEN_Z192_RFLG_CSCIF)) {
		men_z192_err_interrupt(ndev, status);
		handled = true;
	}

	/* schedule NAPI if:
	 * - rx IRQ
	 * - rx timeout IRQ
	 */
	if ((irq_flags & MEN_Z192_RFLG_RXIF) ||
	    (irq_flags & MEN_Z192_RFLG_TOUTF)) {
		men_z192_set_int(priv, MEN_Z192_CAN_NAPI_DIS);
		napi_schedule(&priv->napi);
		handled = true;
	}

out:
	return IRQ_RETVAL(handled);
}

static int men_z192_set_bittiming(struct net_device *ndev)
{
	struct men_z192 *priv = netdev_priv(ndev);
	const struct can_bittiming *bt = &priv->can.bittiming;
	unsigned long flags;
	u32 ctlbtr;
	int ret = 0;

	spin_lock_irqsave(&priv->lock, flags);

	ctlbtr = readl(&priv->regs->ctl_btr);

	if (!(ctlbtr & MEN_Z192_CTL1_INITAK)) {
		netdev_alert(ndev,
			     "cannot set bittiminig while in running mode\n");
		ret = -EPERM;
		goto out_restore;
	}

	ctlbtr &= ~(MEN_Z192_BTR0_BRP(0x3f) |
		    MEN_Z192_BTR0_SJW(0x03) |
		    MEN_Z192_BTR1_TSEG1(0x0f) |
		    MEN_Z192_BTR1_TSEG2(0x07) |
		    MEN_Z192_CTL1_LISTEN |
		    MEN_Z192_CTL1_LOOPB |
		    MEN_Z192_BTR1_SAMP);

	ctlbtr |= MEN_Z192_BTR0_BRP(bt->brp - 1) |
		  MEN_Z192_BTR0_SJW(bt->sjw - 1) |
		  MEN_Z192_BTR1_TSEG1(bt->phase_seg1 + bt->prop_seg - 1) |
		  MEN_Z192_BTR1_TSEG2(bt->phase_seg2 - 1);

	if (priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES)
		ctlbtr |= MEN_Z192_BTR1_SAMP;

	if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
		ctlbtr |= MEN_Z192_CTL1_LISTEN;

	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		ctlbtr |= MEN_Z192_CTL1_LOOPB;

	netdev_dbg(ndev, "CTL_BTR=0x%08x\n", ctlbtr);

	writel(ctlbtr, &priv->regs->ctl_btr);

out_restore:
	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static void men_z192_init_idac(struct net_device *ndev)
{
	struct men_z192 *priv = netdev_priv(ndev);
	struct men_z192_regs __iomem *regs = priv->regs;

	/* hardware filtering (accept everything) */
	writel(0x00000000, &regs->idar_0_to_3);
	writel(0x00000000, &regs->idar_4_to_7);
	writel(0xffffffff, &regs->idmr_0_to_3);
	writel(0xffffffff, &regs->idmr_4_to_7);
}

void men_z192_set_can_state(struct net_device *ndev)
{
	struct men_z192 *priv = netdev_priv(ndev);
	struct men_z192_regs __iomem *regs = priv->regs;
	enum can_state rx_state, tx_state;
	u32 status;

	status = readl(&regs->rx_tx_sts);

	rx_state = bus_state_map[MEN_Z192_GET_RSTATE(status)];
	tx_state = bus_state_map[MEN_Z192_GET_TSTATE(status)];

	priv->can.state = max(tx_state, rx_state);
}

static int men_z192_start(struct net_device *ndev)
{
	struct men_z192 *priv = netdev_priv(ndev);
	int ret;

	ret = men_z192_req_init_mode(priv);
	if (ret)
		return ret;

	ret = men_z192_set_bittiming(ndev);
	if (ret)
		return ret;

	ret = men_z192_req_run_mode(priv);
	if (ret)
		return ret;

	men_z192_init_idac(ndev);

	/* The 16z192 CAN IP does not reset the can bus state
	 * if we enter the init mode. There is also
	 * no software reset to reset the state machine.
	 * We need to read the current state, and
	 * inform the upper layer about the current state.
	 */
	men_z192_set_can_state(ndev);

	men_z192_set_int(priv, MEN_Z192_CAN_EN);

	return 0;
}

static int men_z192_open(struct net_device *ndev)
{
	struct men_z192 *priv = netdev_priv(ndev);
	int ret;

	ret = open_candev(ndev);
	if (ret)
		return ret;

	ret = request_irq(ndev->irq, men_z192_isr, IRQF_SHARED,
			  ndev->name, ndev);
	if (ret)
		goto out_close;

	ret = men_z192_start(ndev);
	if (ret)
		goto out_free_irq;

	napi_enable(&priv->napi);
	netif_start_queue(ndev);

	return 0;

out_free_irq:
	free_irq(ndev->irq, ndev);
out_close:
	close_candev(ndev);
	return ret;
}

static int men_z192_stop(struct net_device *ndev)
{
	struct men_z192 *priv = netdev_priv(ndev);
	int ret;

	men_z192_set_int(priv, MEN_Z192_CAN_DIS);

	ret = men_z192_req_init_mode(priv);
	if (ret)
		return ret;

	priv->can.state = CAN_STATE_STOPPED;

	return 0;
}

static int men_z192_close(struct net_device *ndev)
{
	struct men_z192 *priv = netdev_priv(ndev);
	int ret;

	netif_stop_queue(ndev);

	napi_disable(&priv->napi);

	ret = men_z192_stop(ndev);

	free_irq(ndev->irq, ndev);

	close_candev(ndev);

	return ret;
}

static int men_z192_set_mode(struct net_device *ndev, enum can_mode mode)
{
	int ret;

	switch (mode) {
	case CAN_MODE_START:
		ret = men_z192_start(ndev);
		if (ret)
			return ret;

		netif_wake_queue(ndev);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static const struct net_device_ops men_z192_netdev_ops = {
	.ndo_open	= men_z192_open,
	.ndo_stop	= men_z192_close,
	.ndo_start_xmit	= men_z192_xmit,
	.ndo_change_mtu	= can_change_mtu,
};

static int men_z192_verify_buf_lvl(int buffer_lvl)
{
	if (buffer_lvl < MEN_Z192_MIN_BUF_LVL ||
	    buffer_lvl > MEN_Z192_MAX_BUF_LVL)
		return -EINVAL;

	return 0;
}

static void men_z192_set_buf_lvl_irq(struct net_device *ndev, int rxlvl,
				     int txlvl)
{
	struct men_z192 *priv = netdev_priv(ndev);
	struct men_z192_regs __iomem *regs = priv->regs;
	int reg_val;

	if (men_z192_verify_buf_lvl(rxlvl))
		reg_val = MEN_Z192_RX_BUF_LVL_DEF & MEN_Z192_RX_BUF_LVL;
	else
		reg_val = rxlvl & MEN_Z192_RX_BUF_LVL;

	if (men_z192_verify_buf_lvl(txlvl))
		reg_val |= (MEN_Z192_TX_BUF_LVL_DEF << 16) &
			    MEN_Z192_TX_BUF_LVL;
	else
		reg_val |= (txlvl << 16) & MEN_Z192_TX_BUF_LVL;

	dev_info(priv->dev, "RX IRQ Level: %d TX IRQ Level: %d\n",
		 rxlvl, txlvl);

	writel(reg_val, &regs->buf_lvl);
}

static void men_z192_set_rx_tout(struct net_device *ndev, int tout)
{
	struct men_z192 *priv = netdev_priv(ndev);
	struct men_z192_regs __iomem *regs = priv->regs;
	int reg_val;

	if (tout < MEN_Z192_RX_TOUT_MIN || tout > MEN_Z192_RX_TOUT_MAX)
		reg_val = MEN_Z192_RX_TOUT_MAX;
	else
		reg_val = tout;

	dev_info(priv->dev, "RX IRQ timeout set to: %d\n", reg_val);

	writel(reg_val, &regs->rx_timeout);
}

static int men_z192_register(struct net_device *ndev)
{
	struct men_z192 *priv = netdev_priv(ndev);
	struct men_z192_regs __iomem *regs = priv->regs;
	u32 ctl_btr;
	int ret;

	/* The CAN controller should be always enabled.
	 * There is no way to enable it if disabled.
	 */
	ctl_btr = readl(&regs->ctl_btr);
	if (!(ctl_btr & MEN_Z192_CTL1_CANE))
		return -ENODEV;

	men_z192_set_buf_lvl_irq(ndev, rxlvl, txlvl);
	men_z192_set_rx_tout(ndev, rx_timeout);

	ret = men_z192_req_init_mode(priv);
	if (ret) {
		dev_err(priv->dev, "failed to request init mode\n");
		return ret;
	}

	return register_candev(ndev);
}

static void men_z192_unregister(struct net_device *ndev)
{
	return unregister_candev(ndev);
}

static int men_z192_probe(struct mcb_device *mdev,
			  const struct mcb_device_id *id)
{
	struct device *dev = &mdev->dev;
	struct men_z192 *priv;
	struct net_device *ndev;
	void __iomem *dev_base;
	struct resource *mem;
	u32 timebase;
	int ret = 0;
	int irq;

	mem = mcb_request_mem(mdev, dev_name(dev));
	if (IS_ERR(mem)) {
		dev_err(dev, "failed to request device memory");
		return PTR_ERR(mem);
	}

	dev_base = ioremap(mem->start, resource_size(mem));
	if (!dev_base) {
		dev_err(dev, "failed to ioremap device memory");
		ret = -ENXIO;
		goto out_release;
	}

	irq = mcb_get_irq(mdev);
	if (irq <= 0) {
		ret = -ENODEV;
		goto out_unmap;
	}

	ndev = alloc_candev(sizeof(struct men_z192), 1);
	if (!ndev) {
		dev_err(dev, "failed to allocate the can device");
		ret = -ENOMEM;
		goto out_unmap;
	}

	ndev->netdev_ops = &men_z192_netdev_ops;
	ndev->irq = irq;

	priv = netdev_priv(ndev);
	priv->ndev = ndev;
	priv->dev = dev;

	priv->mem = mem;
	priv->dev_base = dev_base;
	priv->regs = priv->dev_base + MEN_Z192_REGS_OFFS;

	timebase = readl(&priv->regs->timebase);
	if (!timebase) {
		dev_err(dev, "invalid timebase configured (timebase=%d)\n",
			timebase);
		ret = -EINVAL;
		goto out_unmap;
	}

	priv->can.clock.freq = timebase;
	priv->can.bittiming_const = &men_z192_bittiming_const;
	priv->can.do_set_mode = men_z192_set_mode;
	priv->can.do_get_berr_counter = men_z192_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LISTENONLY |
				       CAN_CTRLMODE_3_SAMPLES |
				       CAN_CTRLMODE_LOOPBACK;

	spin_lock_init(&priv->lock);

	netif_napi_add(ndev, &priv->napi, men_z192_poll,
		       MEN_Z192_NAPI_WEIGHT);

	mcb_set_drvdata(mdev, ndev);
	SET_NETDEV_DEV(ndev, dev);

	ret = men_z192_register(ndev);
	if (ret) {
		dev_err(dev, "failed to register CAN device");
		goto out_free_candev;
	}

	dev_info(dev, "MEN 16z192 CAN driver successfully registered\n");

	return 0;

out_free_candev:
	netif_napi_del(&priv->napi);
	free_candev(ndev);
out_unmap:
	iounmap(dev_base);
out_release:
	mcb_release_mem(mem);
	return ret;
}

static void men_z192_remove(struct mcb_device *mdev)
{
	struct net_device *ndev = mcb_get_drvdata(mdev);
	struct men_z192 *priv = netdev_priv(ndev);

	men_z192_unregister(ndev);
	netif_napi_del(&priv->napi);

	iounmap(priv->dev_base);
	mcb_release_mem(priv->mem);

	free_candev(ndev);
}

static const struct mcb_device_id men_z192_ids[] = {
	{ .device = 0xc0 },
	{ }
};
MODULE_DEVICE_TABLE(mcb, men_z192_ids);

static struct mcb_driver men_z192_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = men_z192_probe,
	.remove = men_z192_remove,
	.id_table = men_z192_ids,
};
module_mcb_driver(men_z192_driver);

MODULE_AUTHOR("Andreas Werner <andreas.werner@men.de>");
MODULE_DESCRIPTION("MEN 16z192 CAN Controller");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("mcb:16z192");
