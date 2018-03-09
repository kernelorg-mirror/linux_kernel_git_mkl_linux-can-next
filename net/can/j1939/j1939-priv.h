// SPDX-License-Identifier: GPL-2.0
/* j1939-priv.h
 *
 * Copyright (c) 2010-2011 EIA Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _J1939_PRIV_H_
#define _J1939_PRIV_H_

#include <linux/atomic.h>
#include <linux/if_arp.h>
#include <linux/interrupt.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/can/can-ml.h>
#include <linux/can/j1939.h>

#include <net/sock.h>

#include "../af_can.h"

/* TODO: return ENETRESET on busoff. */

#define PGN_REQUEST 0x0ea00
#define PGN_ADDRESS_CLAIMED 0x0ee00
#define PGN_MAX 0x3ffff

/* j1939 devices */
struct j1939_ecu {
	struct list_head list;
	name_t name;
	u8 sa;

	/* indicates that this ecu successfully claimed @sa as its address */
	struct hrtimer ac_timer;
	struct kref kref;
	struct j1939_priv *priv;

	/* count users, to help transport protocol decide for interaction */
	int nusers;
};

struct j1939_priv {
	struct list_head ecus;
	/* local list entry in priv
	 * These allow irq (& softirq) context lookups on j1939 devices
	 * This approach (separate lists) is done as the other 2 alternatives
	 * are not easier or even wrong
	 * 1) using the pure kobject methods involves mutexes, which are not
	 *    allowed in irq context.
	 * 2) duplicating data structures would require a lot of synchronization
	 *    code
	 * usage:
	 */

	/* segments need a lock to protect the above list */
	rwlock_t lock;

	struct net_device *netdev;

	/* list of 256 ecu ptrs, that cache the claimed addresses.
	 * also protected by the above lock
	 * don't use directly, use j1939_ecu_set_address() instead
	 */
	struct addr_ent {
		struct j1939_ecu *ecu;
		/* count users, to help transport protocol */
		int nusers;
	} ents[256];

	struct kref kref;
};

void j1939_ecu_put(struct j1939_ecu *ecu);

/* keep the cache of what is local */
int j1939_local_get(struct j1939_priv *priv, name_t name, u8 sa);
void j1939_local_put(struct j1939_priv *priv, name_t name, u8 sa);

/* conversion function between (struct sock | struct sk_buff)->sk_priority
 * from linux and j1939 priority field
 */
static inline priority_t j1939_prio(int sk_priority)
{
	if (sk_priority < 0)
		return 6; /* default */
	else if (sk_priority > 7)
		return 0;
	else
		return 7 - sk_priority;
}

static inline bool j1939_address_is_valid(u8 sa)
{
	return sa != J1939_NO_ADDR;
}

static inline bool j1939_address_is_unicast(u8 sa)
{
	return sa <= J1939_MAX_UNICAST_ADDR;
}

static inline bool pgn_is_pdu1(pgn_t pgn)
{
	/* ignore dp & res bits for this */
	return (pgn & 0xff00) < 0xf000;
}

/* utility to correctly unregister a SA */
void _j1939_ecu_remove_sa(struct j1939_ecu *ecu);
void j1939_ecu_remove_sa(struct j1939_ecu *ecu);

u8 j1939_name_to_sa(struct j1939_priv *priv, name_t name);
struct j1939_ecu *j1939_ecu_get_by_addr(struct j1939_priv *priv, u8 sa);
struct j1939_ecu *j1939_ecu_get_by_name(struct j1939_priv *priv, name_t name);

struct j1939_addr {
	name_t src_name;
	name_t dst_name;
	pgn_t pgn;

	u8 sa;
	u8 da;
};

/* control buffer of the sk_buff */
struct j1939_sk_buff_cb {
	struct j1939_addr addr;
	priority_t priority;

	/* Flags for quick lookups during skb processing
	 * These are set in the receive path only
	 */
	int src_flags;
	int dst_flags;

#define ECU_LOCAL 1

	/* for tx, MSG_SYN will be used to sync on sockets */
	int msg_flags;

	/* j1939 clones incoming skb's.
	 * insock saves the incoming skb->sk
	 * to determine local generated packets
	 */
	struct sock *insock;
};

static inline struct j1939_sk_buff_cb *j1939_get_cb(struct sk_buff *skb)
{
	BUILD_BUG_ON(sizeof(struct j1939_sk_buff_cb) > sizeof(skb->cb));

	return (struct j1939_sk_buff_cb *)skb->cb;
}

static inline int j1939cb_is_broadcast(const struct j1939_sk_buff_cb *skcb)
{
	return (!skcb->addr.dst_name && (skcb->addr.da == 0xff));
}

int j1939_send(struct net *net, struct sk_buff *skb);
void j1939_recv(struct sk_buff *skb);

/* stack entries */
int j1939_send_transport(struct net *net, struct j1939_priv *priv, struct sk_buff *skb);
int j1939_recv_transport(struct net *net, struct sk_buff *skb);
int j1939_ac_fixup(struct j1939_priv *priv, struct sk_buff *skb);
void j1939_ac_recv(struct j1939_priv *priv, struct sk_buff *skb);

/* network management */

/* j1939_ecu_get_register
 * 'create' & 'register' & 'get' new ecu
 * when a matching ecu already exists, then that is returned
 */
struct j1939_ecu *_j1939_ecu_get_register(struct j1939_priv *priv,
					  name_t name, bool create_if_necessary);

/* unregister must be called with lock held */
void _j1939_ecu_unregister(struct j1939_ecu *ecu);

int j1939_netdev_start(struct net *net, struct net_device *netdev);
void j1939_netdev_stop(struct net_device *netdev);

void __j1939_priv_release(struct kref *kref);
struct j1939_priv *j1939_priv_get(struct net_device *dev);
struct j1939_priv *j1939_priv_get_by_index(struct net *net, int ifindex);

static inline void j1939_priv_set(struct net_device *dev, struct j1939_priv *priv)
{
	struct can_ml_priv *can_ml_priv = dev->ml_priv;

	can_ml_priv->j1939_priv = priv;
}

static inline void j1939_priv_put(struct j1939_priv *priv)
{
	kref_put(&priv->kref, __j1939_priv_release);
}

/* notify/alert all j1939 sockets bound to ifindex */
void j1939_sk_netdev_event(struct net_device *netdev, int error_code);
int j1939_tp_rmdev_notifier(struct net_device *netdev);

/* decrement pending skb for a j1939 socket */
void j1939_sock_pending_del(struct sock *sk);

/* separate module-init/modules-exit's */
__init int j1939_tp_module_init(void);

void j1939_tp_module_exit(void);

/* CAN protocol */
extern const struct can_proto j1939_can_proto;

#endif /* _J1939_PRIV_H_ */
