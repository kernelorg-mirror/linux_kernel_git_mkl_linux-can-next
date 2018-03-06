// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2010-2011 EIA Electronics
 *
 * Authors:
 * Kurt Van Dijck <kurt.van.dijck@eia.be>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 */

/* bus for j1939 remote devices
 * Since rtnetlink, no real bus is used.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/workqueue.h>

#include "j1939-priv.h"

#define ecu_dbg(_ecu, fmt, ...) \
{ \
	struct j1939_ecu *ecu = _ecu; \
	pr_debug("j1939-%i,%016llx,%02x: " fmt, ecu->priv->netdev->ifindex, \
		 ecu->name, ecu->sa, ##__VA_ARGS__); \
}

/* ECU device interface */
void _j1939_ecu_remove_sa(struct j1939_ecu *ecu)
{
	lockdep_assert_held(&ecu->priv->lock);

	if (!j1939_address_is_unicast(ecu->sa))
		return;
	if (ecu->priv && ecu->priv->ents[ecu->sa].ecu == ecu) {
		ecu->priv->ents[ecu->sa].ecu = NULL;
		ecu->priv->ents[ecu->sa].nusers -= ecu->nusers;
	}
}

void j1939_ecu_remove_sa(struct j1939_ecu *ecu)
{
	if (!j1939_address_is_unicast(ecu->sa))
		return;

	write_lock_bh(&ecu->priv->lock);
	_j1939_ecu_remove_sa(ecu);
	write_unlock_bh(&ecu->priv->lock);
}

static enum hrtimer_restart j1939_ecu_timer_handler(struct hrtimer *hrtimer)
{
	struct j1939_ecu *ecu =
		container_of(hrtimer, struct j1939_ecu, ac_timer);
	struct j1939_priv *priv = ecu->priv;

	write_lock_bh(&priv->lock);
	/* TODO: can we test if ecu->sa is unicast before starting
	 * the timer?
	 */
	if (j1939_address_is_unicast(ecu->sa)) {
		priv->ents[ecu->sa].ecu = ecu;
		priv->ents[ecu->sa].nusers += ecu->nusers;
	}
	write_unlock_bh(&priv->lock);

	return HRTIMER_NORESTART;
}

static void __j1939_ecu_release(struct kref *kref)
{
	struct j1939_ecu *ecu = container_of(kref, struct j1939_ecu, kref);

	kfree(ecu);
}

void j1939_ecu_put(struct j1939_ecu *ecu)
{
	kref_put(&ecu->kref, __j1939_ecu_release);
}

static void j1939_ecu_get(struct j1939_ecu *ecu)
{
	kref_get(&ecu->kref);
}

struct j1939_ecu *_j1939_ecu_get_register(struct j1939_priv *priv, name_t name,
					  bool create_if_necessary)
{
	struct j1939_ecu *ecu;

	lockdep_assert_held(&priv->lock);

	/* find existing */
	/* test for existing name */
	list_for_each_entry(ecu, &priv->ecus, list) {
		if (ecu->name == name)
			return ecu;
	}

	if (!create_if_necessary)
		return ERR_PTR(-ENODEV);

	/* alloc */
	ecu = kzalloc(sizeof(*ecu), gfp_any());
	if (!ecu)
		/* should we look for an existing ecu */
		return ERR_PTR(-ENOMEM);
	kref_init(&ecu->kref);
	ecu->sa = J1939_IDLE_ADDR;
	ecu->name = name;

	hrtimer_init(&ecu->ac_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_SOFT);
	ecu->ac_timer.function = j1939_ecu_timer_handler;
	INIT_LIST_HEAD(&ecu->list);

	/* first add to internal list */
	/* a ref to priv is held */
	ecu->priv = priv;
	list_add_tail(&ecu->list, &priv->ecus);

	ecu_dbg(ecu, "register\n");
	/* do not put_j1939_priv, a new ECU keeps a refcnt open */
	return ecu;
}

void _j1939_ecu_unregister(struct j1939_ecu *ecu)
{
	lockdep_assert_held(&ecu->priv->lock);

	ecu_dbg(ecu, "unregister\n");
	hrtimer_cancel(&ecu->ac_timer);

	_j1939_ecu_remove_sa(ecu);
	list_del_init(&ecu->list);
	j1939_ecu_put(ecu);
}

struct j1939_ecu *j1939_ecu_get_by_addr(struct j1939_priv *priv, u8 sa)
{
	struct j1939_ecu *ecu;

	if (!j1939_address_is_unicast(sa))
		return NULL;

	read_lock_bh(&priv->lock);
	ecu = priv->ents[sa].ecu;
	if (ecu)
		j1939_ecu_get(ecu);
	read_unlock_bh(&priv->lock);

	return ecu;
}

u8 j1939_name_to_sa(struct j1939_priv *priv, name_t name)
{
	struct j1939_ecu *ecu;
	int sa;

	if (!name)
		return J1939_NO_ADDR;

	sa = J1939_IDLE_ADDR;
	read_lock_bh(&priv->lock);
	list_for_each_entry(ecu, &priv->ecus, list) {
		if (ecu->name == name) {
			if (priv->ents[ecu->sa].ecu == ecu)
				/* ecu's SA is registered */
				sa = ecu->sa;
			break;
		}
	}
	read_unlock_bh(&priv->lock);

	return sa;
}

/* ecu lookup by name */
struct j1939_ecu *j1939_ecu_find_by_name(struct net *net, struct j1939_priv *priv, name_t name)
{
	struct j1939_ecu *ecu = NULL;

	if (!name)
		return NULL;

	read_lock_bh(&priv->lock);
	list_for_each_entry(ecu, &priv->ecus, list) {
		if (ecu->name == name) {
			j1939_ecu_get(ecu);
			goto found_on_intf;
		}
	}

 found_on_intf:
	read_unlock_bh(&priv->lock);
	return ecu;
}

/* TX addr/name accounting
 * Transport protocol needs to know if a SA is local or not
 * These functions originate from userspace manipulating sockets,
 * so locking is straigforward
 */

int j1939_local_get(struct j1939_priv *priv, name_t name, u8 sa)
{
	struct j1939_ecu *ecu;
	int err = 0;

	write_lock_bh(&priv->lock);

	if (j1939_address_is_unicast(sa))
		priv->ents[sa].nusers++;

	if (!name)
		goto done;

	ecu = _j1939_ecu_get_register(priv, name, true);
	err = PTR_ERR_OR_ZERO(ecu);
	if (err)
		goto done;

	j1939_ecu_get(ecu);
	ecu->nusers++;
	/* TODO: do we care if ecu->sa != sa? */
	if (priv->ents[ecu->sa].ecu == ecu)
		/* ecu's sa is active already */
		priv->ents[ecu->sa].nusers++;

done:
	write_unlock_bh(&priv->lock);

	return err;
}

void j1939_local_put(struct j1939_priv *priv, name_t name, u8 sa)
{
	struct j1939_ecu *ecu;

	write_lock_bh(&priv->lock);

	if (j1939_address_is_unicast(sa))
		priv->ents[sa].nusers--;

	if (!name)
		goto done;

	ecu = _j1939_ecu_get_register(priv, name, false);
	if (WARN_ON_ONCE(PTR_ERR_OR_ZERO(ecu)))
		goto done;

	ecu->nusers--;
	/* TODO: do we care if ecu->sa != sa? */
	if (priv->ents[ecu->sa].ecu == ecu)
		/* ecu's sa is active already */
		priv->ents[ecu->sa].nusers--;
	j1939_ecu_put(ecu);

done:
	write_unlock_bh(&priv->lock);
}
