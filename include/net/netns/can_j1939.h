/* SPDX-License-Identifier: GPL-2.0 */
/*
 * can in net namespaces
 */

#ifndef __NETNS_CAN_J1939_H__
#define __NETNS_CAN_J1939_H__

struct netns_can_j1939 {
	spinlock_t tp_session_list_lock;
	struct list_head tp_sessionq;
	struct list_head tp_extsessionq;
	spinlock_t tp_dellock;
	struct list_head tp_delsessionq;
	struct work_struct tp_delwork;
	wait_queue_head_t tp_wait;
};

#endif /* __NETNS_CAN_J1939_H__ */
