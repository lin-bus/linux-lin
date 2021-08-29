[1mdiff --git a/sllin/sllin.c b/sllin/sllin.c[m
[1mindex 2db896f..9a40792 100644[m
[1m--- a/sllin/sllin.c[m
[1m+++ b/sllin/sllin.c[m
[36m@@ -57,16 +57,22 @@[m
 #include <linux/if_arp.h>[m
 #include <linux/if_ether.h>[m
 #include <linux/sched.h>[m
[32m+[m[32m#include <uapi/linux/sched/types.h>[m
[32m+[m
 #include <linux/delay.h>[m
 #include <linux/init.h>[m
[31m-#include <linux/can.h>[m
[32m+[m
 #include <linux/kthread.h>[m
 #include <linux/hrtimer.h>[m
 #include <linux/version.h>[m
[32m+[m[32m#include <linux/can.h>[m
[32m+[m[32m#include <linux/can/skb.h>[m
[32m+[m[32m#include <linux/can/can-ml.h>[m
 #include "linux/lin_bus.h"[m
 [m
[32m+[m
 /* Should be in include/linux/tty.h */[m
[31m-#define N_SLLIN			25[m
[32m+[m[32m#define N_SLLIN			28[m
 /* -------------------------------- */[m
 [m
 static __initdata const char banner[] =[m
[36m@@ -447,7 +453,7 @@[m [mstatic int sll_open(struct net_device *dev)[m
 static void sll_free_netdev(struct net_device *dev)[m
 {[m
 	int i = dev->base_addr;[m
[31m-	free_netdev(dev);[m
[32m+[m	[32m//free_netdev(dev);[m
 	sllin_devs[i] = NULL;[m
 }[m
 [m
[36m@@ -460,18 +466,21 @@[m [mstatic const struct net_device_ops sll_netdev_ops = {[m
 static void sll_setup(struct net_device *dev)[m
 {[m
 	dev->netdev_ops		= &sll_netdev_ops;[m
[31m-	dev->destructor		= sll_free_netdev;[m
[32m+[m	[32mdev->needs_free_netdev	= true;[m
[32m+[m	[32mdev->priv_destructor	= sll_free_netdev;[m
 [m
 	dev->hard_header_len	= 0;[m
 	dev->addr_len		= 0;[m
 	dev->tx_queue_len	= 10;[m
 [m
[31m-	dev->mtu		= sizeof(struct can_frame);[m
[32m+[m	[32mdev->mtu		= CAN_MTU;[m
 	dev->type		= ARPHRD_CAN;[m
 [m
 	/* New-style flags. */[m
 	dev->flags		= IFF_NOARP;[m
 	dev->features           = NETIF_F_HW_CSUM; /* NETIF_F_NO_CSUM;*/[m
[32m+[m
[32m+[m
 }[m
 [m
 /******************************************[m
[36m@@ -540,13 +549,16 @@[m [mstatic void sllin_report_error(struct sllin *sl, int err)[m
 	switch (err) {[m
 	case LIN_ERR_CHECKSUM:[m
 		sl->dev->stats.rx_crc_errors++;[m
[32m+[m		[32mnetdev_dbg(sl->dev, "error - checksum error!");[m
 		break;[m
 [m
 	case LIN_ERR_RX_TIMEOUT:[m
[32m+[m		[32mnetdev_dbg(sl->dev, "error - rx timeout!");[m
 		sl->dev->stats.rx_errors++;[m
 		break;[m
 [m
 	case LIN_ERR_FRAMING:[m
[32m+[m		[32mnetdev_dbg(sl->dev, "error - LIN frame error");[m
 		sl->dev->stats.rx_frame_errors++;[m
 		break;[m
 	}[m
[36m@@ -770,12 +782,15 @@[m [mstatic void sllin_slave_receive_buf(struct tty_struct *tty,[m
 				/* 'Duplicated' break character -- ignore */[m
 				if (*cp == 0x00) {[m
 					cp++;[m
[32m+[m[41m					[m
 					continue;[m
 				}[m
 [m
 				/* Wrong sync character */[m
[31m-				if (*cp != 0x55)[m
[32m+[m				[32mif (*cp != 0x55) {[m
[32m+[m					[32mnetdev_dbg(sl->dev, "error - wrong sync char");[m
 					break;[m
[32m+[m				[32m}[m
 			}[m
 [m
 			sl->rx_buff[sl->rx_cnt++] = *cp++;[m
[36m@@ -991,11 +1006,16 @@[m [mstatic enum hrtimer_restart sllin_rx_timeout_handler(struct hrtimer *hrtimer)[m
 			(sl->rx_cnt <= SLLIN_BUFF_DATA) ||[m
 			((!sl->rx_len_unknown) &&[m
 			(sl->rx_cnt < sl->rx_expect))) {[m
[32m+[m[41m				[m
[32m+[m		[32mnetdev_dbg(sl->dev, "error - Master didn't receive as much as expected\n");[m
[32m+[m		[32mnetdev_dbg(sl->dev, "expected: %d\n", sl->rx_expect);[m
[32m+[m		[32mnetdev_dbg(sl->dev, "received: %d\n", sl->rx_cnt);[m
 		sllin_report_error(sl, LIN_ERR_RX_TIMEOUT);[m
 		set_bit(SLF_TMOUTEVENT, &sl->flags);[m
 	} else {[m
 		sllin_slave_finish_rx_msg(sl);[m
 		set_bit(SLF_RXEVENT, &sl->flags);[m
[32m+[m[41m		[m
 	}[m
 	wake_up(&sl->kwt_wq);[m
 [m
[36m@@ -1017,7 +1037,9 @@[m [mstatic int sllin_kwthread(void *ptr)[m
 	struct sllin_conf_entry *sce;[m
 [m
 	netdev_dbg(sl->dev, "sllin_kwthread started.\n");[m
[31m-	sched_setscheduler(current, SCHED_FIFO, &schparam);[m
[32m+[m	[32m//sched_setscheduler(current, SCHED_FIFO, &schparam);[m
[32m+[m	[32m//TODO: set_schedule[m
[32m+[m	[32msched_set_fifo(current);[m
 [m
 	clear_bit(SLF_ERROR, &sl->flags);[m
 	sltty_change_speed(tty, sl->lin_baud);[m
[36m@@ -1332,7 +1354,9 @@[m [mstatic void sll_sync(void)[m
 static struct sllin *sll_alloc(dev_t line)[m
 {[m
 	int i;[m
[32m+[m	[32mint size;[m
 	struct net_device *dev = NULL;[m
[32m+[m	[32m//struct can_ml_priv *can_ml;[m
 	struct sllin       *sl;[m
 [m
 	if (sllin_devs == NULL)[m
[36m@@ -1348,7 +1372,8 @@[m [mstatic struct sllin *sll_alloc(dev_t line)[m
 	/* Sorry, too many, all slots in use */[m
 	if (i >= maxdev)[m
 		return NULL;[m
[31m-[m
[32m+[m[41m	[m
[32m+[m	[32m/* unregister existing net devices first */[m
 	if (dev) {[m
 		sl = netdev_priv(dev);[m
 		if (test_bit(SLF_INUSE, &sl->flags)) {[m
[36m@@ -1357,23 +1382,33 @@[m [mstatic struct sllin *sll_alloc(dev_t line)[m
 			sllin_devs[i] = NULL;[m
 		}[m
 	}[m
[31m-[m
[32m+[m[41m	[m
[32m+[m[41m	[m
 	if (!dev) {[m
[32m+[m		[32m/*FIX ME!!! at some unknown version the kernel changes to the below format[m
[32m+[m		[32mTHEN after aprx 5.10.? it changes AGAIN(See SLCAN changes and implement accordingly)[m
[32m+[m		[32m*/[m
 		char name[IFNAMSIZ];[m
 		sprintf(name, "sllin%d", i);[m
[32m+[m		[32msize = ALIGN(sizeof(*sl), NETDEV_ALIGN) + sizeof(struct can_ml_priv);[m
[32m+[m		[32mdev = alloc_netdev(size, name, NET_NAME_UNKNOWN, sll_setup);[m
[32m+[m
[32m+[m[32m//#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0))[m
[32m+[m[32m//		dev = alloc_netdev(sizeof(*sl), name, sll_setup);[m
[32m+[m[32m//#else[m
[32m+[m[32m//		dev = alloc_netdev(sizeof(*sl), name, NET_NAME_UNKNOWN, sll_setup);[m
[32m+[m[32m//#endif[m
 [m
[31m-#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0))[m
[31m-		dev = alloc_netdev(sizeof(*sl), name, sll_setup);[m
[31m-#else[m
[31m-		dev = alloc_netdev(sizeof(*sl), name, NET_NAME_UNKNOWN, sll_setup);[m
[31m-#endif[m
 [m
 		if (!dev)[m
 			return NULL;[m
 		dev->base_addr  = i;[m
 	}[m
 [m
[32m+[m[41m	[m
 	sl = netdev_priv(dev);[m
[32m+[m	[32mdev->ml_priv = (void *)sl + ALIGN(sizeof(*sl), NETDEV_ALIGN);[m
[32m+[m[41m	[m
 	/* Initialize channel control data */[m
 	sl->magic = SLLIN_MAGIC;[m
 	sl->dev	= dev;[m
[36m@@ -1637,8 +1672,7 @@[m [mstatic void __exit sllin_exit(void)[m
 		sl = netdev_priv(dev);[m
 		if (sl->tty) {[m
 			netdev_dbg(sl->dev, "tty discipline still running\n");[m
[31m-			/* Intentionally leak the control block. */[m
[31m-			dev->destructor = NULL;[m
[32m+[m
 		}[m
 [m
 		unregister_netdev(dev);[m
