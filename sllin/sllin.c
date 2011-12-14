/*
 * sllin.c - serial line LIN interface driver (using tty line discipline)
 *
 * This file is derived from drivers/net/can/slcan.c
 * slcan.c Author: Oliver Hartkopp <socketcan@hartkopp.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307. You can also get it
 * at http://www.gnu.org/licenses/gpl.html
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * Send feedback to <socketcan-users@lists.berlios.de>
 *
 */

#define DEBUG 		1 /* Enables pr_debug() printouts */

#include <linux/module.h>
#include <linux/moduleparam.h>

#include <asm/system.h>
#include <linux/uaccess.h>
#include <linux/bitops.h>
#include <linux/string.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/rtnetlink.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/can.h>
#include <linux/kthread.h>
#include <linux/hrtimer.h>
#include "linux/lin_bus.h"

/* Should be in include/linux/tty.h */
#define N_SLLIN         	25
/* -------------------------------- */

static __initdata const char banner[] =
	KERN_INFO "sllin: serial line LIN interface driver\n";

MODULE_ALIAS_LDISC(N_SLLIN);
MODULE_DESCRIPTION("serial line LIN interface");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("");

#define SLLIN_MAGIC 		0x53CA
/* #define BREAK_BY_BAUD */

static int maxdev = 10;		/* MAX number of SLLIN channels;
				   This can be overridden with
				   insmod sllin.ko maxdev=nnn	*/
module_param(maxdev, int, 0);
MODULE_PARM_DESC(maxdev, "Maximum number of sllin interfaces");

/* maximum buffer len to store whole LIN message*/
#define SLLIN_DATA_MAX	 	8
#define SLLIN_BUFF_LEN		(1 /*break*/ + 1 /*sync*/ + 1 /*ID*/ + \
                         	SLLIN_DATA_MAX + 1 /*checksum*/)
#define SLLIN_BUFF_BREAK 	0
#define SLLIN_BUFF_SYNC	 	1
#define SLLIN_BUFF_ID	 	2
#define SLLIN_BUFF_DATA	 	3

#define SLLIN_SAMPLES_PER_CHAR	10
#define SLLIN_CHARS_TO_TIMEOUT	12

enum slstate {
	SLSTATE_IDLE = 0,
	SLSTATE_BREAK_SENT,
	SLSTATE_ID_SENT,
	SLSTATE_RESPONSE_WAIT, /* Wait for response */
	SLSTATE_RESPONSE_WAIT_BUS, /* Wait for response from LIN bus
				only (CAN frames from network stack
				are not processed in this moment) */
	SLSTATE_RESPONSE_SENT,
};

struct sllin_conf_entry {
	int dlc;		/* Length of data in LIN frame */
	canid_t frame_fl;	/* LIN frame flags. Passed from userspace as canid_t data type */
	u8 data[8];		/* LIN frame data payload */
};

struct sllin {
	int			magic;

	/* Various fields. */
	struct tty_struct	*tty;		/* ptr to TTY structure	     */
	struct net_device	*dev;		/* easy for intr handling    */
	spinlock_t		lock;

	/* LIN message buffer and actual processed data counts */
	unsigned char		rx_buff[SLLIN_BUFF_LEN]; /* LIN Rx buffer */
	unsigned char		tx_buff[SLLIN_BUFF_LEN]; /* LIN Tx buffer */
	int			rx_expect;      /* expected number of Rx chars */
	int			rx_lim;         /* maximum Rx chars for current frame */
	int			rx_cnt;         /* message buffer Rx fill level  */
	int			tx_lim;         /* actual limit of bytes to Tx */
	int			tx_cnt;         /* number of already Tx bytes */
	char			lin_master;	/* node is a master node */
	int			lin_baud;	/* LIN baudrate */
	int 			lin_state;	/* state */
	char  			id_to_send;	/* there is ID to be sent */
	char                    data_to_send;   /* there are data to be sent */
	char			resp_len_known; /* Length of the response is known */

	unsigned long		flags;		/* Flag values/ mode etc     */
#define SLF_INUSE		0		/* Channel in use            */
#define SLF_ERROR		1               /* Parity, etc. error        */
#define SLF_RXEVENT		2               /* Rx wake event             */
#define SLF_TXEVENT		3               /* Tx wake event             */
#define SLF_MSGEVENT		4               /* CAN message to sent       */
#define SLF_TMOUTEVENT          5               /* Timeout on received data  */

	dev_t			line;
	struct task_struct	*kwthread;
	wait_queue_head_t	kwt_wq;		/* Wait queue used by kwthread */
	struct hrtimer          rx_timer;       /* RX timeout timer */
	ktime_t	                rx_timer_timeout; /* RX timeout timer value */
	struct sk_buff          *tx_req_skb;	/* Socket buffer with CAN frame received
						from network stack*/

	struct sllin_conf_entry linfr_cache[SLLIN_ID_MAX + 1]; /* List with configurations for
						each of 0 to SLLIN_ID_MAX LIN IDs */
};

static struct net_device **sllin_devs;
static int sllin_configure_frame_cache(struct sllin *sl, struct can_frame *cf);


/* Values of two parity bits in LIN Protected
   Identifier for each particular LIN ID */
const unsigned char sllin_id_parity_table[] = {
        0x80, 0xc0, 0x40, 0x00, 0xc0, 0x80, 0x00, 0x40,
        0x00, 0x40, 0xc0, 0x80, 0x40, 0x00, 0x80, 0xc0,
        0x40, 0x00, 0x80, 0xc0, 0x00, 0x40, 0xc0, 0x80,
        0xc0, 0x80, 0x00, 0x40, 0x80, 0xc0, 0x40, 0x00,
        0x00, 0x40, 0xc0, 0x80, 0x40, 0x00, 0x80, 0xc0,
        0x80, 0xc0, 0x40, 0x00, 0xc0, 0x80, 0x00, 0x40,
        0xc0, 0x80, 0x00, 0x40, 0x80, 0xc0, 0x40, 0x00,
        0x40, 0x00, 0x80, 0xc0, 0x00, 0x40, 0xc0, 0x80
};

/**
 * sltty_change_speed() -- Change baudrate of Serial device belonging to particular @tty
 *
 * @tty:	Pointer to TTY to change speed for.
 * @speed:	Integer value of new speed. It is possible to 
 *		assign non-standard values, i.e. those which
 *		are not defined in termbits.h.
 */
static int sltty_change_speed(struct tty_struct *tty, unsigned speed)
{
	struct ktermios old_termios;
	int cflag;

	mutex_lock(&tty->termios_mutex);

	old_termios = *(tty->termios);
	cflag = tty->termios->c_cflag;
	cflag &= ~(CBAUD | CIBAUD);
	cflag |= BOTHER;
	tty->termios->c_cflag = cflag;

	tty_encode_baud_rate(tty, speed, speed);

	if (tty->ops->set_termios)
		tty->ops->set_termios(tty, &old_termios);

	mutex_unlock(&tty->termios_mutex);

	return 0;
}


/* Send one can_frame to the network layer */
static void sllin_send_canfr(struct sllin *sl, canid_t id, char *data, int len)
{
	struct sk_buff *skb;
	struct can_frame cf;

	cf.can_id = id;
	cf.can_dlc = len;
	if (cf.can_dlc > 0) {
		memcpy(&cf.data, data, cf.can_dlc);
	}

	skb = dev_alloc_skb(sizeof(struct can_frame));
	if (!skb)
		return;

	skb->dev = sl->dev;
	skb->protocol = htons(ETH_P_CAN);
	skb->pkt_type = PACKET_BROADCAST;
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	memcpy(skb_put(skb, sizeof(struct can_frame)),
	       &cf, sizeof(struct can_frame));
	netif_rx(skb);

	sl->dev->stats.rx_packets++;
	sl->dev->stats.rx_bytes += cf.can_dlc;


}

/**
 * sll_bump() -- Send data of received LIN frame (existing in sl->rx_buff) as CAN frame
 *
 * @sl:
 */
static void sll_bump(struct sllin *sl)
{
	sllin_send_canfr(sl, sl->rx_buff[SLLIN_BUFF_ID] & SLLIN_ID_MASK,
		sl->rx_buff + SLLIN_BUFF_DATA,
		sl->rx_cnt - SLLIN_BUFF_DATA - 1); /* without checksum */
}

/*
 * Called by the driver when there's room for more data.  If we have
 * more packets to send, we send them here.
 */
static void sllin_write_wakeup(struct tty_struct *tty)
{
	int actual;
	int remains;
	struct sllin *sl = (struct sllin *) tty->disc_data;

	/* First make sure we're connected. */
	if (!sl || sl->magic != SLLIN_MAGIC || !netif_running(sl->dev))
		return;

	if (sl->lin_state != SLSTATE_BREAK_SENT)
		remains = sl->tx_lim - sl->tx_cnt;
	else
		remains = SLLIN_BUFF_BREAK + 1 - sl->tx_cnt;

	if (remains > 0) {
		actual = tty->ops->write(tty, sl->tx_buff + sl->tx_cnt, sl->tx_cnt - sl->tx_lim);
		sl->tx_cnt += actual;

		if (sl->tx_cnt < sl->tx_lim) {
			pr_debug("sllin: sllin_write_wakeup sent %d, remains %d, waiting\n",
				sl->tx_cnt, sl->tx_lim - sl->tx_cnt);
			return;
		}
	}

	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	set_bit(SLF_TXEVENT, &sl->flags);
	wake_up(&sl->kwt_wq);

	pr_debug("sllin: sllin_write_wakeup sent %d, wakeup\n", sl->tx_cnt);
}

/**
 * sll_xmit() -- Send a can_frame to a TTY queue.
 *
 * @skb: Pointer to Socket buffer to be sent.
 * @dev: Network device where @skb will be sent.
 */
static netdev_tx_t sll_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct sllin *sl = netdev_priv(dev);
	struct can_frame *cf;

	if (skb->len != sizeof(struct can_frame))
		goto err_out;

	spin_lock(&sl->lock);
	if (!netif_running(dev))  {
		printk(KERN_WARNING "%s: xmit: iface is down\n", dev->name);
		goto err_out_unlock;
	}
	if (sl->tty == NULL) {
		goto err_out_unlock;
	}

	cf = (struct can_frame *) skb->data;
	if (cf->can_id & SLLIN_CTRL_FRAME) {
		sllin_configure_frame_cache(sl, cf);
		goto free_out_unlock;
	}

	netif_stop_queue(sl->dev);

	sl->tx_req_skb = skb;
	set_bit(SLF_MSGEVENT, &sl->flags);
	wake_up(&sl->kwt_wq);
	spin_unlock(&sl->lock);
	return NETDEV_TX_OK;

free_out_unlock:
err_out_unlock:
	spin_unlock(&sl->lock);
err_out:
	kfree_skb(skb);
	return NETDEV_TX_OK;
}


/******************************************
 *   Routines looking at netdevice side.
 ******************************************/

/* Netdevice UP -> DOWN routine */
static int sll_close(struct net_device *dev)
{
	struct sllin *sl = netdev_priv(dev);

	spin_lock_bh(&sl->lock);
	if (sl->tty) {
		/* TTY discipline is running. */
		clear_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
	}
	netif_stop_queue(dev);
	sl->rx_expect = 0;
	sl->tx_lim    = 0;
	spin_unlock_bh(&sl->lock);

	return 0;
}

/* Netdevice DOWN -> UP routine */
static int sll_open(struct net_device *dev)
{
	struct sllin *sl = netdev_priv(dev);

	pr_debug("sllin: %s() invoked\n", __FUNCTION__);

	if (sl->tty == NULL)
		return -ENODEV;

	sl->flags &= (1 << SLF_INUSE);
	netif_start_queue(dev);
	return 0;
}

/* Hook the destructor so we can free sllin devs at the right point in time */
static void sll_free_netdev(struct net_device *dev)
{
	int i = dev->base_addr;
	free_netdev(dev);
	sllin_devs[i] = NULL;
}

static const struct net_device_ops sll_netdev_ops = {
	.ndo_open               = sll_open,
	.ndo_stop               = sll_close,
	.ndo_start_xmit         = sll_xmit,
};

static void sll_setup(struct net_device *dev)
{
	dev->netdev_ops		= &sll_netdev_ops;
	dev->destructor		= sll_free_netdev;

	dev->hard_header_len	= 0;
	dev->addr_len		= 0;
	dev->tx_queue_len	= 10;

	dev->mtu		= sizeof(struct can_frame);
	dev->type		= ARPHRD_CAN;

	/* New-style flags. */
	dev->flags		= IFF_NOARP;
	dev->features           = NETIF_F_NO_CSUM;
}

/******************************************
  Routines looking at TTY side.
 ******************************************/

static void sllin_receive_buf(struct tty_struct *tty,
			      const unsigned char *cp, char *fp, int count)
{
	struct sllin *sl = (struct sllin *) tty->disc_data;

	pr_debug("sllin: sllin_receive_buf invoked\n");

	if (!sl || sl->magic != SLLIN_MAGIC || !netif_running(sl->dev))
		return;

	/* Read the characters out of the buffer */
	while (count--) {
		if (fp && *fp++) {
			if (!test_and_set_bit(SLF_ERROR, &sl->flags))
				sl->dev->stats.rx_errors++;
			pr_debug("sllin: sllin_receive_buf char 0x%02x ignored "
				"due marker 0x%02x, flags 0x%lx\n",
				*cp, *(fp-1), sl->flags);
			cp++;
			continue;
		}

		if (sl->rx_cnt < SLLIN_BUFF_LEN) {
#ifndef BREAK_BY_BAUD
			/* We didn't receive Break character */
			if ((sl->rx_cnt == SLLIN_BUFF_BREAK) && (*cp == 0x55)) {
				sl->rx_buff[sl->rx_cnt++] = 0x00;
			}
#endif
			pr_debug("sllin: LIN_RX[%d]: 0x%02x\n", sl->rx_cnt, *cp);
			sl->rx_buff[sl->rx_cnt++] = *cp++;
		}
	}

	if (sl->rx_cnt >= sl->rx_expect) {
		set_bit(SLF_RXEVENT, &sl->flags);
		wake_up(&sl->kwt_wq);
		pr_debug("sllin: sllin_receive_buf count %d, wakeup\n", sl->rx_cnt);
	} else {
		pr_debug("sllin: sllin_receive_buf count %d, waiting\n", sl->rx_cnt);
	}
}

/*****************************************
 *  sllin message helper routines
 *****************************************/
/**
 * sllin_report_error() -- Report an error by sending CAN frame \
 * 	with particular error flag set in can_id
 *
 * @sl:
 * @err: Error flag to be sent.
 */
void sllin_report_error(struct sllin *sl, int err)
{
	sllin_send_canfr(sl, 0 | CAN_EFF_FLAG | 
		(err & ~SLLIN_ID_MASK), NULL, 0);
}

/**
 * sllin_configure_frame_cache() -- Configure particular entry in linfr_cache
 *
 * @sl:
 * @cf: Pointer to CAN frame sent to this driver
 *	holding configuration information
 */
static int sllin_configure_frame_cache(struct sllin *sl, struct can_frame *cf)
{
	struct sllin_conf_entry *sce;
	if (!(cf->can_id & SLLIN_LIN_ID_CONF))
		return -1;

	sce = &sl->linfr_cache[cf->can_id & SLLIN_ID_MASK];
	pr_debug("sllin: Setting frame cache with EFF CAN frame. "
		"LIN ID = %d\n", cf->can_id & SLLIN_ID_MASK);

	sce->dlc = cf->can_dlc;
	if (sce->dlc > SLLIN_DATA_MAX)
		sce->dlc = SLLIN_DATA_MAX;

	sce->frame_fl = (cf->can_id & ~SLLIN_ID_MASK) & CAN_EFF_MASK;
	memcpy(sce->data, cf->data, cf->can_dlc);

	return 0;
}

/**
 * sllin_checksum() -- Count checksum for particular data
 * 
 * @data: 	 Pointer to the buffer containing whole LIN
 *		 frame (i.e. including break and sync bytes).
 * @length: 	 Length of the buffer.
 * @enhanced_fl: Flag determining whether Enhanced or Classic
 *		 checksum should be counted.
 */
static inline unsigned sllin_checksum(unsigned char *data, int length, int enhanced_fl)
{
	unsigned csum = 0;
	int i;

	if (enhanced_fl) {
		i = SLLIN_BUFF_ID;
	} else {
		i = SLLIN_BUFF_DATA;
	}
	
	for (; i < length; i++) {
		csum += data[i];
		if (csum > 255)
			csum -= 255;
	}

	return ~csum & 0xff;
}

#define SLLIN_STPMSG_RESPONLY		(1) /* Message will be LIN Response only */
#define SLLIN_STPMSG_CHCKSUM_CLS	(1 << 1)
#define SLLIN_STPMSG_CHCKSUM_ENH	(1 << 2)

int sllin_setup_msg(struct sllin *sl, int mode, int id,
		unsigned char *data, int len)
{
	if (id > SLLIN_ID_MASK)
		return -1;

	if (!(mode & SLLIN_STPMSG_RESPONLY)) {
		sl->rx_cnt = 0;
		sl->tx_cnt = 0;
		sl->rx_expect = 0;
		sl->rx_lim = SLLIN_BUFF_LEN;
	}

	sl->tx_buff[SLLIN_BUFF_BREAK] = 0;
	sl->tx_buff[SLLIN_BUFF_SYNC]  = 0x55;
	sl->tx_buff[SLLIN_BUFF_ID]    = id | sllin_id_parity_table[id];
	sl->tx_lim = SLLIN_BUFF_DATA;

	if ((data != NULL) && len) {
		sl->tx_lim += len;
		memcpy(sl->tx_buff + SLLIN_BUFF_DATA, data, len);
		sl->tx_buff[sl->tx_lim++] = sllin_checksum(sl->tx_buff,
				sl->tx_lim, mode & SLLIN_STPMSG_CHCKSUM_ENH);
	}
	if (len != 0)
		sl->rx_lim = SLLIN_BUFF_DATA + len + 1;

	return 0;
}


int sllin_send_tx_buff(struct sllin *sl)
{
	struct tty_struct *tty = sl->tty;
	int remains;
	int res;

#ifdef BREAK_BY_BAUD
	if (sl->lin_state != SLSTATE_BREAK_SENT)
		remains = sl->tx_lim - sl->tx_cnt;
	else
		remains = 1;
#else
	remains = sl->tx_lim - sl->tx_cnt;
#endif

	res = tty->ops->write(tty, sl->tx_buff + sl->tx_cnt, remains);
	if (res < 0)
		return -1;

	remains -= res;
	sl->tx_cnt += res;

	if (remains > 0) {
		set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
		res = tty->ops->write(tty, sl->tx_buff + sl->tx_cnt, remains);
		if (res < 0) {
			clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
			return -1;
		}
		
		remains -= res;
		sl->tx_cnt += res;
	}

	pr_debug("sllin: sllin_send_tx_buff sent %d, remains %d\n",
			sl->tx_cnt, remains);

	return 0;
}

#ifdef BREAK_BY_BAUD
int sllin_send_break(struct sllin *sl)
{
	struct tty_struct *tty = sl->tty;
	unsigned long break_baud;
	int res;

	break_baud = ((sl->lin_baud * 2) / 3);
	sltty_change_speed(tty, break_baud);

	tty->ops->flush_buffer(tty);
	sl->rx_cnt = SLLIN_BUFF_BREAK;

	sl->rx_expect = SLLIN_BUFF_BREAK + 1;
	sl->lin_state = SLSTATE_BREAK_SENT;

	res = sllin_send_tx_buff(sl);
	if (res < 0) {
		sl->lin_state = SLSTATE_IDLE;
		return res;
	}

	return 0;
}
#else /* BREAK_BY_BAUD */

int sllin_send_break(struct sllin *sl)
{
	struct tty_struct *tty = sl->tty;
	int retval;
	unsigned long break_baud;
	unsigned long usleep_range_min;
	unsigned long usleep_range_max;

	break_baud = ((sl->lin_baud * 2) / 3);
	sl->rx_cnt = SLLIN_BUFF_BREAK;
	sl->rx_expect = SLLIN_BUFF_BREAK + 1;
	sl->lin_state = SLSTATE_BREAK_SENT;

	/* Do the break ourselves; Inspired by 
	   http://lxr.linux.no/#linux+v3.1.2/drivers/tty/tty_io.c#L2452 */
	retval = tty->ops->break_ctl(tty, -1);
	if (retval)
		return retval;

	//udelay(712);
	usleep_range_min = (1000000l * SLLIN_SAMPLES_PER_CHAR) / break_baud;
	usleep_range_max = usleep_range_min + 50;
	usleep_range(usleep_range_min, usleep_range_max);

	retval = tty->ops->break_ctl(tty, 0);
	usleep_range_min = (1000000l * 1 /* 1 bit */) / break_baud;
	usleep_range_max = usleep_range_min + 30;
	usleep_range(usleep_range_min, usleep_range_max);
	
	tty->ops->flush_buffer(tty);

	sl->tx_cnt = SLLIN_BUFF_SYNC;

	pr_debug("sllin: Break sent.\n");
	set_bit(SLF_RXEVENT, &sl->flags);
	wake_up(&sl->kwt_wq);

	return 0;
}
#endif /* BREAK_BY_BAUD */


static enum hrtimer_restart sllin_rx_timeout_handler(struct hrtimer *hrtimer)
{
	struct sllin *sl = container_of(hrtimer, struct sllin, rx_timer);

	sllin_report_error(sl, SLLIN_ERR_RX_TIMEOUT);
	set_bit(SLF_TMOUTEVENT, &sl->flags);
	wake_up(&sl->kwt_wq);

	return HRTIMER_NORESTART;
}

/**
 * sllin_rx_validate() -- Validate received frame, i,e. check checksum
 *
 * @sl:
 */
static int sllin_rx_validate(struct sllin *sl)
{
	int actual_id;
	int ext_chcks_fl;
	int lin_dlc;
	unsigned char rec_chcksm = sl->rx_buff[sl->rx_cnt - 1];
	struct sllin_conf_entry *scf;	

	actual_id = sl->rx_buff[SLLIN_BUFF_ID] & SLLIN_ID_MASK; 
	scf = &sl->linfr_cache[actual_id];
	lin_dlc = scf->dlc;
	ext_chcks_fl = scf->frame_fl & SLLIN_CHECKSUM_EXTENDED;

	if (sllin_checksum(sl->rx_buff, sl->rx_cnt - 1, ext_chcks_fl) != 
		rec_chcksm) {

		/* Type of checksum is configured for particular frame */
		if (lin_dlc > 0) {
			return -1;
		} else {
			if (sllin_checksum(sl->rx_buff,	sl->rx_cnt - 1,
				!ext_chcks_fl) != rec_chcksm) {
				return -1;
			}
		}
	}

	return 0;
}

/*****************************************
 *  sllin_kwthread - kernel worker thread
 *****************************************/

int sllin_kwthread(void *ptr)
{
	struct sllin *sl = (struct sllin *)ptr;
	struct tty_struct *tty = sl->tty;
	struct sched_param schparam = { .sched_priority = 40 };
	int tx_bytes = 0; /* Used for Network statistics */


	pr_debug("sllin: sllin_kwthread started.\n");
	sched_setscheduler(current, SCHED_FIFO, &schparam);

	clear_bit(SLF_ERROR, &sl->flags);
	sltty_change_speed(tty, sl->lin_baud);

	while (!kthread_should_stop()) {
		struct can_frame *cf;
		u8 *lin_data;
		int lin_dlc;
		u8 lin_data_buff[SLLIN_DATA_MAX];


		if ((sl->lin_state == SLSTATE_IDLE) && sl->lin_master &&
			sl->id_to_send) {
			if(sllin_send_break(sl) < 0) {
				/* error processing */
			}
		}

		wait_event_killable(sl->kwt_wq, kthread_should_stop() ||
			test_bit(SLF_RXEVENT, &sl->flags) ||
			test_bit(SLF_TXEVENT, &sl->flags) ||
			test_bit(SLF_TMOUTEVENT, &sl->flags) ||
			(((sl->lin_state == SLSTATE_IDLE) || 
				(sl->lin_state == SLSTATE_RESPONSE_WAIT)) 
				&& test_bit(SLF_MSGEVENT, &sl->flags)));

		if (test_and_clear_bit(SLF_RXEVENT, &sl->flags)) {
			pr_debug("sllin: sllin_kthread RXEVENT \n");
		}

		if (test_and_clear_bit(SLF_TXEVENT, &sl->flags)) {
			pr_debug("sllin: sllin_kthread TXEVENT \n");
		}

		if (test_and_clear_bit(SLF_TMOUTEVENT, &sl->flags)) {
			pr_debug("sllin: sllin_kthread TMOUTEVENT \n");
			sl->rx_cnt = 0;
			sl->rx_expect = 0;
			sl->rx_lim = sl->lin_master ? 0 : SLLIN_BUFF_LEN;
			sl->tx_cnt = 0;
			sl->tx_lim = 0;
			sl->id_to_send = false;
			sl->data_to_send = false;
			
			sl->lin_state = SLSTATE_IDLE;
		}

		switch (sl->lin_state) {
			case SLSTATE_IDLE:
				if (!test_bit(SLF_MSGEVENT, &sl->flags))
					break;

				cf = (struct can_frame *)sl->tx_req_skb->data;

				/* SFF RTR CAN frame -> LIN header */ 
				if (cf->can_id & CAN_RTR_FLAG) {
					spin_lock(&sl->lock);
					pr_debug("sllin: %s: RTR SFF CAN frame, ID = %x\n",
						__FUNCTION__, cf->can_id & SLLIN_ID_MASK);

					/* Is there Slave response in linfr_cache to be sent? */
					if ((sl->linfr_cache[cf->can_id & SLLIN_ID_MASK].frame_fl & 
						SLLIN_LOC_SLAVE_CACHE) 
						&& (sl->linfr_cache[cf->can_id & SLLIN_ID_MASK].dlc > 0)) {
						
						pr_debug("sllin: Sending LIN response from linfr_cache\n");
						lin_data = sl->linfr_cache[cf->can_id & SLLIN_ID_MASK].data;
						lin_dlc = sl->linfr_cache[cf->can_id & SLLIN_ID_MASK].dlc;
						if (lin_dlc > SLLIN_DATA_MAX)
							lin_dlc = SLLIN_DATA_MAX;
						memcpy(lin_data_buff, lin_data, lin_dlc);
						lin_data = lin_data_buff;
					} else {
						lin_data = NULL;
						lin_dlc = sl->linfr_cache[cf->can_id & SLLIN_ID_MASK].dlc;
					}
					spin_unlock(&sl->lock);
				} else { /* SFF NON-RTR CAN frame -> LIN header + LIN response */
					pr_debug("sllin: %s: NON-RTR SFF CAN frame, ID = %x\n",
						__FUNCTION__, (int)cf->can_id & SLLIN_ID_MASK);

					lin_data = cf->data;
					lin_dlc = cf->can_dlc;
					if (lin_dlc > SLLIN_DATA_MAX)
						lin_dlc = SLLIN_DATA_MAX;
					tx_bytes = lin_dlc;
				}

				if (sllin_setup_msg(sl, 0, cf->can_id & SLLIN_ID_MASK,
					lin_data, lin_dlc) != -1) {

					sl->id_to_send = true;
					sl->data_to_send = (lin_data != NULL) ? true : false;
					sl->resp_len_known = (lin_dlc > 0) ? true : false;
					sl->dev->stats.tx_packets++;
					sl->dev->stats.tx_bytes += tx_bytes;
				}

				clear_bit(SLF_MSGEVENT, &sl->flags);
				kfree_skb(sl->tx_req_skb);
				netif_wake_queue(sl->dev);
				break;

			case SLSTATE_BREAK_SENT:
#ifdef BREAK_BY_BAUD
				if (sl->rx_cnt <= SLLIN_BUFF_BREAK)
					continue;

				res = sltty_change_speed(tty, sl->lin_baud);
#endif

				sl->lin_state = SLSTATE_ID_SENT;
				sllin_send_tx_buff(sl);
				break;

			case SLSTATE_ID_SENT:
				sl->id_to_send = false;
				if (sl->data_to_send) {
					sllin_send_tx_buff(sl);
					sl->lin_state = SLSTATE_RESPONSE_SENT;
					sl->rx_expect = sl->tx_lim;
					goto slstate_response_sent;
				} else {
					if (sl->resp_len_known) {
						sl->rx_expect = sl->rx_lim;
					} else {
						sl->rx_expect = SLLIN_BUFF_DATA + 2;
					}
					sl->lin_state = SLSTATE_RESPONSE_WAIT;
					/* If we don't receive anything, timer will "unblock" us */
					hrtimer_start(&sl->rx_timer, 
						ktime_add(ktime_get(), sl->rx_timer_timeout),
						HRTIMER_MODE_ABS);
					goto slstate_response_wait;
				}
				break;

			case SLSTATE_RESPONSE_WAIT:
			slstate_response_wait:
				if (test_bit(SLF_MSGEVENT, &sl->flags)) {
					unsigned char *lin_buff;
					cf = (struct can_frame *)sl->tx_req_skb->data;

					lin_buff = (sl->lin_master) ? sl->tx_buff : sl->rx_buff;
					if (cf->can_id == (lin_buff[SLLIN_BUFF_ID] & SLLIN_ID_MASK)) {
						if (sllin_setup_msg(sl, SLLIN_STPMSG_RESPONLY, 
							cf->can_id & SLLIN_ID_MASK,
							cf->data, cf->can_dlc) != -1) {

							sl->rx_expect = sl->tx_lim;
							sl->data_to_send = true;
							sl->dev->stats.tx_packets++;
							sl->dev->stats.tx_bytes += tx_bytes;

							if (!sl->lin_master) {
								sl->tx_cnt = SLLIN_BUFF_DATA;
							}
							
							sllin_send_tx_buff(sl);
							clear_bit(SLF_MSGEVENT, &sl->flags);
							kfree_skb(sl->tx_req_skb);
							netif_wake_queue(sl->dev);

							sl->lin_state = SLSTATE_RESPONSE_SENT;
							goto slstate_response_sent;
						}
					} else {
						sl->lin_state = SLSTATE_RESPONSE_WAIT_BUS;
					}
				}

			case SLSTATE_RESPONSE_WAIT_BUS:
				if (sl->rx_cnt < sl->rx_expect)
					continue;
			
				hrtimer_cancel(&sl->rx_timer);
				pr_debug("sllin: response received ID %d len %d\n",
					sl->rx_buff[SLLIN_BUFF_ID], sl->rx_cnt - SLLIN_BUFF_DATA - 1);

				if (sllin_rx_validate(sl) == -1) {
					pr_debug("sllin: RX validation failed.\n");
					sllin_report_error(sl, SLLIN_ERR_CHECKSUM);
					//FIXME tx_stat.err++
				} else {
					// send CAN non-RTR frame with data
					pr_debug("sllin: sending NON-RTR CAN"
						"frame with LIN payload.");
					sll_bump(sl); //send packet to the network layer
				}

				sl->id_to_send = false;
				sl->lin_state = SLSTATE_IDLE;
				break;
			
			case SLSTATE_RESPONSE_SENT:
			slstate_response_sent:
				if (sl->rx_cnt < sl->tx_lim)
					continue;
				
				sll_bump(sl); //send packet to the network layer
				pr_debug("sllin: response sent ID %d len %d\n",
					sl->rx_buff[SLLIN_BUFF_ID], sl->rx_cnt - SLLIN_BUFF_DATA - 1);

				sl->id_to_send = false;
				sl->lin_state = SLSTATE_IDLE;
				break;
		}




		/* sl->dev->stats.tx_packets++; send frames statistic */
		/* netif_wake_queue(sl->dev); allow next Tx packet arrival */
	}

	hrtimer_cancel(&sl->rx_timer);
	pr_debug("sllin: sllin_kwthread stopped.\n");

	return 0;
}


/************************************
 *  sllin_open helper routines.
 ************************************/

/* Collect hanged up channels */
static void sll_sync(void)
{
	int i;
	struct net_device *dev;
	struct sllin	  *sl;

	for (i = 0; i < maxdev; i++) {
		dev = sllin_devs[i];
		if (dev == NULL)
			break;

		sl = netdev_priv(dev);
		if (sl->tty)
			continue;
		if (dev->flags & IFF_UP)
			dev_close(dev);
	}
}

/* Find a free SLLIN channel, and link in this `tty' line. */
static struct sllin *sll_alloc(dev_t line)
{
	int i;
	struct net_device *dev = NULL;
	struct sllin       *sl;

	if (sllin_devs == NULL)
		return NULL;	/* Master array missing ! */

	for (i = 0; i < maxdev; i++) {
		dev = sllin_devs[i];
		if (dev == NULL)
			break;

	}

	/* Sorry, too many, all slots in use */
	if (i >= maxdev)
		return NULL;

	if (dev) {
		sl = netdev_priv(dev);
		if (test_bit(SLF_INUSE, &sl->flags)) {
			unregister_netdevice(dev);
			dev = NULL;
			sllin_devs[i] = NULL;
		}
	}

	if (!dev) {
		char name[IFNAMSIZ];
		sprintf(name, "sllin%d", i);

		dev = alloc_netdev(sizeof(*sl), name, sll_setup);
		if (!dev)
			return NULL;
		dev->base_addr  = i;
	}

	sl = netdev_priv(dev);
	/* Initialize channel control data */
	sl->magic = SLLIN_MAGIC;
	sl->dev	= dev;
	spin_lock_init(&sl->lock);
	sllin_devs[i] = dev;

	return sl;
}

/*
 * Open the high-level part of the SLLIN channel.
 * This function is called by the TTY module when the
 * SLLIN line discipline is called for.  Because we are
 * sure the tty line exists, we only have to link it to
 * a free SLLIN channel...
 *
 * Called in process context serialized from other ldisc calls.
 */

static int sllin_open(struct tty_struct *tty)
{
	struct sllin *sl;
	int err;
	pr_debug("sllin: %s() invoked\n", __FUNCTION__);

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (tty->ops->write == NULL)
		return -EOPNOTSUPP;

	/* RTnetlink lock is misused here to serialize concurrent
	   opens of sllin channels. There are better ways, but it is
	   the simplest one.
	 */
	rtnl_lock();

	/* Collect hanged up channels. */
	sll_sync();

	sl = tty->disc_data;

	err = -EEXIST;
	/* First make sure we're not already connected. */
	if (sl && sl->magic == SLLIN_MAGIC)
		goto err_exit;

	/* OK.  Find a free SLLIN channel to use. */
	err = -ENFILE;
	sl = sll_alloc(tty_devnum(tty));
	if (sl == NULL)
		goto err_exit;

	sl->tty = tty;
	tty->disc_data = sl;
	sl->line = tty_devnum(tty);

	if (!test_bit(SLF_INUSE, &sl->flags)) {
		/* Perform the low-level SLLIN initialization. */
		sl->lin_master = true;

		sl->rx_cnt = 0;
		sl->rx_expect = 0;
		sl->rx_lim = sl->lin_master ? 0 : SLLIN_BUFF_LEN;
		sl->tx_cnt = 0;
		sl->tx_lim = 0;
		sl->id_to_send = false;
		sl->data_to_send = false;

		sl->lin_baud  = 19200;

		sl->lin_state = SLSTATE_IDLE;

		hrtimer_init(&sl->rx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		sl->rx_timer.function = sllin_rx_timeout_handler;
		/* timeval_to_ktime(msg_head->ival1); */
		sl->rx_timer_timeout = ns_to_ktime(
			(1000000000l / sl->lin_baud) * 
			SLLIN_SAMPLES_PER_CHAR * SLLIN_CHARS_TO_TIMEOUT); 
 
		set_bit(SLF_INUSE, &sl->flags);

		init_waitqueue_head(&sl->kwt_wq);
		sl->kwthread = kthread_run(sllin_kwthread, sl, "sllin");
		if (sl->kwthread == NULL)
			goto err_free_chan;

		err = register_netdevice(sl->dev);
		if (err)
			goto err_free_chan_and_thread;
	}

	/* Done.  We have linked the TTY line to a channel. */
	rtnl_unlock();
	tty->receive_room = SLLIN_BUFF_LEN * 40;	/* We don't flow control */

	/* TTY layer expects 0 on success */
	return 0;

err_free_chan_and_thread:
	kthread_stop(sl->kwthread);
	sl->kwthread = NULL;

err_free_chan:
	sl->tty = NULL;
	tty->disc_data = NULL;
	clear_bit(SLF_INUSE, &sl->flags);

err_exit:
	rtnl_unlock();

	/* Count references from TTY module */
	return err;
}

/*
 * Close down a SLLIN channel.
 * This means flushing out any pending queues, and then returning. This
 * call is serialized against other ldisc functions.
 *
 * We also use this method for a hangup event.
 */

static void sllin_close(struct tty_struct *tty)
{
	struct sllin *sl = (struct sllin *) tty->disc_data;

	/* First make sure we're connected. */
	if (!sl || sl->magic != SLLIN_MAGIC || sl->tty != tty)
		return;

	kthread_stop(sl->kwthread);
	sl->kwthread = NULL;

	tty->disc_data = NULL;
	sl->tty = NULL;

	/* Flush network side */
	unregister_netdev(sl->dev);
	/* This will complete via sl_free_netdev */
}

static int sllin_hangup(struct tty_struct *tty)
{
	sllin_close(tty);
	return 0;
}

/* Perform I/O control on an active SLLIN channel. */
static int sllin_ioctl(struct tty_struct *tty, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	struct sllin *sl = (struct sllin *) tty->disc_data;
	unsigned int tmp;

	/* First make sure we're connected. */
	if (!sl || sl->magic != SLLIN_MAGIC)
		return -EINVAL;

	switch (cmd) {
	case SIOCGIFNAME:
		tmp = strlen(sl->dev->name) + 1;
		if (copy_to_user((void __user *)arg, sl->dev->name, tmp))
			return -EFAULT;
		return 0;

	case SIOCSIFHWADDR:
		return -EINVAL;

	default:
		return tty_mode_ioctl(tty, file, cmd, arg);
	}
}

static struct tty_ldisc_ops sll_ldisc = {
	.owner		= THIS_MODULE,
	.magic		= TTY_LDISC_MAGIC,
	.name		= "sllin",
	.open		= sllin_open,
	.close		= sllin_close,
	.hangup		= sllin_hangup,
	.ioctl		= sllin_ioctl,
	.receive_buf	= sllin_receive_buf,
	.write_wakeup	= sllin_write_wakeup,
};

static int __init sllin_init(void)
{
	int status;

	if (maxdev < 4)
		maxdev = 4; /* Sanity */

	printk(banner);
	pr_debug("sllin: %d dynamic interface channels.\n", maxdev);

	sllin_devs = kzalloc(sizeof(struct net_device *)*maxdev, GFP_KERNEL);
	if (!sllin_devs) {
		printk(KERN_ERR "sllin: can't allocate sllin device array!\n");
		return -ENOMEM;
	}

	/* Fill in our line protocol discipline, and register it */
	status = tty_register_ldisc(N_SLLIN, &sll_ldisc);
	if (status)  {
		printk(KERN_ERR "sllin: can't register line discipline\n");
		kfree(sllin_devs);
	}

#ifdef BREAK_BY_BAUD
	pr_debug("sllin: Break is generated by baud-rate change.");
#else
	pr_debug("sllin: Break is generated manually with tiny sleep.");
#endif

	return status;
}

static void __exit sllin_exit(void)
{
	int i;
	struct net_device *dev;
	struct sllin *sl;
	unsigned long timeout = jiffies + HZ;
	int busy = 0;

	if (sllin_devs == NULL)
		return;

	/* First of all: check for active disciplines and hangup them.
	 */
	do {
		if (busy)
			msleep_interruptible(100);

		busy = 0;
		for (i = 0; i < maxdev; i++) {
			dev = sllin_devs[i];
			if (!dev)
				continue;
			sl = netdev_priv(dev);
			spin_lock_bh(&sl->lock);
			if (sl->tty) {
				busy++;
				tty_hangup(sl->tty);
			}
			spin_unlock_bh(&sl->lock);
		}
	} while (busy && time_before(jiffies, timeout));

	/* FIXME: hangup is async so we should wait when doing this second
	   phase */

	for (i = 0; i < maxdev; i++) {
		dev = sllin_devs[i];
		if (!dev)
			continue;
		sllin_devs[i] = NULL;

		sl = netdev_priv(dev);
		if (sl->tty) {
			printk(KERN_ERR "%s: tty discipline still running\n",
			       dev->name);
			/* Intentionally leak the control block. */
			dev->destructor = NULL;
		}

		unregister_netdev(dev);
	}

	kfree(sllin_devs);
	sllin_devs = NULL;

	i = tty_unregister_ldisc(N_SLLIN);
	if (i)
		printk(KERN_ERR "sllin: can't unregister ldisc (err %d)\n", i);
}

module_init(sllin_init);
module_exit(sllin_exit);
