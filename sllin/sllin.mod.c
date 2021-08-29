#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x4766f3ab, "module_layout" },
	{ 0x69a4d9e9, "param_ops_bool" },
	{ 0xadee4c82, "param_ops_int" },
	{ 0xa120d33c, "tty_unregister_ldisc" },
	{ 0xcc5005fe, "msleep_interruptible" },
	{ 0x31dfa005, "tty_hangup" },
	{ 0x526c3a6c, "jiffies" },
	{ 0x37a0cba, "kfree" },
	{ 0xc79dc49, "tty_register_ldisc" },
	{ 0x2d6fcc06, "__kmalloc" },
	{ 0x6e720ff2, "rtnl_unlock" },
	{ 0xcd4753e5, "register_netdevice" },
	{ 0x56653629, "wake_up_process" },
	{ 0x9d67ded2, "kthread_create_on_node" },
	{ 0x5bbe49f4, "__init_waitqueue_head" },
	{ 0xa362bf8f, "hrtimer_init" },
	{ 0x642d79b7, "alloc_netdev_mqs" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0xf6f93543, "tty_devnum" },
	{ 0xd22a9f00, "dev_close" },
	{ 0xc7a4fbed, "rtnl_lock" },
	{ 0xc6cbbc89, "capable" },
	{ 0xc5850110, "printk" },
	{ 0xf1e8c0ee, "netdev_warn" },
	{ 0xae577d60, "_raw_spin_lock" },
	{ 0xec523f88, "hrtimer_start_range_ns" },
	{ 0xb43f9365, "ktime_get" },
	{ 0x641f7f33, "netif_tx_wake_queue" },
	{ 0x9870b5cd, "kfree_skb" },
	{ 0x695bf5e9, "hrtimer_cancel" },
	{ 0x2a3aa678, "_test_and_clear_bit" },
	{ 0x49970de8, "finish_wait" },
	{ 0x1000e51, "schedule" },
	{ 0x647af474, "prepare_to_wait_event" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0xa1c76e0a, "_cond_resched" },
	{ 0x12a38747, "usleep_range" },
	{ 0xb3f7646e, "kthread_should_stop" },
	{ 0x91e258b0, "sched_set_fifo" },
	{ 0x2196324, "__aeabi_idiv" },
	{ 0xe707d823, "__aeabi_uidiv" },
	{ 0x7e0ce0c3, "up_write" },
	{ 0x1584781c, "tty_encode_baud_rate" },
	{ 0x4253aa7e, "down_write" },
	{ 0x308d59ce, "unregister_netdev" },
	{ 0xfc5485d0, "kthread_stop" },
	{ 0x51a910c0, "arm_copy_to_user" },
	{ 0x97255bdf, "strlen" },
	{ 0xcea24e46, "tty_mode_ioctl" },
	{ 0x86332725, "__stack_chk_fail" },
	{ 0x5bdfd9c8, "netif_rx" },
	{ 0x13a660c6, "skb_put" },
	{ 0xa085d65b, "__netdev_alloc_skb" },
	{ 0x8f678b07, "__stack_chk_guard" },
	{ 0x9d669763, "memcpy" },
	{ 0x3dcf1ffa, "__wake_up" },
	{ 0x6230aaca, "netdev_printk" },
	{ 0xca54fee, "_test_and_set_bit" },
	{ 0x3ec80fa0, "_raw_spin_unlock_bh" },
	{ 0x676bbc0f, "_set_bit" },
	{ 0x49ebacbd, "_clear_bit" },
	{ 0xe7e4d52a, "_raw_spin_lock_bh" },
	{ 0xf3d0b495, "_raw_spin_unlock_irqrestore" },
	{ 0xde55e795, "_raw_spin_lock_irqsave" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "D0BAFD2DF04073478DABBC0");
