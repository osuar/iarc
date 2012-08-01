#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
 .name = KBUILD_MODNAME,
 .init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
 .exit = cleanup_module,
#endif
 .arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xf6628fc9, "module_layout" },
	{ 0x87d069b9, "device_remove_file" },
	{ 0x53822150, "kmalloc_caches" },
	{ 0x3ec8886f, "param_ops_int" },
	{ 0xdb97fba0, "dev_set_drvdata" },
	{ 0xcf7df8a8, "snd_pcm_period_elapsed" },
	{ 0xc8b57c27, "autoremove_wake_function" },
	{ 0x1b36e933, "snd_card_create" },
	{ 0x17e64e93, "video_device_release" },
	{ 0xff965a13, "v4l2_device_unregister" },
	{ 0x47939e0d, "__tasklet_hi_schedule" },
	{ 0x6efb6547, "usb_kill_urb" },
	{ 0x206b57f0, "__video_register_device" },
	{ 0xb78c61e8, "param_ops_bool" },
	{ 0x59f300eb, "mutex_unlock" },
	{ 0x999e8297, "vfree" },
	{ 0x91715312, "sprintf" },
	{ 0x67940bc4, "snd_pcm_hw_constraint_integer" },
	{ 0x959160a0, "v4l2_device_register" },
	{ 0x6395be94, "__init_waitqueue_head" },
	{ 0x4f8b5ddb, "_copy_to_user" },
	{ 0xbfb3f6f1, "v4l2_device_disconnect" },
	{ 0xde0bdcff, "memset" },
	{ 0xcc7a06b5, "video_device_alloc" },
	{ 0xde1c9b0e, "dev_err" },
	{ 0x8f64aa4, "_raw_spin_unlock_irqrestore" },
	{ 0xf5d0b8cb, "current_task" },
	{ 0x3ac8c215, "usb_deregister" },
	{ 0x1b9ecedf, "__mutex_init" },
	{ 0x27e1a049, "printk" },
	{ 0xbed7a2d, "video_unregister_device" },
	{ 0xd06cbc9c, "snd_pcm_set_ops" },
	{ 0xfd1e3796, "usb_set_interface" },
	{ 0xa1c76e0a, "_cond_resched" },
	{ 0xb4390f9a, "mcount" },
	{ 0xd6dbe3b3, "usb_control_msg" },
	{ 0x672144bd, "strlcpy" },
	{ 0x868e2bb, "mutex_lock" },
	{ 0x471ef3a, "snd_pcm_lib_free_pages" },
	{ 0x9545af6d, "tasklet_init" },
	{ 0x249faad1, "snd_pcm_lib_ioctl" },
	{ 0x288fd9c4, "usb_free_coherent" },
	{ 0xccda8fb7, "vm_insert_page" },
	{ 0xf6babe1a, "snd_pcm_lib_malloc_pages" },
	{ 0x82072614, "tasklet_kill" },
	{ 0x9051bce3, "device_create_file" },
	{ 0xdc9f2ef7, "usb_submit_urb" },
	{ 0x2776c39f, "usb_get_dev" },
	{ 0x782e38db, "video_devdata" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0xed6547af, "usb_put_dev" },
	{ 0x1000e51, "schedule" },
	{ 0xa0b04675, "vmalloc_32" },
	{ 0x4a21112a, "kmem_cache_alloc_trace" },
	{ 0x9327f5ce, "_raw_spin_lock_irqsave" },
	{ 0xcf21d241, "__wake_up" },
	{ 0x1d2e87c6, "do_gettimeofday" },
	{ 0x37a0cba, "kfree" },
	{ 0x236c8c64, "memcpy" },
	{ 0x5c8b5ce8, "prepare_to_wait" },
	{ 0x6128b5fc, "__printk_ratelimit" },
	{ 0x63afefa5, "usb_register_driver" },
	{ 0xef7c6172, "request_firmware" },
	{ 0xfa66f77c, "finish_wait" },
	{ 0x7f38608d, "snd_pcm_lib_preallocate_pages_for_all" },
	{ 0x9e3bd8cc, "snd_card_free" },
	{ 0x7a46de5e, "snd_card_register" },
	{ 0x50720c5f, "snprintf" },
	{ 0x8ec7ae4a, "snd_pcm_new" },
	{ 0xb008c80b, "usb_alloc_coherent" },
	{ 0xef3aca51, "vmalloc_to_page" },
	{ 0xb2798ef9, "dev_get_drvdata" },
	{ 0x459459ad, "usb_free_urb" },
	{ 0xeb433df8, "video_ioctl2" },
	{ 0xf3251e7b, "v4l2_norm_to_name" },
	{ 0x76e9dff4, "usb_alloc_urb" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=snd-pcm,snd,videodev";

MODULE_ALIAS("usb:v1C88p0007d*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v1C88p003Cd*dc*dsc*dp*ic*isc*ip*");

MODULE_INFO(srcversion, "179EDE8B82DB90242131A57");
