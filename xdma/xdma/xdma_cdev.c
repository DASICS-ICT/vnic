/*
 * This file is part of the Xilinx DMA IP Core driver for Linux
 *
 * Copyright (c) 2016-present,  Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */

#define pr_fmt(fmt)     KBUILD_MODNAME ":%s: " fmt, __func__

#include "xdma_cdev.h"

static struct class *g_xdma_class;

struct kmem_cache *cdev_cache;

enum cdev_type {
	CHAR_USER,
	CHAR_CTRL,
	CHAR_XVC,
	CHAR_EVENTS,
	CHAR_XDMA_H2C,
	CHAR_XDMA_C2H,
	CHAR_BYPASS_H2C,
	CHAR_BYPASS_C2H,
	CHAR_BYPASS,
};

static const char * const devnode_names[] = {
	XDMA_NODE_NAME "%d_user",
	XDMA_NODE_NAME "%d_control",
	XDMA_NODE_NAME "%d_xvc",
	XDMA_NODE_NAME "%d_events_%d",
	XDMA_NODE_NAME "%d_h2c_%d",
	XDMA_NODE_NAME "%d_c2h_%d",
	XDMA_NODE_NAME "%d_bypass_h2c_%d",
	XDMA_NODE_NAME "%d_bypass_c2h_%d",
	XDMA_NODE_NAME "%d_bypass",
};

enum xpdev_flags_bits {
	XDF_CDEV_USER,
	XDF_CDEV_CTRL,
	XDF_CDEV_XVC,
	XDF_CDEV_EVENT,
	XDF_CDEV_SG,
	XDF_CDEV_BYPASS,
};

static inline void xpdev_flag_set(struct xdma_pci_dev *xpdev,
				enum xpdev_flags_bits fbit)
{
	xpdev->flags |= 1 << fbit;
}

static inline void xcdev_flag_clear(struct xdma_pci_dev *xpdev,
				enum xpdev_flags_bits fbit)
{
	xpdev->flags &= ~(1 << fbit);
}

static inline int xpdev_flag_test(struct xdma_pci_dev *xpdev,
				enum xpdev_flags_bits fbit)
{
	return xpdev->flags & (1 << fbit);
}

#ifdef __XDMA_SYSFS__
ssize_t xdma_dev_instance_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct xdma_pci_dev *xpdev =
		(struct xdma_pci_dev *)dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\t%d\n",
			xpdev->major, xpdev->xdev->idx);
}

static DEVICE_ATTR_RO(xdma_dev_instance);
#endif

static int config_kobject(struct xdma_cdev *xcdev, enum cdev_type type)
{
	int rv = -EINVAL;
	struct xdma_dev *xdev = xcdev->xdev;
	struct xdma_engine *engine = xcdev->engine;

	switch (type) {
	case CHAR_XDMA_H2C:
	case CHAR_XDMA_C2H:
	case CHAR_BYPASS_H2C:
	case CHAR_BYPASS_C2H:
		if (!engine) {
			pr_err("Invalid DMA engine\n");
			return rv;
		}
		rv = kobject_set_name(&xcdev->cdev.kobj, devnode_names[type],
			xdev->idx, engine->channel);
		break;
	case CHAR_BYPASS:
	case CHAR_USER:
	case CHAR_CTRL:
	case CHAR_XVC:
		rv = kobject_set_name(&xcdev->cdev.kobj, devnode_names[type],
			xdev->idx);
		break;
	case CHAR_EVENTS:
		rv = kobject_set_name(&xcdev->cdev.kobj, devnode_names[type],
			xdev->idx, xcdev->bar);
		break;
	default:
		pr_warn("%s: UNKNOWN type 0x%x.\n", __func__, type);
		break;
	}

	if (rv)
		pr_err("%s: type 0x%x, failed %d.\n", __func__, type, rv);
	return rv;
}

int xcdev_check(const char *fname, struct xdma_cdev *xcdev, bool check_engine)
{
	struct xdma_dev *xdev;

	if (!xcdev || xcdev->magic != MAGIC_CHAR) {
		pr_info("%s, xcdev 0x%p, magic 0x%lx.\n",
			fname, xcdev, xcdev ? xcdev->magic : 0xFFFFFFFF);
		return -EINVAL;
	}

	xdev = xcdev->xdev;
	if (!xdev || xdev->magic != MAGIC_DEVICE) {
		pr_info("%s, xdev 0x%p, magic 0x%lx.\n",
			fname, xdev, xdev ? xdev->magic : 0xFFFFFFFF);
		return -EINVAL;
	}

	if (check_engine) {
		struct xdma_engine *engine = xcdev->engine;

		if (!engine || engine->magic != MAGIC_ENGINE) {
			pr_info("%s, engine 0x%p, magic 0x%lx.\n", fname,
				engine, engine ? engine->magic : 0xFFFFFFFF);
			return -EINVAL;
		}
	}

	return 0;
}

int char_open(struct inode *inode, struct file *file)
{
	struct xdma_cdev *xcdev;

	/* pointer to containing structure of the character device inode */
	xcdev = container_of(inode->i_cdev, struct xdma_cdev, cdev);
	if (xcdev->magic != MAGIC_CHAR) {
		pr_err("xcdev 0x%p inode 0x%lx magic mismatch 0x%lx\n",
			xcdev, inode->i_ino, xcdev->magic);
		return -EINVAL;
	}
	/* create a reference to our char device in the opened file */
	file->private_data = xcdev;

	return 0;
}

/*
 * Called when the device goes from used to unused.
 */
int char_close(struct inode *inode, struct file *file)
{
	struct xdma_dev *xdev;
	struct xdma_cdev *xcdev = (struct xdma_cdev *)file->private_data;

	if (!xcdev) {
		pr_err("char device with inode 0x%lx xcdev NULL\n",
			inode->i_ino);
		return -EINVAL;
	}

	if (xcdev->magic != MAGIC_CHAR) {
		pr_err("xcdev 0x%p magic mismatch 0x%lx\n",
				xcdev, xcdev->magic);
		return -EINVAL;
	}

	/* fetch device specific data stored earlier during open */
	xdev = xcdev->xdev;
	if (!xdev) {
		pr_err("char device with inode 0x%lx xdev NULL\n",
			inode->i_ino);
		return -EINVAL;
	}

	if (xdev->magic != MAGIC_DEVICE) {
		pr_err("xdev 0x%p magic mismatch 0x%lx\n", xdev, xdev->magic);
		return -EINVAL;
	}

	return 0;
}

/* create_xcdev() -- create a character device interface to data or control bus
 *
 * If at least one SG DMA engine is specified, the character device interface
 * is coupled to the SG DMA file operations which operate on the data bus. If
 * no engines are specified, the interface is coupled with the control bus.
 */

static int create_sys_device(struct xdma_cdev *xcdev, enum cdev_type type)
{
	struct xdma_dev *xdev = xcdev->xdev;
	struct xdma_engine *engine = xcdev->engine;
	int last_param;

	if (type == CHAR_EVENTS)
		last_param = xcdev->bar;
	else
		last_param = engine ? engine->channel : 0;

	xcdev->sys_device = device_create(g_xdma_class, &xdev->pdev->dev,
		xcdev->cdevno, NULL, devnode_names[type], xdev->idx,
		last_param);

	if (!xcdev->sys_device) {
		pr_err("device_create(%s) failed\n", devnode_names[type]);
		return -1;
	}

	return 0;
}

static int destroy_xcdev(struct xdma_cdev *cdev)
{
	if (!cdev) {
		pr_warn("cdev NULL.\n");
		return -EINVAL;
	}
	if (cdev->magic != MAGIC_CHAR) {
		pr_warn("cdev 0x%p magic mismatch 0x%lx\n", cdev, cdev->magic);
		return -EINVAL;
	}

	if (!cdev->xdev) {
		pr_err("xdev NULL\n");
		return -EINVAL;
	}

	if (!g_xdma_class) {
		pr_err("g_xdma_class NULL\n");
		return -EINVAL;
	}

	if (!cdev->sys_device) {
		pr_err("cdev sys_device NULL\n");
		return -EINVAL;
	}

	if (cdev->sys_device)
		device_destroy(g_xdma_class, cdev->cdevno);

	cdev_del(&cdev->cdev);

	return 0;
}

static int create_xcdev(struct xdma_pci_dev *xpdev, struct xdma_cdev *xcdev,
			int bar, struct xdma_engine *engine,
			enum cdev_type type)
{
	int rv;
	int minor;
	struct xdma_dev *xdev = xpdev->xdev;
	dev_t dev;

	spin_lock_init(&xcdev->lock);
	/* new instance? */
	if (!xpdev->major) {
		/* allocate a dynamically allocated char device node */
		int rv = alloc_chrdev_region(&dev, XDMA_MINOR_BASE,
					XDMA_MINOR_COUNT, XDMA_NODE_NAME);

		if (rv) {
			pr_err("unable to allocate cdev region %d.\n", rv);
			return rv;
		}
		xpdev->major = MAJOR(dev);
	}

	/*
	 * do not register yet, create kobjects and name them,
	 */
	xcdev->magic = MAGIC_CHAR;
	xcdev->cdev.owner = THIS_MODULE;
	xcdev->xpdev = xpdev;
	xcdev->xdev = xdev;
	xcdev->engine = engine;
	xcdev->bar = bar;

	rv = config_kobject(xcdev, type);
	if (rv < 0)
		return rv;

	switch (type) {
	case CHAR_USER:
	case CHAR_CTRL:
		/* minor number is type index for non-SGDMA interfaces */
		minor = type;
		cdev_ctrl_init(xcdev);
		break;
	case CHAR_XVC:
		/* minor number is type index for non-SGDMA interfaces */
		minor = type;
		cdev_xvc_init(xcdev);
		break;
	case CHAR_XDMA_H2C:
		minor = 32 + engine->channel;
		cdev_sgdma_init(xcdev);
		break;
	case CHAR_XDMA_C2H:
		minor = 36 + engine->channel;
		cdev_sgdma_init(xcdev);
		break;
	case CHAR_EVENTS:
		minor = 10 + bar;
		cdev_event_init(xcdev);
		break;
	case CHAR_BYPASS_H2C:
		minor = 64 + engine->channel;
		cdev_bypass_init(xcdev);
		break;
	case CHAR_BYPASS_C2H:
		minor = 68 + engine->channel;
		cdev_bypass_init(xcdev);
		break;
	case CHAR_BYPASS:
		minor = 100;
		cdev_bypass_init(xcdev);
		break;
	default:
		pr_info("type 0x%x NOT supported.\n", type);
		return -EINVAL;
	}
	xcdev->cdevno = MKDEV(xpdev->major, minor);

	/* bring character device live */
	rv = cdev_add(&xcdev->cdev, xcdev->cdevno, 1);
	if (rv < 0) {
		pr_err("cdev_add failed %d, type 0x%x.\n", rv, type);
		goto unregister_region;
	}

	dbg_init("xcdev 0x%p, %u:%u, %s, type 0x%x.\n",
		xcdev, xpdev->major, minor, xcdev->cdev.kobj.name, type);

	/* create device on our class */
	if (g_xdma_class) {
		rv = create_sys_device(xcdev, type);
		if (rv < 0)
			goto del_cdev;
	}

	return 0;

del_cdev:
	cdev_del(&xcdev->cdev);
unregister_region:
	unregister_chrdev_region(xcdev->cdevno, XDMA_MINOR_COUNT);
	return rv;
}

void xpdev_destroy_interfaces(struct xdma_pci_dev *xpdev)
{
	int i = 0;
	int rv;
#ifdef __XDMA_SYSFS__
	device_remove_file(&xpdev->pdev->dev, &dev_attr_xdma_dev_instance);
#endif

	if (xpdev_flag_test(xpdev, XDF_CDEV_SG)) {
		/* iterate over channels */
		for (i = 0; i < xpdev->h2c_channel_max; i++) {
			/* remove SG DMA character device */
			rv = destroy_xcdev(&xpdev->sgdma_h2c_cdev[i]);
			if (rv < 0)
				pr_err("Failed to destroy h2c xcdev %d error :0x%x\n",
						i, rv);
		}
		for (i = 0; i < xpdev->c2h_channel_max; i++) {
			rv = destroy_xcdev(&xpdev->sgdma_c2h_cdev[i]);
			if (rv < 0)
				pr_err("Failed to destroy c2h xcdev %d error 0x%x\n",
						i, rv);
		}
	}

	if (xpdev_flag_test(xpdev, XDF_CDEV_EVENT)) {
		for (i = 0; i < xpdev->user_max; i++) {
			rv = destroy_xcdev(&xpdev->events_cdev[i]);
			if (rv < 0)
				pr_err("Failed to destroy cdev event %d error 0x%x\n",
					i, rv);
		}
	}

	/* remove control character device */
	if (xpdev_flag_test(xpdev, XDF_CDEV_CTRL)) {
		rv = destroy_xcdev(&xpdev->ctrl_cdev);
		if (rv < 0)
			pr_err("Failed to destroy cdev ctrl event %d error 0x%x\n",
				i, rv);
	}

	/* remove user character device */
	if (xpdev_flag_test(xpdev, XDF_CDEV_USER)) {
		rv = destroy_xcdev(&xpdev->user_cdev);
		if (rv < 0)
			pr_err("Failed to destroy user cdev %d error 0x%x\n",
				i, rv);
	}

	if (xpdev_flag_test(xpdev, XDF_CDEV_XVC)) {
		rv = destroy_xcdev(&xpdev->xvc_cdev);
		if (rv < 0)
			pr_err("Failed to destroy xvc cdev %d error 0x%x\n",
				i, rv);
	}

	if (xpdev_flag_test(xpdev, XDF_CDEV_BYPASS)) {
		/* iterate over channels */
		for (i = 0; i < xpdev->h2c_channel_max; i++) {
			/* remove DMA Bypass character device */
			rv = destroy_xcdev(&xpdev->bypass_h2c_cdev[i]);
			if (rv < 0)
				pr_err("Failed to destroy bypass h2c cdev %d error 0x%x\n",
					i, rv);
		}
		for (i = 0; i < xpdev->c2h_channel_max; i++) {
			rv = destroy_xcdev(&xpdev->bypass_c2h_cdev[i]);
			if (rv < 0)
				pr_err("Failed to destroy bypass c2h %d error 0x%x\n",
					i, rv);
		}
		rv = destroy_xcdev(&xpdev->bypass_cdev_base);
		if (rv < 0)
			pr_err("Failed to destroy base cdev\n");
	}

	if (xpdev->major)
		unregister_chrdev_region(
				MKDEV(xpdev->major, XDMA_MINOR_BASE),
				XDMA_MINOR_COUNT);
}

int xpdev_create_interfaces(struct xdma_pci_dev *xpdev)
{
	struct xdma_dev *xdev = xpdev->xdev;
	struct xdma_engine *engine;
	int i;
	int rv = 0;

	/* initialize control character device */
	rv = create_xcdev(xpdev, &xpdev->ctrl_cdev, xdev->config_bar_idx,
			NULL, CHAR_CTRL);
	if (rv < 0) {
		pr_err("create_char(ctrl_cdev) failed\n");
		goto fail;
	}
	xpdev_flag_set(xpdev, XDF_CDEV_CTRL);

	/* initialize events character device */
	for (i = 0; i < xpdev->user_max; i++) {
		rv = create_xcdev(xpdev, &xpdev->events_cdev[i], i, NULL,
			CHAR_EVENTS);
		if (rv < 0) {
			pr_err("create char event %d failed, %d.\n", i, rv);
			goto fail;
		}
	}
	xpdev_flag_set(xpdev, XDF_CDEV_EVENT);

	/* iterate over channels */
	for (i = 0; i < xpdev->h2c_channel_max; i++) {
		engine = &xdev->engine_h2c[i];

		if (engine->magic != MAGIC_ENGINE)
			continue;

		rv = create_xcdev(xpdev, &xpdev->sgdma_h2c_cdev[i], i, engine,
				 CHAR_XDMA_H2C);
		if (rv < 0) {
			pr_err("create char h2c %d failed, %d.\n", i, rv);
			goto fail;
		}
	}

	for (i = 0; i < xpdev->c2h_channel_max; i++) {
		engine = &xdev->engine_c2h[i];

		if (engine->magic != MAGIC_ENGINE)
			continue;

		rv = create_xcdev(xpdev, &xpdev->sgdma_c2h_cdev[i], i, engine,
				 CHAR_XDMA_C2H);
		if (rv < 0) {
			pr_err("create char c2h %d failed, %d.\n", i, rv);
			goto fail;
		}
	}
	xpdev_flag_set(xpdev, XDF_CDEV_SG);

	/* Initialize Bypass Character Device */
	if (xdev->bypass_bar_idx > 0) {
		for (i = 0; i < xpdev->h2c_channel_max; i++) {
			engine = &xdev->engine_h2c[i];

			if (engine->magic != MAGIC_ENGINE)
				continue;

			rv = create_xcdev(xpdev, &xpdev->bypass_h2c_cdev[i], i,
					engine, CHAR_BYPASS_H2C);
			if (rv < 0) {
				pr_err("create h2c %d bypass I/F failed, %d.\n",
					i, rv);
				goto fail;
			}
		}

		for (i = 0; i < xpdev->c2h_channel_max; i++) {
			engine = &xdev->engine_c2h[i];

			if (engine->magic != MAGIC_ENGINE)
				continue;

			rv = create_xcdev(xpdev, &xpdev->bypass_c2h_cdev[i], i,
					engine, CHAR_BYPASS_C2H);
			if (rv < 0) {
				pr_err("create c2h %d bypass I/F failed, %d.\n",
					i, rv);
				goto fail;
			}
		}

		rv = create_xcdev(xpdev, &xpdev->bypass_cdev_base,
				xdev->bypass_bar_idx, NULL, CHAR_BYPASS);
		if (rv < 0) {
			pr_err("create bypass failed %d.\n", rv);
			goto fail;
		}
		xpdev_flag_set(xpdev, XDF_CDEV_BYPASS);
	}

	/* initialize user character device */
	if (xdev->user_bar_idx >= 0) {
		rv = create_xcdev(xpdev, &xpdev->user_cdev, xdev->user_bar_idx,
			NULL, CHAR_USER);
		if (rv < 0) {
			pr_err("create_char(user_cdev) failed\n");
			goto fail;
		}
		xpdev_flag_set(xpdev, XDF_CDEV_USER);

		/* xvc */
		rv = create_xcdev(xpdev, &xpdev->xvc_cdev, xdev->user_bar_idx,
				 NULL, CHAR_XVC);
		if (rv < 0) {
			pr_err("create xvc failed, %d.\n", rv);
			goto fail;
		}
		xpdev_flag_set(xpdev, XDF_CDEV_XVC);
	}

#ifdef __XDMA_SYSFS__
	/* sys file */
	rv = device_create_file(&xpdev->pdev->dev,
				&dev_attr_xdma_dev_instance);
	if (rv) {
		pr_err("Failed to create device file\n");
		goto fail;
	}
#endif

	return 0;

fail:
	rv = -1;
	xpdev_destroy_interfaces(xpdev);
	return rv;
}

int xdma_cdev_init(void)
{
#if defined(RHEL_RELEASE_CODE)
    #if (RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(9, 4))
        g_xdma_class = class_create(XDMA_NODE_NAME);
    #else
        g_xdma_class = class_create(THIS_MODULE, XDMA_NODE_NAME);
    #endif
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
        g_xdma_class = class_create(XDMA_NODE_NAME);
#else
        g_xdma_class = class_create(THIS_MODULE, XDMA_NODE_NAME);
#endif
	if (IS_ERR(g_xdma_class)) {
		dbg_init(XDMA_NODE_NAME ": failed to create class");
		return -EINVAL;
	}

	/* using kmem_cache_create to enable sequential cleanup */
	cdev_cache = kmem_cache_create("cdev_cache",
					sizeof(struct cdev_async_io), 0,
					SLAB_HWCACHE_ALIGN, NULL);

	if (!cdev_cache) {
		pr_info("memory allocation for cdev_cache failed. OOM\n");
		return -ENOMEM;
	}

	return 0;
}

void xdma_cdev_cleanup(void)
{
	if (cdev_cache)
		kmem_cache_destroy(cdev_cache);

	if (g_xdma_class)
		class_destroy(g_xdma_class);
}
