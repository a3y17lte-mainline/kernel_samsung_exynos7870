/* SPDX-License-Identifier: GPL-2.0-or-later WITH Linux-syscall-note */
/*
 * Raw NCI character device of the Samsung S3FWRN5 family (/dev/s3fwrn5)
 *
 * read() returns one NCI packet, write() sends one. The chip is off until
 * S3FWRN5_RAW_IOC_POWER switches it on, and is switched off on close.
 */
#ifndef _UAPI_LINUX_S3FWRN5_RAW_H
#define _UAPI_LINUX_S3FWRN5_RAW_H

#include <linux/ioctl.h>
#include <linux/types.h>

/* 0: power off; 1: power cycle into NCI mode (received packets are dropped) */
#define S3FWRN5_RAW_IOC_POWER	_IOW('S', 0x01, __u32)

#endif /* _UAPI_LINUX_S3FWRN5_RAW_H */
