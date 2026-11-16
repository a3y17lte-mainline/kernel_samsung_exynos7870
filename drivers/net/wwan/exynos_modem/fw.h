/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Samsung Exynos integrated modem: layout of the modem firmware image
 *
 * The image starts with a table of contents: 32-byte entries, each naming
 * one part of the image, where it is in the file and where it is loaded in
 * the modem's memory. The parts the modem needs are the loader ("BOOT") and
 * the firmware ("MAIN"). The "NV" entry describes the modem's non-volatile
 * data, which is kept in a separate file.
 */
#ifndef __EXYNOS_MODEM_FW_H__
#define __EXYNOS_MODEM_FW_H__

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define EXMODEM_TOC_ENTRY_SIZE	32
#define EXMODEM_TOC_ENTRIES	16
#define EXMODEM_TOC_NAME_LEN	12
#define EXMODEM_TOC_SIZE	(EXMODEM_TOC_ENTRIES * EXMODEM_TOC_ENTRY_SIZE)

/**
 * struct exmodem_toc_entry - one part of the firmware image
 * @offset: offset in the image file
 * @load_addr: address in the modem's view of its memory
 * @size: size in bytes
 */
struct exmodem_toc_entry {
	u32 offset;
	u32 load_addr;
	u32 size;
};

/**
 * exmodem_toc_find() - look up a part in the table of contents
 * @data: the image
 * @len: length of the image
 * @name: name of the part
 * @entry: filled in with the part's location
 *
 * Return: 0, -EINVAL if the image has no table of contents or -ENOENT if the
 * part is not listed.
 */
static inline int exmodem_toc_find(const u8 *data, size_t len,
				   const char *name,
				   struct exmodem_toc_entry *entry)
{
	size_t name_len = strlen(name);
	int i;

	if (len < EXMODEM_TOC_SIZE ||
	    name_len >= EXMODEM_TOC_NAME_LEN)
		return -EINVAL;

	for (i = 0; i < EXMODEM_TOC_ENTRIES; i++) {
		const u8 *e = data + i * EXMODEM_TOC_ENTRY_SIZE;

		if (memcmp(e, name, name_len) || e[name_len])
			continue;

		entry->offset = get_unaligned_le32(e + 12);
		entry->load_addr = get_unaligned_le32(e + 16);
		entry->size = get_unaligned_le32(e + 20);
		return 0;
	}

	return -ENOENT;
}

#endif /* __EXYNOS_MODEM_FW_H__ */
