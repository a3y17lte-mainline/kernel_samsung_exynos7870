/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Samsung IPC version 5 (SIPC5) link frames and the ring buffers of the
 * shared memory link between the application processor and the modem.
 *
 * Everything in here works on plain memory so that it can be unit tested.
 */

#ifndef __EXYNOS_MODEM_SIPC5_H__
#define __EXYNOS_MODEM_SIPC5_H__

#include <linux/align.h>
#include <linux/bits.h>
#include <linux/compiler.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

/*
 * Frame header:
 *
 *   byte 0     configuration: start pattern and field flags
 *   byte 1     channel
 *   byte 2-3   length of header and payload, without padding
 *   byte 4     control field of a multi-frame message (flags == MULTI), or
 *   byte 2-5   32-bit length (flags == EXT_LEN)
 */
#define SIPC5_START		0xf8
#define SIPC5_START_MASK	0xf8
#define SIPC5_PADDING		BIT(2)
#define SIPC5_EXT_FIELD		BIT(1)
#define SIPC5_CTL_FIELD		BIT(0)
#define SIPC5_EXT_MASK		(SIPC5_EXT_FIELD | SIPC5_CTL_FIELD)
#define SIPC5_EXT_MULTI		(SIPC5_EXT_FIELD | SIPC5_CTL_FIELD)
#define SIPC5_EXT_LEN		SIPC5_EXT_FIELD

#define SIPC5_HDR_MIN		4
#define SIPC5_HDR_MULTI		5
#define SIPC5_HDR_EXT_LEN	6
#define SIPC5_HDR_MAX		SIPC5_HDR_EXT_LEN

/* Control field of a multi-frame message */
#define SIPC5_CTL_MORE		BIT(7)
#define SIPC5_CTL_ID		GENMASK(6, 0)

/*
 * The vendor implementation pads frames to the size of a long of the
 * application processor's kernel, which is 64-bit on every SoC with this
 * link.
 */
#define SIPC5_PAD_ALIGN		8

/* Channels */
#define SIPC5_CH_PDP_0		10
#define SIPC5_CH_PDP_LAST	24
#define SIPC5_CH_BOOT_0		215
#define SIPC5_CH_DUMP_0		225
#define SIPC5_CH_DUMP_9		234
#define SIPC5_CH_FMT_0		235
#define SIPC5_CH_FMT_9		244
#define SIPC5_CH_RFS_0		245
#define SIPC5_CH_RFS_9		254

static inline bool sipc5_ch_is_fmt(u8 ch)
{
	return ch >= SIPC5_CH_FMT_0 && ch <= SIPC5_CH_FMT_9;
}

/* Boot and dump channels: used while no IPC is possible */
static inline bool sipc5_ch_is_udl(u8 ch)
{
	return ch >= SIPC5_CH_BOOT_0 && ch <= SIPC5_CH_DUMP_9;
}

static inline bool sipc5_ch_is_pdp(u8 ch)
{
	return ch >= SIPC5_CH_PDP_0 && ch <= SIPC5_CH_PDP_LAST;
}

static inline bool sipc5_start_valid(const u8 *hdr)
{
	return (hdr[0] & SIPC5_START_MASK) == SIPC5_START;
}

static inline bool sipc5_is_multi(const u8 *hdr)
{
	return (hdr[0] & SIPC5_EXT_MASK) == SIPC5_EXT_MULTI;
}

static inline unsigned int sipc5_hdr_len(const u8 *hdr)
{
	switch (hdr[0] & SIPC5_EXT_MASK) {
	case SIPC5_EXT_MULTI:
		return SIPC5_HDR_MULTI;
	case SIPC5_EXT_LEN:
		return SIPC5_HDR_EXT_LEN;
	default:
		return SIPC5_HDR_MIN;
	}
}

/* Length of header and payload. @hdr must hold SIPC5_HDR_MAX bytes. */
static inline unsigned int sipc5_frame_len(const u8 *hdr)
{
	if ((hdr[0] & SIPC5_EXT_MASK) == SIPC5_EXT_LEN)
		return get_unaligned_le32(hdr + 2);

	return get_unaligned_le16(hdr + 2);
}

static inline unsigned int sipc5_pad_len(unsigned int frame_len)
{
	return ALIGN(frame_len, SIPC5_PAD_ALIGN) - frame_len;
}

/* Length of the frame in the ring, with padding */
static inline unsigned int sipc5_total_len(const u8 *hdr)
{
	unsigned int len = sipc5_frame_len(hdr);

	if (hdr[0] & SIPC5_PADDING)
		len += sipc5_pad_len(len);

	return len;
}

/* Header length needed to send @payload_len bytes in one frame */
static inline unsigned int sipc5_tx_hdr_len(unsigned int payload_len)
{
	return payload_len + SIPC5_HDR_MIN > U16_MAX ? SIPC5_HDR_EXT_LEN :
						       SIPC5_HDR_MIN;
}

/**
 * sipc5_build_hdr() - write the header of a single frame
 * @hdr: destination, sipc5_tx_hdr_len(@payload_len) bytes
 * @ch: channel
 * @payload_len: number of payload bytes that follow the header
 * @padded: the frame is padded to SIPC5_PAD_ALIGN
 *
 * Return: number of padding bytes to append after the payload.
 */
static inline unsigned int sipc5_build_hdr(u8 *hdr, u8 ch,
					   unsigned int payload_len,
					   bool padded)
{
	unsigned int hdr_len = sipc5_tx_hdr_len(payload_len);
	unsigned int len = hdr_len + payload_len;

	hdr[0] = SIPC5_START | (padded ? SIPC5_PADDING : 0);
	hdr[1] = ch;
	if (hdr_len == SIPC5_HDR_EXT_LEN) {
		hdr[0] |= SIPC5_EXT_LEN;
		put_unaligned_le32(len, hdr + 2);
	} else {
		put_unaligned_le16(len, hdr + 2);
	}

	return padded ? sipc5_pad_len(len) : 0;
}

/**
 * struct sipc_ring - one direction of a ring buffer pair in shared memory
 * @head: index of the next byte to write, advanced by the writer
 * @tail: index of the next byte to read, advanced by the reader
 * @buf: the data buffer
 * @size: size of @buf
 *
 * One byte always stays free, so head == tail means empty. The other side is
 * a different processor: the indices are read once and validated before use.
 */
struct sipc_ring {
	__le32 *head;
	__le32 *tail;
	u8 *buf;
	u32 size;
};

static inline u32 sipc_ring_head(const struct sipc_ring *ring)
{
	return le32_to_cpu(READ_ONCE(*ring->head));
}

static inline u32 sipc_ring_tail(const struct sipc_ring *ring)
{
	return le32_to_cpu(READ_ONCE(*ring->tail));
}

static inline void sipc_ring_reset(struct sipc_ring *ring)
{
	WRITE_ONCE(*ring->head, 0);
	WRITE_ONCE(*ring->tail, 0);
}

static inline bool sipc_ring_valid(const struct sipc_ring *ring, u32 head,
				   u32 tail)
{
	return head < ring->size && tail < ring->size;
}

static inline u32 sipc_ring_used(const struct sipc_ring *ring, u32 head,
				 u32 tail)
{
	return head >= tail ? head - tail : ring->size - tail + head;
}

static inline u32 sipc_ring_space(const struct sipc_ring *ring, u32 head,
				  u32 tail)
{
	return ring->size - sipc_ring_used(ring, head, tail) - 1;
}

/**
 * sipc_ring_empty() - check whether the reader has caught up
 * @ring: the ring
 *
 * Return: true if the ring holds no data.
 */
static inline bool sipc_ring_empty(const struct sipc_ring *ring)
{
	return sipc_ring_head(ring) == sipc_ring_tail(ring);
}

/**
 * sipc_ring_write() - append data and advance the head
 * @ring: a ring this side writes to
 * @data: the bytes to write
 * @len: number of bytes
 *
 * Return: 0, -ENOSPC if the data does not fit now, or -EIO if the indices
 * are outside of the buffer.
 */
static inline int sipc_ring_write(struct sipc_ring *ring, const void *data,
				  u32 len)
{
	u32 head = sipc_ring_head(ring);
	u32 tail = sipc_ring_tail(ring);
	u32 chunk;

	if (!sipc_ring_valid(ring, head, tail))
		return -EIO;

	if (sipc_ring_space(ring, head, tail) < len)
		return -ENOSPC;

	chunk = min(len, ring->size - head);
	memcpy(ring->buf + head, data, chunk);
	memcpy(ring->buf, data + chunk, len - chunk);

	head += len;
	if (head >= ring->size)
		head -= ring->size;

	/* The data must be in place before the reader can see the new head */
	smp_wmb();
	WRITE_ONCE(*ring->head, cpu_to_le32(head));

	return 0;
}

/**
 * sipc_ring_peek() - copy data out without consuming it
 * @ring: a ring this side reads from
 * @tail: the read position, from sipc_ring_tail()
 * @data: destination
 * @len: number of bytes; the caller checked that they are available
 */
static inline void sipc_ring_peek(const struct sipc_ring *ring, u32 tail,
				  void *data, u32 len)
{
	u32 chunk = min(len, ring->size - tail);

	memcpy(data, ring->buf + tail, chunk);
	memcpy(data + chunk, ring->buf, len - chunk);
}

/**
 * sipc_ring_consume() - advance the tail
 * @ring: a ring this side reads from
 * @tail: the read position, from sipc_ring_tail()
 * @len: number of bytes read
 */
static inline void sipc_ring_consume(struct sipc_ring *ring, u32 tail, u32 len)
{
	tail += len;
	if (tail >= ring->size)
		tail -= ring->size;

	WRITE_ONCE(*ring->tail, cpu_to_le32(tail));
}

#endif /* __EXYNOS_MODEM_SIPC5_H__ */
