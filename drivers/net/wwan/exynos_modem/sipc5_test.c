// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the SIPC5 frame and ring buffer helpers and the firmware
 * image's table of contents
 */

#include <kunit/test.h>
#include <linux/unaligned.h>

#include "fw.h"
#include "sipc5.h"

#define TEST_RING_SIZE	64

struct sipc_test_ring {
	__le32 head;
	__le32 tail;
	u8 buf[TEST_RING_SIZE];
	u8 guard[8];
	struct sipc_ring ring;
};

static struct sipc_test_ring *sipc_test_ring_alloc(struct kunit *test)
{
	struct sipc_test_ring *tr;

	tr = kunit_kzalloc(test, sizeof(*tr), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, tr);

	memset(tr->guard, 0xa5, sizeof(tr->guard));
	tr->ring.head = &tr->head;
	tr->ring.tail = &tr->tail;
	tr->ring.buf = tr->buf;
	tr->ring.size = TEST_RING_SIZE;

	return tr;
}

static void sipc_test_guard(struct kunit *test, struct sipc_test_ring *tr)
{
	int i;

	for (i = 0; i < sizeof(tr->guard); i++)
		KUNIT_EXPECT_EQ(test, tr->guard[i], 0xa5);
}

static void sipc5_test_hdr_min(struct kunit *test)
{
	u8 hdr[SIPC5_HDR_MAX] = {};

	KUNIT_EXPECT_EQ(test, sipc5_tx_hdr_len(4), SIPC5_HDR_MIN);
	KUNIT_EXPECT_EQ(test, sipc5_build_hdr(hdr, 235, 4, false), 0);
	KUNIT_EXPECT_EQ(test, hdr[0], 0xf8);
	KUNIT_EXPECT_EQ(test, hdr[1], 235);
	KUNIT_EXPECT_EQ(test, hdr[2], 8);
	KUNIT_EXPECT_EQ(test, hdr[3], 0);

	KUNIT_EXPECT_TRUE(test, sipc5_start_valid(hdr));
	KUNIT_EXPECT_FALSE(test, sipc5_is_multi(hdr));
	KUNIT_EXPECT_EQ(test, sipc5_hdr_len(hdr), SIPC5_HDR_MIN);
	KUNIT_EXPECT_EQ(test, sipc5_frame_len(hdr), 8);
	KUNIT_EXPECT_EQ(test, sipc5_total_len(hdr), 8);
}

static void sipc5_test_hdr_padded(struct kunit *test)
{
	u8 hdr[SIPC5_HDR_MAX] = {};

	/* The loader handshake: four payload bytes need no padding */
	KUNIT_EXPECT_EQ(test, sipc5_build_hdr(hdr, 215, 4, true), 0);
	KUNIT_EXPECT_EQ(test, hdr[0], 0xfc);
	KUNIT_EXPECT_EQ(test, sipc5_total_len(hdr), 8);

	KUNIT_EXPECT_EQ(test, sipc5_build_hdr(hdr, 245, 5, true), 7);
	KUNIT_EXPECT_EQ(test, sipc5_frame_len(hdr), 9);
	KUNIT_EXPECT_EQ(test, sipc5_total_len(hdr), 16);

	/* Without the flag the reader must not skip anything */
	hdr[0] &= ~SIPC5_PADDING;
	KUNIT_EXPECT_EQ(test, sipc5_total_len(hdr), 9);
}

static void sipc5_test_hdr_ext_len(struct kunit *test)
{
	u8 hdr[SIPC5_HDR_MAX] = {};
	unsigned int payload = U16_MAX - SIPC5_HDR_MIN;

	KUNIT_EXPECT_EQ(test, sipc5_tx_hdr_len(payload), SIPC5_HDR_MIN);
	payload++;
	KUNIT_EXPECT_EQ(test, sipc5_tx_hdr_len(payload), SIPC5_HDR_EXT_LEN);

	KUNIT_EXPECT_EQ(test, sipc5_build_hdr(hdr, 10, payload, false), 0);
	KUNIT_EXPECT_EQ(test, hdr[0], 0xfa);
	KUNIT_EXPECT_EQ(test, sipc5_hdr_len(hdr), SIPC5_HDR_EXT_LEN);
	KUNIT_EXPECT_EQ(test, sipc5_frame_len(hdr),
			payload + SIPC5_HDR_EXT_LEN);
	KUNIT_EXPECT_FALSE(test, sipc5_is_multi(hdr));
}

static void sipc5_test_hdr_multi(struct kunit *test)
{
	/* Fragment 5 of a control message, more to come */
	static const u8 hdr[SIPC5_HDR_MAX] = { 0xfb, 235, 0x10, 0x00, 0x85 };

	KUNIT_EXPECT_TRUE(test, sipc5_start_valid(hdr));
	KUNIT_EXPECT_TRUE(test, sipc5_is_multi(hdr));
	KUNIT_EXPECT_EQ(test, sipc5_hdr_len(hdr), SIPC5_HDR_MULTI);
	KUNIT_EXPECT_EQ(test, sipc5_frame_len(hdr), 0x10);
	KUNIT_EXPECT_TRUE(test, hdr[4] & SIPC5_CTL_MORE);
	KUNIT_EXPECT_EQ(test, hdr[4] & SIPC5_CTL_ID, 5);
}

static void sipc5_test_hdr_invalid(struct kunit *test)
{
	static const u8 zero[SIPC5_HDR_MAX];
	static const u8 text[SIPC5_HDR_MAX] = "BOOT";

	KUNIT_EXPECT_FALSE(test, sipc5_start_valid(zero));
	KUNIT_EXPECT_FALSE(test, sipc5_start_valid(text));
}

static void sipc5_test_channels(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, sipc5_ch_is_fmt(235));
	KUNIT_EXPECT_TRUE(test, sipc5_ch_is_fmt(244));
	KUNIT_EXPECT_FALSE(test, sipc5_ch_is_fmt(245));
	KUNIT_EXPECT_FALSE(test, sipc5_ch_is_fmt(10));

	KUNIT_EXPECT_TRUE(test, sipc5_ch_is_udl(215));
	KUNIT_EXPECT_TRUE(test, sipc5_ch_is_udl(225));
	KUNIT_EXPECT_TRUE(test, sipc5_ch_is_udl(234));
	KUNIT_EXPECT_FALSE(test, sipc5_ch_is_udl(214));
	KUNIT_EXPECT_FALSE(test, sipc5_ch_is_udl(235));

	KUNIT_EXPECT_TRUE(test, sipc5_ch_is_pdp(10));
	KUNIT_EXPECT_TRUE(test, sipc5_ch_is_pdp(24));
	KUNIT_EXPECT_FALSE(test, sipc5_ch_is_pdp(25));
	KUNIT_EXPECT_FALSE(test, sipc5_ch_is_pdp(9));
}

static void sipc_ring_test_basic(struct kunit *test)
{
	struct sipc_test_ring *tr = sipc_test_ring_alloc(test);
	struct sipc_ring *ring = &tr->ring;
	u8 out[8] = {};

	KUNIT_EXPECT_TRUE(test, sipc_ring_empty(ring));
	KUNIT_EXPECT_EQ(test, sipc_ring_space(ring, 0, 0), TEST_RING_SIZE - 1);

	KUNIT_EXPECT_EQ(test, sipc_ring_write(ring, "abcdefgh", 8), 0);
	KUNIT_EXPECT_FALSE(test, sipc_ring_empty(ring));
	KUNIT_EXPECT_EQ(test, sipc_ring_head(ring), 8);
	KUNIT_EXPECT_EQ(test, sipc_ring_tail(ring), 0);
	KUNIT_EXPECT_EQ(test, sipc_ring_used(ring, 8, 0), 8);

	sipc_ring_peek(ring, 0, out, 8);
	KUNIT_EXPECT_MEMEQ(test, out, "abcdefgh", 8);
	KUNIT_EXPECT_EQ(test, sipc_ring_tail(ring), 0);

	sipc_ring_consume(ring, 0, 8);
	KUNIT_EXPECT_EQ(test, sipc_ring_tail(ring), 8);
	KUNIT_EXPECT_TRUE(test, sipc_ring_empty(ring));
}

static void sipc_ring_test_wrap(struct kunit *test)
{
	struct sipc_test_ring *tr = sipc_test_ring_alloc(test);
	struct sipc_ring *ring = &tr->ring;
	u8 data[16], out[16];
	int i;

	for (i = 0; i < sizeof(data); i++)
		data[i] = i + 1;

	/* Six bytes fit before the end of the buffer, ten after the wrap */
	tr->head = cpu_to_le32(TEST_RING_SIZE - 6);
	tr->tail = cpu_to_le32(TEST_RING_SIZE - 6);

	KUNIT_EXPECT_EQ(test, sipc_ring_write(ring, data, sizeof(data)), 0);
	KUNIT_EXPECT_EQ(test, sipc_ring_head(ring), 10);
	KUNIT_EXPECT_EQ(test, tr->buf[TEST_RING_SIZE - 1], 6);
	KUNIT_EXPECT_EQ(test, tr->buf[0], 7);
	sipc_test_guard(test, tr);

	KUNIT_EXPECT_EQ(test, sipc_ring_used(ring, 10, TEST_RING_SIZE - 6), 16);

	sipc_ring_peek(ring, TEST_RING_SIZE - 6, out, sizeof(out));
	KUNIT_EXPECT_MEMEQ(test, out, data, sizeof(data));

	sipc_ring_consume(ring, TEST_RING_SIZE - 6, sizeof(out));
	KUNIT_EXPECT_EQ(test, sipc_ring_tail(ring), 10);
	KUNIT_EXPECT_TRUE(test, sipc_ring_empty(ring));
}

static void sipc_ring_test_full(struct kunit *test)
{
	struct sipc_test_ring *tr = sipc_test_ring_alloc(test);
	struct sipc_ring *ring = &tr->ring;
	u8 data[TEST_RING_SIZE];

	memset(data, 0x5a, sizeof(data));

	/* One byte always stays free */
	KUNIT_EXPECT_EQ(test, sipc_ring_write(ring, data, TEST_RING_SIZE),
			-ENOSPC);
	KUNIT_EXPECT_TRUE(test, sipc_ring_empty(ring));

	KUNIT_EXPECT_EQ(test, sipc_ring_write(ring, data, TEST_RING_SIZE - 1),
			0);
	KUNIT_EXPECT_EQ(test, sipc_ring_head(ring), TEST_RING_SIZE - 1);
	KUNIT_EXPECT_EQ(test, sipc_ring_write(ring, data, 1), -ENOSPC);
	sipc_test_guard(test, tr);

	/* The reader frees ten bytes: the writer wraps into them */
	sipc_ring_consume(ring, 0, 10);
	KUNIT_EXPECT_EQ(test, sipc_ring_write(ring, data, 11), -ENOSPC);
	KUNIT_EXPECT_EQ(test, sipc_ring_write(ring, data, 10), 0);
	KUNIT_EXPECT_EQ(test, sipc_ring_head(ring), 9);
	sipc_test_guard(test, tr);
}

static void sipc_ring_test_bad_index(struct kunit *test)
{
	struct sipc_test_ring *tr = sipc_test_ring_alloc(test);
	struct sipc_ring *ring = &tr->ring;

	/* The other processor wrote nonsense */
	tr->tail = cpu_to_le32(TEST_RING_SIZE);
	KUNIT_EXPECT_FALSE(test, sipc_ring_valid(ring, sipc_ring_head(ring),
						 sipc_ring_tail(ring)));
	KUNIT_EXPECT_EQ(test, sipc_ring_write(ring, "x", 1), -EIO);

	tr->tail = 0;
	tr->head = cpu_to_le32(0xffffffff);
	KUNIT_EXPECT_EQ(test, sipc_ring_write(ring, "x", 1), -EIO);
	KUNIT_EXPECT_EQ(test, tr->buf[0], 0);
	sipc_test_guard(test, tr);

	sipc_ring_reset(ring);
	KUNIT_EXPECT_EQ(test, sipc_ring_write(ring, "x", 1), 0);
}

/* A frame written the way the link does it comes back out intact */
static void sipc_ring_test_frames(struct kunit *test)
{
	struct sipc_test_ring *tr = sipc_test_ring_alloc(test);
	struct sipc_ring *ring = &tr->ring;
	static const u8 payload[] = { 1, 2, 3, 4, 5 };
	u8 frame[SIPC5_HDR_MAX + sizeof(payload) + SIPC5_PAD_ALIGN] = {};
	u8 hdr[SIPC5_HDR_MAX], out[sizeof(payload)];
	unsigned int hdr_len, pad, total;
	u32 head, tail, pos;
	int i;

	hdr_len = sipc5_tx_hdr_len(sizeof(payload));
	pad = sipc5_build_hdr(frame, 245, sizeof(payload), true);
	memcpy(frame + hdr_len, payload, sizeof(payload));
	total = hdr_len + sizeof(payload) + pad;
	KUNIT_ASSERT_EQ(test, total, 16);

	/* Enough rounds to cross the end of the buffer several times */
	for (i = 0; i < 20; i++) {
		KUNIT_ASSERT_EQ(test, sipc_ring_write(ring, frame, total), 0);

		head = sipc_ring_head(ring);
		tail = sipc_ring_tail(ring);
		KUNIT_ASSERT_TRUE(test, sipc_ring_valid(ring, head, tail));
		KUNIT_ASSERT_EQ(test, sipc_ring_used(ring, head, tail), total);

		sipc_ring_peek(ring, tail, hdr, sizeof(hdr));
		KUNIT_ASSERT_TRUE(test, sipc5_start_valid(hdr));
		KUNIT_EXPECT_EQ(test, hdr[1], 245);
		KUNIT_EXPECT_EQ(test, sipc5_frame_len(hdr),
				hdr_len + sizeof(payload));
		KUNIT_EXPECT_EQ(test, sipc5_total_len(hdr), total);

		pos = tail + sipc5_hdr_len(hdr);
		if (pos >= ring->size)
			pos -= ring->size;
		sipc_ring_peek(ring, pos, out, sizeof(out));
		KUNIT_EXPECT_MEMEQ(test, out, payload, sizeof(payload));

		sipc_ring_consume(ring, tail, sipc5_total_len(hdr));
		KUNIT_EXPECT_TRUE(test, sipc_ring_empty(ring));
	}

	sipc_test_guard(test, tr);
}

static void toc_test_put(u8 *toc, int i, const char *name, u32 offset,
			 u32 load_addr, u32 size)
{
	u8 *e = toc + i * EXMODEM_TOC_ENTRY_SIZE;

	strscpy_pad(e, name, EXMODEM_TOC_NAME_LEN);
	put_unaligned_le32(offset, e + 12);
	put_unaligned_le32(load_addr, e + 16);
	put_unaligned_le32(size, e + 20);
	put_unaligned_le32(0xdeadbeef, e + 24);
	put_unaligned_le32(i, e + 28);
}

/* The layout of the Galaxy A3 (2017) image */
static u8 *toc_test_alloc(struct kunit *test)
{
	u8 *toc = kunit_kzalloc(test, EXMODEM_TOC_SIZE, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, toc);
	toc_test_put(toc, 0, "TOC", 0, 0, 0x200);
	toc_test_put(toc, 1, "BOOT", 0x200, 0x40000000, 0x170c);
	toc_test_put(toc, 2, "MAIN", 0x1920, 0x40010000, 0x2479508);
	toc_test_put(toc, 3, "NV", 0, 0x44900000, 0x100000);
	toc_test_put(toc, 4, "OFFSET", 0x7aa00, 0, 0x85600);

	return toc;
}

static void toc_test_find(struct kunit *test)
{
	struct exmodem_toc_entry e;
	u8 *toc = toc_test_alloc(test);

	KUNIT_ASSERT_EQ(test, exmodem_toc_find(toc, EXMODEM_TOC_SIZE, "MAIN", &e),
			0);
	KUNIT_EXPECT_EQ(test, e.offset, 0x1920);
	KUNIT_EXPECT_EQ(test, e.load_addr, 0x40010000);
	KUNIT_EXPECT_EQ(test, e.size, 0x2479508);

	KUNIT_ASSERT_EQ(test, exmodem_toc_find(toc, EXMODEM_TOC_SIZE, "NV", &e),
			0);
	KUNIT_EXPECT_EQ(test, e.load_addr, 0x44900000);
	KUNIT_EXPECT_EQ(test, e.size, 0x100000);
}

static void toc_test_names(struct kunit *test)
{
	struct exmodem_toc_entry e;
	u8 *toc = toc_test_alloc(test);

	/* A prefix of a name is not a match, nor is a longer name */
	KUNIT_EXPECT_EQ(test, exmodem_toc_find(toc, EXMODEM_TOC_SIZE, "N", &e),
			-ENOENT);
	KUNIT_EXPECT_EQ(test, exmodem_toc_find(toc, EXMODEM_TOC_SIZE, "NVX", &e),
			-ENOENT);
	KUNIT_EXPECT_EQ(test, exmodem_toc_find(toc, EXMODEM_TOC_SIZE, "BOO", &e),
			-ENOENT);
	KUNIT_EXPECT_EQ(test,
			exmodem_toc_find(toc, EXMODEM_TOC_SIZE,
					 "NAME_TOO_LONG", &e),
			-EINVAL);
}

static void toc_test_short(struct kunit *test)
{
	struct exmodem_toc_entry e;
	u8 *toc = toc_test_alloc(test);

	KUNIT_EXPECT_EQ(test,
			exmodem_toc_find(toc, EXMODEM_TOC_SIZE - 1, "BOOT", &e),
			-EINVAL);
}

static struct kunit_case sipc5_test_cases[] = {
	KUNIT_CASE(sipc5_test_hdr_min),
	KUNIT_CASE(sipc5_test_hdr_padded),
	KUNIT_CASE(sipc5_test_hdr_ext_len),
	KUNIT_CASE(sipc5_test_hdr_multi),
	KUNIT_CASE(sipc5_test_hdr_invalid),
	KUNIT_CASE(sipc5_test_channels),
	KUNIT_CASE(sipc_ring_test_basic),
	KUNIT_CASE(sipc_ring_test_wrap),
	KUNIT_CASE(sipc_ring_test_full),
	KUNIT_CASE(sipc_ring_test_bad_index),
	KUNIT_CASE(sipc_ring_test_frames),
	KUNIT_CASE(toc_test_find),
	KUNIT_CASE(toc_test_names),
	KUNIT_CASE(toc_test_short),
	{ }
};

static struct kunit_suite sipc5_test_suite = {
	.name = "exynos-modem-sipc5",
	.test_cases = sipc5_test_cases,
};

kunit_test_suite(sipc5_test_suite);

MODULE_DESCRIPTION("KUnit tests for the Exynos modem link helpers");
MODULE_LICENSE("GPL");
