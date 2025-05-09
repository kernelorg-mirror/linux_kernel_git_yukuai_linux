// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/blkdev.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/file.h>
#include <linux/mount.h>
#include <linux/buffer_head.h>
#include <linux/seq_file.h>
#include <trace/events/block.h>

#include "md.h"
#include "md-bitmap.h"

/*
 * #### Background
 *
 * Redundant data is used to enhance data fault tolerance, and the storage
 * method for redundant data vary depending on the RAID levels. And it's
 * important to maintain the consistency of redundant data.
 *
 * Bitmap is used to record which data blocks have been synchronized and which
 * ones need to be resynchronized or recovered. Each bit in the bitmap
 * represents a segment of data in the array. When a bit is set, it indicates
 * that the multiple redundant copies of that data segment may not be
 * consistent. Data synchronization can be performed based on the bitmap after
 * power failure or readding a disk. If there is no bitmap, a full disk
 * synchronization is required.
 *
 * #### Key Features
 *
 *  - IO fastpath is lockless, if user issues lots of write IO to the same
 *  bitmap bit in a short time, only the first write have additional overhead
 *  to update bitmap bit, no additional overhead for the following writes;
 *  - support only resync or recover written data, means in the case creating
 *  new array or replacing with a new disk, there is no need to do a full disk
 *  resync/recovery;
 *
 * #### Key Concept
 *
 * ##### State Machine
 *
 * Each bit is one byte, contain 6 difference state, see llbitmap_state. And
 * there are total 8 differenct actions, see llbitmap_action, can change state:
 *
 * llbitmap state machine: transitions between states
 *
 * |           | Startwrite | Startsync | Endsync | Abortsync| Reload   | Daemon | Discard   | Stale     |
 * | --------- | ---------- | --------- | ------- | -------  | -------- | ------ | --------- | --------- |
 * | Unwritten | Dirty      | x         | x       | x        | x        | x      | x         | x         |
 * | Clean     | Dirty      | x         | x       | x        | x        | x      | Unwritten | NeedSync  |
 * | Dirty     | x          | x         | x       | x        | NeedSync | Clean  | Unwritten | NeedSync  |
 * | NeedSync  | x          | Syncing   | x       | x        | x        | x      | Unwritten | x         |
 * | Syncing   | x          | Syncing   | Dirty   | NeedSync | NeedSync | x      | Unwritten | NeedSync  |
 *
 * Typical scenarios:
 *
 * 1) Create new array
 * All bits will be set to Unwritten by default, if --assume-clean is set,
 * All bits will be set to Clean instead.
 *
 * 2) write data, raid1/raid10 have full copy of data, while raid456 doesn't and
 * rely on xor data
 *
 * 2.1) write new data to raid1/raid10:
 * Unwritten --StartWrite--> Dirty
 *
 * 2.2) write new data to raid456:
 * Unwritten --StartWrite--> NeedSync
 *
 * Because the initial recover for raid456 is skipped, the xor data is not build
 * yet, the bit must set to NeedSync first and after lazy initial recover is
 * finished, the bit will finially set to Dirty(see 4.1 and 4.4);
 *
 * 2.3) cover write
 * Clean --StartWrite--> Dirty
 *
 * 3) daemon, if the array is not degraded:
 * Dirty --Daemon--> Clean
 *
 * For degraded array, the Dirty bit will never be cleared, prevent full disk
 * recovery while readding a removed disk.
 *
 * 4) discard
 * {Clean, Dirty, NeedSync, Syncing} --Discard--> Unwritten
 *
 * 5) resync and recover
 *
 * 5.1) common process
 * NeedSync --Startsync--> Syncing --Endsync--> Dirty --Daemon--> Clean
 *
 * 5.2) resync after power failure
 * Dirty --Reload--> NeedSync
 *
 * 5.3) recover while replacing with a new disk
 * By default, the old bitmap framework will recover all data, and llbitmap
 * implement this by a new helper llbitmap_skip_sync_blocks:
 *
 * skip recover for bits other than dirty or clean;
 *
 * 5.4) lazy initial recover for raid5:
 * By default, the old bitmap framework will only allow new recover when there
 * are spares(new disk), a new recovery flag MD_RECOVERY_LAZY_RECOVER is add
 * to perform raid456 lazy recover for set bits(from 2.2).
 *
 * ##### Bitmap IO
 *
 * ##### Chunksize
 *
 * The default bitmap size is 128k, incluing 1k bitmap super block, and
 * the default size of segment of data in the array each bit(chunksize) is 64k,
 * and chunksize will adjust to twice the old size each time if the total number
 * bits is not less than 127k.(see llbitmap_init)
 *
 * ##### READ
 *
 * While creating bitmap, all pages will be allocated and read for llbitmap,
 * there won't be read afterwards
 *
 * ##### WRITE
 *
 * WRITE IO is divided into logical_block_size of the array, the dirty state
 * of each block is tracked independently, for example:
 *
 * each page is 4k, contain 8 blocks; each block is 512 bytes contain 512 bit;
 *
 * | page0 | page1 | ... | page 31 |
 * |       |
 * |        \-----------------------\
 * |                                |
 * | block0 | block1 | ... | block 8|
 * |        |
 * |         \-----------------\
 * |                            |
 * | bit0 | bit1 | ... | bit511 |
 *
 * From IO path, if one bit is changed to Dirty or NeedSync, the corresponding
 * block will be marked dirty, such block must write first before the IO is
 * issued. This behaviour will affect IO performance, to reduce the impact, if
 * multiple bits are changed in the same block in a short time, all bits in this
 * block will be changed to Dirty/NeedSync, so that there won't be any overhead
 * until daemon clears dirty bits.
 *
 * ##### Dirty Bits syncronization
 *
 * IO fast path will set bits to dirty, and those dirty bits will be cleared
 * by daemon after IO is done. llbitmap_barrier is used to synchronize between
 * IO path and daemon;
 *
 * IO path:
 *  1) try to grab a reference, if succeed, set expire time after 5s and return;
 *  2) if failed to grab a reference, wait for daemon to finish clearing dirty
 *  bits;
 *
 * Daemon(Daemon will be waken up every daemon_sleep seconds):
 * For each page:
 *  1) check if page expired, if not skip this page; for expired page:
 *  2) suspend the page and wait for inflight write IO to be done;
 *  3) change dirty page to clean;
 *  4) resume the page;
 */

#define LLBITMAP_MAJOR_HI 6

#define BITMAP_MAX_SECTOR (128 * 2)
#define BITMAP_MAX_PAGES 32
#define BITMAP_SB_SIZE 1024
/* 64k is the max IO size of sync IO for raid1/raid10 */
#define MIN_CHUNK_SIZE (64 * 2)

#define DEFAULT_DAEMON_SLEEP 30

#define BARRIER_IDLE 5

enum llbitmap_state {
	/* No valid data, init state after assemble the array */
	BitUnwritten = 0,
	/* data is consistent */
	BitClean,
	/* data will be consistent after IO is done, set directly for writes */
	BitDirty,
	/*
	 * data need to be resynchronized:
	 * 1) set directly for writes if array is degraded, prevent full disk
	 * synchronization after readding a disk;
	 * 2) reassemble the array after power failure, and dirty bits are
	 * found after reloading the bitmap;
	 */
	BitNeedSync,
	/* data is synchronizing */
	BitSyncing,
	nr_llbitmap_state,
	BitNone = 0xff,
};

enum llbitmap_action {
	/* User write new data, this is the only acton from IO fast path */
	BitmapActionStartwrite = 0,
	/* Start recovery */
	BitmapActionStartsync,
	/* Finish recovery */
	BitmapActionEndsync,
	/* Failed recovery */
	BitmapActionAbortsync,
	/* Reassemble the array */
	BitmapActionReload,
	/* Daemon thread is trying to clear dirty bits */
	BitmapActionDaemon,
	/* Data is deleted */
	BitmapActionDiscard,
	/*
	 * Bitmap is stale, mark all bits in addition to BitUnwritten to
	 * BitNeedSync.
	 */
	BitmapActionStale,
	nr_llbitmap_action,
	/* Init state is BitUnwritten */
	BitmapActionInit,
};

enum barrier_state {
	LLPageFlush = 0,
	LLPageDirty,
};
/*
 * page level barrier to synchronize between dirty bit by write IO and clean bit
 * by daemon.
 */
struct llbitmap_barrier {
	char *data;
	struct percpu_ref active;
	unsigned long expire;
	unsigned long flags;
	/* Per block size dirty state, maximum 64k page / 512 sector = 128 */
	DECLARE_BITMAP(dirty, 128);
	wait_queue_head_t wait;
} ____cacheline_aligned_in_smp;

struct llbitmap {
	struct mddev *mddev;
	int nr_pages;
	struct page *pages[BITMAP_MAX_PAGES];
	struct llbitmap_barrier barrier[BITMAP_MAX_PAGES];

	/* shift of one chunk */
	unsigned long chunkshift;
	/* size of one chunk in sector */
	unsigned long chunksize;
	/* total number of chunks */
	unsigned long chunks;
	int io_size;
	int bits_per_page;
	/* fires on first BitDirty state */
	struct timer_list pending_timer;
	struct work_struct daemon_work;

	unsigned long flags;
	__u64	events_cleared;
};

struct llbitmap_unplug_work {
	struct work_struct work;
	struct llbitmap *llbitmap;
	struct completion *done;
};

static struct workqueue_struct *md_llbitmap_io_wq;
static struct workqueue_struct *md_llbitmap_unplug_wq;

static char state_machine[nr_llbitmap_state][nr_llbitmap_action] = {
	[BitUnwritten] = {BitDirty, BitNone, BitNone, BitNone, BitNone, BitNone, BitNone, BitNone},
	[BitClean] = {BitDirty, BitNone, BitNone, BitNone, BitNone, BitNone, BitUnwritten, BitNeedSync},
	[BitDirty] = {BitNone, BitNone, BitNone, BitNone, BitNeedSync, BitClean, BitUnwritten, BitNeedSync},
	[BitNeedSync] = {BitNone, BitSyncing, BitNone, BitNone, BitNone, BitNone, BitUnwritten, BitNone},
	[BitSyncing] = {BitNone, BitSyncing, BitDirty, BitNeedSync, BitNeedSync, BitNone, BitUnwritten, BitNeedSync},
};

static bool is_raid456(struct mddev *mddev)
{
	return (mddev->level == 4 || mddev->level == 5 || mddev->level == 6);
}

static int llbitmap_read(struct llbitmap *llbitmap, enum llbitmap_state *state,
			 loff_t pos)
{
	pos += BITMAP_SB_SIZE;
	*state = llbitmap->barrier[pos >> PAGE_SHIFT].data[offset_in_page(pos)];
	return 0;
}

static void llbitmap_set_page_dirty(struct llbitmap *llbitmap, int idx, int offset)
{
	struct llbitmap_barrier *barrier = &llbitmap->barrier[idx];
	bool level_456 = is_raid456(llbitmap->mddev);
	int io_size = llbitmap->io_size;
	int bit = offset / io_size;
	bool infectious = false;
	int pos;

	if (!test_bit(LLPageDirty, &barrier->flags))
		set_bit(LLPageDirty, &barrier->flags);

	/*
	 * if the bit is already dirty, or other page bytes is the same bit is
	 * already BitDirty, then mark the whole bytes in the bit as dirty
	 */
	if (test_and_set_bit(bit, barrier->dirty)) {
		infectious = true;
	} else {
		for (pos = bit * io_size; pos < (bit + 1) * io_size - 1;
		     pos++) {
			if (pos == offset)
				continue;
			if (barrier->data[pos] == BitDirty ||
			    barrier->data[pos] == BitNeedSync) {
				infectious = true;
				break;
			}
		}

	}

	if (!infectious)
		return;

	for (pos = bit * io_size; pos < (bit + 1) * io_size - 1; pos++) {
		if (pos == offset)
			continue;

		switch (barrier->data[pos]) {
		case BitUnwritten:
			barrier->data[pos] = level_456 ? BitNeedSync : BitDirty;
			break;
		case BitClean:
			barrier->data[pos] = BitDirty;
			break;
		};
	}
}

static int llbitmap_write(struct llbitmap *llbitmap, enum llbitmap_state state,
			  loff_t pos)
{
	int idx;
	int offset;

	pos += BITMAP_SB_SIZE;
	idx = pos >> PAGE_SHIFT;
	offset = offset_in_page(pos);

	llbitmap->barrier[idx].data[offset] = state;
	if (state == BitDirty || state == BitNeedSync)
		llbitmap_set_page_dirty(llbitmap, idx, offset);
	return 0;
}

static void llbitmap_free_pages(struct llbitmap *llbitmap)
{
	int i;

	for (i = 0; i < BITMAP_MAX_PAGES; i++) {
		struct page *page = llbitmap->pages[i];

		if (!page)
			return;

		llbitmap->pages[i] = NULL;
		__free_page(page);
		percpu_ref_exit(&llbitmap->barrier[i].active);
	}
}

static struct page *llbitmap_read_page(struct llbitmap *llbitmap, int idx)
{
	struct page *page = llbitmap->pages[idx];
	struct mddev *mddev = llbitmap->mddev;
	struct md_rdev *rdev;

	if (page)
		return page;

	page = alloc_page(GFP_KERNEL | __GFP_ZERO);
	if (!page)
		return ERR_PTR(-ENOMEM);

	rdev_for_each(rdev, mddev) {
		sector_t sector;

		if (rdev->raid_disk < 0 || test_bit(Faulty, &rdev->flags))
			continue;

		sector = mddev->bitmap_info.offset + (idx << PAGE_SECTORS_SHIFT);

		if (sync_page_io(rdev, sector, PAGE_SIZE, page, REQ_OP_READ, true))
			return page;

		md_error(mddev, rdev);
	}

	__free_page(page);
	return ERR_PTR(-EIO);
}

static void llbitmap_write_page(struct llbitmap *llbitmap, int idx)
{
	struct page *page = llbitmap->pages[idx];
	struct mddev *mddev = llbitmap->mddev;
	struct md_rdev *rdev;
	int bit;

	for (bit = 0; bit < llbitmap->bits_per_page; bit++) {
		struct llbitmap_barrier *barrier = &llbitmap->barrier[idx];

		if (!test_and_clear_bit(bit, barrier->dirty))
			continue;

		rdev_for_each(rdev, mddev) {
			sector_t sector;
			sector_t bit_sector = llbitmap->io_size >> SECTOR_SHIFT;

			if (rdev->raid_disk < 0 || test_bit(Faulty, &rdev->flags))
				continue;

			sector = mddev->bitmap_info.offset + rdev->sb_start +
				 (idx << PAGE_SECTORS_SHIFT) +
				 bit * bit_sector;
			md_super_write(mddev, rdev, sector, llbitmap->io_size,
				       page, bit * llbitmap->io_size);
		}
	}
}

static void active_release(struct percpu_ref *ref)
{
	struct llbitmap_barrier *barrier =
		container_of(ref, struct llbitmap_barrier, active);

	wake_up(&barrier->wait);
}

static int llbitmap_cache_pages(struct llbitmap *llbitmap)
{
	int nr_pages = (llbitmap->chunks + BITMAP_SB_SIZE + PAGE_SIZE - 1) / PAGE_SIZE;
	struct page *page;
	int i = 0;

	llbitmap->nr_pages = nr_pages;
	while (i < nr_pages) {
		page = llbitmap_read_page(llbitmap, i);
		if (IS_ERR(page)) {
			llbitmap_free_pages(llbitmap);
			return PTR_ERR(page);
		}

		if (percpu_ref_init(&llbitmap->barrier[i].active, active_release,
				    PERCPU_REF_ALLOW_REINIT, GFP_KERNEL)) {
			__free_page(page);
			return -ENOMEM;
		}

		init_waitqueue_head(&llbitmap->barrier[i].wait);
		llbitmap->barrier[i].data = page_address(page);
		llbitmap->pages[i++] = page;
	}

	return 0;
}

static void llbitmap_init_state(struct llbitmap *llbitmap)
{
	enum llbitmap_state state = BitUnwritten;
	unsigned long i;

	if (test_and_clear_bit(BITMAP_CLEAN, &llbitmap->flags))
		state = BitClean;

	for (i = 0; i < llbitmap->chunks; i++) {
		int ret = llbitmap_write(llbitmap, state, i);

		if (ret < 0) {
			set_bit(BITMAP_WRITE_ERROR, &llbitmap->flags);
			break;
		}
	}
}

/* The return value is only used from resync, where @start == @end. */
static enum llbitmap_state llbitmap_state_machine(struct llbitmap *llbitmap,
						  unsigned long start,
						  unsigned long end,
						  enum llbitmap_action action)
{
	struct mddev *mddev = llbitmap->mddev;
	enum llbitmap_state state = BitNone;
	bool need_resync = false;
	bool need_recovery = false;

	if (test_bit(BITMAP_WRITE_ERROR, &llbitmap->flags))
		return BitNone;

	if (action == BitmapActionInit) {
		llbitmap_init_state(llbitmap);
		return BitNone;
	}

	while (start <= end) {
		ssize_t ret;
		enum llbitmap_state c;

		ret = llbitmap_read(llbitmap, &c, start);
		if (ret < 0) {
			set_bit(BITMAP_WRITE_ERROR, &llbitmap->flags);
			return BitNone;
		}

		if (c < 0 || c >= nr_llbitmap_state) {
			pr_err("%s: invalid bit %lu state %d action %d, forcing resync\n",
			       __func__, start, c, action);
			state = BitNeedSync;
			goto write_bitmap;
		}

		if (c == BitNeedSync)
			need_resync = true;

		state = state_machine[c][action];
		if (state == BitNone) {
			start++;
			continue;
		}

write_bitmap:
		/* Delay raid456 initial recovery to first write. */
		if (c == BitUnwritten && state == BitDirty &&
		    action == BitmapActionStartwrite && is_raid456(mddev)) {
			state = BitNeedSync;
			need_recovery = true;
		}

		ret = llbitmap_write(llbitmap, state, start);
		if (ret < 0) {
			set_bit(BITMAP_WRITE_ERROR, &llbitmap->flags);
			return BitNone;
		}

		if (state == BitNeedSync)
			need_resync = true;
		else if (state == BitDirty &&
			 !timer_pending(&llbitmap->pending_timer))
			mod_timer(&llbitmap->pending_timer,
				  jiffies + mddev->bitmap_info.daemon_sleep * HZ);

		start++;
	}

	if (need_recovery) {
		set_bit(MD_RECOVERY_NEEDED, &mddev->recovery);
		set_bit(MD_RECOVERY_LAZY_RECOVER, &mddev->recovery);
		md_wakeup_thread(mddev->thread);
	} else if (need_resync) {
		set_bit(MD_RECOVERY_NEEDED, &mddev->recovery);
		set_bit(MD_RECOVERY_SYNC, &mddev->recovery);
		md_wakeup_thread(mddev->thread);
	}

	return state;
}

static void llbitmap_raise_barrier(struct llbitmap *llbitmap, int page_idx)
{
	struct llbitmap_barrier *barrier = &llbitmap->barrier[page_idx];

retry:
	if (likely(percpu_ref_tryget_live(&barrier->active))) {
		WRITE_ONCE(barrier->expire, jiffies + BARRIER_IDLE * HZ);
		return;
	}

	wait_event(barrier->wait, !percpu_ref_is_dying(&barrier->active));
	goto retry;
}

static void llbitmap_release_barrier(struct llbitmap *llbitmap, int page_idx)
{
	struct llbitmap_barrier *barrier = &llbitmap->barrier[page_idx];

	percpu_ref_put(&barrier->active);
}

static void llbitmap_suspend(struct llbitmap *llbitmap, int page_idx)
{
	struct llbitmap_barrier *barrier = &llbitmap->barrier[page_idx];

	percpu_ref_kill(&barrier->active);
	wait_event(barrier->wait, percpu_ref_is_zero(&barrier->active));
}

static void llbitmap_resume(struct llbitmap *llbitmap, int page_idx)
{
	struct llbitmap_barrier *barrier = &llbitmap->barrier[page_idx];

	barrier->expire = LONG_MAX;
	percpu_ref_resurrect(&barrier->active);
	wake_up(&barrier->wait);
}

static int llbitmap_check_support(struct mddev *mddev)
{
	if (test_bit(MD_HAS_JOURNAL, &mddev->flags)) {
		pr_notice("md/llbitmap: %s: array with journal cannot have bitmap\n",
			  mdname(mddev));
		return -EBUSY;
	}

	if (mddev->bitmap_info.space == 0) {
		if (mddev->bitmap_info.default_space == 0) {
			pr_notice("md/llbitmap: %s: no space for bitmap\n",
				  mdname(mddev));
			return -ENOSPC;
		}
	}

	if (!mddev->persistent) {
		pr_notice("md/llbitmap: %s: array must be persistent\n",
			  mdname(mddev));
		return -EOPNOTSUPP;
	}

	if (mddev->bitmap_info.file) {
		pr_notice("md/llbitmap: %s: doesn't support bitmap file\n",
			  mdname(mddev));
		return -EOPNOTSUPP;
	}

	if (mddev->bitmap_info.external) {
		pr_notice("md/llbitmap: %s: doesn't support external metadata\n",
			  mdname(mddev));
		return -EOPNOTSUPP;
	}

	if (mddev_is_dm(mddev)) {
		pr_notice("md/llbitmap: %s: doesn't support dm-raid\n",
			  mdname(mddev));
		return -EOPNOTSUPP;
	}

	return 0;
}

static int llbitmap_init(struct llbitmap *llbitmap)
{
	struct mddev *mddev = llbitmap->mddev;
	sector_t blocks = mddev->resync_max_sectors;
	unsigned long chunksize = MIN_CHUNK_SIZE;
	unsigned long chunks = DIV_ROUND_UP(blocks, chunksize);
	unsigned long space = mddev->bitmap_info.space << SECTOR_SHIFT;
	int ret;

	while (chunks > space) {
		chunksize = chunksize << 1;
		chunks = DIV_ROUND_UP(blocks, chunksize);
	}

	llbitmap->chunkshift = ffz(~chunksize);
	llbitmap->chunksize = chunksize;
	llbitmap->chunks = chunks;
	mddev->bitmap_info.daemon_sleep = DEFAULT_DAEMON_SLEEP;

	ret = llbitmap_cache_pages(llbitmap);
	if (ret)
		return ret;

	llbitmap_state_machine(llbitmap, 0, llbitmap->chunks - 1, BitmapActionInit);
	return 0;
}

static int llbitmap_read_sb(struct llbitmap *llbitmap)
{
	struct mddev *mddev = llbitmap->mddev;
	unsigned long daemon_sleep;
	unsigned long chunksize;
	unsigned long events;
	struct page *sb_page;
	bitmap_super_t *sb;
	int ret = -EINVAL;

	if (!mddev->bitmap_info.offset) {
		pr_err("md/llbitmap: %s: no super block found", mdname(mddev));
		return -EINVAL;
	}

	sb_page = llbitmap_read_page(llbitmap, 0);
	if (IS_ERR(sb_page)) {
		pr_err("md/llbitmap: %s: read super block failed",
		       mdname(mddev));
		ret = -EIO;
		goto out;
	}

	sb = kmap_local_page(sb_page);
	if (sb->magic != cpu_to_le32(BITMAP_MAGIC)) {
		pr_err("md/llbitmap: %s: invalid super block magic number",
		       mdname(mddev));
		goto out_put_page;
	}

	if (sb->version != cpu_to_le32(LLBITMAP_MAJOR_HI)) {
		pr_err("md/llbitmap: %s: invalid super block version",
		       mdname(mddev));
		goto out_put_page;
	}

	if (memcmp(sb->uuid, mddev->uuid, 16)) {
		pr_err("md/llbitmap: %s: bitmap superblock UUID mismatch\n",
		       mdname(mddev));
		goto out_put_page;
	}

	if (mddev->bitmap_info.space == 0) {
		int room = le32_to_cpu(sb->sectors_reserved);

		if (room)
			mddev->bitmap_info.space = room;
		else
			mddev->bitmap_info.space = mddev->bitmap_info.default_space;
	}
	llbitmap->flags = le32_to_cpu(sb->state);
	if (test_and_clear_bit(BITMAP_FIRST_USE, &llbitmap->flags)) {
		ret = llbitmap_init(llbitmap);
		goto out_put_page;
	}

	chunksize = le32_to_cpu(sb->chunksize);
	if (!is_power_of_2(chunksize)) {
		pr_err("md/llbitmap: %s: chunksize not a power of 2",
		       mdname(mddev));
		goto out_put_page;
	}

	if (chunksize < DIV_ROUND_UP(mddev->resync_max_sectors,
				     mddev->bitmap_info.space << SECTOR_SHIFT)) {
		pr_err("md/llbitmap: %s: chunksize too small %lu < %llu / %lu",
		       mdname(mddev), chunksize, mddev->resync_max_sectors,
		       mddev->bitmap_info.space);
		goto out_put_page;
	}

	daemon_sleep = le32_to_cpu(sb->daemon_sleep);
	if (daemon_sleep < 1 || daemon_sleep > MAX_SCHEDULE_TIMEOUT / HZ) {
		pr_err("md/llbitmap: %s: daemon sleep %lu period out of range",
		       mdname(mddev), daemon_sleep);
		goto out_put_page;
	}

	if (le32_to_cpu(sb->write_behind))
		pr_warn("md/llbitmap: %s: slow disk is not supported",
			mdname(mddev));

	events = le64_to_cpu(sb->events);
	if (events < mddev->events) {
		pr_warn("md/llbitmap :%s: bitmap file is out of date (%lu < %llu) -- forcing full recovery",
			mdname(mddev), events, mddev->events);
		set_bit(BITMAP_STALE, &llbitmap->flags);
	}

	sb->sync_size = cpu_to_le64(mddev->resync_max_sectors);
	mddev->bitmap_info.chunksize = chunksize;
	mddev->bitmap_info.daemon_sleep = daemon_sleep;

	llbitmap->chunksize = chunksize;
	llbitmap->chunks = DIV_ROUND_UP(mddev->resync_max_sectors, chunksize);
	llbitmap->chunkshift = ffz(~chunksize);
	ret = llbitmap_cache_pages(llbitmap);

out_put_page:
	__free_page(sb_page);
out:
	kunmap_local(sb);
	return ret;
}

static void llbitmap_pending_timer_fn(struct timer_list *t)
{
	struct llbitmap *llbitmap = from_timer(llbitmap, t, pending_timer);

	if (work_busy(&llbitmap->daemon_work)) {
		pr_warn("daemon_work not finished\n");
		set_bit(BITMAP_DAEMON_BUSY, &llbitmap->flags);
		return;
	}

	queue_work(md_llbitmap_io_wq, &llbitmap->daemon_work);
}

static void md_llbitmap_daemon_fn(struct work_struct *work)
{
	struct llbitmap *llbitmap =
		container_of(work, struct llbitmap, daemon_work);
	unsigned long start;
	unsigned long end;
	bool restart;
	int idx;

	if (llbitmap->mddev->degraded)
		return;

retry:
	start = 0;
	end = min(llbitmap->chunks, PAGE_SIZE - BITMAP_SB_SIZE) - 1;
	restart = false;

	for (idx = 0; idx < llbitmap->nr_pages; idx++) {
		struct llbitmap_barrier *barrier = &llbitmap->barrier[idx];

		if (idx > 0) {
			start = end + 1;
			end = min(end + PAGE_SIZE, llbitmap->chunks - 1);
		}

		if (!test_bit(LLPageFlush, &barrier->flags) &&
		    time_before(jiffies, barrier->expire)) {
			restart = true;
			continue;
		}

		llbitmap_suspend(llbitmap, idx);
		llbitmap_state_machine(llbitmap, start, end, BitmapActionDaemon);
		llbitmap_resume(llbitmap, idx);
	}

	/*
	 * If the daemon took a long time to finish, retry to prevent missing
	 * clearing dirty bits.
	 */
	if (test_and_clear_bit(BITMAP_DAEMON_BUSY, &llbitmap->flags))
		goto retry;

	/* If some page is dirty but not expired, setup timer again */
	if (restart)
		mod_timer(&llbitmap->pending_timer,
			  jiffies + llbitmap->mddev->bitmap_info.daemon_sleep * HZ);
}

static int llbitmap_create(struct mddev *mddev)
{
	struct llbitmap *llbitmap;
	int ret;

	ret = llbitmap_check_support(mddev);
	if (ret)
		return ret;

	llbitmap = kzalloc(sizeof(*llbitmap), GFP_KERNEL);
	if (!llbitmap)
		return -ENOMEM;

	llbitmap->mddev = mddev;
	llbitmap->io_size = bdev_logical_block_size(mddev->gendisk->part0);
	llbitmap->bits_per_page = PAGE_SIZE / llbitmap->io_size;

	timer_setup(&llbitmap->pending_timer, llbitmap_pending_timer_fn, 0);
	INIT_WORK(&llbitmap->daemon_work, md_llbitmap_daemon_fn);

	mutex_lock(&mddev->bitmap_info.mutex);
	mddev->bitmap = llbitmap;
	ret = llbitmap_read_sb(llbitmap);
	mutex_unlock(&mddev->bitmap_info.mutex);
	if (ret)
		goto err_out;

	return 0;

err_out:
	kfree(llbitmap);
	return ret;
}

static int llbitmap_resize(struct mddev *mddev, sector_t blocks, int chunksize)
{
	struct llbitmap *llbitmap = mddev->bitmap;
	unsigned long chunks;

	if (chunksize == 0)
		chunksize = llbitmap->chunksize;

	/* If there is enough space, leave the chunksize unchanged. */
	chunks = DIV_ROUND_UP(blocks, chunksize);
	while (chunks > mddev->bitmap_info.space << SECTOR_SHIFT) {
		chunksize = chunksize << 1;
		chunks = DIV_ROUND_UP(blocks, chunksize);
	}

	llbitmap->chunkshift = ffz(~chunksize);
	llbitmap->chunksize = chunksize;
	llbitmap->chunks = chunks;

	return 0;
}

static int llbitmap_load(struct mddev *mddev)
{
	enum llbitmap_action action = BitmapActionReload;
	struct llbitmap *llbitmap = mddev->bitmap;

	if (test_and_clear_bit(BITMAP_STALE, &llbitmap->flags))
		action = BitmapActionStale;

	llbitmap_state_machine(llbitmap, 0, llbitmap->chunks - 1, action);
	return 0;
}

static void llbitmap_destroy(struct mddev *mddev)
{
	struct llbitmap *llbitmap = mddev->bitmap;

	if (!llbitmap)
		return;

	mutex_lock(&mddev->bitmap_info.mutex);

	timer_delete_sync(&llbitmap->pending_timer);
	flush_workqueue(md_llbitmap_io_wq);
	flush_workqueue(md_llbitmap_unplug_wq);

	mddev->bitmap = NULL;
	llbitmap_free_pages(llbitmap);
	kfree(llbitmap);
	mutex_unlock(&mddev->bitmap_info.mutex);
}

static int llbitmap_startwrite(struct mddev *mddev, sector_t offset,
			       unsigned long sectors)
{
	struct llbitmap *llbitmap = mddev->bitmap;
	unsigned long start = offset >> llbitmap->chunkshift;
	unsigned long end = (offset + sectors - 1) >> llbitmap->chunkshift;
	int page_start = (start + BITMAP_SB_SIZE) >> PAGE_SHIFT;
	int page_end = (end + BITMAP_SB_SIZE) >> PAGE_SHIFT;

	llbitmap_state_machine(llbitmap, start, end, BitmapActionStartwrite);


	while (page_start <= page_end) {
		llbitmap_raise_barrier(llbitmap, page_start);
		page_start++;
	}

	return 0;
}

static void llbitmap_endwrite(struct mddev *mddev, sector_t offset,
			      unsigned long sectors)
{
	struct llbitmap *llbitmap = mddev->bitmap;
	unsigned long start = offset >> llbitmap->chunkshift;
	unsigned long end = (offset + sectors - 1) >> llbitmap->chunkshift;
	int page_start = (start + BITMAP_SB_SIZE) >> PAGE_SHIFT;
	int page_end = (end + BITMAP_SB_SIZE) >> PAGE_SHIFT;

	while (page_start <= page_end) {
		llbitmap_release_barrier(llbitmap, page_start);
		page_start++;
	}
}

static int llbitmap_start_discard(struct mddev *mddev, sector_t offset,
				  unsigned long sectors)
{
	struct llbitmap *llbitmap = mddev->bitmap;
	unsigned long start = DIV_ROUND_UP(offset, llbitmap->chunksize);
	unsigned long end = (offset + sectors - 1) >> llbitmap->chunkshift;
	int page_start = (start + BITMAP_SB_SIZE) >> PAGE_SHIFT;
	int page_end = (end + BITMAP_SB_SIZE) >> PAGE_SHIFT;

	llbitmap_state_machine(llbitmap, start, end, BitmapActionDiscard);

	while (page_start <= page_end) {
		llbitmap_raise_barrier(llbitmap, page_start);
		page_start++;
	}

	return 0;
}

static void llbitmap_end_discard(struct mddev *mddev, sector_t offset,
				 unsigned long sectors)
{
	struct llbitmap *llbitmap = mddev->bitmap;
	unsigned long start = DIV_ROUND_UP(offset, llbitmap->chunksize);
	unsigned long end = (offset + sectors - 1) >> llbitmap->chunkshift;
	int page_start = (start + BITMAP_SB_SIZE) >> PAGE_SHIFT;
	int page_end = (end + BITMAP_SB_SIZE) >> PAGE_SHIFT;

	while (page_start <= page_end) {
		llbitmap_release_barrier(llbitmap, page_start);
		page_start++;
	}
}

static void llbitmap_unplug_fn(struct work_struct *work)
{
	struct llbitmap_unplug_work *unplug_work =
		container_of(work, struct llbitmap_unplug_work, work);
	struct llbitmap *llbitmap = unplug_work->llbitmap;
	struct blk_plug plug;
	int i;

	blk_start_plug(&plug);

	for (i = 0; i < llbitmap->nr_pages; i++) {
		if (!test_bit(LLPageDirty, &llbitmap->barrier[i].flags) ||
		    !test_and_clear_bit(LLPageDirty, &llbitmap->barrier[i].flags))
			continue;

		llbitmap_write_page(llbitmap, i);
	}

	blk_finish_plug(&plug);
	md_super_wait(llbitmap->mddev);
	complete(unplug_work->done);
}

static bool llbitmap_dirty(struct llbitmap *llbitmap)
{
	int i;

	for (i = 0; i < llbitmap->nr_pages; i++)
		if (test_bit(LLPageDirty, &llbitmap->barrier[i].flags))
			return true;

	return false;
}

static void llbitmap_unplug(struct mddev *mddev, bool sync)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct llbitmap *llbitmap = mddev->bitmap;
	struct llbitmap_unplug_work unplug_work = {
		.llbitmap = llbitmap,
		.done = &done,
	};

	if (!llbitmap_dirty(llbitmap))
		return;

	INIT_WORK_ONSTACK(&unplug_work.work, llbitmap_unplug_fn);
	queue_work(md_llbitmap_unplug_wq, &unplug_work.work);
	wait_for_completion(&done);
	destroy_work_on_stack(&unplug_work.work);
}

static void llbitmap_flush(struct mddev *mddev)
{
	struct llbitmap *llbitmap = mddev->bitmap;
	struct blk_plug plug;
	int i;

	for (i = 0; i < llbitmap->nr_pages; i++)
		set_bit(LLPageFlush, &llbitmap->barrier[i].flags);

	timer_delete_sync(&llbitmap->pending_timer);
	queue_work(md_llbitmap_io_wq, &llbitmap->daemon_work);
	flush_work(&llbitmap->daemon_work);

	blk_start_plug(&plug);
	for (i = 0; i < llbitmap->nr_pages; i++) {
		/* mark all bits as dirty */
		bitmap_fill(llbitmap->barrier[i].dirty, llbitmap->bits_per_page);
		llbitmap_write_page(llbitmap, i);
	}
	blk_finish_plug(&plug);
	md_super_wait(llbitmap->mddev);
}

/* This is used for raid5 lazy initial recovery */
static bool llbitmap_blocks_synced(struct mddev *mddev, sector_t offset)
{
	struct llbitmap *llbitmap = mddev->bitmap;
	unsigned long p = offset >> llbitmap->chunkshift;
	enum llbitmap_state c;
	int ret;

	ret = llbitmap_read(llbitmap, &c, p);
	if (ret < 0) {
		set_bit(BITMAP_WRITE_ERROR, &llbitmap->flags);
		return false;
	}

	return c == BitClean || c == BitDirty;
}

static sector_t llbitmap_skip_sync_blocks(struct mddev *mddev, sector_t offset)
{
	struct llbitmap *llbitmap = mddev->bitmap;
	unsigned long p = offset >> llbitmap->chunkshift;
	int blocks = llbitmap->chunksize - (offset & (llbitmap->chunksize - 1));
	enum llbitmap_state c;
	int ret;

	ret = llbitmap_read(llbitmap, &c, p);
	if (ret < 0) {
		set_bit(BITMAP_WRITE_ERROR, &llbitmap->flags);
		return 0;
	}

	/* always skip unwritten blocks */
	if (c == BitUnwritten)
		return blocks;

	/* For resync also skip clean/dirty blocks */
	if ((c == BitClean || c == BitDirty) &&
	    test_bit(MD_RECOVERY_SYNC, &mddev->recovery) &&
	    !test_bit(MD_RECOVERY_REQUESTED, &mddev->recovery))
		return blocks;

	return 0;
}

static bool llbitmap_start_sync(struct mddev *mddev, sector_t offset,
				sector_t *blocks, bool degraded)
{
	struct llbitmap *llbitmap = mddev->bitmap;
	unsigned long p = offset >> llbitmap->chunkshift;

	/*
	 * Handle one bit at a time, this is much simpler. And it doesn't matter
	 * if md_do_sync() loop more times.
	 */
	*blocks = llbitmap->chunksize - (offset & (llbitmap->chunksize - 1));
	return llbitmap_state_machine(llbitmap, p, p, BitmapActionStartsync) == BitSyncing;
}

static void llbitmap_end_sync(struct mddev *mddev, sector_t offset,
			      sector_t *blocks)
{
	struct llbitmap *llbitmap = mddev->bitmap;
	unsigned long p = offset >> llbitmap->chunkshift;

	*blocks = llbitmap->chunksize - (offset & (llbitmap->chunksize - 1));
	llbitmap_state_machine(llbitmap, p, llbitmap->chunks - 1, BitmapActionAbortsync);
}

static void llbitmap_close_sync(struct mddev *mddev)
{
	struct llbitmap *llbitmap = mddev->bitmap;
	int i;

	for (i = 0; i < llbitmap->nr_pages; i++) {
		struct llbitmap_barrier *barrier = &llbitmap->barrier[i];

		/* let daemon_fn clear dirty bits immediately */
		WRITE_ONCE(barrier->expire, jiffies);
	}

	llbitmap_state_machine(llbitmap, 0, llbitmap->chunks - 1, BitmapActionEndsync);
}

static bool llbitmap_enabled(void *data)
{
	struct llbitmap *llbitmap = data;

	return llbitmap && !test_bit(BITMAP_WRITE_ERROR, &llbitmap->flags);
}

static void llbitmap_dirty_bits(struct mddev *mddev, unsigned long s,
				unsigned long e)
{
	llbitmap_state_machine(mddev->bitmap, s, e, BitmapActionStartwrite);
}

static void llbitmap_write_sb(struct llbitmap *llbitmap)
{
	int nr_bits = round_up(BITMAP_SB_SIZE, llbitmap->io_size) / llbitmap->io_size;

	bitmap_fill(llbitmap->barrier[0].dirty, nr_bits);
	llbitmap_write_page(llbitmap, 0);
	md_super_wait(llbitmap->mddev);
}

static void llbitmap_update_sb(void *data)
{
	struct llbitmap *llbitmap = data;
	struct mddev *mddev = llbitmap->mddev;
	struct page *sb_page;
	bitmap_super_t *sb;

	if (test_bit(BITMAP_WRITE_ERROR, &llbitmap->flags))
		return;

	sb_page = llbitmap_read_page(llbitmap, 0);
	if (IS_ERR(sb_page)) {
		pr_err("%s: %s: read super block failed", __func__,
		       mdname(mddev));
		set_bit(BITMAP_WRITE_ERROR, &llbitmap->flags);
		return;
	}

	if (mddev->events < llbitmap->events_cleared)
		llbitmap->events_cleared = mddev->events;

	sb = kmap_local_page(sb_page);
	sb->events = cpu_to_le64(mddev->events);
	sb->state = cpu_to_le32(llbitmap->flags);
	sb->chunksize = cpu_to_le32(llbitmap->chunksize);
	sb->sync_size = cpu_to_le64(mddev->resync_max_sectors);
	sb->events_cleared = cpu_to_le64(llbitmap->events_cleared);
	sb->sectors_reserved = cpu_to_le32(mddev->bitmap_info.space);
	sb->daemon_sleep = cpu_to_le32(mddev->bitmap_info.daemon_sleep);

	kunmap_local(sb);
	llbitmap_write_sb(llbitmap);
}

static int llbitmap_get_stats(void *data, struct md_bitmap_stats *stats)
{
	struct llbitmap *llbitmap = data;

	memset(stats, 0, sizeof(*stats));

	stats->missing_pages = 0;
	stats->pages = llbitmap->nr_pages;
	stats->file_pages = llbitmap->nr_pages;

	return 0;
}

static void llbitmap_write_all(struct mddev *mddev)
{

}

static void llbitmap_daemon_work(struct mddev *mddev)
{

}

static void llbitmap_start_behind_write(struct mddev *mddev)
{

}

static void llbitmap_end_behind_write(struct mddev *mddev)
{

}

static void llbitmap_wait_behind_writes(struct mddev *mddev)
{

}

static void llbitmap_cond_end_sync(struct mddev *mddev, sector_t sector,
				   bool force)
{
}

static void llbitmap_sync_with_cluster(struct mddev *mddev,
				       sector_t old_lo, sector_t old_hi,
				       sector_t new_lo, sector_t new_hi)
{

}

static void *llbitmap_get_from_slot(struct mddev *mddev, int slot)
{
	return ERR_PTR(-EOPNOTSUPP);
}

static int llbitmap_copy_from_slot(struct mddev *mddev, int slot, sector_t *low,
				   sector_t *high, bool clear_bits)
{
	return -EOPNOTSUPP;
}

static void llbitmap_set_pages(void *data, unsigned long pages)
{
}

static void llbitmap_free(void *data)
{
}

static ssize_t bits_show(struct mddev *mddev, char *page)
{
	struct llbitmap *llbitmap;
	int bits[nr_llbitmap_state] = {0};
	loff_t start = 0;

	mutex_lock(&mddev->bitmap_info.mutex);
	llbitmap = mddev->bitmap;
	if (!llbitmap) {
		mutex_unlock(&mddev->bitmap_info.mutex);
		return sprintf(page, "no bitmap\n");
	}

	if (test_bit(BITMAP_WRITE_ERROR, &llbitmap->flags)) {
		mutex_unlock(&mddev->bitmap_info.mutex);
		return sprintf(page, "bitmap io error\n");
	}

	while (start < llbitmap->chunks) {
		ssize_t ret;
		enum llbitmap_state c;

		ret = llbitmap_read(llbitmap, &c, start);
		if (ret < 0) {
			set_bit(BITMAP_WRITE_ERROR, &llbitmap->flags);
			mutex_unlock(&mddev->bitmap_info.mutex);
			return sprintf(page, "bitmap io error\n");
		}

		if (c < 0 || c >= nr_llbitmap_state)
			pr_err("%s: invalid bit %llu state %d\n",
			       __func__, start, c);
		else
			bits[c]++;
		start++;
	}

	mutex_unlock(&mddev->bitmap_info.mutex);
	return sprintf(page, "unwritten %d\nclean %d\ndirty %d\nneed sync %d\nsyncing %d\n",
		       bits[BitUnwritten], bits[BitClean], bits[BitDirty],
		       bits[BitNeedSync], bits[BitSyncing]);
}

static struct md_sysfs_entry llbitmap_bits =
__ATTR_RO(bits);

static ssize_t metadata_show(struct mddev *mddev, char *page)
{
	struct llbitmap *llbitmap;
	ssize_t ret;

	mutex_lock(&mddev->bitmap_info.mutex);
	llbitmap = mddev->bitmap;
	if (!llbitmap) {
		mutex_unlock(&mddev->bitmap_info.mutex);
		return sprintf(page, "no bitmap\n");
	}

	ret =  sprintf(page, "chunksize %lu\nchunkshift %lu\nchunks %lu\noffset %llu\ndaemon_sleep %lu\n",
		       llbitmap->chunksize, llbitmap->chunkshift,
		       llbitmap->chunks, mddev->bitmap_info.offset,
		       llbitmap->mddev->bitmap_info.daemon_sleep);
	mutex_unlock(&mddev->bitmap_info.mutex);

	return ret;
}

static struct md_sysfs_entry llbitmap_metadata =
__ATTR_RO(metadata);

static ssize_t
daemon_sleep_show(struct mddev *mddev, char *page)
{
	return sprintf(page, "%lu\n", mddev->bitmap_info.daemon_sleep);
}

static ssize_t
daemon_sleep_store(struct mddev *mddev, const char *buf, size_t len)
{
	unsigned long timeout;
	int rv = kstrtoul(buf, 10, &timeout);

	if (rv)
		return rv;

	mddev->bitmap_info.daemon_sleep = timeout;
	return len;
}

static struct md_sysfs_entry llbitmap_daemon_sleep =
__ATTR_RW(daemon_sleep);

static struct attribute *md_llbitmap_attrs[] = {
	&llbitmap_bits.attr,
	&llbitmap_metadata.attr,
	&llbitmap_daemon_sleep.attr,
	NULL
};
