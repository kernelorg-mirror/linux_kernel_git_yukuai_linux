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
 * special illustration:
 * - Unwritten is special state, which means user never write data, hence there
 *   is no need to resync/recover data. This is safe if user create filesystems
 *   for the array, filesystem will make sure user will get zero data for
 *   unwritten blocks.
 * - After resync is done, change state from Syncing to Dirty first, in case
 *   Startwrite happen before the state is Clean.
 */

#define BITMAP_MAX_PAGES 32
#define BITMAP_SB_SIZE 1024

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
	 * */
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

struct llbitmap {
	struct mddev *mddev;
	/* hidden disk to manage bitmap IO */
	struct gendisk *bitmap_disk;
	/* opened hidden disk */
	struct file *bitmap_file;
	int nr_pages;
	struct page *pages[BITMAP_MAX_PAGES];

	struct bio_set bio_set;
	struct bio_list retry_list;
	struct work_struct retry_work;
	spinlock_t retry_lock;

	/* shift of one chunk */
	unsigned long chunkshift;
	/* size of one chunk in sector */
	unsigned long chunksize;
	/* total number of chunks */
	unsigned long chunks;
	/* fires on first BitDirty state */
	struct timer_list pending_timer;
	struct work_struct daemon_work;

	unsigned long flags;
	__u64	events_cleared;
};

static char state_machine[nr_llbitmap_state][nr_llbitmap_action] = {
	[BitUnwritten] = {BitDirty, BitNone, BitNone, BitNone, BitNone, BitNone, BitNone, BitNone},
	[BitClean] = {BitDirty, BitNone, BitNone, BitNone, BitNone, BitNone, BitUnwritten, BitNeedSync},
	[BitDirty] = {BitNone, BitNone, BitNone, BitNone, BitNeedSync, BitClean, BitUnwritten, BitNeedSync},
	[BitNeedSync] = {BitNone, BitSyncing, BitNone, BitNone, BitNone, BitNone, BitUnwritten, BitNone},
	[BitSyncing] = {BitNone, BitSyncing, BitDirty, BitNeedSync, BitNeedSync, BitNone, BitUnwritten, BitNeedSync},
};

static enum llbitmap_state state_from_page(struct page *page, loff_t pos)
{
	u8 *p = kmap_local_page(page);
	enum llbitmap_state state = p[offset_in_page(pos)];

	kunmap_local(p);
	return state;
}

static void state_to_page(struct page *page, enum llbitmap_state state,
			  loff_t pos)
{
	u8 *p = kmap_local_page(page);

	p[offset_in_page(pos)] = state;
	set_page_dirty(page);
	kunmap_local(p);
}

static int llbitmap_read(struct llbitmap *llbitmap, enum llbitmap_state *state,
			 loff_t pos)
{
	pos += BITMAP_SB_SIZE;
	*state = state_from_page(llbitmap->pages[pos >> PAGE_SHIFT], pos);
	return 0;
}

static int llbitmap_write(struct llbitmap *llbitmap, enum llbitmap_state state,
			  loff_t pos)
{
	pos += BITMAP_SB_SIZE;
	state_to_page(llbitmap->pages[pos >> PAGE_SHIFT], state, pos);
	return 0;
}

/* The return value is only used from resync, where @start == @end. */
static enum llbitmap_state llbitmap_state_machine(struct llbitmap *llbitmap,
						  unsigned long start,
						  unsigned long end,
						  enum llbitmap_action action)
{
	struct mddev *mddev = llbitmap->mddev;
	enum llbitmap_state state = BitNone;
	bool need_recovery = false;

	if (test_bit(BITMAP_WRITE_ERROR, &llbitmap->flags))
		return BitNone;

	while (start <= end) {
		ssize_t ret;
		enum llbitmap_state c;

		if (action == BitmapActionInit) {
			state = BitUnwritten;
			ret = llbitmap_write(llbitmap, state, start);
			if (ret < 0) {
				set_bit(BITMAP_WRITE_ERROR, &llbitmap->flags);
				return BitNone;
			}

			start++;
			continue;
		}

		ret = llbitmap_read(llbitmap, &c, start);
		if (ret < 0) {
			set_bit(BITMAP_WRITE_ERROR, &llbitmap->flags);
			return BitNone;
		}

		if (c < 0 || c >= nr_llbitmap_state) {
			pr_err("%s: invalid bit %lu state %d action %d, forcing resync\n",
			       __func__, start, c, action);
			c = BitNeedSync;
			goto write_bitmap;
		}

		if (c == BitNeedSync)
			need_recovery = true;

		state = state_machine[c][action];
		if (state == BitNone) {
			start++;
			continue;
		}

write_bitmap:
		ret = llbitmap_write(llbitmap, state, start);
		if (ret < 0) {
			set_bit(BITMAP_WRITE_ERROR, &llbitmap->flags);
			return BitNone;
		}

		if (state == BitNeedSync)
			need_recovery = true;
		else if (state == BitDirty &&
			 !timer_pending(&llbitmap->pending_timer))
			mod_timer(&llbitmap->pending_timer,
				  jiffies + mddev->bitmap_info.daemon_sleep * HZ);

		start++;
	}

	if (need_recovery) {
		set_bit(MD_RECOVERY_NEEDED, &mddev->recovery);
		set_bit(MD_RECOVERY_SYNC, &mddev->recovery);
		md_wakeup_thread(mddev->thread);
	}

	return state;
}
