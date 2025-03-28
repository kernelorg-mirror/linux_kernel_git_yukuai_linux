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
 *
 * ##### Bitmap IO
 *
 * A hidden disk, named mdxxx_bitmap, is created for bitmap, see details in
 * llbitmap_add_disk(). And a file is created as well to manage bitmap IO for
 * this disk, see details in llbitmap_open_disk(). Read/write bitmap is
 * converted to buffer IO to this file.
 */

#define BITMAP_MAX_SECTOR (128 * 2)
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

struct llbitmap_bio {
	struct md_rdev *rdev;
	struct bio bio;
};

static struct workqueue_struct *md_llbitmap_io_wq;

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

static void llbitmap_end_write(struct bio *bio)
{
	struct bio *parent = bio->bi_private;
	struct llbitmap_bio *llbitmap_bio;
	struct md_rdev *rdev;

	if (bio->bi_status == BLK_STS_OK) {
		WRITE_ONCE(parent->bi_status, BLK_STS_OK);
	} else {
		llbitmap_bio = container_of(bio, struct llbitmap_bio, bio);
		rdev = llbitmap_bio->rdev;

		pr_err("%s: %s: bitmap write failed for %pg\n", __func__,
		       mdname(rdev->mddev), rdev->bdev);
		md_error(rdev->mddev, rdev);
	}

	bio_put(bio);
	bio_endio(parent);
}

static void md_llbitmap_retry_read(struct llbitmap *llbitmap, struct bio *bio)
{
	unsigned long flags;

	spin_lock_irqsave(&llbitmap->retry_lock, flags);
	bio_list_add(&llbitmap->retry_list, bio);
	queue_work(md_llbitmap_io_wq, &llbitmap->retry_work);
	spin_unlock_irqrestore(&llbitmap->retry_lock, flags);
}

static void llbitmap_end_read(struct bio *bio)
{
	struct bio *parent = bio->bi_private;
	struct llbitmap_bio *llbitmap_bio;
	struct llbitmap *llbitmap;
	struct md_rdev *rdev;

	if (bio->bi_status == BLK_STS_OK) {
		WRITE_ONCE(parent->bi_status, BLK_STS_OK);
		bio_put(bio);
		bio_endio(parent);
		return;
	}

	llbitmap_bio = container_of(bio, struct llbitmap_bio, bio);
	rdev = llbitmap_bio->rdev;
	pr_err("%s: %s: bitmap read failed for %pg\n", __func__,
	       mdname(rdev->mddev), rdev->bdev);
	md_error(rdev->mddev, rdev);
	bio_put(bio);
	md_llbitmap_retry_read(llbitmap, parent);
}

static void md_llbitmap_retry_fn(struct work_struct *work)
{
	struct llbitmap *llbitmap =
		container_of(work, struct llbitmap, retry_work);
	struct mddev *mddev = llbitmap->mddev;
	struct md_rdev *rdev;
	struct bio *bio;

again:
	spin_lock_irq(&llbitmap->retry_lock);
	bio = bio_list_pop(&llbitmap->retry_list);
	spin_unlock_irq(&llbitmap->retry_lock);

	if (!bio)
		return;

	rdev_for_each(rdev, mddev) {
		struct llbitmap_bio *llbitmap_bio;
		struct bio *new;

		if (rdev->raid_disk < 0 || test_bit(Faulty, &rdev->flags))
			continue;

		new = bio_alloc_clone(rdev->bdev, bio, GFP_NOIO,
				      &llbitmap->bio_set);
		new->bi_iter.bi_sector = bio->bi_iter.bi_sector +
					 rdev->sb_start +
					 mddev->bitmap_info.offset;
		new->bi_opf |= REQ_SYNC | REQ_IDLE | REQ_META;
		new->bi_private = bio;
		new->bi_end_io = llbitmap_end_read;

		llbitmap_bio = container_of(new, struct llbitmap_bio, bio);
		llbitmap_bio->rdev = rdev;

		submit_bio_noacct(new);
		goto again;
	}
}

static void llbitmap_submit_bio(struct bio *bio)
{
	struct mddev *mddev = bio->bi_bdev->bd_disk->private_data;
	struct llbitmap *llbitmap = mddev->bitmap;
	struct llbitmap_bio *llbitmap_bio;
	struct md_rdev *rdev;
	struct bio *new;

	if (unlikely(bio->bi_opf & REQ_PREFLUSH))
		bio->bi_opf &= ~REQ_PREFLUSH;

	if (!bio_sectors(bio)) {
		bio_endio(bio);
		return;
	}

	/* status will be cleared if any member disk IO succeed */
	bio->bi_status = BLK_STS_IOERR;

	rdev_for_each(rdev, mddev) {
		if (rdev->raid_disk < 0 || test_bit(Faulty, &rdev->flags))
			continue;

		new = bio_alloc_clone(rdev->bdev, bio, GFP_NOIO,
				      &llbitmap->bio_set);
		new->bi_iter.bi_sector = bio->bi_iter.bi_sector +
					 rdev->sb_start +
					 mddev->bitmap_info.offset;
		new->bi_opf |= REQ_SYNC | REQ_IDLE | REQ_META;

		llbitmap_bio = container_of(new, struct llbitmap_bio, bio);
		llbitmap_bio->rdev = rdev;
		bio_inc_remaining(bio);
		new->bi_private = bio;

		if (bio_data_dir(bio) == WRITE) {
			new->bi_end_io = llbitmap_end_write;
			new->bi_opf |= REQ_FUA;
			submit_bio_noacct(new);
			continue;
		}

		new->bi_end_io = llbitmap_end_read;
		submit_bio_noacct(new);
		break;
	}

	bio_endio(bio);
}

const struct block_device_operations llbitmap_fops = {
	.owner = THIS_MODULE,
	.submit_bio = llbitmap_submit_bio,
};

static int llbitmap_add_disk(struct llbitmap *llbitmap)
{
	struct mddev *mddev = llbitmap->mddev;
	struct gendisk *disk = blk_alloc_disk(&mddev->gendisk->queue->limits,
					      NUMA_NO_NODE);
	int ret;

	if (IS_ERR(disk))
		return PTR_ERR(disk);

	sprintf(disk->disk_name, "%s_bitmap", mdname(mddev));
	disk->flags |= GENHD_FL_HIDDEN;
	disk->fops = &llbitmap_fops;

	ret = add_disk(disk);
	if (ret) {
		put_disk(disk);
		return ret;
	}

	set_capacity(disk, BITMAP_MAX_SECTOR);
	disk->private_data = mddev;
	llbitmap->bitmap_disk = disk;
	return 0;
}

static void llbitmap_del_disk(struct llbitmap *llbitmap)
{
	struct gendisk *disk = llbitmap->bitmap_disk;

	if (!disk)
		return;

	llbitmap->bitmap_disk = NULL;
	del_gendisk(disk);
	put_disk(disk);
}

static int llbitmap_open_disk(struct llbitmap *llbitmap)
{
	struct gendisk *disk = llbitmap->bitmap_disk;
	struct file *bitmap_file;

	bitmap_file = bdev_file_alloc(disk->part0,
				      BLK_OPEN_READ | BLK_OPEN_WRITE);
	if (IS_ERR(bitmap_file))
		return PTR_ERR(bitmap_file);

	/* corresponding to the blkdev_put_no_open() from blkdev_release() */
	get_device(disk_to_dev(disk));

	bitmap_file->f_flags |= O_LARGEFILE;
	bitmap_file->f_mode |= FMODE_CAN_ODIRECT;
	bitmap_file->f_mapping = disk->part0->bd_mapping;
	bitmap_file->f_wb_err = filemap_sample_wb_err(bitmap_file->f_mapping);

	/* not actually opened, let blkdev_release() know */
	bitmap_file->private_data = ERR_PTR(-ENODEV);
	llbitmap->bitmap_file = bitmap_file;
	return 0;
}

static void llbitmap_close_disk(struct llbitmap *llbitmap)
{
	struct file *bitmap_file = llbitmap->bitmap_file;

	if (!bitmap_file)
		return;

	llbitmap->bitmap_file = NULL;
	fput(bitmap_file);
}

