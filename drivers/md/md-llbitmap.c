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
 *
 * IO fast path will set bits to dirty, and those dirty bits will be cleared
 * by daemon after IO is done. llbitmap_barrier is used to syncronize between
 * IO path and daemon;
 *
 * IO patch:
 *  1) try to grab a reference, if succeed, set expire time after 5s and return;
 *  2) wait for daemon to finish clearing dirty bits;
 *
 * Daemon(Daemon will be wake up every daemon_sleep seconds):
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

/*
 * page level barrier to synchronize between dirty bit by write IO and clean bit
 * by daemon.
 */
struct llbitmap_barrier {
	struct percpu_ref active;
	unsigned long expire;
	bool flush;
	wait_queue_head_t wait;
} ____cacheline_aligned_in_smp;

struct llbitmap {
	struct mddev *mddev;
	/* hidden disk to manage bitmap IO */
	struct gendisk *bitmap_disk;
	/* opened hidden disk */
	struct file *bitmap_file;
	int nr_pages;
	struct page *pages[BITMAP_MAX_PAGES];
	struct llbitmap_barrier barrier[BITMAP_MAX_PAGES];

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

static void llbitmap_free_pages(struct llbitmap *llbitmap)
{
	int i;

	for (i = 0; i < BITMAP_MAX_PAGES; i++) {
		struct page *page = llbitmap->pages[i];

		if (!page)
			return;

		llbitmap->pages[i] = NULL;
		put_page(page);
		percpu_ref_exit(&llbitmap->barrier[i].active);
	}
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
		page = read_mapping_page(llbitmap->bitmap_file->f_mapping, i, NULL);
		if (IS_ERR(page)) {
			int ret = PTR_ERR(page);

			llbitmap_free_pages(llbitmap);
			return ret;
		}

		if (percpu_ref_init(&llbitmap->barrier[i].active, active_release,
				    PERCPU_REF_ALLOW_REINIT, GFP_KERNEL)) {
			put_page(page);
			return -ENOMEM;
		}

		init_waitqueue_head(&llbitmap->barrier[i].wait);
		llbitmap->pages[i++] = page;
	}

	return 0;
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

	sb_page = read_mapping_page(llbitmap->bitmap_file->f_mapping, 0, NULL);
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
	put_page(sb_page);
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
	unsigned long start = 0;
	unsigned long end = min(llbitmap->chunks, PAGE_SIZE - BITMAP_SB_SIZE) - 1;
	bool restart = false;
	int page_idx = 0;

	while (page_idx < llbitmap->nr_pages) {
		struct llbitmap_barrier *barrier = &llbitmap->barrier[page_idx];

		if (page_idx > 0) {
			start = end + 1;
			end = min(end + PAGE_SIZE, llbitmap->chunks - 1);
		}

		if (!barrier->flush && time_before(jiffies, barrier->expire)) {
			restart = true;
			page_idx++;
			continue;
		}

		llbitmap_suspend(llbitmap, page_idx);
		llbitmap_state_machine(llbitmap, start, end, BitmapActionDaemon);
		llbitmap_resume(llbitmap, page_idx);

		page_idx++;
	}

	if (test_and_clear_bit(BITMAP_DAEMON_BUSY, &llbitmap->flags) || restart)
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
	bio_list_init(&llbitmap->retry_list);
	spin_lock_init(&llbitmap->retry_lock);

	timer_setup(&llbitmap->pending_timer, llbitmap_pending_timer_fn, 0);
	INIT_WORK(&llbitmap->retry_work, md_llbitmap_retry_fn);
	INIT_WORK(&llbitmap->daemon_work, md_llbitmap_daemon_fn);

	ret = bioset_init(&llbitmap->bio_set, BIO_POOL_SIZE,
			  offsetof(struct llbitmap_bio, bio), 0);
	if (ret)
		goto err_out;

	ret = llbitmap_add_disk(llbitmap);
	if (ret)
		goto err_bio_set;

	ret = llbitmap_open_disk(llbitmap);
	if (ret)
		goto err_del_disk;

	mutex_lock(&mddev->bitmap_info.mutex);
	mddev->bitmap = llbitmap;
	ret = llbitmap_read_sb(llbitmap);
	mutex_unlock(&mddev->bitmap_info.mutex);
	if (ret)
		goto err_close_disk;

	return 0;

err_close_disk:
	mddev->bitmap = NULL;
	llbitmap_close_disk(llbitmap);
err_del_disk:
	llbitmap_del_disk(llbitmap);
err_bio_set:
	bioset_exit(&llbitmap->bio_set);
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

	del_timer_sync(&llbitmap->pending_timer);
	flush_workqueue(md_llbitmap_io_wq);
	flush_workqueue(md_llbitmap_unplug_wq);

	llbitmap_del_disk(llbitmap);
	llbitmap_close_disk(llbitmap);
	mddev->bitmap = NULL;

	llbitmap_free_pages(llbitmap);
	bioset_exit(&llbitmap->bio_set);
	kfree(llbitmap);
	mutex_unlock(&mddev->bitmap_info.mutex);
}

static int llbitmap_startwrite(struct mddev *mddev, sector_t offset,
			       unsigned long sectors, bool is_discard)
{
	struct llbitmap *llbitmap = mddev->bitmap;
	enum llbitmap_action action;
	unsigned long start;
	unsigned long end;
	int page_start;
	int page_end;

	if (likely(!is_discard)) {
		start = offset >> llbitmap->chunkshift;
		end = (offset + sectors - 1) >> llbitmap->chunkshift;
		action = BitmapActionStartwrite;
	} else {
		/*
		 * For discard, the bit can be handled only if the discard range
		 * cover the whole bit, hence round start up, and end down.
		 */
		start = DIV_ROUND_UP(offset, llbitmap->chunksize);
		end = (offset + sectors - 1) >> llbitmap->chunkshift;
		action = BitmapActionDiscard;
	}

	llbitmap_state_machine(llbitmap, start, end, action);

	page_start = (start + BITMAP_SB_SIZE) >> PAGE_SHIFT;
	page_end = (end + BITMAP_SB_SIZE) >> PAGE_SHIFT;

	while (page_start <= page_end) {
		llbitmap_raise_barrier(llbitmap, page_start);
		page_start++;
	}

	return 0;
}

static void llbitmap_endwrite(struct mddev *mddev, sector_t offset,
			      unsigned long sectors, bool is_discard)
{
	struct llbitmap *llbitmap = mddev->bitmap;
	unsigned long start;
	unsigned long end;
	int page_start;
	int page_end;

	if (likely(!is_discard)) {
		start = offset >> llbitmap->chunkshift;
		end = (offset + sectors - 1) >> llbitmap->chunkshift;
	} else {
		/*
		 * For discard, the bit can be handled only if the discard range
		 * cover the whole bit, hence round start up, and end down.
		 */
		start = DIV_ROUND_UP(offset, llbitmap->chunksize);
		end = (offset + sectors - 1) >> llbitmap->chunkshift;
	}

	page_start = (start + BITMAP_SB_SIZE) >> PAGE_SHIFT;
	page_end = (end + BITMAP_SB_SIZE) >> PAGE_SHIFT;

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

	filemap_write_and_wait_range(llbitmap->bitmap_file->f_mapping,
				     BITMAP_SB_SIZE,
				     BITMAP_SB_SIZE + llbitmap->chunks - 1);
	complete(unplug_work->done);
}

static void llbitmap_unplug(struct mddev *mddev, bool sync)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct llbitmap *llbitmap = mddev->bitmap;
	struct llbitmap_unplug_work unplug_work = {
		.llbitmap = llbitmap,
		.done = &done,
	};

	if (!mapping_tagged(llbitmap->bitmap_file->f_mapping,
			    PAGECACHE_TAG_DIRTY))
		return;

	INIT_WORK_ONSTACK(&unplug_work.work, llbitmap_unplug_fn);
	queue_work(md_llbitmap_unplug_wq, &unplug_work.work);
	wait_for_completion(&done);
	destroy_work_on_stack(&unplug_work.work);
}

static void llbitmap_flush(struct mddev *mddev)
{
	struct llbitmap *llbitmap = mddev->bitmap;
	int i;

	for (i = 0; i < llbitmap->nr_pages; i++)
		llbitmap->barrier[i].flush = true;

	del_timer_sync(&llbitmap->pending_timer);
	queue_work(md_llbitmap_io_wq, &llbitmap->daemon_work);
	flush_work(&llbitmap->daemon_work);
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

static void llbitmap_update_sb(void *data)
{
	struct llbitmap *llbitmap = data;
	struct mddev *mddev = llbitmap->mddev;
	struct page *sb_page;
	bitmap_super_t *sb;

	if (test_bit(BITMAP_WRITE_ERROR, &llbitmap->flags))
		return;

	sb_page = read_mapping_page(llbitmap->bitmap_file->f_mapping, 0, NULL);
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
	set_page_dirty(sb_page);
	put_page(sb_page);

	if (filemap_write_and_wait_range(llbitmap->bitmap_file->f_mapping, 0,
					 BITMAP_SB_SIZE) != 0) {
		pr_err("%s: %s: write super block failed", __func__,
		       mdname(mddev));
		set_bit(BITMAP_WRITE_ERROR, &llbitmap->flags);
	}
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

static struct attribute_group md_llbitmap_group = {
	.name = "llbitmap",
	.attrs = md_llbitmap_attrs,
};

static struct bitmap_operations llbitmap_ops = {
	.head = {
		.type	= MD_BITMAP,
		.id	= ID_LLBITMAP,
		.name	= "llbitmap",
	},

	.enabled		= llbitmap_enabled,
	.create			= llbitmap_create,
	.resize			= llbitmap_resize,
	.load			= llbitmap_load,
	.destroy		= llbitmap_destroy,

	.startwrite		= llbitmap_startwrite,
	.endwrite		= llbitmap_endwrite,
	.unplug			= llbitmap_unplug,
	.flush			= llbitmap_flush,

	.start_sync		= llbitmap_start_sync,
	.end_sync		= llbitmap_end_sync,
	.close_sync		= llbitmap_close_sync,

	.update_sb		= llbitmap_update_sb,
	.get_stats		= llbitmap_get_stats,
	.dirty_bits		= llbitmap_dirty_bits,

	/* not needed */
	.write_all		= llbitmap_write_all,
	.daemon_work		= llbitmap_daemon_work,
	.cond_end_sync		= llbitmap_cond_end_sync,

	/* not supported */
	.start_behind_write	= llbitmap_start_behind_write,
	.end_behind_write	= llbitmap_end_behind_write,
	.wait_behind_writes	= llbitmap_wait_behind_writes,
	.sync_with_cluster	= llbitmap_sync_with_cluster,
	.get_from_slot		= llbitmap_get_from_slot,
	.copy_from_slot		= llbitmap_copy_from_slot,
	.set_pages		= llbitmap_set_pages,
	.free			= llbitmap_free,

	.group			= &md_llbitmap_group,
};
