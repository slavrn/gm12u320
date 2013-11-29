#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/usb/storage.h>
#include <linux/fb.h>
#include <linux/mm.h>

#define DRIVER_NAME		"gm12u320"
#define DRIVER_DESC		"Driver for GM12U320-based video projectors"
#define DRIVER_AUTHOR	"Viacheslav Nurmekhamitov <slavrn@yandex.ru>"
#define DRIVER_VERSION	"1.0"

#define VENDOR_ID_ACER_C120		0x1de1
#define PRODUCT_ID_ACER_C120	0xc102

#define X_RES		854
#define Y_RES		480
#define COLOR_DEPTH	24
#define FB_SIZE X_RES * Y_RES * COLOR_DEPTH >> 3

#define DATA_IFACE_NUM	0
#define DATA_SND_EPT	3
#define DATA_RCV_EPT	2
#define BL_IFACE_NUM	1
#define BL_SND_EPT	4
#define BL_RCV_EPT	1

#define DATA_BLOCK_HEADER_SIZE		84
#define DATA_BLOCK_RAW_SIZE			64512
#define DATA_LAST_BLOCK_RAW_SIZE	4032
#define DATA_BLOCK_FOOTER_SIZE		20
#define DATA_BLOCK_SIZE				DATA_BLOCK_HEADER_SIZE + \
							DATA_BLOCK_RAW_SIZE + DATA_BLOCK_FOOTER_SIZE
#define DATA_LAST_BLOCK_SIZE		DATA_BLOCK_HEADER_SIZE + \
							DATA_LAST_BLOCK_RAW_SIZE + DATA_BLOCK_FOOTER_SIZE
#define DATA_BLOCK_COUNT			20

#define FRAME_COUNT 2

#define CMD_SIZE		31
#define CMD_CYCLE		DATA_BLOCK_COUNT

#define READ_BLOCK_SIZE	13

#define IDLE_PERIOD 2000

#define DELAY_10MS HZ / 100

enum gm12u320_brightness_levels {
	BRIGHTNESS_ECO, /* Econo mode */
	BRIGHTNESS_AUTO, /* Normal brigtness */
	BRIGHTNESS_UNKNOWN,
};

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

struct gm12u320_dev;

static int gm12u320_init(void);
static void gm12u320_exit(void);
static int gm12u320_probe(struct usb_interface *interface, const struct usb_device_id *id);
static void gm12u320_disconnect(struct usb_interface *interface);
static void gm12u320_free_work(struct work_struct *work);
static void gm12u320_defio_cb(struct fb_info *info, struct list_head *pagelist);
static struct fb_info *gm12u320_fb_alloc(void);
static void gm12u320_fb_free(struct fb_info *info);
static int gm12u320_fb_open(struct fb_info *info, int user);
static int gm12u320_fb_release(struct fb_info *info, int user);
static int gm12u320_check_var(struct fb_var_screeninfo *var, struct fb_info *info);
static int gm12u320_set_par(struct fb_info *info);
static int gm12u320_fb_mmap(struct fb_info *info, struct vm_area_struct *vma);
static void gm12u320_usb_free(struct gm12u320_dev *dev);
static int gm12u320_usb_alloc(struct gm12u320_dev *dev, struct usb_device *udev);
static void gm12u320_cmd_write_complete(struct urb *urb);
static void gm12u320_cmd_read_complete(struct urb *urb);
static void gm12u320_update_frame(struct gm12u320_dev *dev);
static void gm12u320_cycle(struct work_struct *work);
static void gm12u320_bl_write_complete(struct urb *urb);
static void gm12u320_bl_ans_complete(struct urb *urb);
static void gm12u320_bl_ack_complete(struct urb *urb);
static int gm12u320_bl_get(struct backlight_device *bl_dev);
static int gm12u320_bl_update(struct backlight_device *bl_dev);
static int gm12u320_bl_request(struct gm12u320_dev *dev);

static struct usb_device_id gm12u320_id_table[] = {
	{	.match_flags = USB_DEVICE_ID_MATCH_VENDOR |
					USB_DEVICE_ID_MATCH_PRODUCT |
					USB_DEVICE_ID_MATCH_INT_CLASS |
					USB_DEVICE_ID_MATCH_INT_SUBCLASS |
					USB_DEVICE_ID_MATCH_INT_PROTOCOL,
		.idVendor = VENDOR_ID_ACER_C120,
		.idProduct = PRODUCT_ID_ACER_C120,
		.bInterfaceClass = USB_CLASS_MASS_STORAGE,
		.bInterfaceSubClass = USB_SC_SCSI,
		.bInterfaceProtocol = USB_PR_BULK,
	},
	{},
};

MODULE_DEVICE_TABLE(usb, gm12u320_id_table);

static const char cmd_data[CMD_SIZE] =
	{0x55, 0x53, 0x42, 0x43, 0x00, 0x00, 0x00, 0x00, 0x68, 0xfc, 0x00, 0x00,
	0x00, 0x00, 0x10, 0xff, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x80, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const char cmd_draw[CMD_SIZE] =
	{0x55, 0x53, 0x42, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x10, 0xfe, 0x00, 0x00, 0x00, 0xc0, 0xd1, 0x05, 0x00, 0x40,
	0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const char data_block_header[DATA_BLOCK_HEADER_SIZE] =
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xfb, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x04, 0x15, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x01, 0x00, 0x00, 0xdb };

static const char data_last_block_header[DATA_BLOCK_HEADER_SIZE] =
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xfb, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x2a, 0x00, 0x20, 0x00, 0xc0, 0x0f, 0x00, 0x00, 0x01, 0x00, 0x00, 0xd7 };

static const char data_block_footer[DATA_BLOCK_FOOTER_SIZE] =
	{ 0xfb, 0x14, 0x02, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x4f };

static const char bl_get_set_brightness[CMD_SIZE] =
	{0x55, 0x53, 0x42, 0x43, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
	0x80, 0x01, 0x10, 0xfd, 0x00, 0x00, 0x00, 0xc0, 0xff, 0x35, 0x00, 0x00,
	0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };

struct gm12u320_dev {
	struct usb_device *udev;
	struct fb_info *info;
	struct backlight_device *bl;
	struct urb *cmd_data_urb[FRAME_COUNT][DATA_BLOCK_COUNT];
	struct urb *data_urb[FRAME_COUNT][DATA_BLOCK_COUNT];
	struct urb *cmd_draw_urb;
	struct urb *read_urb;
	struct urb *bl_urb;
	struct urb *bl_ans_urb;
	struct urb *bl_ack_urb;
	struct workqueue_struct *cycle_wq;
	struct work_struct cycle_work;
	struct delayed_work free_work;
	struct semaphore sync_sem;
	struct semaphore in_use;
	struct semaphore bl_sem;
	atomic_t current_cmd; /* 0-19 = data cmds, 20 = draw cmd, neg = error */
	atomic_t current_frame;
	atomic_t next_frame;
	atomic_t usb_ref_cnt;
	atomic_t virtual;
	atomic_t brightness;
};

static struct backlight_ops gm12u320_backlight_ops = {
	.get_brightness = gm12u320_bl_get,
	.update_status = gm12u320_bl_update,
};

static struct fb_ops gm12u320_fb_ops = {
	.owner = THIS_MODULE,
	.fb_open = gm12u320_fb_open,
	.fb_release = gm12u320_fb_release,
	.fb_read = fb_sys_read,
	.fb_write = fb_sys_write,
	.fb_check_var = gm12u320_check_var,
	.fb_set_par = gm12u320_set_par,
	.fb_fillrect = sys_fillrect,
	.fb_copyarea = sys_copyarea,
	.fb_imageblit = sys_imageblit,
	.fb_mmap = gm12u320_fb_mmap,
};

static struct fb_fix_screeninfo gm12u320_fb_fix = {
	.id = DRIVER_NAME,
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.line_length = X_RES * COLOR_DEPTH >> 3,
	.accel = FB_ACCEL_NONE,
};

static struct fb_var_screeninfo gm12u320_fb_var = {
	.xres = X_RES,
	.yres = Y_RES,
	.xres_virtual = X_RES,
	.yres_virtual = Y_RES,
	.bits_per_pixel = COLOR_DEPTH,
	.red = { 16, 8, 0 },
	.green = { 8, 8, 0 },
	.blue = { 0, 8, 0 },
	.vmode = FB_VMODE_NONINTERLACED,
};

static const u32 gm12u320_fb_flags = FBINFO_DEFAULT | FBINFO_HWACCEL_DISABLED | FBINFO_VIRTFB | FBINFO_READS_FAST;

static struct usb_driver gm12u320_driver = {
	.name = DRIVER_NAME,
	.probe = gm12u320_probe,
	.disconnect = gm12u320_disconnect,
	.id_table = gm12u320_id_table,
};

static struct fb_deferred_io gm12u320_fb_defio = {
	.delay = DELAY_10MS,
	.deferred_io = gm12u320_defio_cb,
};

module_init(gm12u320_init);
module_exit(gm12u320_exit);

static int __init gm12u320_init(void)
{
	int ret = 0;
	ret = usb_register(&gm12u320_driver);
	if (ret)
		pr_err("gm12u320: Driver loading failed");
	else
		pr_info("gm12u320: Driver loaded");
	return ret;
}

static void __exit gm12u320_exit(void)
{
	usb_deregister(&gm12u320_driver);
	pr_info("gm12u320: Driver unloaded");
}

static int gm12u320_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_device *udev = 0;
	struct gm12u320_dev *dev = 0;
	struct fb_info *info = 0;
	int ret = 0;
	struct backlight_properties props;
	/* GM12U320 provides 2 interfaces: data and ctl. Proceed for data
						interface probes only - ignore control interface probes. */
	if (DATA_IFACE_NUM != interface->cur_altsetting->desc.bInterfaceNumber)
		return ret;

	pr_info("gm12u320: GM12U320-based device connected");
	udev = interface_to_usbdev(interface);
	dev = kzalloc(sizeof(*dev) ,GFP_KERNEL);
	if (!dev || gm12u320_usb_alloc(dev, udev)) {
		pr_err("gm12u320: Memory allocation error");
		ret = -ENOMEM;
		goto err;
	}
	info = gm12u320_fb_alloc();
	if (!dev) {
		pr_err("gm12u320: Framebuffer allocation error");
		ret = -ENOMEM;
		goto err;
	}
	dev->info = info;
	info->par = dev;
	usb_set_intfdata(interface, dev);
	INIT_WORK(&dev->cycle_work, gm12u320_cycle);
	INIT_DELAYED_WORK(&dev->free_work, gm12u320_free_work);
	atomic_set(&dev->current_cmd, CMD_CYCLE);
	atomic_set(&dev->current_frame, 0);
	atomic_set(&dev->next_frame, 0);
	atomic_set(&dev->usb_ref_cnt, 0);
	atomic_set(&dev->virtual, 0);
	atomic_set(&dev->brightness, BRIGHTNESS_UNKNOWN);
	sema_init(&dev->sync_sem, 0);
	sema_init(&dev->in_use, 1);
	sema_init(&dev->bl_sem, 1);

	/* Init backlight control */
	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_PLATFORM;
	props.max_brightness = BRIGHTNESS_AUTO;
	props.brightness = BRIGHTNESS_AUTO;
	dev->bl = backlight_device_register(DRIVER_NAME, NULL, dev, &gm12u320_backlight_ops, &props);
	if (IS_ERR(dev->bl)) {
		pr_err("gm12u320: Backlight init error: 0x%lx", PTR_ERR(dev->bl));
		ret = -ENOMEM;
		dev->bl = 0;
		goto err;
	}
	if (gm12u320_bl_request(dev)) {
		pr_err("gm12u320: Error requesting brightness");
		ret = -EINVAL;
		goto err;
	}
	dev->cycle_wq = alloc_workqueue(DRIVER_NAME, 0, 1);
	if (!dev->cycle_wq) {
		pr_err("gm12u320: Work queue allocation error");
		ret = -ENOMEM;
		goto err;
	}
	ret = register_framebuffer(info);
	if (ret < 0) {
		pr_err("gm12u320: Framebuffer register error");
		goto err;
	}
	/* Starting main cycle work */
	queue_work(dev->cycle_wq, &dev->cycle_work);
	return ret;
err:
	if (dev) {
		if (dev->cycle_wq) destroy_workqueue(dev->cycle_wq);
		if (dev->bl) backlight_device_unregister(dev->bl);
		if (info) gm12u320_fb_free(info);
		gm12u320_usb_free(dev);
		kfree(dev);
	}
	return ret;
}

static void gm12u320_disconnect(struct usb_interface *interface)
{
	struct gm12u320_dev *dev;
	int i;
	/* GM12U320 provides 2 interfaces: data and control. Proceed for data
									interface only - ignore control interface. */
	if (DATA_IFACE_NUM != interface->cur_altsetting->desc.bInterfaceNumber)
		return;
	pr_info("gm12u320: GM12U320 disconnected");
	if ( !(dev = usb_get_intfdata(interface)) ) return;
	/* Set error state */
	atomic_sub(CMD_CYCLE * 2, &dev->current_cmd);
	/* Let cycle work finish its job */
	up(&dev->sync_sem);
	up(&dev->bl_sem);
	if (dev->cycle_wq) destroy_workqueue(dev->cycle_wq);

	/* Wait for USB aync requests finish */
	for (i = 0; atomic_read(&dev->usb_ref_cnt) && i++ < 100; msleep(10));
	usb_set_intfdata(interface, NULL);

	if (dev->bl) backlight_device_unregister(dev->bl);
	gm12u320_usb_free(dev);
	if (!down_trylock(&dev->in_use)) {
		unregister_framebuffer(dev->info);
		gm12u320_fb_free(dev->info);
		kfree(dev);
	}
	else
		atomic_set(&dev->virtual, 1);
}

static void gm12u320_free_work(struct work_struct *work)
{
	struct gm12u320_dev *dev = container_of(work, struct gm12u320_dev, free_work.work);
	unregister_framebuffer(dev->info);
	gm12u320_fb_free(dev->info);
	kfree(dev);
}

static void gm12u320_defio_cb(struct fb_info *info, struct list_head *pagelist)
{
	struct gm12u320_dev *dev = (struct gm12u320_dev *) info->par;
	if (0 <= atomic_read(&dev->current_cmd))
		gm12u320_update_frame(dev);
}

static struct fb_info *gm12u320_fb_alloc()
{
	/* Allocate framebuffer */
	struct fb_info *info;
	info = framebuffer_alloc(0, 0);
	if (!info) goto err;
	info->screen_base = vmalloc(FB_SIZE);
	if (!info->screen_base) goto err;
	memset(info->screen_base, 0, FB_SIZE);
	memcpy(&info->fix, &gm12u320_fb_fix, sizeof(gm12u320_fb_fix));
	memcpy(&info->var, &gm12u320_fb_var, sizeof(gm12u320_fb_var));
	info->fix.smem_len = PAGE_ALIGN(FB_SIZE);
	info->fix.smem_start = (unsigned long) info->screen_base;
	info->flags = gm12u320_fb_flags;
	info->fbops = &gm12u320_fb_ops;
	info->fbdefio = &gm12u320_fb_defio;
	fb_deferred_io_init(info);
	return info;
err:
	if (info) gm12u320_fb_free(info);
	return 0;
}

static void gm12u320_fb_free(struct fb_info *info)
{
	fb_deferred_io_cleanup(info);
	vfree(info->screen_base);
	framebuffer_release(info);
}

static int gm12u320_fb_open(struct fb_info *info, int user)
{
	struct gm12u320_dev *dev = info->par;
	if (0 > atomic_read(&dev->current_cmd)) return -EIO;
	if (down_trylock(&dev->in_use)) return -EBUSY;
	return 0;
}

static int gm12u320_fb_release(struct fb_info *info, int user)
{
	struct gm12u320_dev *dev = info->par;
	if (0 <= atomic_read(&dev->current_cmd)) {
		/* Blank framebuffer */
		memset(info->screen_base, 0, FB_SIZE);
		gm12u320_update_frame(dev);
		up(&dev->in_use);
	}
	else if (atomic_read(&dev->virtual)) {
		/* Schedule deferred cleanup */
		schedule_delayed_work(&dev->free_work, DELAY_10MS);
	}
	return 0;
}

static int gm12u320_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	if (var)
		memcpy(var, &gm12u320_fb_var, sizeof(gm12u320_fb_var));
	return 0;
}

static int gm12u320_set_par(struct fb_info *info)
{
	return 0;
}

static int gm12u320_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long page, pos;

	if (offset + size > info->fix.smem_len)
		return -EINVAL;

	pos = (unsigned long)info->fix.smem_start + offset;

	pr_notice("mmap() framebuffer addr:%lu size:%lu\n", pos, size);

	while (size > 0) {
		page = vmalloc_to_pfn((void *)pos);
		if (remap_pfn_range(vma, start, page, PAGE_SIZE, PAGE_SHARED))
			return -EAGAIN;

		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}

	return 0;
}

static void gm12u320_usb_free(struct gm12u320_dev *dev)
{
	int i, k;
	int block_size;

	for (k = 0; k < FRAME_COUNT; k++)
	for (i = 0; i < DATA_BLOCK_COUNT; i++) {
		if (i == DATA_BLOCK_COUNT - 1) /* Last block */
			block_size = DATA_LAST_BLOCK_SIZE;
		else
			block_size = DATA_BLOCK_SIZE;
		/* Free data command URBs */
		if (dev->cmd_data_urb[k][i]) {
			if (dev->cmd_data_urb[k][i]->transfer_buffer) usb_free_coherent(dev->udev, CMD_SIZE,
					dev->cmd_data_urb[k][i]->transfer_buffer, dev->cmd_data_urb[k][i]->transfer_dma);
			usb_free_urb(dev->cmd_data_urb[k][i]);
		}
		/* Free data block URBs */
		if (dev->data_urb[k][i]) {
			if (dev->data_urb[k][i]->transfer_buffer) usb_free_coherent(dev->udev, block_size,
						dev->data_urb[k][i]->transfer_buffer, dev->data_urb[k][i]->transfer_dma);
		 	usb_free_urb(dev->data_urb[k][i]);
		}
	}
	/* Free draw command URB */
	if (dev->cmd_draw_urb) {
		if (dev->cmd_draw_urb->transfer_buffer) usb_free_coherent(dev->udev, CMD_SIZE,
			dev->cmd_draw_urb->transfer_buffer, dev->cmd_draw_urb->transfer_dma);
		usb_free_urb(dev->cmd_draw_urb);
	}
	/* Free read URB */
	if (dev->read_urb) {
		if (dev->read_urb->transfer_buffer) usb_free_coherent(dev->udev, READ_BLOCK_SIZE,
						dev->read_urb->transfer_buffer, dev->read_urb->transfer_dma);
		usb_free_urb(dev->read_urb);
	}
	/* Free bl URB */
	if (dev->bl_urb) {
		if (dev->bl_urb->transfer_buffer) usb_free_coherent(dev->udev, READ_BLOCK_SIZE,
						dev->bl_urb->transfer_buffer, dev->bl_urb->transfer_dma);
		usb_free_urb(dev->bl_urb);
	}
	/* Free bl read URBs */
	if (dev->bl_ans_urb) {
		if (dev->bl_ans_urb->transfer_buffer) usb_free_coherent(dev->udev, READ_BLOCK_SIZE,
						dev->bl_ans_urb->transfer_buffer, dev->bl_ans_urb->transfer_dma);
		usb_free_urb(dev->bl_ans_urb);
	}
	if (dev->bl_ack_urb) {
		if (dev->bl_ack_urb->transfer_buffer) usb_free_coherent(dev->udev, READ_BLOCK_SIZE,
						dev->bl_ack_urb->transfer_buffer, dev->bl_ack_urb->transfer_dma);
		usb_free_urb(dev->bl_ack_urb);
	}
}

static int gm12u320_usb_alloc(struct gm12u320_dev *dev, struct usb_device *udev)
{
	int i, k;
	int block_size;
	int last_block;
	char *buf;
	dev->udev = udev;

	/* Prepare data command and data block URBs */
	for (k = 0; k < FRAME_COUNT; k++)
	for (i = 0; i < DATA_BLOCK_COUNT; i++) {
		if (i == DATA_BLOCK_COUNT - 1) {/* Last block */
			last_block = 1;
			block_size = DATA_LAST_BLOCK_SIZE;
		}
		else {
			last_block = 0;
			block_size = DATA_BLOCK_SIZE;
		}
		/* Prepare data command URB */
		dev->cmd_data_urb[k][i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!dev->cmd_data_urb[k][i]) goto err;
		buf = usb_alloc_coherent(dev->udev, CMD_SIZE, GFP_KERNEL, &dev->cmd_data_urb[k][i]->transfer_dma);
		if (!buf) goto err;
		memcpy(buf, cmd_data, CMD_SIZE);
		/* Commands are made from template, increment data counters */
		buf[20] = 0xfc - i * 4;
		buf[21] = i | (k << 7);
		usb_fill_bulk_urb(dev->cmd_data_urb[k][i], dev->udev, usb_sndbulkpipe(dev->udev, DATA_SND_EPT),
													buf, CMD_SIZE, gm12u320_cmd_write_complete, dev);
		if (last_block) {/* The last data block is less than the others */
			buf[8] = 0x28;
			buf[9] = 0x10;
		}
		/* Prepare data block URB */
		dev->data_urb[k][i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!dev->data_urb[k][i]) goto err;
		buf = usb_alloc_coherent(dev->udev, block_size, GFP_KERNEL, &dev->data_urb[k][i]->transfer_dma);
		if (!buf) goto err;
		memcpy(buf, last_block ? data_last_block_header : data_block_header, DATA_BLOCK_HEADER_SIZE);
		memcpy(buf + block_size - DATA_BLOCK_FOOTER_SIZE, data_block_footer, DATA_BLOCK_FOOTER_SIZE);
		usb_fill_bulk_urb(dev->data_urb[k][i], dev->udev, usb_sndbulkpipe(dev->udev, DATA_SND_EPT),
												buf, block_size, gm12u320_cmd_write_complete, dev);
	}
	/* Prepare draw command URB */
	dev->cmd_draw_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->cmd_draw_urb) goto err;
	buf = usb_alloc_coherent(dev->udev, CMD_SIZE, GFP_KERNEL, &dev->cmd_draw_urb->transfer_dma);
	if (!buf) goto err;
	memcpy(buf, cmd_draw, CMD_SIZE);
	usb_fill_bulk_urb(dev->cmd_draw_urb, dev->udev, usb_sndbulkpipe(dev->udev, DATA_SND_EPT),
													buf, CMD_SIZE, gm12u320_cmd_write_complete, dev);
	/* Prepare read URB */
	dev->read_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->read_urb) goto err;
	buf = usb_alloc_coherent(dev->udev, READ_BLOCK_SIZE, GFP_KERNEL, &dev->read_urb->transfer_dma);
	if (!buf) goto err;
	usb_fill_bulk_urb(dev->read_urb, dev->udev, usb_rcvbulkpipe(dev->udev, DATA_RCV_EPT),
											buf, READ_BLOCK_SIZE, gm12u320_cmd_read_complete, dev);
	/* Prepare bl URB */
	dev->bl_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->bl_urb) goto err;
	buf = usb_alloc_coherent(dev->udev, CMD_SIZE, GFP_KERNEL, &dev->bl_urb->transfer_dma);
	if (!buf) goto err;
	memcpy(buf, &bl_get_set_brightness, CMD_SIZE);
	usb_fill_bulk_urb(dev->bl_urb, dev->udev, usb_sndbulkpipe(dev->udev, BL_SND_EPT),
													buf, CMD_SIZE, gm12u320_bl_write_complete, dev);
	/* Prepare bl read URBs */
	dev->bl_ans_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->bl_ans_urb) goto err;
	buf = usb_alloc_coherent(dev->udev, READ_BLOCK_SIZE, GFP_KERNEL, &dev->bl_ans_urb->transfer_dma);
	if (!buf) goto err;
	usb_fill_bulk_urb(dev->bl_ans_urb, dev->udev, usb_rcvbulkpipe(dev->udev, BL_RCV_EPT),
												buf, READ_BLOCK_SIZE, gm12u320_bl_ans_complete, dev);
	dev->bl_ack_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->bl_ack_urb) goto err;
	buf = usb_alloc_coherent(dev->udev, READ_BLOCK_SIZE, GFP_KERNEL, &dev->bl_ack_urb->transfer_dma);
	if (!buf) goto err;
	usb_fill_bulk_urb(dev->bl_ack_urb, dev->udev, usb_rcvbulkpipe(dev->udev, BL_RCV_EPT),
												buf, READ_BLOCK_SIZE, gm12u320_bl_ack_complete, dev);
	return 0;
err:
	if (dev) gm12u320_usb_free(dev);
	return -ENOMEM;
}

static void gm12u320_cmd_write_complete(struct urb *urb)
{
	struct gm12u320_dev *dev = (struct gm12u320_dev *) urb->context;
	if(urb->status) {
		atomic_sub(CMD_CYCLE * 2, &dev->current_cmd);
		pr_err("gm12u320: Write callback error: %d", urb->status);
	}
	atomic_sub(1, &dev->usb_ref_cnt);
}

static void gm12u320_cmd_read_complete(struct urb *urb)
{
	struct gm12u320_dev *dev = (struct gm12u320_dev *) urb->context;
	if(urb->status) {
		atomic_sub(CMD_CYCLE * 2, &dev->current_cmd);
		pr_err("gm12u320: Read callback error: %d", urb->status);
	}
	else if (0 <= atomic_read(&dev->current_cmd))
		queue_work(dev->cycle_wq, &dev->cycle_work);
	atomic_sub(1, &dev->usb_ref_cnt);
}

static void gm12u320_update_frame(struct gm12u320_dev *dev)
{
	int i;
	int frame = !atomic_read(&dev->current_frame);
	for (i = 0; i < DATA_BLOCK_COUNT; i++)
		memcpy(dev->data_urb[frame][i]->transfer_buffer + DATA_BLOCK_HEADER_SIZE, dev->info->screen_base + DATA_BLOCK_RAW_SIZE * i,
												i == DATA_BLOCK_COUNT - 1 ? DATA_LAST_BLOCK_RAW_SIZE : DATA_BLOCK_RAW_SIZE);
	atomic_set(&dev->next_frame, frame);
	/* To avoid unlimited semaphore incrementing */
	(void)down_trylock(&dev->sync_sem);
	up(&dev->sync_sem);
}

static void gm12u320_cycle(struct work_struct *work)
{
	struct gm12u320_dev *dev = container_of(work, struct gm12u320_dev, cycle_work);
	int ret = 0;
	int current_cmd = atomic_read(&dev->current_cmd);
	int frame = atomic_read(&dev->current_frame);

	if (current_cmd < 0) return; /* Error state - exiting */

	if (current_cmd == CMD_CYCLE) { /* End of cycle - restarting */
		/* Wait for idle period, but wake up on fb_defio */
		down_timeout(&dev->sync_sem, msecs_to_jiffies(IDLE_PERIOD));
		if (0 > atomic_read(&dev->current_cmd)) return;
		atomic_sub(CMD_CYCLE, &dev->current_cmd); /* Reset command counter */
		current_cmd = 0;
		frame = atomic_read(&dev->next_frame);
		atomic_set(&dev->current_frame, frame);
	}
	else {
		atomic_add(1, &dev->current_cmd);
		current_cmd++;
	}
	if (current_cmd < DATA_BLOCK_COUNT) {
		/* Send data command to device */
		atomic_add(1, &dev->usb_ref_cnt);
		ret = usb_submit_urb(dev->cmd_data_urb[frame][current_cmd], GFP_KERNEL);
		if (ret) {
			atomic_sub(1, &dev->usb_ref_cnt);
			goto err;
		}
		/* Send data block to device */
		atomic_add(1, &dev->usb_ref_cnt);
		ret = usb_submit_urb(dev->data_urb[frame][current_cmd], GFP_KERNEL);
		if (ret) {
			atomic_sub(1, &dev->usb_ref_cnt);
			goto err;
		}
	}
	else {
		/* Send draw command to device */
		atomic_add(1, &dev->usb_ref_cnt);
		ret = usb_submit_urb(dev->cmd_draw_urb, GFP_KERNEL);
		if (ret) {
			atomic_sub(1, &dev->usb_ref_cnt);
			goto err;
		}
	}
	/* Schedule response receiving from device */
	atomic_add(1, &dev->usb_ref_cnt);
	ret = usb_submit_urb(dev->read_urb, GFP_KERNEL);
	if (ret) {
		atomic_sub(1, &dev->usb_ref_cnt);
		goto err;
	}
	return;
err:
	atomic_sub(CMD_CYCLE * 2, &dev->current_cmd);
	pr_err("gm12u320: Submit command %d error: %d", current_cmd, ret);
	return;
}

static void gm12u320_bl_write_complete(struct urb *urb)
{
	struct gm12u320_dev *dev = (struct gm12u320_dev *) urb->context;
	if(urb->status) {
		atomic_sub(CMD_CYCLE * 2, &dev->current_cmd);
		pr_err("gm12u320: Write callback error: %d", urb->status);
	}
	atomic_sub(1, &dev->usb_ref_cnt);
}

static void gm12u320_bl_ans_complete(struct urb *urb)
{
	struct gm12u320_dev *dev = (struct gm12u320_dev *) urb->context;
	if(urb->status) {
		atomic_sub(CMD_CYCLE * 2, &dev->current_cmd);
		pr_err("gm12u320: Read callback error: %d", urb->status);
	}
	if (atomic_read(&dev->brightness) == BRIGHTNESS_UNKNOWN) {
		/* We are running for the first time - get brightness from device */
		switch (((unsigned char*)urb->transfer_buffer)[0]) {
			case 0x00:
				atomic_set(&dev->brightness, BRIGHTNESS_AUTO);
				break;
			case 0x01:
				atomic_set(&dev->brightness, BRIGHTNESS_ECO);
				break;
			default:
				/* Unknown brightness */
				pr_warn("Unexpected brightness response: 0x%x", ((unsigned char*)urb->transfer_buffer)[0]);
				atomic_set(&dev->brightness, BRIGHTNESS_AUTO);
				break;
		}
	}
	atomic_sub(1, &dev->usb_ref_cnt);
}

static void gm12u320_bl_ack_complete(struct urb *urb)
{
	struct gm12u320_dev *dev = (struct gm12u320_dev *) urb->context;
	if(urb->status) {
		atomic_sub(CMD_CYCLE * 2, &dev->current_cmd);
		pr_err("gm12u320: Read callback error: %d", urb->status);
	}
	up(&dev->bl_sem);
	atomic_sub(1, &dev->usb_ref_cnt);
}

static int gm12u320_bl_get(struct backlight_device *bl_dev)
{
	struct gm12u320_dev *dev = bl_get_data(bl_dev);
	return atomic_read(&dev->brightness);
}

static int gm12u320_bl_update(struct backlight_device *bl_dev)
{
	struct gm12u320_dev *dev = bl_get_data(bl_dev);
	int brightness = bl_dev->props.brightness;
	int ret = 0;
	if (0 > atomic_read(&dev->current_cmd)) return -1; /* Error state - exiting */
	if (atomic_read(&dev->brightness) == brightness)
		return 0;

	down(&dev->bl_sem);
	/* Brightness set command */
	((char*) dev->bl_urb->transfer_buffer)[22] = 0x01;
	switch (brightness) {
		case BRIGHTNESS_AUTO:
			((char*) dev->bl_urb->transfer_buffer)[23] = 0x00;
			break;
		case BRIGHTNESS_ECO:
			((char*) dev->bl_urb->transfer_buffer)[23] = 0x01;
			break;
		default:
			ret = -1;
			goto err;
	}
	atomic_set(&dev->brightness, brightness);
	atomic_add(1, &dev->usb_ref_cnt);
	ret = usb_submit_urb(dev->bl_urb, GFP_KERNEL);
	if (ret) {
		atomic_sub(1, &dev->usb_ref_cnt);
		goto err;
	}
	/* Schedule response receiving from device */
	atomic_add(1, &dev->usb_ref_cnt);
	ret = usb_submit_urb(dev->bl_ans_urb, GFP_KERNEL);
	if (ret) {
		atomic_sub(1, &dev->usb_ref_cnt);
		goto err;
	}
	atomic_add(1, &dev->usb_ref_cnt);
	ret = usb_submit_urb(dev->bl_ack_urb, GFP_KERNEL);
	if (ret) {
		atomic_sub(1, &dev->usb_ref_cnt);
		goto err;
	}
	return ret;
err:
	atomic_sub(CMD_CYCLE * 2, &dev->current_cmd);
	pr_err("gm12u320: Error setting brightness");
	return ret;
}

static int gm12u320_bl_request(struct gm12u320_dev *dev)
{
	int ret = 0;
	down(&dev->bl_sem);
	/* Brightness get command */
	((char*) dev->bl_urb->transfer_buffer)[22] = 0x00;
	((char*) dev->bl_urb->transfer_buffer)[23] = 0x00;
	atomic_add(1, &dev->usb_ref_cnt);
	ret = usb_submit_urb(dev->bl_urb, GFP_KERNEL);
	if (ret) {
		atomic_sub(1, &dev->usb_ref_cnt);
		goto err;
	}
	/* Schedule response receiving from device */
	atomic_add(1, &dev->usb_ref_cnt);
	ret = usb_submit_urb(dev->bl_ans_urb, GFP_KERNEL);
	if (ret) {
		atomic_sub(1, &dev->usb_ref_cnt);
		goto err;
	}
	atomic_add(1, &dev->usb_ref_cnt);
	ret = usb_submit_urb(dev->bl_ack_urb, GFP_KERNEL);
	if (ret) {
		atomic_sub(1, &dev->usb_ref_cnt);
		goto err;
	}
	return ret;
err:
	atomic_sub(CMD_CYCLE * 2, &dev->current_cmd);
	pr_err("gm12u320: Error requesting brightness");
	return ret;
}
