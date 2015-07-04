/*
 * axi-csi-rx.c
 *
 * Based on axi-hdmi-rx.c
 *
 * Copyright (C) 2015 Sylvain Munaut <tnt@246tNt.com>
 * Copyright (C) 2012-2013 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-event.h>
#include <media/v4l2-of.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>


#define DRIVER_NAME "axi-csi-rx"


#define AXI_CSI_RX_REG_CR	0x00
#define AXI_CSI_RX_REG_SR	0x04
#define AXI_CSI_RX_REG_ER	0x08
#define AXI_CSI_RX_REG_IMR	0x0C
#define AXI_CSI_RX_REG_PDCR	0x10
#define AXI_CSI_RX_REG_VPCR	0x14
#define AXI_CSI_RX_REG_VCCS	0x20
#define AXI_CSI_RX_REG_VCCE	0x24
#define AXI_CSI_RX_REG_VCLS	0x28
#define AXI_CSI_RX_REG_VCLE	0x2C


struct axi_csi_rx_buffer {
	struct vb2_buffer vb;
	struct list_head head;
};

struct axi_csi_rx_stream {
	struct video_device vdev;
	struct vb2_queue q;
	struct v4l2_subdev *subdev;

	struct mutex lock;

	struct dma_chan *chan;
};

struct axi_csi_rx {
	struct v4l2_device v4l2_dev;
	struct vb2_alloc_ctx *alloc_ctx;

	struct axi_csi_rx_stream stream;

	void __iomem *base;
	struct gpio_desc *gpio_led;

	struct v4l2_async_notifier notifier;
	struct v4l2_async_subdev asd;
	struct v4l2_async_subdev *asds[1];
};

#if 0
struct axi_csi_rx_fmt_desc {

	const char *desc;
	u32 
};

static const struct axi_csi_rx_fmt_desc axi_csi_rx_fmts[] = {
	{ "sbggr10p",   0x000000 },       /* x xx xxxxxxxx 00 00 */
	{ "sbggr10",    0xa00022 },       /* 1 10 xxxxxxxx 10 10 */
	{ "sbggr8",     0x800023 },       /* 1 00 xxxxxxxx 10 11 */
	{ "rgbz32",     0x803930 },       /* 1 xx 00111001 11 00 */
	{ "bgrz32",     0x801b30 },       /* 1 xx 00011011 11 00 */
	{ "gray8",      0x800233 },       /* 1 xx xxxxxx10 11 11 */
};
#endif



/* Helpers ---------------------------------------------------------------- */

static void
axi_csi_rx_write(struct axi_csi_rx *csi,
                 unsigned int reg, unsigned int val)
{
	writel(val, csi->base + reg);
}

static u32
axi_csi_rx_read(struct axi_csi_rx *csi,
                unsigned int reg)
{
	return readl(csi->base + reg);
}



/* V4L device ------------------------------------------------------------- */


 /* File Operations */

static const struct v4l2_file_operations axi_csi_rx_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.unlocked_ioctl	= video_ioctl2,
	.read		= vb2_fop_read,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
};


 /* IOCTL Operations */

static int
axi_csi_rx_ioctl_querycap(struct file *file, void *priv_fh,
                          struct v4l2_capability *vcap)
{
	strlcpy(vcap->driver, DRIVER_NAME, sizeof(vcap->driver));
	strlcpy(vcap->card, DRIVER_NAME, sizeof(vcap->card));
	snprintf(vcap->bus_info, sizeof(vcap->bus_info), "platform:" DRIVER_NAME);
	vcap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	vcap->capabilities = vcap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int
axi_csi_rx_ioctl_enum_fmt_vid_cap(struct file *file, void *priv_fh,
                                  struct v4l2_fmtdesc *f)
{
	return 0;	/* FIXME */
}

static int
axi_csi_rx_ioctl_get_fmt_vid_cap(struct file *file, void *priv_fh,
                                 struct v4l2_format *f)
{
	struct axi_csi_rx *hdmi_rx = video_drvdata(file);
	struct axi_csi_rx_stream *s = &hdmi_rx->stream;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	/* FIXME */
	return 0;
}

static int
axi_csi_rx_ioctl_set_fmt_vid_cap(struct file *file, void *priv_fh,
                                 struct v4l2_format *f)
{
	return 0;	/* FIXME */
}

static int
axi_csi_rx_ioctl_try_fmt_vid_cap(struct file *file, void *priv_fh,
                                 struct v4l2_format *f)
{
	return 0;	/* FIXME */
}

static int
axi_csi_rx_ioctl_log_status(struct file *file, void *priv_fh)
{
	struct axi_csi_rx *csi = video_drvdata(file);

	v4l2_device_call_all(&csi->v4l2_dev, 0, core, log_status);

	return 0;
}


#ifdef CONFIG_VIDEO_ADV_DEBUG
static int
axi_csi_rx_ioctl_get_register(struct file *file, void *priv_fh,
                              struct v4l2_dbg_register *reg)
{
	struct axi_csi_rx *csi = video_drvdata(file);

	reg->val = axi_csi_rx_read(csi, reg->reg);
	reg->size = 4;

	return 0;
}

static int
axi_csi_rx_ioctl_set_register(struct file *file, void *priv_fh,
                              const struct v4l2_dbg_register *reg)
{
	struct axi_csi_rx *csi = video_drvdata(file);

	axi_csi_rx_write(csi, reg->reg, reg->val);

	return 0;
}
#endif

static const struct v4l2_ioctl_ops axi_csi_rx_ioctl_ops = {
	.vidioc_querycap		= axi_csi_rx_ioctl_querycap,
	.vidioc_enum_fmt_vid_cap	= axi_csi_rx_ioctl_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= axi_csi_rx_ioctl_get_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		= axi_csi_rx_ioctl_set_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap		= axi_csi_rx_ioctl_try_fmt_vid_cap,
	.vidioc_log_status		= axi_csi_rx_ioctl_log_status,
	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_register		= axi_csi_rx_ioctl_get_register,
	.vidioc_s_register		= axi_csi_rx_ioctl_set_register,
#endif
};


 /* Queue Operations */

static int
axi_csi_rx_qops_queue_setup(struct vb2_queue *q,
	const struct v4l2_format *fmt, unsigned int *num_buffers,
	unsigned int *num_planes, unsigned int sizes[], void *alloc_ctxs[])
{
	struct axi_csi_rx *csi = vb2_get_drv_priv(q);
	struct axi_csi_rx_stream *s = &csi->stream;

	v4l2_info(&csi->v4l2_dev, "%s\n", __func__);

	return 0;
}

static int 
axi_csi_rx_qops_buf_prepare(struct vb2_buffer *vb)
{
	struct axi_csi_rx *csi = vb2_get_drv_priv(vb->vb2_queue);
	struct axi_csi_rx_stream *s = &csi->stream;

	v4l2_info(&csi->v4l2_dev, "%s\n", __func__);

	return 0; /* FIXME */
}

static void
axi_csi_rx_qops_buf_queue(struct vb2_buffer *vb)
{
	struct axi_csi_rx *csi = vb2_get_drv_priv(vb->vb2_queue);
	struct axi_csi_rx_stream *s = &csi->stream;

	v4l2_info(&csi->v4l2_dev, "%s\n", __func__);
	/* FIXME */
}

static int
axi_csi_rx_qops_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct axi_csi_rx *csi = vb2_get_drv_priv(q);
	struct axi_csi_rx_stream *s = &csi->stream;

	v4l2_info(&csi->v4l2_dev, "%s\n", __func__);
	return 0; /* FIXME */
}

static int
axi_csi_rx_qops_stop_streaming(struct vb2_queue *q)
{
	struct axi_csi_rx *csi = vb2_get_drv_priv(q);
	struct axi_csi_rx_stream *s = &csi->stream;

	v4l2_info(&csi->v4l2_dev, "%s\n", __func__);
	return 0; /* FIXME */
}

static const struct vb2_ops axi_csi_rx_qops = {
	.queue_setup		= axi_csi_rx_qops_queue_setup,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.buf_prepare		= axi_csi_rx_qops_buf_prepare,
	.buf_queue		= axi_csi_rx_qops_buf_queue,
	.start_streaming	= axi_csi_rx_qops_start_streaming,
	.stop_streaming		= axi_csi_rx_qops_stop_streaming,
};


 /* */

static int
axi_csi_rx_nodes_register(struct axi_csi_rx *csi)
{
	struct axi_csi_rx_stream *s = &csi->stream;
	struct video_device *vdev = &s->vdev;
	struct vb2_queue *q = &s->q;
	int rv;

	mutex_init(&s->lock);

	vdev->v4l2_dev = &csi->v4l2_dev;
	vdev->fops     = &axi_csi_rx_fops;
	vdev->release  = video_device_release_empty;
	vdev->ctrl_handler = s->subdev->ctrl_handler;
	set_bit(V4L2_FL_USE_FH_PRIO, &vdev->flags);
	vdev->lock     = &s->lock;
	vdev->queue    = q;

	q->lock = &s->lock;

	vdev->ioctl_ops = &axi_csi_rx_ioctl_ops;

	q->type            = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes        = VB2_MMAP | VB2_USERPTR | VB2_READ;
	q->drv_priv        = csi;
	q->buf_struct_size = sizeof(struct axi_csi_rx_buffer);
	q->ops             = &axi_csi_rx_qops;
	q->mem_ops         = &vb2_dma_contig_memops;
	q->timestamp_type  = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	rv = vb2_queue_init(q);
	if (rv)
		return rv;

	return video_register_device(vdev, VFL_TYPE_GRABBER, -1);
}

 /* Async subdev probing */
static struct axi_csi_rx *
notifier_to_axi_csi_rx(struct v4l2_async_notifier *n)
{
	return container_of(n, struct axi_csi_rx, notifier);
}

static int
axi_csi_rx_async_bound(struct v4l2_async_notifier *notifier,
                       struct v4l2_subdev *subdev,
                       struct v4l2_async_subdev *asd)
{
	struct axi_csi_rx *csi = notifier_to_axi_csi_rx(notifier);

	v4l2_info(&csi->v4l2_dev, "async bound\n");

	csi->stream.subdev = subdev;

	return 0;
}

static int
axi_csi_rx_async_complete(struct v4l2_async_notifier *notifier)
{
	struct axi_csi_rx *csi = notifier_to_axi_csi_rx(notifier);
	int rv;

	v4l2_info(&csi->v4l2_dev, "async complete\n");

	rv = v4l2_device_register_subdev_nodes(&csi->v4l2_dev);
	if (rv < 0)
		return rv;

	return axi_csi_rx_nodes_register(csi);
}




/* Platform Driver  ------------------------------------------------------- */

static int
axi_csi_rx_probe(struct platform_device *pdev)
{
	struct axi_csi_rx *csi;
	struct resource *regs;
	struct device_node *ep_node;
	int irq, rv;

	dev_info(&pdev->dev, "probe\n");

	/* Alloc our private struct */
	csi = devm_kzalloc(&pdev->dev, sizeof(struct axi_csi_rx), GFP_KERNEL);
	if (!csi)
		return -ENOMEM;

	/* Get info & resources from the DT */
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "Invalid (or missing) base address\n");
		return -ENXIO;
	}
	csi->base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(csi->base))
		return PTR_ERR(csi->base);

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(&pdev->dev, "Invalid (or missing) IRQ\n");
		return -ENXIO;
	}

	csi->gpio_led = devm_gpiod_get(&pdev->dev, "led");
	if (!IS_ERR(csi->gpio_led)) {
		gpiod_direction_output(csi->gpio_led, 0);
	} else {
		csi->gpio_led = NULL;
	}

	csi->stream.chan = dma_request_slave_channel(&pdev->dev, "video");
	if (!csi->stream.chan)
		return -EPROBE_DEFER;

	/* V4L2 setup */
	csi->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(csi->alloc_ctx)) {
		rv = PTR_ERR(csi->alloc_ctx);
		dev_err(&pdev->dev, "Failed to init dma ctx: %d\n", rv);
		goto err_dma_release_channel;
	}

	rv = v4l2_device_register(&pdev->dev, &csi->v4l2_dev);
	if (rv) {
		dev_err(&pdev->dev, "Failed to register V4L device: %d\n", rv);
		goto err_dma_cleanup_ctx;
	}

	ep_node = v4l2_of_get_next_endpoint(pdev->dev.of_node, NULL);
	if (!ep_node) {
		rv = -EINVAL;
		goto err_device_unregister;
	}

	csi->asd.match_type = V4L2_ASYNC_MATCH_OF;
	csi->asd.match.of.node = v4l2_of_get_remote_port_parent(ep_node);

	csi->asds[0] = &csi->asd;
	csi->notifier.subdevs     = csi->asds;
	csi->notifier.num_subdevs = ARRAY_SIZE(csi->asds);
	csi->notifier.bound       = axi_csi_rx_async_bound;
	csi->notifier.complete    = axi_csi_rx_async_complete;

	rv = v4l2_async_notifier_register(&csi->v4l2_dev, &csi->notifier);
	if (rv) {
		dev_err(&pdev->dev, "Error %d registering device nodes\n", rv);
		goto err_device_unregister;
	}

	/* Init result */
	dev_info(&pdev->dev, "Regs     : 0x%08x -> 0x%08x [%s]\n",
		regs->start, regs->end, regs->name);
	dev_info(&pdev->dev, "IRQ      : %d\n", irq);
	dev_info(&pdev->dev, "GPIO LED : %d\n",
		csi->gpio_led ? desc_to_gpio(csi->gpio_led) : -1);

	return 0;

err_device_unregister:
	v4l2_device_unregister(&csi->v4l2_dev);
err_dma_cleanup_ctx:
	vb2_dma_contig_cleanup_ctx(csi->alloc_ctx);
err_dma_release_channel:
	dma_release_channel(csi->stream.chan);

	return rv;
}

static int
axi_csi_rx_remove(struct platform_device *pdev)
{
	struct axi_csi_rx *csi = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "remove\n");

	v4l2_async_notifier_unregister(&csi->notifier);
	video_unregister_device(&csi->stream.vdev);
	v4l2_device_unregister(&csi->v4l2_dev);
	vb2_dma_contig_cleanup_ctx(csi->alloc_ctx);
	dma_release_channel(csi->stream.chan);

	return 0;
}


static struct of_device_id axi_csi_rx_of_match[] = {
	{ .compatible = "s47,axi-csi-rx-1.00.a", },
	{ }
};
MODULE_DEVICE_TABLE(of, axi_csi_rx_of_match);

static struct platform_driver axi_csi_rx_driver = {
	.probe  = axi_csi_rx_probe,
	.remove = axi_csi_rx_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name  = DRIVER_NAME,
		.of_match_table = axi_csi_rx_of_match,
	},
};
module_platform_driver(axi_csi_rx_driver);

MODULE_AUTHOR("Sylvain Munaut <tnt@246tNt.com>");
MODULE_DESCRIPTION("MIPI CSI RX interface on AXI bus");
MODULE_LICENSE("GPL");
