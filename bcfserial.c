
/*
 *  bcfserial.c - Serial interface driver for Beagle Connect Freedom.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/serdev.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <net/cfg802154.h>
#include <net/mac802154.h>

#define BCFSERIAL_DRV_VERSION "0.1.0"
#define BCFSERIAL_DRV_NAME "bcfserial"

struct bcfserial {
	struct serdev_device *serdev;
	struct ieee802154_hw *hw;
};

// TODO Add serial buffers and async workers for serdev

// TODO Add HDLC parsing and packing

static int tty_receive(struct serdev_device *serdev, 
	const unsigned char *data, size_t count)
{
	// struct bcfserial *hi = serdev_device_get_drvdata(serdev);
	// size_t i;

	count = serdev_device_write_buf(serdev, data, count);
	printk("Echo %u\n", count);
	return count;
}

static void tty_wakeup(struct serdev_device *serdev)
{
	// TODO
	printk("bcfserial Wakeup\n");
}

static struct serdev_device_ops bcfserial_serdev_ops = {
	.receive_buf = tty_receive,
	.write_wakeup = tty_wakeup,
};

// TODO Add implementations for 802154 functions

static int bcfserial_start(struct ieee802154_hw *hw)
{
	return 0;
}

static void bcfserial_stop(struct ieee802154_hw *hw)
{
}

static int bcfserial_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	return 0;
}

static int bcfserial_ed(struct ieee802154_hw *hw, u8 *level)
{
	return 0;
}

static int bcfserial_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	return 0;
}

static int bcfserial_set_hw_addr_filt(struct ieee802154_hw *hw,
			       	      struct ieee802154_hw_addr_filt *filt,
			       	      unsigned long changed)
{
	return 0;
}

static int bcfserial_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
	return 0;
}

static int bcfserial_set_lbt(struct ieee802154_hw *hw, bool on)
{
	return 0;
}

static int bcfserial_set_cca_mode(struct ieee802154_hw *hw,
			   const struct wpan_phy_cca *cca)
{
	return 0;
}

static int bcfserial_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm)
{
	return 0;
}

static int bcfserial_set_csma_params(struct ieee802154_hw *hw, u8 min_be, u8 max_be, 
			      u8 retries)
{
	return 0;
}

static int bcfserial_set_frame_retries(struct ieee802154_hw *hw, s8 retries)
{
	return 0;
}

static int bcfserial_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on)
{
	return 0;
}

static const struct ieee802154_ops bcfserial_ops = {
	.owner			= THIS_MODULE,
	.start			= bcfserial_start,
	.stop			= bcfserial_stop,
	.xmit_async		= bcfserial_xmit,
	.ed			= bcfserial_ed,
	.set_channel		= bcfserial_set_channel,
	.set_hw_addr_filt	= bcfserial_set_hw_addr_filt,
	.set_txpower		= bcfserial_set_txpower,
	.set_lbt		= bcfserial_set_lbt,
	.set_cca_mode		= bcfserial_set_cca_mode,
	.set_cca_ed_level	= bcfserial_set_cca_ed_level,
	.set_csma_params	= bcfserial_set_csma_params,
	.set_frame_retries	= bcfserial_set_frame_retries,
	.set_promiscuous_mode	= bcfserial_set_promiscuous_mode,
};

static const struct of_device_id bcfserial_of_match[] = {
	{
	.compatible = "beagle,bcfserial",
	},
	{}
};
MODULE_DEVICE_TABLE(of, bcfserial_of_match);

static int bcfserial_probe(struct serdev_device *serdev)
{
	struct ieee802154_hw *hw;
	struct bcfserial *bcfserial = NULL;
	u32 speed = 115200;
	int ret;

	printk("Loading bcfserial\n");

	hw = ieee802154_alloc_hw(sizeof(struct bcfserial), &bcfserial_ops);
	if (!hw)
		return -ENOMEM;

	bcfserial = hw->priv;
	bcfserial->hw = hw;
	hw->parent = &serdev->dev;

	serdev_device_set_client_ops(serdev, &bcfserial_serdev_ops);
	
	ret = serdev_device_open(serdev);
	if (ret) {
		printk("Unable to open device\n");
		goto fail_hw;
	}

	speed = serdev_device_set_baudrate(serdev, speed);
	printk("Using baudrate %u\n", speed);

	serdev_device_set_flow_control(serdev, false);

	// TODO connect with BeagleConnect Freedom using serial cmds


	ret = ieee802154_register_hw(hw);
	if (ret)
		goto fail;

	return 0;

fail:
	printk("Closing serial device on failure");
	serdev_device_close(serdev);
fail_hw:
	printk("Closing wpan hw on failure");
	ieee802154_free_hw(hw);
	return ret;
}

static void bcfserial_remove(struct serdev_device *serdev)
{
	printk("Closing serial device");
	serdev_device_close(serdev);
}

static struct serdev_device_driver bcfserial_driver = {
	.probe = bcfserial_probe,
	.remove = bcfserial_remove,
	.driver = {
		.name = BCFSERIAL_DRV_NAME,
		.of_match_table = of_match_ptr(bcfserial_of_match),
	},
};

module_serdev_device_driver(bcfserial_driver);

MODULE_DESCRIPTION("WPAN serial driver for BeagleConnect Freedom");
MODULE_AUTHOR("Erik Larson <erik@statropy.com>");
MODULE_VERSION("0.1.0");
MODULE_LICENSE("GPL v2");
