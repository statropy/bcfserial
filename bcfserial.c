
/*
 *  bcfserial.c - Serial interface driver for Beagle Connect Freedom.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/serdev.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/module.h>

#define BCFSERIAL_DRV_VERSION "0.1.0"
#define BCFSERIAL_DRV_NAME "bcfserial"

struct bcfserial {
	struct serdev_device *serdev;
};

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

static const struct of_device_id bcfserial_of_match[] = {
	{
	.compatible = "beagle,bcfserial",
	},
	{}
};
MODULE_DEVICE_TABLE(of, bcfserial_of_match);

static int bcfserial_probe(struct serdev_device *serdev)
{
	u32 speed = 115200;
	int ret;

	serdev_device_set_client_ops(serdev, &bcfserial_serdev_ops);

	ret = serdev_device_open(serdev);
	if (ret) {
		printk("Unable to open device\n");
		return ret;
	}

	speed = serdev_device_set_baudrate(serdev, speed);
	printk("Using baudrate %u\n", speed);

	serdev_device_set_flow_control(serdev, false);

	return 0;
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

MODULE_LICENSE("GPL v2");
