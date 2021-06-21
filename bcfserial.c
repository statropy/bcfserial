
/*
 *  bcfserial.c - Serial interface driver for Beagle Connect Freedom.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/serdev.h>
#include <linux/sched.h>
#include <linux/skbuff.h>

#include <net/cfg802154.h>
#include <net/mac802154.h>

#define BCFSERIAL_DRV_VERSION "0.1.0"
#define BCFSERIAL_DRV_NAME "bcfserial"

#define HDLC_FRAME      0x7E
#define HDLC_ESC        0x7D
#define HDLC_XOR        0x20

#define MAX_PSDU		127
#define MAX_RX_XFER		(1 + MAX_PSDU + 2 + 1)	/* PHR+PSDU+CRC+LQI */
#define HDLC_HEADER_LEN 	2
#define PACKET_HEADER_LEN	8
#define CRC_LEN 		2
#define MAX_TX_HDLC		(1 + HDLC_HEADER_LEN + PACKET_HEADER_LEN + MAX_RX_XFER + CRC_LEN + 1)

enum bcfserial_requests {
	RESET,
	TX,
	XMIT_ASYNC,
	ED,
	SET_CHANNEL,
	START,
	STOP,
	SET_SHORT_ADDR,
	SET_PAN_ID,
	SET_IEEE_ADDR,
	SET_TXPOWER,
	SET_CCA_MODE,
	SET_CCA_ED_LEVEL,
	SET_CSMA_PARAMS,
	SET_LBT,
	SET_FRAME_RETRIES,
	SET_PROMISCUOUS_MODE,
	GET_EXTENDED_ADDR,
	GET_SUPPORTED_CHANNELS,
};

struct bcfserial {
	struct serdev_device *serdev;
	struct ieee802154_hw *hw;
	struct work_struct tx_work;

	struct sk_buff *tx_skb;
	u8 tx_ack_seq;		/* current TX ACK sequence number */
	u8 *tx_head;
	u8 *tx_tail;
	int tx_remaining;
	u8 *tx_buffer;
	u16 tx_crc;
};

// TODO Add serial buffers and async workers for serdev

// TODO Add HDLC parsing and packing

static int bcfserial_tty_receive(struct serdev_device *serdev, 
	const unsigned char *data, size_t count)
{
	// struct bcfserial *bcfserial = serdev_device_get_drvdata(serdev);
	// size_t i;

	count = serdev_device_write_buf(serdev, data, count);
	printk("Echo %u\n", count);
	return count;
}

static void bcfserial_uart_transmit(struct work_struct *work)
{
	struct bcfserial *bcfserial = container_of(work, struct bcfserial, tx_work);
	int written;

	if (bcfserial->tx_remaining) {
		written = serdev_device_write_buf(bcfserial->serdev, bcfserial->tx_buffer,
					  bcfserial->tx_remaining);
		if (written > 0) {
			printk("Work send %d\n", written);
			bcfserial->tx_head += written;
			bcfserial->tx_remaining -= written;
		}
	}
}

static void bcfserial_tty_wakeup(struct serdev_device *serdev)
{
	struct bcfserial *bcfserial = serdev_device_get_drvdata(serdev);

	schedule_work(&bcfserial->tx_work);
}

static struct serdev_device_ops bcfserial_serdev_ops = {
	.receive_buf = bcfserial_tty_receive,
	.write_wakeup = bcfserial_tty_wakeup,
};

static void bcfserial_append_tx_frame(struct bcfserial *bcfserial)
{
	*bcfserial->tx_tail++ = HDLC_FRAME;
}

static void bcfserial_append_tx_u8(struct bcfserial *bcfserial, u8 value)
{
	// TODO Update CRC
	if (value == HDLC_FRAME || value == HDLC_ESC) {
		*bcfserial->tx_tail++ = HDLC_ESC;
		value ^= HDLC_XOR;
	}
	*bcfserial->tx_tail++ = value^HDLC_XOR;
}

static void bcfserial_append_tx_buffer(struct bcfserial *bcfserial, const u8 *buffer, size_t len)
{
	for(size_t i=0; i<len; i++) {
		bcfserial_append_tx_u8(bcfserial, buffer[i]);
	}
}

static void bcfserial_append_tx_le16(struct bcfserial *bcfserial, u16 value)
{
	value = cpu_to_le16(value);
	bcfserial_append_tx_buffer(bcfserial, (u8 *)&value, sizeof(u16));
}

static void bcfserial_append_crc(struct bcfserial *bcfserial, u16 value)
{
	bcfserial_append_tx_le16(bcfserial, bcfserial->tx_crc);
}

static void bcfserial_hdlc_send(struct bcfserial *bcfserial, u8 cmd, u16 value, u16 index, u16 length, const u8* buffer)
{
	int written;
	// HDLC_FRAME
	// 0 address : 0x01
	// 1 control : 0x03
	// 2 [bmRequestType] : 0x00
	// 3 cmd (TX, START, STOP, etc)
	// 4/5 value
	// 6/7 index
	// 8/9 length
	// contents
	// x/y crc
	// HDLC_FRAME
	WARN_ON(bcfserial->tx_remaining);

	bcfserial->tx_remaining = 0;
	bcfserial->tx_head = bcfserial->tx_buffer;
	bcfserial->tx_tail = bcfserial->tx_head;
	bcfserial->tx_crc = 0xFFFF;

	bcfserial_append_tx_frame(bcfserial);
	bcfserial_append_tx_u8(bcfserial, 0x01); //address
	bcfserial_append_tx_u8(bcfserial, 0x03); //control
	bcfserial_append_tx_u8(bcfserial, 0x00); //ignored
	bcfserial_append_tx_u8(bcfserial, cmd);
	bcfserial_append_tx_le16(bcfserial, value);
	bcfserial_append_tx_le16(bcfserial, index);
	bcfserial_append_tx_le16(bcfserial, length);
	bcfserial_append_tx_buffer(bcfserial, buffer, length);
	bcfserial_append_tx_crc(bcfserial);
	bcfserial_append_tx_frame(bcfserial);

	written = serdev_device_write_buf(bcfserial->serdev, bcfserial->tx_buffer,
					  bcfserial->tx_remaining);
	if (written > 0) {
		printk("Send %d\n", written);
		bcfserial->tx_head += written;
		bcfserial->tx_remaining = bcfserial->tx_tail - bcfserial->tx_head;
	}
}

static void bcfserial_hdlc_send_cmd(struct bcfserial *bcfserial, u8 cmd)
{
	bcfserial_hdlc_send(bcfserial, cmd, 0, 0, 0, NULL);
}

// TODO Add implementations for 802154 functions

static int bcfserial_start(struct ieee802154_hw *hw)
{
	struct bcfserial *bcfserial = hw->priv;
	printk("Start\n");
	bcfserial_hdlc_send_cmd(bcfserial, START);
	return 0;
}

static void bcfserial_stop(struct ieee802154_hw *hw)
{
	struct bcfserial *bcfserial = hw->priv;
	printk("Stop\n");
	bcfserial_hdlc_send_cmd(bcfserial, STOP);
}

// TODO call ieee802154_xmit_complete after good tx ack

static int bcfserial_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct bcfserial *bcfserial = hw->priv;

	bcfserial->tx_skb = skb;
	bcfserial->tx_ack_seq++;

	bcfserial_hdlc_send(bcfserial, TX, 0, bcfserial->tx_ack_seq, skb->len);
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
	size_t len;

	printk("Loading bcfserial\n");

	hw = ieee802154_alloc_hw(sizeof(struct bcfserial), &bcfserial_ops);
	if (!hw)
		return -ENOMEM;

	bcfserial = hw->priv;
	bcfserial->hw = hw;
	hw->parent = &serdev->dev;
	bcfserial->serdev = serdev;

	INIT_WORK(&bcfserial->tx_work, bcfserial_uart_transmit);

	serdev_device_set_drvdata(serdev, bcfserial);
	serdev_device_set_client_ops(serdev, &bcfserial_serdev_ops);
	
	ret = serdev_device_open(serdev);
	if (ret) {
		printk("Unable to open device\n");
		goto fail_hw;
	}

	serdev_device_set_drvdata(serdev, bcfserial);

	speed = serdev_device_set_baudrate(serdev, speed);
	printk("Using baudrate %u\n", speed);

	serdev_device_set_flow_control(serdev, false);

	// TODO connect with BeagleConnect Freedom using serial cmds


	ret = ieee802154_register_hw(hw);
	if (ret)
		goto fail;

	bcfserial->tx_buffer = devm_kmalloc(&serdev->dev, MAX_TX_HDLC, GFP_KERNEL);

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
