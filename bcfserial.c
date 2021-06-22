
/*
 *  bcfserial.c - Serial interface driver for Beagle Connect Freedom.
 */
#include <linux/crc-ccitt.h>
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

#define HDLC_FRAME	0x7E
#define HDLC_ESC	0x7D
#define HDLC_XOR	0x20

#define MAX_PSDU		127
#define MAX_RX_XFER		(1 + MAX_PSDU + 2 + 1)	/* PHR+PSDU+CRC+LQI */
#define HDLC_HEADER_LEN 	2
#define PACKET_HEADER_LEN	8
#define CRC_LEN 		2
#define RX_HDLC_PAYLOAD		140
#define MAX_TX_HDLC		(1 + HDLC_HEADER_LEN + PACKET_HEADER_LEN + MAX_RX_XFER + CRC_LEN + 1)
#define MAX_RX_HDLC		(1 + RX_HDLC_PAYLOAD + CRC_LEN)

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
	spinlock_t tx_lock;
	struct sk_buff *tx_skb;
	u8 tx_ack_seq;		/* current TX ACK sequence number */
	u8 *tx_head;
	u8 *tx_tail;
	int tx_remaining;
	u8 *tx_buffer;
	u16 tx_crc;

	u8 rx_in_esc;
	u8 rx_address;
	u16 rx_offset;
	u8 *rx_buffer;
};

// RX Packet Format:
// - WPAN RX PACKET:	[len] payload [lqi]
// - WPAN TX ACK:	[seq]
// - WPAN CAPABILITIES:	supported_channels_mask(4)
// - CDC:		printable_chars

// TX Packet Format:


// TODO Add HDLC parsing and packing
// - explore using ieee802154_wake_queue in bcfserial_uart_transmit (see qca)
// - Always require ACK? (not supported correctly in wpanusb_bc)

static void bcfserial_append_tx_frame(struct bcfserial *bcfserial)
{
	*bcfserial->tx_tail++ = HDLC_FRAME;
}

static void bcfserial_append_tx_u8(struct bcfserial *bcfserial, u8 value)
{
	bcfserial->crc = crc_ccitt(bcfserial->crc, &value, 1);
	if (value == HDLC_FRAME || value == HDLC_ESC) {
		*bcfserial->tx_tail++ = HDLC_ESC;
		value ^= HDLC_XOR;
	}
	*bcfserial->tx_tail++ = value;
}

static void bcfserial_append_tx_buffer(struct bcfserial *bcfserial, const u8 *buffer, size_t len)
{
	size_t i;
	for (i=0; i<len; i++) {
		bcfserial_append_tx_u8(bcfserial, buffer[i]);
	}
}

static void bcfserial_append_tx_le16(struct bcfserial *bcfserial, u16 value)
{
	value = cpu_to_le16(value);
	bcfserial_append_tx_buffer(bcfserial, (u8 *)&value, sizeof(u16));
}

static void bcfserial_append_tx_crc(struct bcfserial *bcfserial)
{
	bcfserial->tx_crc ^= 0xffff;
	bcfserial_append_tx_u8(bcfserial, bcfserial->tx_crc & 0xff);
	bcfserial_append_tx_u8(bcfserial, (bcfserial->tx_crc >> 8) & 0xff);
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

	spin_lock(&bcfserial->tx_lock);
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

	bcfserial->tx_remaining = bcfserial->tx_tail - bcfserial->tx_head;
	written = serdev_device_write_buf(bcfserial->serdev, bcfserial->tx_buffer,
					  bcfserial->tx_remaining);

	printk("Sending %d\n", bcfserial->tx_remaining);

	if (written > 0) {
		printk("Sent %d\n", written);
		bcfserial->tx_head += written;
		bcfserial->tx_remaining -= written;

		// TODO move to TX ACK handler
		if (bcfserial->tx_remaining <= 0 && bcfserial->tx_buffer[4] == TX) {
			ieee802154_xmit_complete(bcfserial->hw, bcfserial->tx_skb, false);
 		}

	}
	spin_unlock(&bcfserial->tx_lock);
}

static void bcfserial_hdlc_send_cmd(struct bcfserial *bcfserial, u8 cmd)
{
	bcfserial_hdlc_send(bcfserial, cmd, 0, 0, 0, NULL);
}

// TODO Add implementations for 802154 functions

static int bcfserial_start(struct ieee802154_hw *hw)
{
	struct bcfserial *bcfserial = hw->priv;
	printk("START\n");
	bcfserial_hdlc_send_cmd(bcfserial, START);
	return 0;
}

static void bcfserial_stop(struct ieee802154_hw *hw)
{
	struct bcfserial *bcfserial = hw->priv;
	printk("STOP\n");
	bcfserial_hdlc_send_cmd(bcfserial, STOP);
}

static int bcfserial_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct bcfserial *bcfserial = hw->priv;

	printk("XMIT\n");

	bcfserial->tx_skb = skb;
	bcfserial->tx_ack_seq++;

	bcfserial_hdlc_send(bcfserial, TX, 0, bcfserial->tx_ack_seq, skb->len, skb->data);
	return 0;
}

static int bcfserial_ed(struct ieee802154_hw *hw, u8 *level)
{
	printk("ED\n");
	return 0;
}

static int bcfserial_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	printk("SET CHANNEL %u %u\n", page, channel);
	return 0;
}

static int bcfserial_set_hw_addr_filt(struct ieee802154_hw *hw,
				      struct ieee802154_hw_addr_filt *filt,
				      unsigned long changed)
{
	printk("HW ADDR\n");
	return 0;
}

static int bcfserial_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
	printk("SET TXPOWER\n");
	return 0;
}

static int bcfserial_set_lbt(struct ieee802154_hw *hw, bool on)
{
	printk("SET LBT\n");
	return 0;
}

static int bcfserial_set_cca_mode(struct ieee802154_hw *hw,
			   const struct wpan_phy_cca *cca)
{
	printk("SET CCA MODE\n");
	return 0;
}

static int bcfserial_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm)
{
	printk("SET CCA ED LEVEL\n");
	return 0;
}

static int bcfserial_set_csma_params(struct ieee802154_hw *hw, u8 min_be, u8 max_be,
			      u8 retries)
{
	printk("SET CSMA PARAMS\n");
	return 0;
}

static int bcfserial_set_frame_retries(struct ieee802154_hw *hw, s8 retries)
{
	printk("SET FRAME RETRIES\n");
	return 0;
}

static int bcfserial_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on)
{
	printk("SET PROMISCUOUS\n");
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

static void bcfserial_wpan_rx(struct bcfserial *bcfserial, const u8 *buffer, size_t count)
{
	// TODO Handle get capabilities blocking

	if (count == 1) {
		// TX ACK
		//dev_dbg(&udev->dev, "seq 0x%02x expect 0x%02x\n", seq, expect);
		printk("TX ACK: 0x%02x:0x%02x\n", buffer[1], bcfserial->tx_ack_seq);

		if (buffer[0] == bcfserial->tx_ack_seq) {
			ieee802154_xmit_complete(bcfserial->hw, bcfserial->tx_skb, false);
		} else {
			//dev_dbg(&udev->dev, "unknown ack %u\n", seq);

			ieee802154_wake_queue(bcfserial->hw);
			if (bcfserial->tx_skb)
				dev_kfree_skb_irq(bcfserial->tx_skb);
		}
	} else {
		// RX Packet
		printk("RX Packet Len:%u LQI:%u\n", buffer[0], buffer[count-1]);
	}
}

static int bcfserial_tty_receive(struct serdev_device *serdev, 
	const unsigned char *data, size_t count)
{
	struct bcfserial *bcfserial = serdev_device_get_drvdata(serdev);
	u16 crc_check = 0;
	u16 abort_size = 0;
	size_t i;
	u8 c;

	for (i = 0; i < count; i++) {
		c = data[i];
	
		if (c == HDLC_FRAME) {
			if (bcfserial->rx_address != 0xFF) {

				bcfserial->rx_crc = crc_ccitt(0xffff, &bcfserial->rx_address, 1);
				bcfserial->rx_crc = crc_ccitt(bcfserial->rx_crc, bcfserial->rx_buffer, bcfserial->rx_offset);

				if (crc_check == 0xf0b8) {
					printk("RX HDLC: %u:%u \n", bcfserial->rx_address, bcfserial->rx_offset - 3);
					// TODO send ACK packet - contention?

					// if ((bcfserial->rx_buffer[0] & 1) == 0) {
					// //I-Frame, send S-Frame ACK
					// USBWPAN_sendAck(bcfserial->rx_address, (bcfserial->rx_buffer[0] >> 1) & 0x7);

					if (bcfserial->rx_address == ADDRESS_WPAN) {
						// if (USBWPAN_getInterfaceStatus(WPAN0_INTFNUM) & USBWPAN_WAITING_FOR_SEND) {
						// 	USBWPAN_abortSend(&abort_size, WPAN0_INTFNUM);
						// }
						// USBWPAN_sendData(bcfserial->rx_buffer+1, bcfserial->rx_offset-3, WPAN0_INTFNUM);
						bcfserial_wpan_rx(bcfserial, bcfserial->rx_buffer + 1, bcfserial->rx_offset - 3);
					}
					else if (bcfserial->rx_address == ADDRESS_CDC) {
						// if (USBCDC_getInterfaceStatus(CDC0_INTFNUM,&bytesSent,&bytesReceived) & USBCDC_WAITING_FOR_SEND) {
						// 	USBCDC_abortSend(&abort_size, CDC0_INTFNUM);
						// }
						// USBCDC_sendData(bcfserial->rx_buffer+1, bcfserial->rx_offset - 3, CDC0_INTFNUM);
					}
				}
				else {
					printk("CRC Failed: 0x%04x\n", crc_check);
				}
			}
			bcfserial->rx_offset = 0;
			bcfserial->rx_address = 0xFF;
		} else if (c == HDLC_ESC) {
			inEsc = TRUE;
		} else {
			if (inEsc) {
				c ^= 0x20;
				inEsc = FALSE;
			}

			if (bcfserial->rx_address == 0xFF) {
				bcfserial->rx_address = c;
				if (bcfserial->rx_address == ADDRESS_WPAN || 
				   bcfserial->rx_address == ADDRESS_CDC ||
				   bcfserial->rx_address == ADDRESS_HW) {
				} else {
					bcfserial->rx_address = 0xFF;
				}
					bcfserial->rx_offset = 0;
			} else {
				if (bcfserial->rx_offset < BUFFER_SIZE) {
					bcfserial->rx_buffer[bcfserial->rx_offset] = c;
					bcfserial->rx_offset++;
				} else {
					//buffer overflow
					bcfserial->rx_address = 0xFF;
					bcfserial->rx_offset = 0;
				}
			}
		}
	}
}

static void bcfserial_uart_transmit(struct work_struct *work)
{
	struct bcfserial *bcfserial = container_of(work, struct bcfserial, tx_work);
	int written;

	spin_lock_bh(&bcfserial->tx_lock);

	if (bcfserial->tx_remaining) {
		written = serdev_device_write_buf(bcfserial->serdev, bcfserial->tx_head,
					  bcfserial->tx_remaining);
		if (written > 0) {
			printk("Work sent %d\n", written);
			bcfserial->tx_head += written;
			bcfserial->tx_remaining -= written;

			// TODO move to TX ACK handler
			if (bcfserial->tx_remaining <= 0 && bcfserial->tx_buffer[4] == TX) {
				ieee802154_xmit_complete(bcfserial->hw, bcfserial->tx_skb, false);
 			}
		}
	}
	spin_unlock_bh(&bcfserial->tx_lock);
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

static const struct of_device_id bcfserial_of_match[] = {
	{
	.compatible = "beagle,bcfserial",
	},
	{}
};
MODULE_DEVICE_TABLE(of, bcfserial_of_match);

static const s32 channel_powers[] = {
	300, 280, 230, 180, 130, 70, 0, -100, -200, -300, -400, -500, -700,
	-900, -1200, -1700,
};

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
	hw->flags = IEEE802154_HW_TX_OMIT_CKSUM | IEEE802154_HW_AFILT;

	/* FIXME: these need to come from device capabilities */
	hw->phy->flags = WPAN_PHY_FLAG_TXPOWER;

	/* Set default and supported channels */
	hw->phy->current_page = 0;
	hw->phy->current_channel = 1; //set to lowest valid channel
	hw->phy->supported.channels[0] = 0x07FFFFFF;

	/* FIXME: these need to come from device capabilities */
	hw->phy->supported.tx_powers = channel_powers;
	hw->phy->supported.tx_powers_size = ARRAY_SIZE(channel_powers);
	hw->phy->transmit_power = hw->phy->supported.tx_powers[0];

	ret = ieee802154_register_hw(hw);
	if (ret)
		goto fail;

	spin_lock_init(&bcfserial->tx_lock);
	bcfserial->tx_buffer = devm_kmalloc(&serdev->dev, MAX_TX_HDLC, GFP_KERNEL);
	bcfserial->tx_remaining = 0;
	bcfserial->rx_buffer = devm_kmalloc(&serdev->dev, MAX_RX_HDLC, GFP_KERNEL);
	bcfserial->rx_offset = 0;
	bcfserial->rx_address = 0xff;
	return 0;

fail:
	printk("Closing serial device on failure\n");
	serdev_device_close(serdev);
fail_hw:
	printk("Closing wpan hw on failure\n");
	ieee802154_free_hw(hw);
	return ret;
}

static void bcfserial_remove(struct serdev_device *serdev)
{
	struct bcfserial *bcfserial = serdev_device_get_drvdata(serdev);
	printk("Closing serial device\n");
	cancel_work_sync(&bcfserial->tx_work);
	serdev_device_close(serdev);
	ieee802154_unregister_hw(bcfserial->hw);
	ieee802154_free_hw(bcfserial->hw);
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
