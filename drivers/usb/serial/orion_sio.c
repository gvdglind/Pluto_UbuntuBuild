/*
 * USB FTDI SIO driver
 *
 *	Copyright (C) 2009 - 2013
 *	    Johan Hovold (jhovold@gmail.com)
 *	Copyright (C) 1999 - 2001
 *	    Greg Kroah-Hartman (greg@kroah.com)
 *          Bill Ryder (bryder@sgi.com)
 *	Copyright (C) 2002
 *	    Kuba Ober (kuba@mareimbrium.org)
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 * See Documentation/usb/usb-serial.txt for more information on using this
 * driver
 *
 * See http://ftdi-usb-sio.sourceforge.net for up to date testing info
 *	and extra documentation
 *
 * Change entries from 2004 and earlier can be found in versions of this
 * file in kernel versions prior to the 2.6.24 release.
 *
 */

/* Bill Ryder - bryder@sgi.com - wrote the FTDI_SIO implementation */
/* Thanx to FTDI for so kindly providing details of the protocol required */
/*   to talk to the device */
/* Thanx to gkh and the rest of the usb dev group for all code I have
   assimilated :-) */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/serial.h>
#include <linux/usb/serial.h>
#include "orion_sio.h"

#define DRIVER_AUTHOR "Greg Kroah-Hartman <greg@kroah.com>, Bill Ryder <bryder@sgi.com>, Kuba Ober <kuba@mareimbrium.org>, Andreas Mohr, Johan Hovold <jhovold@gmail.com>"
#define DRIVER_DESC "USB FTDI Serial Converters Driver"


struct ftdi_private {
	enum ftdi_chip_type chip_type;
				/* type of device, either SIO or FT8U232AM */
	int baud_base;		/* baud base clock for divisor setting */
	int custom_divisor;	/* custom_divisor kludge, this is for
				   baud_base (different from what goes to the
				   chip!) */
	__u16 last_set_data_urb_value ;
				/* the last data state set - needed for doing
				 * a break
				 */
	int flags;		/* some ASYNC_xxxx flags are supported */
	unsigned long last_dtr_rts;	/* saved modem control outputs */
	char prev_status;        /* Used for TIOCMIWAIT */
	char transmit_empty;	/* If transmitter is empty or not */
	__u16 interface;	/* FT2232C, FT2232H or FT4232H port interface
				   (0 for FT232/245) */

	speed_t force_baud;	/* if non-zero, force the baud rate to
				   this value */
	int force_rtscts;	/* if non-zero, force RTS-CTS to always
				   be enabled */

	unsigned int latency;		/* latency setting in use */
	unsigned short max_packet_size;
	struct mutex cfg_lock; /* Avoid mess by parallel calls of config ioctl() and change_speed() */
};

/* struct ftdi_sio_quirk is used by devices requiring special attention. */
struct ftdi_sio_quirk {
	int (*probe)(struct usb_serial *);
	/* Special settings for probed ports. */
	void (*port_probe)(struct ftdi_private *);
};
static int ftdi_KP_orion_probe(struct usb_serial *serial);

static struct ftdi_sio_quirk KP_Orion_quirk={
	.probe	= ftdi_KP_orion_probe,
};


/*
 * The 8U232AM has the same API as the sio except for:
 * - it can support MUCH higher baudrates; up to:
 *   o 921600 for RS232 and 2000000 for RS422/485 at 48MHz
 *   o 230400 at 12MHz
 *   so .. 8U232AM's baudrate setting codes are different
 * - it has a two byte status code.
 * - it returns characters every 16ms (the FTDI does it every 40ms)
 *
 * the bcdDevice value is used to differentiate FT232BM and FT245BM from
 * the earlier FT8U232AM and FT8U232BM.  For now, include all known VID/PID
 * combinations in both tables.
 * FIXME: perhaps bcdDevice can also identify 12MHz FT8U232AM devices,
 * but I don't know if those ever went into mass production. [Ian Abbott]
 */



/*
 * Device ID not listed? Test it using
 * /sys/bus/usb-serial/drivers/ftdi_sio/new_id and send a patch or report.
 */
static struct usb_device_id id_table_combined [] = {
	{ USB_DEVICE(KP_VID, KP_ORION_PID) ,
		.driver_info = (kernel_ulong_t)&KP_Orion_quirk },
	{ }					/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, id_table_combined);

static const char *ftdi_chip_name[] = {
	[FT2232C] = "KP ORION",
};


/* Used for TIOCMIWAIT */
#define FTDI_STATUS_B0_MASK	(FTDI_RS0_CTS | FTDI_RS0_DSR | FTDI_RS0_RI | FTDI_RS0_RLSD)
#define FTDI_STATUS_B1_MASK	(FTDI_RS_BI)
/* End TIOCMIWAIT */

/* function prototypes for a FTDI serial converter */
static int  ftdi_sio_probe(struct usb_serial *serial,
					const struct usb_device_id *id);
static int  ftdi_sio_port_probe(struct usb_serial_port *port);
static int  ftdi_sio_port_remove(struct usb_serial_port *port);
static int  ftdi_open(struct tty_struct *tty, struct usb_serial_port *port);
static void ftdi_dtr_rts(struct usb_serial_port *port, int on);
static void ftdi_process_read_urb(struct urb *urb);
static int ftdi_prepare_write_buffer(struct usb_serial_port *port,
						void *dest, size_t size);
static void ftdi_set_termios(struct tty_struct *tty,
			struct usb_serial_port *port, struct ktermios *old);
static int  ftdi_tiocmget(struct tty_struct *tty);
static int  ftdi_tiocmset(struct tty_struct *tty,
			unsigned int set, unsigned int clear);
static int  ftdi_ioctl(struct tty_struct *tty,
			unsigned int cmd, unsigned long arg);
static void ftdi_break_ctl(struct tty_struct *tty, int break_state);
static bool ftdi_tx_empty(struct usb_serial_port *port);
static int ftdi_get_modem_status(struct usb_serial_port *port,
						unsigned char status[2]);

static struct usb_serial_driver ftdi_sio_device = {
	.driver = {
		.owner =	THIS_MODULE,
		.name =		"orion_sio",
	},
	.description =		"ORION USB Serial Device",
	.id_table =		id_table_combined,
	.num_ports =		1,
	.bulk_in_size =		512,
	.bulk_out_size =	512,
	.probe =		ftdi_sio_probe,
	.port_probe =		ftdi_sio_port_probe,
	.port_remove =		ftdi_sio_port_remove,
	.open =			ftdi_open,
	.dtr_rts =		ftdi_dtr_rts,
	.throttle =		usb_serial_generic_throttle,
	.unthrottle =		usb_serial_generic_unthrottle,
	.process_read_urb =	ftdi_process_read_urb,
	.prepare_write_buffer =	ftdi_prepare_write_buffer,
	.tiocmget =		ftdi_tiocmget,
	.tiocmset =		ftdi_tiocmset,
	.tiocmiwait =		usb_serial_generic_tiocmiwait,
	.get_icount =           usb_serial_generic_get_icount,
	.ioctl =		ftdi_ioctl,
	.set_termios =		ftdi_set_termios,
	.break_ctl =		ftdi_break_ctl,
	.tx_empty =		ftdi_tx_empty,
};

static struct usb_serial_driver * const serial_drivers[] = {
	&ftdi_sio_device, NULL
};


#define WDR_TIMEOUT 5000 /* default urb timeout */
#define WDR_SHORT_TIMEOUT 1000	/* shorter urb timeout */

/*
 * ***************************************************************************
 * Utility functions
 * ***************************************************************************
 */

#define set_mctrl(port, set)		update_mctrl((port), (set), 0)
#define clear_mctrl(port, clear)	update_mctrl((port), 0, (clear))

static int update_mctrl(struct usb_serial_port *port, unsigned int set,
							unsigned int clear)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	struct device *dev = &port->dev;
	unsigned urb_value;
	int rv;

	if (((set | clear) & (TIOCM_DTR | TIOCM_RTS)) == 0) {
		dev_dbg(dev, "%s - DTR|RTS not being set|cleared\n", __func__);
		return 0;	/* no change */
	}

	clear &= ~set;	/* 'set' takes precedence over 'clear' */
	urb_value = 0;
	if (clear & TIOCM_DTR)
		urb_value |= FTDI_SIO_SET_DTR_LOW;
	if (clear & TIOCM_RTS)
		urb_value |= FTDI_SIO_SET_RTS_LOW;
	if (set & TIOCM_DTR)
		urb_value |= FTDI_SIO_SET_DTR_HIGH;
	if (set & TIOCM_RTS)
		urb_value |= FTDI_SIO_SET_RTS_HIGH;
	rv = usb_control_msg(port->serial->dev,
			       usb_sndctrlpipe(port->serial->dev, 0),
			       FTDI_SIO_SET_MODEM_CTRL_REQUEST,
			       FTDI_SIO_SET_MODEM_CTRL_REQUEST_TYPE,
			       urb_value, priv->interface,
			       NULL, 0, WDR_TIMEOUT);
	if (rv < 0) {
		dev_dbg(dev, "%s Error from MODEM_CTRL urb: DTR %s, RTS %s\n",
			__func__,
			(set & TIOCM_DTR) ? "HIGH" : (clear & TIOCM_DTR) ? "LOW" : "unchanged",
			(set & TIOCM_RTS) ? "HIGH" : (clear & TIOCM_RTS) ? "LOW" : "unchanged");
		rv = usb_translate_errors(rv);
	} else {
		dev_dbg(dev, "%s - DTR %s, RTS %s\n", __func__,
			(set & TIOCM_DTR) ? "HIGH" : (clear & TIOCM_DTR) ? "LOW" : "unchanged",
			(set & TIOCM_RTS) ? "HIGH" : (clear & TIOCM_RTS) ? "LOW" : "unchanged");
		/* FIXME: locking on last_dtr_rts */
		priv->last_dtr_rts = (priv->last_dtr_rts & ~clear) | set;
	}
	return rv;
}


static int change_speed(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	__u16 urb_value;
	__u16 urb_index;
	__u32 urb_index_value;
	int rv;

	urb_index_value = tty_get_baud_rate(tty);
	urb_value = (__u16)urb_index_value;
	urb_index = (__u16)(urb_index_value >> 16);
	urb_index = (__u16)((urb_index << 8) | priv->interface);

	rv = usb_control_msg(port->serial->dev,
			    usb_sndctrlpipe(port->serial->dev, 0),
			    FTDI_SIO_SET_BAUDRATE_REQUEST,
			    FTDI_SIO_SET_BAUDRATE_REQUEST_TYPE,
			    urb_value, urb_index,
			    NULL, 0, WDR_SHORT_TIMEOUT);
	return rv;
}

static int write_latency_timer(struct usb_serial_port *port)
{
	return 0;
}

static int read_latency_timer(struct usb_serial_port *port)
{
	return 16;
}

static int get_serial_info(struct usb_serial_port *port,
				struct serial_struct __user *retinfo)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	struct serial_struct tmp;

	if (!retinfo)
		return -EFAULT;
	memset(&tmp, 0, sizeof(tmp));
	tmp.flags = priv->flags;
	tmp.baud_base = priv->baud_base;
	tmp.custom_divisor = priv->custom_divisor;
	if (copy_to_user(retinfo, &tmp, sizeof(*retinfo)))
		return -EFAULT;
	return 0;
}

static int set_serial_info(struct tty_struct *tty,
	struct usb_serial_port *port, struct serial_struct __user *newinfo)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	struct serial_struct new_serial;
	struct ftdi_private old_priv;

	if (copy_from_user(&new_serial, newinfo, sizeof(new_serial)))
		return -EFAULT;

	mutex_lock(&priv->cfg_lock);
	old_priv = *priv;

	/* Do error checking and permission checking */

	if (!capable(CAP_SYS_ADMIN)) {
		if (((new_serial.flags & ~ASYNC_USR_MASK) !=
		     (priv->flags & ~ASYNC_USR_MASK))) {
			mutex_unlock(&priv->cfg_lock);
			return -EPERM;
		}
		priv->flags = ((priv->flags & ~ASYNC_USR_MASK) |
			       (new_serial.flags & ASYNC_USR_MASK));
		priv->custom_divisor = new_serial.custom_divisor;
		goto check_and_exit;
	}

	if (new_serial.baud_base != priv->baud_base) {
		mutex_unlock(&priv->cfg_lock);
		return -EINVAL;
	}

	/* Make the changes - these are privileged changes! */

	priv->flags = ((priv->flags & ~ASYNC_FLAGS) |
					(new_serial.flags & ASYNC_FLAGS));
	priv->custom_divisor = new_serial.custom_divisor;

	write_latency_timer(port);

check_and_exit:
	if ((old_priv.flags & ASYNC_SPD_MASK) !=
	     (priv->flags & ASYNC_SPD_MASK)) {
		if ((priv->flags & ASYNC_SPD_MASK) == ASYNC_SPD_HI)
			tty->alt_speed = 57600;
		else if ((priv->flags & ASYNC_SPD_MASK) == ASYNC_SPD_VHI)
			tty->alt_speed = 115200;
		else if ((priv->flags & ASYNC_SPD_MASK) == ASYNC_SPD_SHI)
			tty->alt_speed = 230400;
		else if ((priv->flags & ASYNC_SPD_MASK) == ASYNC_SPD_WARP)
			tty->alt_speed = 460800;
		else
			tty->alt_speed = 0;
	}
	if (((old_priv.flags & ASYNC_SPD_MASK) !=
	     (priv->flags & ASYNC_SPD_MASK)) ||
	    (((priv->flags & ASYNC_SPD_MASK) == ASYNC_SPD_CUST) &&
	     (old_priv.custom_divisor != priv->custom_divisor))) {
		change_speed(tty, port);
		mutex_unlock(&priv->cfg_lock);
	}
	else
		mutex_unlock(&priv->cfg_lock);
	return 0;
}

static int get_lsr_info(struct usb_serial_port *port,
			struct serial_struct __user *retinfo)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	unsigned int result = 0;

	if (!retinfo)
		return -EFAULT;

	if (priv->transmit_empty)
		result = TIOCSER_TEMT;

	if (copy_to_user(retinfo, &result, sizeof(unsigned int)))
		return -EFAULT;
	return 0;
}


/* Determine type of FTDI chip based on USB config and descriptor. */
static void ftdi_determine_type(struct usb_serial_port *port)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	struct usb_serial *serial = port->serial;
	struct usb_device *udev = serial->dev;
	unsigned version;
	unsigned interfaces;

	/* Assume it is not the original SIO device for now. */
	priv->baud_base = 48000000 / 2;

	version = le16_to_cpu(udev->descriptor.bcdDevice);
	interfaces = udev->actconfig->desc.bNumInterfaces;
	dev_dbg(&port->dev, "%s: bcdDevice = 0x%x, bNumInterfaces = %u\n", __func__,
		version, interfaces);
	if (interfaces > 1) {
		int inter;

		priv->chip_type = FT2232C;

		/* Determine interface code. */
		inter = serial->interface->altsetting->desc.bInterfaceNumber;
		if (inter == 1) {
			priv->interface = INTERFACE_A;
		} else  if (inter == 2) {
			priv->interface = INTERFACE_B;
		}
	}
	dev_info(&udev->dev, "Detected %s\n", ftdi_chip_name[priv->chip_type]);
}


/* Determine the maximum packet size for the device.  This depends on the chip
 * type and the USB host capabilities.  The value should be obtained from the
 * device descriptor as the chip will use the appropriate values for the host.*/
static void ftdi_set_max_packet_size(struct usb_serial_port *port)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	struct usb_serial *serial = port->serial;
	struct usb_device *udev = serial->dev;

	struct usb_interface *interface = serial->interface;
	struct usb_endpoint_descriptor *ep_desc = &interface->cur_altsetting->endpoint[1].desc;

	unsigned num_endpoints;
	int i;

	num_endpoints = interface->cur_altsetting->desc.bNumEndpoints;
	dev_info(&udev->dev, "Number of endpoints %d\n", num_endpoints);

	/* NOTE: some customers have programmed FT232R/FT245R devices
	 * with an endpoint size of 0 - not good.  In this case, we
	 * want to override the endpoint descriptor setting and use a
	 * value of 64 for wMaxPacketSize */
	for (i = 0; i < num_endpoints; i++) {
		dev_info(&udev->dev, "Endpoint %d MaxPacketSize %d\n", i+1,
			interface->cur_altsetting->endpoint[i].desc.wMaxPacketSize);
		ep_desc = &interface->cur_altsetting->endpoint[i].desc;
		if (ep_desc->wMaxPacketSize == 0) {
			ep_desc->wMaxPacketSize = cpu_to_le16(0x40);
			dev_info(&udev->dev, "Overriding wMaxPacketSize on endpoint %d\n", i);
		}
	}

	/* set max packet size based on descriptor */
	priv->max_packet_size = usb_endpoint_maxp(ep_desc);

	dev_info(&udev->dev, "Setting MaxPacketSize %d\n", priv->max_packet_size);
}


/*
 * ***************************************************************************
 * Sysfs Attribute
 * ***************************************************************************
 */

static ssize_t latency_timer_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	if (priv->flags & ASYNC_LOW_LATENCY)
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "%i\n", priv->latency);
}

/* Write a new value of the latency timer, in units of milliseconds. */
static ssize_t latency_timer_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *valbuf, size_t count)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	int v = simple_strtoul(valbuf, NULL, 10);
	int rv;

	priv->latency = v;
	rv = write_latency_timer(port);
	if (rv < 0)
		return -EIO;
	return count;
}
static DEVICE_ATTR_RW(latency_timer);

/* Write an event character directly to the FTDI register.  The ASCII
   value is in the low 8 bits, with the enable bit in the 9th bit. */
static ssize_t store_event_char(struct device *dev,
	struct device_attribute *attr, const char *valbuf, size_t count)
{
	struct usb_serial_port *port = to_usb_serial_port(dev);
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	struct usb_device *udev = port->serial->dev;
	int v = simple_strtoul(valbuf, NULL, 10);
	int rv;

	dev_dbg(&port->dev, "%s: setting event char = %i\n", __func__, v);

	rv = usb_control_msg(udev,
			     usb_sndctrlpipe(udev, 0),
			     FTDI_SIO_SET_EVENT_CHAR_REQUEST,
			     FTDI_SIO_SET_EVENT_CHAR_REQUEST_TYPE,
			     v, priv->interface,
			     NULL, 0, WDR_TIMEOUT);
	if (rv < 0) {
		dev_dbg(&port->dev, "Unable to write event character: %i\n", rv);
		return -EIO;
	}

	return count;
}
static DEVICE_ATTR(event_char, S_IWUSR, NULL, store_event_char);

static int create_sysfs_attrs(struct usb_serial_port *port)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	int retval = 0;

	/* XXX I've no idea if the original SIO supports the event_char
	 * sysfs parameter, so I'm playing it safe.  */
	dev_dbg(&port->dev, "sysfs attributes for %s\n", ftdi_chip_name[priv->chip_type]);
	retval = device_create_file(&port->dev, &dev_attr_event_char);
	if ((!retval) &&
		(priv->chip_type == FT2232C)) {
		retval = device_create_file(&port->dev,
						&dev_attr_latency_timer);
	}

	return retval;
}

static void remove_sysfs_attrs(struct usb_serial_port *port)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);

	/* XXX see create_sysfs_attrs */

	device_remove_file(&port->dev, &dev_attr_event_char);
	if (priv->chip_type == FT2232C) {
		device_remove_file(&port->dev, &dev_attr_latency_timer);
	}

}

/*
 * ***************************************************************************
 * FTDI driver specific functions
 * ***************************************************************************
 */

/* Probe function to check for special devices */
static int ftdi_sio_probe(struct usb_serial *serial,
					const struct usb_device_id *id)
{
	struct ftdi_sio_quirk *quirk =
				(struct ftdi_sio_quirk *)id->driver_info;

	if (quirk && quirk->probe) {
		int ret = quirk->probe(serial);
		if (ret != 0)
			return ret;
	}

	usb_set_serial_data(serial, (void *)id->driver_info);

	return 0;
}

static int ftdi_sio_port_probe(struct usb_serial_port *port)
{
	struct ftdi_private *priv;
	struct ftdi_sio_quirk *quirk = usb_get_serial_data(port->serial);


	priv = kzalloc(sizeof(struct ftdi_private), GFP_KERNEL);
	if (!priv) {
		dev_err(&port->dev, "%s- kmalloc(%Zd) failed.\n", __func__,
					sizeof(struct ftdi_private));
		return -ENOMEM;
	}

	mutex_init(&priv->cfg_lock);

	priv->flags = ASYNC_LOW_LATENCY;

	if (quirk && quirk->port_probe)
		quirk->port_probe(priv);

	usb_set_serial_port_data(port, priv);

	ftdi_determine_type(port);
	ftdi_set_max_packet_size(port);
	if (read_latency_timer(port) < 0)
		priv->latency = 16;
	write_latency_timer(port);
	create_sysfs_attrs(port);
	return 0;
}

/*
 * Module parameter to control latency timer for NDI FTDI-based USB devices.
 * If this value is not set in /etc/modprobe.d/ its value will be set
 * to 1ms.
 */
static int ndi_latency_timer = 1;

static int ftdi_KP_orion_probe(struct usb_serial *serial)
{
	struct usb_device *udev = serial->dev;
	struct usb_interface *interface = serial->interface;

	if (interface == udev->actconfig->interface[1]||interface == udev->actconfig->interface[2]) {
		dev_info(&udev->dev,
			 "Ignoring other usb ports\n");
		return 0;
	}

	return -ENODEV;
}

static int ftdi_sio_port_remove(struct usb_serial_port *port)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);

	remove_sysfs_attrs(port);

	kfree(priv);

	return 0;
}

static int ftdi_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct usb_device *dev = port->serial->dev;
	struct ftdi_private *priv = usb_get_serial_port_data(port);

	/* No error checking for this (will get errors later anyway) */
	/* See ftdi_sio.h for description of what is reset */
	usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
			FTDI_SIO_RESET_REQUEST, FTDI_SIO_RESET_REQUEST_TYPE,
			FTDI_SIO_RESET_SIO,
			priv->interface, NULL, 0, WDR_TIMEOUT);

	/* Termios defaults are set by usb_serial_init. We don't change
	   port->tty->termios - this would lose speed settings, etc.
	   This is same behaviour as serial.c/rs_open() - Kuba */

	/* ftdi_set_termios  will send usb control messages */
	if (tty)
		ftdi_set_termios(tty, port, NULL);

	return usb_serial_generic_open(tty, port);
}

static void ftdi_dtr_rts(struct usb_serial_port *port, int on)
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);

	/* Disable flow control */
	if (!on) {
		if (usb_control_msg(port->serial->dev,
			    usb_sndctrlpipe(port->serial->dev, 0),
			    FTDI_SIO_SET_FLOW_CTRL_REQUEST,
			    FTDI_SIO_SET_FLOW_CTRL_REQUEST_TYPE,
			    0, priv->interface, NULL, 0,
			    WDR_TIMEOUT) < 0) {
			dev_err(&port->dev, "error from flowcontrol urb\n");
		}
	}
	/* drop RTS and DTR */
	if (on)
		set_mctrl(port, TIOCM_DTR | TIOCM_RTS);
	else
		clear_mctrl(port, TIOCM_DTR | TIOCM_RTS);
}

/* The SIO requires the first byte to have:
 *  B0 1
 *  B1 0
 *  B2..7 length of message excluding byte 0
 *
 * The new devices do not require this byte
 */
static int ftdi_prepare_write_buffer(struct usb_serial_port *port,
						void *dest, size_t size)
{
	struct ftdi_private *priv;
	int count;

	priv = usb_get_serial_port_data(port);

	count = kfifo_out_locked(&port->write_fifo, dest, size,
						&port->lock);
	port->icount.tx += count;

	return count;
}

#define FTDI_RS_ERR_MASK (FTDI_RS_BI | FTDI_RS_PE | FTDI_RS_FE | FTDI_RS_OE)

static int ftdi_process_packet(struct usb_serial_port *port,
		struct ftdi_private *priv, char *packet, int len)
{
	int i;
	char status;
	char flag;
	char *ch;

	if (len < 2) {
		dev_dbg(&port->dev, "malformed packet\n");
		return 0;
	}

	/* Compare new line status to the old one, signal if different/
	   N.B. packet may be processed more than once, but differences
	   are only processed once.  */
	status = packet[0] & FTDI_STATUS_B0_MASK;
	if (status != priv->prev_status) {
		char diff_status = status ^ priv->prev_status;

		if (diff_status & FTDI_RS0_CTS)
			port->icount.cts++;
		if (diff_status & FTDI_RS0_DSR)
			port->icount.dsr++;
		if (diff_status & FTDI_RS0_RI)
			port->icount.rng++;
		if (diff_status & FTDI_RS0_RLSD) {
			struct tty_struct *tty;

			port->icount.dcd++;
			tty = tty_port_tty_get(&port->port);
			if (tty)
				usb_serial_handle_dcd_change(port, tty,
						status & FTDI_RS0_RLSD);
			tty_kref_put(tty);
		}

		wake_up_interruptible(&port->port.delta_msr_wait);
		priv->prev_status = status;
	}

	flag = TTY_NORMAL;
	if (packet[1] & FTDI_RS_ERR_MASK) {
		/* Break takes precedence over parity, which takes precedence
		 * over framing errors */
		if (packet[1] & FTDI_RS_BI) {
			flag = TTY_BREAK;
			port->icount.brk++;
			usb_serial_handle_break(port);
		} else if (packet[1] & FTDI_RS_PE) {
			flag = TTY_PARITY;
			port->icount.parity++;
		} else if (packet[1] & FTDI_RS_FE) {
			flag = TTY_FRAME;
			port->icount.frame++;
		}
		/* Overrun is special, not associated with a char */
		if (packet[1] & FTDI_RS_OE) {
			port->icount.overrun++;
			tty_insert_flip_char(&port->port, 0, TTY_OVERRUN);
		}
	}

	/* save if the transmitter is empty or not */
	if (packet[1] & FTDI_RS_TEMT)
		priv->transmit_empty = 1;
	else
		priv->transmit_empty = 0;

	len -= 2;
	if (!len)
		return 0;	/* status only */
	port->icount.rx += len;
	ch = packet + 2;

	if (port->port.console && port->sysrq) {
		for (i = 0; i < len; i++, ch++) {
			if (!usb_serial_handle_sysrq_char(port, *ch))
				tty_insert_flip_char(&port->port, *ch, flag);
		}
	} else {
		tty_insert_flip_string_fixed_flag(&port->port, ch, flag, len);
	}

	return len;
}

static void ftdi_process_read_urb(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	char *data = (char *)urb->transfer_buffer;
	int i;
	int len;
	int count = 0;

	for (i = 0; i < urb->actual_length; i += priv->max_packet_size) {
		len = min_t(int, urb->actual_length - i, priv->max_packet_size);
		count += ftdi_process_packet(port, priv, &data[i], len);
	}

	if (count)
		tty_flip_buffer_push(&port->port);
}

static void ftdi_break_ctl(struct tty_struct *tty, int break_state)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	__u16 urb_value;

	/* break_state = -1 to turn on break, and 0 to turn off break */
	/* see drivers/char/tty_io.c to see it used */
	/* last_set_data_urb_value NEVER has the break bit set in it */

	if (break_state)
		urb_value = priv->last_set_data_urb_value | FTDI_SIO_SET_BREAK;
	else
		urb_value = priv->last_set_data_urb_value;

	if (usb_control_msg(port->serial->dev,
			usb_sndctrlpipe(port->serial->dev, 0),
			FTDI_SIO_SET_DATA_REQUEST,
			FTDI_SIO_SET_DATA_REQUEST_TYPE,
			urb_value , priv->interface,
			NULL, 0, WDR_TIMEOUT) < 0) {
		dev_err(&port->dev, "%s FAILED to enable/disable break state (state was %d)\n",
			__func__, break_state);
	}

	dev_dbg(&port->dev, "%s break state is %d - urb is %d\n", __func__,
		break_state, urb_value);

}

static bool ftdi_tx_empty(struct usb_serial_port *port)
{
	unsigned char buf[2];
	int ret;

	ret = ftdi_get_modem_status(port, buf);
	if (ret == 2) {
		if (!(buf[1] & FTDI_RS_TEMT))
			return false;
	}

	return true;
}

/* old_termios contains the original termios settings and tty->termios contains
 * the new setting to be used
 * WARNING: set_termios calls this with old_termios in kernel space
 */
static void ftdi_set_termios(struct tty_struct *tty,
		struct usb_serial_port *port, struct ktermios *old_termios)
{
	struct usb_device *dev = port->serial->dev;
	struct device *ddev = &port->dev;
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	struct ktermios *termios = &tty->termios;
	unsigned int cflag = termios->c_cflag;
	__u16 urb_value; /* will hold the new flags */

	/* Added for xon/xoff support */
	unsigned int iflag = termios->c_iflag;
	unsigned char vstop;
	unsigned char vstart;

	/* Force baud rate if this device requires it, unless it is set to
	   B0. */
	if (priv->force_baud && ((termios->c_cflag & CBAUD) != B0)) {
		dev_dbg(ddev, "%s: forcing baud rate for this device\n", __func__);
		tty_encode_baud_rate(tty, priv->force_baud,
					priv->force_baud);
	}

	/* Force RTS-CTS if this device requires it. */
	if (priv->force_rtscts) {
		dev_dbg(ddev, "%s: forcing rtscts for this device\n", __func__);
		termios->c_cflag |= CRTSCTS;
	}

	/*
	 * All FTDI UART chips are limited to CS7/8. We shouldn't pretend to
	 * support CS5/6 and revert the CSIZE setting instead.
	 *
	 * CS5 however is used to control some smartcard readers which abuse
	 * this limitation to switch modes. Original FTDI chips fall back to
	 * eight data bits.
	 *
	 * TODO: Implement a quirk to only allow this with mentioned
	 *       readers. One I know of (Argolis Smartreader V1)
	 *       returns "USB smartcard server" as iInterface string.
	 *       The vendor didn't bother with a custom VID/PID of
	 *       course.
	 */
	if (C_CSIZE(tty) == CS6) {
		dev_warn(ddev, "requested CSIZE setting not supported\n");

		termios->c_cflag &= ~CSIZE;
		if (old_termios)
			termios->c_cflag |= old_termios->c_cflag & CSIZE;
		else
			termios->c_cflag |= CS8;
	}

	cflag = termios->c_cflag;

	if (!old_termios)
		goto no_skip;

	if (old_termios->c_cflag == termios->c_cflag
	    && old_termios->c_ispeed == termios->c_ispeed
	    && old_termios->c_ospeed == termios->c_ospeed)
		goto no_c_cflag_changes;

	/* NOTE These routines can get interrupted by
	   ftdi_sio_read_bulk_callback  - need to examine what this means -
	   don't see any problems yet */

	if ((old_termios->c_cflag & (CSIZE|PARODD|PARENB|CMSPAR|CSTOPB)) ==
	    (termios->c_cflag & (CSIZE|PARODD|PARENB|CMSPAR|CSTOPB)))
		goto no_data_parity_stop_changes;

no_skip:
	/* Set number of data bits, parity, stop bits */

	urb_value = 0;
	urb_value |= (cflag & CSTOPB ? FTDI_SIO_SET_DATA_STOP_BITS_2 :
		      FTDI_SIO_SET_DATA_STOP_BITS_1);
	if (cflag & PARENB) {
		if (cflag & CMSPAR)
			urb_value |= cflag & PARODD ?
				     FTDI_SIO_SET_DATA_PARITY_MARK :
				     FTDI_SIO_SET_DATA_PARITY_SPACE;
		else
			urb_value |= cflag & PARODD ?
				     FTDI_SIO_SET_DATA_PARITY_ODD :
				     FTDI_SIO_SET_DATA_PARITY_EVEN;
	} else {
		urb_value |= FTDI_SIO_SET_DATA_PARITY_NONE;
	}
	switch (cflag & CSIZE) {
	case CS5:
		dev_dbg(ddev, "Setting CS5 quirk\n");
		break;
	case CS7:
		urb_value |= 7;
		dev_dbg(ddev, "Setting CS7\n");
		break;
	default:
	case CS8:
		urb_value |= 8;
		dev_dbg(ddev, "Setting CS8\n");
		break;
	}

	/* This is needed by the break command since it uses the same command
	   - but is or'ed with this value  */
	priv->last_set_data_urb_value = urb_value;

	if (usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
			    FTDI_SIO_SET_DATA_REQUEST,
			    FTDI_SIO_SET_DATA_REQUEST_TYPE,
			    urb_value , priv->interface,
			    NULL, 0, WDR_SHORT_TIMEOUT) < 0) {
		dev_err(ddev, "%s FAILED to set databits/stopbits/parity\n",
			__func__);
	}

	/* Now do the baudrate */
no_data_parity_stop_changes:
	if ((cflag & CBAUD) == B0) {
		/* Disable flow control */
		/* Drop RTS and DTR */
		clear_mctrl(port, TIOCM_DTR | TIOCM_RTS);
	} else {
		/* set the baudrate determined before */
		mutex_lock(&priv->cfg_lock);
		if (change_speed(tty, port))
			dev_err(ddev, "%s urb failed to set baudrate\n", __func__);
		mutex_unlock(&priv->cfg_lock);
		/* Ensure RTS and DTR are raised when baudrate changed from 0 */
		if (old_termios && (old_termios->c_cflag & CBAUD) == B0)
			set_mctrl(port, TIOCM_DTR | TIOCM_RTS);
	}

	/* Set flow control */
	/* Note device also supports DTR/CD (ugh) and Xon/Xoff in hardware */
no_c_cflag_changes:
	if (cflag & CRTSCTS) {
		dev_dbg(ddev, "%s Setting to CRTSCTS flow control\n", __func__);
		if (usb_control_msg(dev,
				    usb_sndctrlpipe(dev, 0),
				    FTDI_SIO_SET_FLOW_CTRL_REQUEST,
				    FTDI_SIO_SET_FLOW_CTRL_REQUEST_TYPE,
				    0 , (FTDI_SIO_RTS_CTS_HS | priv->interface),
				    NULL, 0, WDR_TIMEOUT) < 0) {
			dev_err(ddev, "urb failed to set to rts/cts flow control\n");
		}
	} else {
		/*
		 * Xon/Xoff code
		 *
		 * Check the IXOFF status in the iflag component of the
		 * termios structure. If IXOFF is not set, the pre-xon/xoff
		 * code is executed.
		 */
		if (iflag & IXOFF) {
			dev_dbg(ddev, "%s  request to enable xonxoff iflag=%04x\n",
				__func__, iflag);
			/* Try to enable the XON/XOFF on the ftdi_sio
			 * Set the vstart and vstop -- could have been done up
			 * above where a lot of other dereferencing is done but
			 * that would be very inefficient as vstart and vstop
			 * are not always needed.
			 */
			vstart = termios->c_cc[VSTART];
			vstop = termios->c_cc[VSTOP];
			urb_value = (vstop << 8) | (vstart);

		} else {
			/* else clause to only run if cflag ! CRTSCTS and iflag
			 * ! XOFF. CHECKME Assuming XON/XOFF handled by tty
			 * stack - not by device */
			dev_dbg(ddev, "%s Turning off hardware flow control\n", __func__);
		}
	}
}

/*
 * Get modem-control status.
 *
 * Returns the number of status bytes retrieved (device dependant), or
 * negative error code.
 */
static int ftdi_get_modem_status(struct usb_serial_port *port,
						unsigned char status[2])
{
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	unsigned char *buf;
	int len;
	int ret;

	buf = kmalloc(2, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	/*
	 * The 8U232AM returns a two byte value (the SIO a 1 byte value) in
	 * the same format as the data returned from the in point.
	 */
	switch (priv->chip_type) {
		len = 2;
		break;
	default:
		ret = -EFAULT;
		goto out;
	}

	ret = usb_control_msg(port->serial->dev,
			usb_rcvctrlpipe(port->serial->dev, 0),
			FTDI_SIO_GET_MODEM_STATUS_REQUEST,
			FTDI_SIO_GET_MODEM_STATUS_REQUEST_TYPE,
			0, priv->interface,
			buf, len, WDR_TIMEOUT);
	if (ret < 0) {
		dev_err(&port->dev, "failed to get modem status: %d\n", ret);
		ret = usb_translate_errors(ret);
		goto out;
	}

	status[0] = buf[0];
	if (ret > 1)
		status[1] = buf[1];
	else
		status[1] = 0;

	dev_dbg(&port->dev, "%s - 0x%02x%02x\n", __func__, status[0],
								status[1]);
out:
	kfree(buf);

	return ret;
}

static int ftdi_tiocmget(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ftdi_private *priv = usb_get_serial_port_data(port);
	unsigned char buf[2]={0,0};
	int ret;

	ret = ftdi_get_modem_status(port, buf);
	if (ret < 0)
		return ret;

	ret =	(buf[0] & FTDI_SIO_DSR_MASK  ? TIOCM_DSR : 0) |
		(buf[0] & FTDI_SIO_CTS_MASK  ? TIOCM_CTS : 0) |
		(buf[0] & FTDI_SIO_RI_MASK   ? TIOCM_RI  : 0) |
		(buf[0] & FTDI_SIO_RLSD_MASK ? TIOCM_CD  : 0) |
		priv->last_dtr_rts;

	return ret;
}

static int ftdi_tiocmset(struct tty_struct *tty,
			unsigned int set, unsigned int clear)
{
	struct usb_serial_port *port = tty->driver_data;

	return update_mctrl(port, set, clear);
}

static int ftdi_ioctl(struct tty_struct *tty,
					unsigned int cmd, unsigned long arg)
{
	struct usb_serial_port *port = tty->driver_data;

	dev_dbg(&port->dev, "%s cmd 0x%04x\n", __func__, cmd);

	/* Based on code from acm.c and others */
	switch (cmd) {

	case TIOCGSERIAL: /* gets serial port data */
		return get_serial_info(port,
					(struct serial_struct __user *) arg);

	case TIOCSSERIAL: /* sets serial port data */
		return set_serial_info(tty, port,
					(struct serial_struct __user *) arg);
	case TIOCSERGETLSR:
		return get_lsr_info(port, (struct serial_struct __user *)arg);
		break;
	default:
		break;
	}
	/* This is not necessarily an error - turns out the higher layers
	 * will do some ioctls themselves (see comment above)
	 */
	dev_dbg(&port->dev, "%s arg not supported - it was 0x%04x - check /usr/include/asm/ioctls.h\n",
		__func__, cmd);
	return -ENOIOCTLCMD;
}

module_usb_serial_driver(serial_drivers, id_table_combined);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

module_param(ndi_latency_timer, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(ndi_latency_timer, "NDI device latency timer override");
