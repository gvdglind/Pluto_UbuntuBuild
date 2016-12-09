/*
 * Notes:
 */

#include <linux/module.h>
#include <linux/init.h>

#include <linux/slab.h>                /* kmalloc() */
#include <linux/usb.h>                 /* USB stuff */
#include <linux/mutex.h>               /* mutexes */
#include <linux/ioctl.h>

#include <asm/uaccess.h>               /* copy_*_user */


#define DEBUG_LEVEL_DEBUG              0x1F
#define DEBUG_LEVEL_INFO               0x0F
#define DEBUG_LEVEL_WARN               0x07
#define DEBUG_LEVEL_ERROR              0x03
#define DEBUG_LEVEL_CRITICAL	         0x01

#define DBG_DEBUG(fmt, args...) \
if ((debug_level & DEBUG_LEVEL_DEBUG) == DEBUG_LEVEL_DEBUG) \
   printk( KERN_DEBUG "[debug] %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ## args)

#define DBG_INFO(fmt, args...) \
if ((debug_level & DEBUG_LEVEL_INFO) == DEBUG_LEVEL_INFO) \
   printk( KERN_DEBUG "[info]  %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ## args)

#define DBG_WARN(fmt, args...) \
if ((debug_level & DEBUG_LEVEL_WARN) == DEBUG_LEVEL_WARN) \
   printk( KERN_DEBUG "[warn]  %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ## args)

#define DBG_ERR(fmt, args...) \
if ((debug_level & DEBUG_LEVEL_ERROR) == DEBUG_LEVEL_ERROR) \
   printk( KERN_DEBUG "[err]   %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ## args)

#define DBG_CRIT(fmt, args...) \
if ((debug_level & DEBUG_LEVEL_CRITICAL) == DEBUG_LEVEL_CRITICAL) \
   printk( KERN_DEBUG "[crit]  %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ## args)


#define ORION_VENDOR_ID                0x2bcd
#define ORION_PRODUCT_ID               0x0200

#ifdef CONFIG_USB_DYNAMIC_MINORS
   #define ORION_CORE_MINOR_BASE       0
#else
   #define ORION_CORE_MINOR_BASE       96
#endif



struct usb_orion_core {
   struct usb_device              *udev;
   struct usb_interface           *interface;
   unsigned char                  minor;
   char                           serial_number[8];

   int                            open_count;    /* Open count for this port */
   struct                         semaphore sem; /* Locks this structure */
   spinlock_t                     cmd_spinlock; 
   
   char                           *int_in_buffer;
   struct usb_endpoint_descriptor *int_in_endpoint;
   struct urb                     *int_in_urb;
   int                            int_in_running;
   char                           *buffered_in_msg; /* for copy to userland ...*/
   char                           *buffered_in_ptr;  
   int                            buffered_in_len;  

   char                           *int_out_buffer;
   struct usb_endpoint_descriptor *int_out_endpoint;
   struct urb                     *int_out_urb;
   int                            int_out_running;
   int                            txbusy;
};



static struct usb_device_id orion_core_table [] = {
   { USB_DEVICE(ORION_VENDOR_ID, ORION_PRODUCT_ID) },
   { }
};



MODULE_DEVICE_TABLE (usb, orion_core_table);

static int debug_level = DEBUG_LEVEL_INFO;
static int debug_trace = 0;
module_param(debug_level, int, S_IRUGO | S_IWUSR);
module_param(debug_trace, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug_level, "debug level (bitmask)");
MODULE_PARM_DESC(debug_trace, "enable function tracing");

/* Prevent races between open() and disconnect */
static DEFINE_MUTEX(disconnect_mutex);
static struct usb_driver orion_core_driver;


static inline void orion_core_debug_data(const char *function, int size, const unsigned char *data)
{
   int i;

   if ((debug_level & DEBUG_LEVEL_DEBUG) == DEBUG_LEVEL_DEBUG) {
      printk(KERN_DEBUG "[debug] %s: length = %d, data = ", function, size);
      for (i = 0; i < size; ++i)
         printk("%.2x ", data[i]);
      printk("\n");
   }
}



static void orion_core_abort_transfers(struct usb_orion_core *dev)
{
   if (!dev) {
      DBG_ERR("dev is NULL");
      return;
   }

   if (!dev->udev) {
      DBG_ERR("udev is NULL");
      return;
   }

   if (dev->udev->state == USB_STATE_NOTATTACHED) {
      DBG_ERR("udev not attached");
      return;
   }

   /* Shutdown transfer */
   if (dev->int_in_running) {
      dev->int_in_running = 0;
      mb();
      if (dev->int_in_urb)
         usb_kill_urb(dev->int_in_urb);
   }

   if (dev->int_out_running) {
      dev->int_out_running = 0;
      mb();
      if (dev->int_out_urb)
         usb_kill_urb(dev->int_out_urb);
   }
}



static inline void orion_core_delete(struct usb_orion_core *dev)
{
   orion_core_abort_transfers(dev);

   /* Free data structures. */

   if (dev->int_in_urb)
      usb_free_urb(dev->int_in_urb);

   if (dev->int_out_urb)
      usb_free_urb(dev->int_out_urb);

   kfree(dev->int_in_buffer);
   kfree(dev->buffered_in_msg);
   kfree(dev->int_out_buffer);
   kfree(dev);
}



static void orion_core_int_out_callback(struct urb *urb)
{
   struct usb_orion_core *dev = urb->context;
   DBG_DEBUG("clr txbusy");

   spin_lock(&dev->cmd_spinlock);
   dev->txbusy = 0;
   spin_unlock(&dev->cmd_spinlock);
}



static void orion_core_int_in_callback(struct urb *urb)
{
   struct usb_orion_core *dev = urb->context;
   int retval;

   orion_core_debug_data(__FUNCTION__, urb->actual_length, urb->transfer_buffer);

   if (urb->status) {
      if ((urb->status == -ENOENT) || (urb->status == -ECONNRESET) || (urb->status == -ESHUTDOWN)) 
         goto exit;
      DBG_ERR("non-zero urb status (%d)", urb->status);
      goto exit;
   }

   if (urb->actual_length > le16_to_cpu(dev->int_in_endpoint->wMaxPacketSize)) {
      DBG_ERR("ignore too big rcvd packet (%d)", urb->actual_length);
      goto exit;
   }

   spin_lock(&dev->cmd_spinlock);
   
   if (urb->actual_length > 0) {
      memcpy(dev->buffered_in_msg, urb->transfer_buffer, urb->actual_length);
      dev->buffered_in_ptr = dev->buffered_in_msg;  
      dev->buffered_in_len = urb->actual_length;
   }

   spin_unlock(&dev->cmd_spinlock);

exit: 
   if (dev->int_in_running) { // retrigger receiver
      retval = usb_submit_urb(dev->int_in_urb, GFP_KERNEL);
      if (retval) {
         DBG_ERR("submitting int 'in' urb failed (%d)", retval);
      }
   }
}



static int orion_core_open(struct inode *inode, struct file *file)
{
   struct usb_orion_core *dev = NULL;
   struct usb_interface *interface;
   int subminor;
   int retval = 0;

   DBG_INFO("open device");
   subminor = iminor(inode);

   mutex_lock(&disconnect_mutex);

   interface = usb_find_interface(&orion_core_driver, subminor);
   if (!interface) {
      DBG_ERR("can't find device for minor %d", subminor);
      retval = -ENODEV;
      goto exit;
   }

   dev = usb_get_intfdata(interface);
   if (!dev) {
      retval = -ENODEV;
      goto exit;
   }

   /* lock this device */
   if (down_interruptible (&dev->sem)) {
      DBG_ERR("sem down failed");
      retval = -ERESTARTSYS;
      goto exit;
   }

   dev->buffered_in_len = 0;

   /* Increment our usage count for the device. */
   ++dev->open_count;
   if (dev->open_count > 1)
      DBG_DEBUG("open_count = %d", dev->open_count);

   /* Initialize 'in' interrupt URB and trigger
    */
   usb_fill_int_urb(dev->int_in_urb, 
                    dev->udev, 
                    usb_rcvintpipe(dev->udev, dev->int_in_endpoint->bEndpointAddress),
                    dev->int_in_buffer, 
                    le16_to_cpu(dev->int_in_endpoint->wMaxPacketSize), 
                    orion_core_int_in_callback,
                    dev,
                    dev->int_in_endpoint->bInterval);
   dev->int_in_running = 1;
   
   mb();
   retval = usb_submit_urb(dev->int_in_urb, GFP_KERNEL);
   if (retval) {
      DBG_ERR("submitting int 'in' urb failed (%d)", retval);
      --dev->open_count;
      orion_core_abort_transfers(dev);
      goto unlock_exit;
   }

   /* Initialize 'out' interrupt URB
    */
   usb_fill_int_urb(dev->int_out_urb, 
                    dev->udev, 
                    usb_sndintpipe(dev->udev, dev->int_out_endpoint->bEndpointAddress),
                    dev->int_out_buffer, 
                    le16_to_cpu(dev->int_out_endpoint->wMaxPacketSize), 
                    orion_core_int_out_callback,
                    dev,
                    dev->int_out_endpoint->bInterval);
   dev->int_out_running = 1;

   /* Save our object in the file's private structure. */
   file->private_data = dev;

unlock_exit:
   up(&dev->sem);

exit:
   mutex_unlock(&disconnect_mutex);
   return retval;
}



static int orion_core_release(struct inode *inode, struct file *file)
{
   struct usb_orion_core *dev = NULL;
   int retval = 0;

   DBG_INFO("release driver");
   dev = file->private_data;

   if (!dev) {
      DBG_ERR("dev is NULL");
      retval =  -ENODEV;
      goto exit;
   }

   /* Lock our device */
   if (down_interruptible(&dev->sem)) {
      DBG_ERR("sem down failed");
      retval = -ERESTARTSYS;
      goto exit;
   }

   if (dev->open_count <= 0) {
      DBG_ERR("device not opened");
      retval = -ENODEV;
      goto unlock_exit;
   }

   if (!dev->udev) {
      DBG_DEBUG("device unplugged before the file was released");
      up(&dev->sem);	/* Unlock here as orion_core_delete frees dev. */
      orion_core_delete(dev);
      goto exit;
   }

   if (dev->open_count > 1)
      DBG_DEBUG("open_count = %d", dev->open_count);

   orion_core_abort_transfers(dev);
   --dev->open_count;

unlock_exit:
   up(&dev->sem);

exit:
   return retval;
}



static ssize_t orion_core_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
   struct usb_orion_core *dev;
   int bytes_read = 0, retval = 0;

   dev = file->private_data;

   /* Lock this object. */
   if (down_interruptible(&dev->sem)) {
      DBG_ERR("sem down failed");
      retval = -ERESTARTSYS;
      goto exit;
   }

   /* Verify that the device wasn't unplugged. */
   if (!dev->udev) {
      retval = -ENODEV;
      DBG_ERR("No device or device unplugged (%d)", retval);
      goto unlock_exit;
   }

   /* Some basic checks */
   if ((dev->buffered_in_len == 0) || (count == 0))
      goto unlock_exit;

   /* Actually put the data into the buffer */
   while (dev->buffered_in_len && count) {
      put_user(*dev->buffered_in_ptr++, user_buf++);
      dev->buffered_in_len--;
      count--;
      bytes_read++;
   }
   retval = bytes_read;

unlock_exit:
   up(&dev->sem);

exit:
   return retval;
}



static ssize_t orion_core_write(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
   struct usb_orion_core *dev;
   int retval = 0;

   DBG_DEBUG("sending command of %d bytes...", (int)count);

   dev = file->private_data;

   /* Lock this object. */
   if (down_interruptible(&dev->sem)) {
      DBG_ERR("sem down failed");
      retval = -ERESTARTSYS;
      goto exit;
   }

   /* Verify that the device wasn't unplugged. */
   if (!dev->udev) {
      retval = -ENODEV;
      DBG_ERR("No device or device unplugged (%d)", retval);
      goto unlock_exit;
   }

   /* Verify that we actually have some data to write and not too much */
   if ((count == 0) || (count > le16_to_cpu(dev->int_out_endpoint->wMaxPacketSize)))
      goto unlock_exit;

   /* Too fast writes..., try later */
   if (dev->txbusy) {
      retval = -EWOULDBLOCK;
      goto unlock_exit;
   }

   /* Copy from userspace to kernelspace */
   if (copy_from_user(dev->int_out_buffer, user_buf, count)) {
      retval = -EFAULT;
      goto unlock_exit;
   }

   orion_core_debug_data(__FUNCTION__, count, dev->int_out_buffer);
   
   /* and transmit... */
   ++dev->txbusy;
   retval = usb_submit_urb(dev->int_out_urb, GFP_KERNEL);
   if (retval) {
      DBG_ERR("submitting int 'out' urb failed (%d)", retval);
   }
   else 
      retval = count;

unlock_exit:
   up(&dev->sem);

exit:
   if (retval <= 0) {
      DBG_ERR("retval=%d", retval);
   }
   return retval;
}



static struct file_operations orion_core_fops = {
   .owner   = THIS_MODULE,
   .read    = orion_core_read,
   .write   = orion_core_write,
   .open    = orion_core_open,
   .release = orion_core_release,
};



static struct usb_class_driver orion_core_class = {
   .name = "orion_core%d", // we will replace this to add the unique serial number in
   .fops = &orion_core_fops,
   .minor_base = ORION_CORE_MINOR_BASE,
};



static int orion_core_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
   static char devicename[32];
   struct usb_device *udev;
   struct usb_orion_core *dev = NULL;
   struct usb_host_interface *iface_desc;
   struct usb_endpoint_descriptor *endpoint;
   int i, int_end_size;
   int retval = -ENODEV;
   

   /* Skip bulk transfertypes (ttyUSB0 and ttyUSB1) */
   for (i = 0; i < interface->cur_altsetting->desc.bNumEndpoints; ++i) {
      endpoint = &interface->cur_altsetting->endpoint[i].desc;
      if ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) != USB_ENDPOINT_XFER_INT) {
         // DBG_ERR("orion_core launcher: ignore interface not based on interrupt (i.e. bulk)");
         return retval;
      }
   }

   udev = interface_to_usbdev(interface);
   if (!udev) {
      DBG_ERR("udev is NULL");
      goto exit;
   }
   DBG_INFO("launching %s", udev->product);

   dev = kzalloc(sizeof(struct usb_orion_core), GFP_KERNEL);
   if (!dev) {
      DBG_ERR("cannot allocate memory for struct usb_orion_core");
      retval = -ENOMEM;
      goto exit;
   }

   sema_init(&dev->sem, 1);
   spin_lock_init(&dev->cmd_spinlock);

   dev->udev = udev;
   dev->interface = interface;
   iface_desc = interface->cur_altsetting;


   /* Set up interrupt endpoint 'in' information. 
    */
   
   for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
      endpoint = &iface_desc->endpoint[i].desc;

      if (((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) && 
          ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_INT))
         dev->int_in_endpoint = endpoint;
   }
	
   if (!dev->int_in_endpoint) {
      DBG_ERR("could not find interrupt in endpoint");
      goto error;
   }

   int_end_size = le16_to_cpu(dev->int_in_endpoint->wMaxPacketSize);
   dev->int_in_buffer = kmalloc(int_end_size, GFP_KERNEL);
   if (!dev->int_in_buffer) {
      DBG_ERR("could not allocate int_in_buffer");
      retval = -ENOMEM;
      goto error;
   }

   dev->buffered_in_msg = kmalloc(int_end_size, GFP_KERNEL);
   if (!dev->buffered_in_msg) {
      DBG_ERR("could not allocate buffered_in_msg");
      retval = -ENOMEM;
      goto error;
   }

   dev->int_in_urb = usb_alloc_urb(0, GFP_KERNEL);
   if (!dev->int_in_urb) {
      DBG_ERR("could not allocate int_in_urb");
      retval = -ENOMEM;
      goto error;
   }

   DBG_DEBUG("'in' endpoint is %d, size is %d", dev->int_in_endpoint->bEndpointAddress & 0xf, int_end_size);

   
   /* Set up interrupt endpoint 'out' information. 
    */
   
   for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
      endpoint = &iface_desc->endpoint[i].desc;

      if (((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT) && 
          ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_INT))
         dev->int_out_endpoint = endpoint;
   }
	
   if (!dev->int_out_endpoint) {
      DBG_ERR("could not find interrupt out endpoint");
      goto error;
   }

   int_end_size = le16_to_cpu(dev->int_out_endpoint->wMaxPacketSize);
   dev->int_out_buffer = kmalloc(int_end_size, GFP_KERNEL);
   if (!dev->int_out_buffer) {
      DBG_ERR("could not allocate int_out_buffer");
      retval = -ENOMEM;
      goto error;
   }

   dev->int_out_urb = usb_alloc_urb(0, GFP_KERNEL);
   if (!dev->int_out_urb) {
      DBG_ERR("could not allocate int_out_urb");
      retval = -ENOMEM;
      goto error;
   }

   DBG_DEBUG("'out' endpoint is %d, size is %d", dev->int_out_endpoint->bEndpointAddress & 0xf, int_end_size);


   /* Retrieve a serial. */
   if (!usb_string(udev, udev->descriptor.iSerialNumber, dev->serial_number, sizeof(dev->serial_number))) {
      DBG_ERR("could not retrieve serial number");
      goto error;
   }
   DBG_DEBUG("serial number: %s", dev->serial_number);

   /* Save our data pointer in this interface device. */
   usb_set_intfdata(interface, dev);

   /* We can register the device now, as it is ready. */
   sprintf(devicename, "orion_core_%s", dev->serial_number);
   orion_core_class.name = devicename;
   retval = usb_register_dev(interface, &orion_core_class);
   if (retval) {
      DBG_ERR("not able to get a minor for this device.");
      usb_set_intfdata(interface, NULL);
      goto error;
   }

   dev->minor = interface->minor;
   DBG_INFO("driver now attached to /dev/orion_core_%s", /* interface->minor - ORION_CORE_MINOR_BASE */ dev->serial_number);

exit:
   return retval;

error:
   orion_core_delete(dev);
   return retval;
}



static void orion_core_disconnect(struct usb_interface *interface)
{
   struct usb_orion_core *dev;
   int minor;

   mutex_lock(&disconnect_mutex); /* Not interruptible */

   dev = usb_get_intfdata(interface);
   usb_set_intfdata(interface, NULL);

   down(&dev->sem); /* Not interruptible */

   minor = dev->minor;

   /* Give back our minor. */
   usb_deregister_dev(interface, &orion_core_class);

   /* If the device is not opened, then we clean up right now. */
   if (!dev->open_count) {
      up(&dev->sem);
      orion_core_delete(dev);
   } 
   else {
      dev->udev = NULL;
      up(&dev->sem);
   }

   mutex_unlock(&disconnect_mutex);

   DBG_INFO("/dev/orion_core%d now disconnected", minor - ORION_CORE_MINOR_BASE);
}



static struct usb_driver orion_core_driver = {
   .name = "orion_core_driver",
   .id_table = orion_core_table,
   .probe = orion_core_probe,
   .disconnect = orion_core_disconnect,
};



static int __init orion_core_init(void)
{
   int result;

   DBG_INFO("register module 1.01");
   result = usb_register(&orion_core_driver);
   if (result) {
      DBG_ERR("registering module failed");
   }
   else { 
      DBG_INFO("registered successfully");
   }
   return result;
}



static void __exit orion_core_exit(void)
{
   DBG_INFO("deregister module");
   usb_deregister(&orion_core_driver);
}



module_init(orion_core_init);
module_exit(orion_core_exit);

MODULE_AUTHOR("Gerard van de Glind");
MODULE_LICENSE("GPL");

