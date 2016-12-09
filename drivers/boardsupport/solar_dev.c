#include <linux/init.h>           // Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>         // Core header for loading LKMs into the kernel
#include <linux/device.h>         // Header to support the kernel Driver Model
#include <linux/kernel.h>         // Contains types, macros, functions for the kernel
#include <linux/fs.h>             // Header for the Linux file system support
#include <asm/uaccess.h>          // Required for the copy to user function
#include <linux/slab.h>           // kfree, kzalloc	
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/delay.h>          // msleep

// #define PC
#ifdef PC
   // #define SIMU_READ
   // After one time read then issue (with unloaded module) : sudo i2cdump -r 0-0xf 1 0x50 to reinit read 
   // A re-inited read gives at first read 0x28, 0x14 ...  
   #define SIMU_WRITE
#endif

#define DEVICE_NAME               "solar_dev"
#define CLASS_NAME                "solar"
#define PACKET_SIZE_READ_MAX      265 // length max 255 = cdm+status+data + 10 (stx, 4x sno, 2x len, 2x crc, etx)
#define PACKET_SIZE_WRITE_MAX     128
#define PACKET_SIZE_WRITE_MIN     11


#define DBG_INFO(fmt, args...) \
if (debug_info) printk( KERN_DEBUG "[info]  %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ## args)

#define DBG_ERR(fmt, args...) \
printk( KERN_DEBUG "[err]   %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ## args)



typedef enum {RC_STX, RC_SNR0, RC_SNR1, RC_SNR2, RC_SNR3, RC_LEN0, RC_LEN1, RC_CMD, RC_STATUS, RC_DATA, RC_CRC0, RC_CRC1, RC_ETX, RC_READY, RC_ERROR} rcState;


struct solar_strc {
   struct i2c_client *client;
   struct semaphore  sem;              // Locks this structure 
   char              *buffered_in_msg; // for copy to userland...
   int               buffered_in_len;
   #ifdef PC
      struct i2c_adapter *adapter;
   #endif
};



static int                   debug_info = 1;
static struct class          *solar_class = NULL;  // The device class -- this is a character device driver
static struct device         *solar_device = NULL; // The device-driver device struct pointer
static struct solar_strc     *solar = NULL;
static int                   majorNumber;  
static int                   numberOpens = 0;

static const struct i2c_device_id solar_id[] = {
   { DEVICE_NAME, 0 },
   {},
};



MODULE_DEVICE_TABLE(i2c, solar_id);



static int solar_dev_open(struct inode *inode, struct file *file)
{
   // inodep A pointer to an inode object (defined in linux/fs.h)
   // filep A pointer to a file object (defined in linux/fs.h)
   
   numberOpens++;
   DBG_INFO("opened %d time(s)", numberOpens);
   return 0;
};



static int solar_dev_release(struct inode *inode, struct file *file)
{
   numberOpens--;
   DBG_INFO("opened %d time(s)", numberOpens);
   return 0;
};



static uint16_t crc16_update_ccitt(uint16_t crc, uint8_t data)
{
   #define lo8(x) (x & 0xff)
   #define hi8(x) (x >> 8)

   data ^= lo8 (crc);
   data ^= data << 4;
   return ((((uint16_t)data << 8) | hi8 (crc)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}



static rcState processRead(rcState state, u8 databyte)
{
   static u8 length;
   static unsigned short crc;

   // RC_STX, RC_SNR0, RC_SNR1, RC_SNR2, RC_SNR3, RC_LEN0, RC_LEN1, RC_CMD, RC_STATUS, RC_DATA, RC_CRC0, RC_CRC1, RC_ETX, RC_READY, RC_ERROR} rcState; 
   // DBG_INFO("processRead state=%d, databyte=%02x", state, databyte);
   
   if ((state > RC_STX) && (state < RC_CRC0))
      crc = crc16_update_ccitt(crc, databyte);

   switch (state) {
      case RC_STX : 
         crc = 0xffff;
         return (databyte == 0x82) ? RC_SNR0 : RC_ERROR;
      
      case RC_SNR0 : case RC_SNR1 : case RC_SNR2 : 
         return (state+1);
      
      case RC_SNR3 : 
         return RC_LEN0;
 
      case RC_LEN0 : 
         return (databyte == 0) ? RC_LEN1 : RC_ERROR;
      
      case RC_LEN1 :
         length = databyte; 
         return (databyte >= 2) ? RC_CMD : RC_ERROR; // By CMD and STATUS always >= 2 
      
      case RC_CMD : 
         length--;
         return RC_STATUS;

      case RC_STATUS :
         length--;
         return (length == 0) ? RC_CRC0 : RC_DATA;
 
      case RC_DATA :
         length--;
         return (length == 0) ? RC_CRC0 : RC_DATA;

      case RC_CRC0 :
         return (databyte == ((crc & 0xff00) >> 8)) ? RC_CRC1 : RC_ERROR;

      case RC_CRC1 :
         return (databyte == (crc & 0xff)) ? RC_ETX : RC_ERROR;

      case RC_ETX : 
         return (databyte == 0x83) ? RC_READY : RC_ERROR;

      default : 
         return state;
   }

   return state;
}



#ifdef SIMU_READ

static int simulate_read(int start) 
{
   static int idx;
   const  int ar[] = {/* 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, */ 0x00, 0x82, 0x00, 0x17, 0xae, 0xb2, 0x00, 0x06, 0xd2, 0x00, 0x84, 0x01, 0x01, 0x0b, 0x21, 0xb7, 0x83};
   int        ret;

   if (start) {
      idx = 0;
      return 0;
   } 
   ret = ar[idx];
   if (++idx >= (sizeof(ar)/sizeof(int)))
      idx--;
   return ret;
}

#endif



static ssize_t solar_dev_write(struct file *file, const char *buf, size_t length, loff_t *offset)
{
   rcState state;
   int     retval = 0, ii, read, retries;
   u8      writebuf[PACKET_SIZE_WRITE_MAX], databyte;
   char    *buffered_in_ptr;  
   
   // Lock this object.
   if (down_interruptible(&solar->sem)) {
      DBG_ERR("sem down failed");
      retval = -ERESTARTSYS;
      goto exit;
   }

   if (solar == NULL) {
      DBG_ERR("solar == null");
      goto unlock_exit; 
   }
   
   if (solar->client == NULL) {
      DBG_ERR("solar->client == null");
      goto unlock_exit;
   }
      
   if (solar->client->adapter == NULL) {
      DBG_ERR("solar->client->adapter == null");
      goto unlock_exit;
   }

   // PM check...
   if ((length > PACKET_SIZE_WRITE_MAX) || (length < PACKET_SIZE_WRITE_MIN)) {
      DBG_ERR("unsupported amount to write (%d)", (int)length);
      goto unlock_exit;
   }
      

   // OK, write the ucam sequence to the I2C slave...
   copy_from_user(writebuf, buf, length);
   DBG_INFO("write ucam seq with len=%d, cmd=0x%02x", (int)length, writebuf[7]);
   solar->buffered_in_len = 0;
   buffered_in_ptr        = solar->buffered_in_msg;    
   state                  = RC_STX;
   retries                = 10;

   #ifdef SIMU_READ
      simulate_read(1);                                    // Init the read simulator
   #endif

   #ifdef SIMU_WRITE
      retval = 0;
   #else
      retval = i2c_master_send(solar->client, writebuf, length);
   #endif

   if (retval < 0) {
      DBG_ERR("i2c_master_send returns %d", retval);
      retval = -EIO;
      goto unlock_exit;
   }
   

   // Write OK, give I2C slave (Solar) at least 50ms to prepare a reply and start reading...
   msleep(50);

   while (1) {
      #ifdef SIMU_READ
         databyte = simulate(0);
         read = 0;
      #else   
         read = i2c_master_recv(solar->client, &databyte, 1); // read one byte from the slave
      #endif
      if (read < 0) {                                      // error while reading, quit
         DBG_ERR("i2c_master_recv returns %d", read);
         solar->buffered_in_len = 0;
         retval = -EIO;
         goto unlock_exit;
      }

      // databyte = (u8)(read & 0xFF);                     // read a valid byte
      if ((databyte == 0x00) && retries) {                 // first bytes with value of 0x00 means 'wait, slave not ready'
         if (--retries == 0) {                             // retry the read max 10 times (=500 ms), else quit
            DBG_ERR("response error timeout");
            retval = -EIO;
            goto unlock_exit;
         }
         DBG_INFO("wait request from slave");
         msleep(50);
         continue;
      }
      
      retries = 0;                                         // OK, one byte was read correctly
      solar->buffered_in_len++;
      *buffered_in_ptr++ = databyte;
      state = processRead(state, databyte);
      if ((state == RC_ERROR) || (state == RC_READY))
         break; 
   }

   // Read finished
   if (state != RC_READY) {
      solar->buffered_in_len = 0;
      DBG_ERR("ready but with error in state %d\n", state);
   }
   else {
      DBG_INFO("ready and okay, len=%d : ", solar->buffered_in_len);
      for (ii = 0; ii < solar->buffered_in_len; ii++)
         printk("%02x ", (u8)solar->buffered_in_msg[ii]);
      printk("\n");
   }


unlock_exit:
   up(&solar->sem);

exit:
   return retval;
}



static ssize_t solar_dev_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
   int bytes_read = 0, retval = 0;
   char *buffered_in_ptr;  

   // Lock this object.
   if (down_interruptible(&solar->sem)) {
      DBG_ERR("sem down failed");
      retval = -ERESTARTSYS;
      goto exit;
   }

   // DBG_INFO("ask for %d bytes to read, has %d bytes", (int)count, solar->buffered_in_len);

   // Some basic checks
   if ((solar->buffered_in_len == 0) || (count == 0))
      goto unlock_exit;

   // Put the data into the userland buffer
   buffered_in_ptr = solar->buffered_in_msg; 
   while (solar->buffered_in_len && count) {
      put_user(*buffered_in_ptr++, buf++);
      solar->buffered_in_len--;
      count--;
      bytes_read++;
   }
   retval = bytes_read;

unlock_exit:
   up(&solar->sem);

exit:
   return retval;
}



static struct file_operations solar_dev_ops = {
   .read    = solar_dev_read,
   .write   = solar_dev_write,
   .open    = solar_dev_open,
   .release = solar_dev_release
};



static int solar_dev_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
   DBG_INFO("I2C probed !");
 
   if (  (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_READ_BYTE)) || 
         (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WRITE_BLOCK_DATA))  ) {
      DBG_ERR("no I2C_FUNC_SMBUS_READ_BYTE or I2C_FUNC_SMBUS_WRITE_BLOCK_DATA functionality");
      return -EIO;
   }

   // Allocate the memory for this device 
   solar = kzalloc(sizeof(struct solar_strc), GFP_KERNEL);   
   if (solar == NULL) {
      DBG_ERR("could not allocate solar_strc");
      return -ENOMEM;
   }    

   solar->buffered_in_msg = kzalloc(PACKET_SIZE_READ_MAX, GFP_KERNEL);
   if (!solar->buffered_in_msg) {
      DBG_ERR("could not allocate buffered_in_msg");
      kfree(solar);
      return -ENOMEM;
   }

   sema_init(&solar->sem, 1);

   solar->client = client;
   return 0;
}



static int solar_dev_remove(struct i2c_client *client)
{
   DBG_INFO("removed !");
   kfree(solar->buffered_in_msg);
   kfree(solar);
   return 0;
}



static struct i2c_driver solar_driver =
{
   .driver = {
      .name  = DEVICE_NAME,
   },
   .probe    = solar_dev_probe,
   .remove   = solar_dev_remove,
   .id_table = solar_id,
};



static int __init solar_dev_init(void)
{
   int ret = 0;

   DBG_INFO("initializing...");

   // Try to dynamically allocate a major number for the device
   ret = register_chrdev(0, DEVICE_NAME, &solar_dev_ops);
   if (ret < 0) {
      DBG_ERR("failed to register a major number");
      return ret;
   }
   majorNumber = ret;
   DBG_INFO("registered correctly with major number %d", majorNumber);


   // Register the device class
   solar_class = class_create(THIS_MODULE, CLASS_NAME);
   if (IS_ERR(solar_class)){                               // Check for error and clean up if there is
      DBG_ERR("failed to register device class");
      ret = PTR_ERR(solar_class);                          // Correct way to return an error on a pointer
      goto unregister;
   }


   // Register the device driver
   solar_device = device_create(solar_class, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
   if (IS_ERR(solar_device)){                              // Clean up if there is an error
      DBG_ERR("failed to create the device");
      ret = PTR_ERR(solar_device);
      goto cls_destroy;
   }

  
   // I2C add the driver
   ret = i2c_add_driver(&solar_driver); 
   if (ret) {                                              // Clean up if there is an error
      DBG_ERR("registering I2C driver failed (%d)", ret);
      goto dev_destroy;
   }

#ifdef PC
   // Allocate the memory for this device 
   solar = kzalloc(sizeof(struct solar_strc), GFP_KERNEL);   
   if (solar == NULL) {
      DBG_ERR("could not allocate solar_strc");
      return -ENOMEM;
   }    

   solar->buffered_in_msg = kzalloc(PACKET_SIZE_READ_MAX, GFP_KERNEL);
   if (!solar->buffered_in_msg) {
      DBG_ERR("could not allocate buffered_in_msg");
      kfree(solar);
      return -ENOMEM;
   }
   
   solar->adapter = i2c_get_adapter(1); // 1 means i2c-1 bus
   solar->client  = i2c_new_dummy(solar->adapter, 0x50); // 0x50 - slave address on i2c bus
  
   sema_init(&solar->sem, 1);
#endif

   return 0;

dev_destroy:
   device_destroy(solar_class, MKDEV(majorNumber, 0)); 

cls_destroy:
   class_unregister(solar_class);                          // Not in example of http://derekmolloy.ie/writing-a-linux-kernel-module-part-2-a-character-device/
   class_destroy(solar_class);

unregister:
   unregister_chrdev(majorNumber, DEVICE_NAME);

   return ret;
}



static void __exit solar_dev_exit(void)
{
   DBG_INFO("exiting...");

   device_destroy(solar_class, MKDEV(majorNumber, 0));     // remove the device
   class_unregister(solar_class);                          // unregister the device class
   class_destroy(solar_class);                             // remove the device class
   unregister_chrdev(majorNumber, DEVICE_NAME);            // unregister the major number
   i2c_del_driver(&solar_driver);
   #ifdef PC
      i2c_unregister_device(solar->client);
   #endif
}


module_init(solar_dev_init);
module_exit(solar_dev_exit);

MODULE_DESCRIPTION("Solar i2c driver running UCAM commands 29sept2016, v1.02");
MODULE_AUTHOR("Gerard van de Glind");
MODULE_LICENSE("GPL");
