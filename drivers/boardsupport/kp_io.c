// #define PCTEST

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/buffer_head.h>
#include <asm/segment.h>
#include <asm-generic/errno-base.h>

#ifdef PCTEST
   #include <linux/configfs.h>
   #include <linux/tty.h>                  /* For fg_console, MAX_NR_CONSOLES */
   #include <linux/kd.h>                   /* For KDSETLED */
   #include <linux/vt.h>
   #include <linux/console_struct.h>       /* For vc_cons */
   #include <linux/vt_kern.h>
#endif

static struct proc_dir_entry *proc_parent_io;
static struct proc_dir_entry *proc_parent_in;
static struct proc_dir_entry *proc_parent_out; 
static int buzzer, statusled_red, statusled_green, statusled_blue;

#ifdef PCTEST
   
   static struct tty_driver *my_driver;

   static void init_leds(void)
   {
      int i;

      printk(KERN_INFO "kp_io: kbleds loading, fgconsole is %x\n", fg_console);
      for (i = 0; i < MAX_NR_CONSOLES; i++) {
         if (!vc_cons[i].d)
            break;
         // printk(KERN_INFO "poet_atkm: console[%i/%i] #%i, tty %lx\n", i,
         //        MAX_NR_CONSOLES, vc_cons[i].d->vc_num, (unsigned long)vc_cons[i].d->port.tty);
      }
     
      my_driver = vc_cons[fg_console].d->port.tty->driver;
      printk(KERN_INFO "kp_io: kbleds loaded, tty driver magic %x\n", my_driver->magic);
   }
 
   static void exit_leds(void) 
   {
	   printk(KERN_INFO "kp_io: kbleds unloading\n");
      (my_driver->ops->ioctl)(vc_cons[fg_console].d->port.tty, KDSETLED, 0xff);
   }

#endif // PCTEST



static struct file* file_open(const char* path, int flags, int rights) 
{
   struct file* filp = NULL;
   mm_segment_t oldfs;
   int err = 0;

   oldfs = get_fs();
   set_fs(get_ds());
   filp = filp_open(path, flags, rights);
   set_fs(oldfs);
   if (IS_ERR(filp)) {
      err = PTR_ERR(filp);
      return NULL;
   }
   return filp;
}



static void file_close(struct file* file) 
{
   filp_close(file, NULL);
}



static int file_read(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size) 
{
   mm_segment_t oldfs;
   int ret;

   oldfs = get_fs();
   set_fs(get_ds());

   ret = vfs_read(file, data, size, &offset);

   set_fs(oldfs);
   return ret;
}  
 


static int file_write(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size) 
{
   mm_segment_t oldfs;
   int ret;

   oldfs = get_fs();
   set_fs(get_ds());

   ret = vfs_write(file, data, size, &offset);

   set_fs(oldfs);
   return ret;
}



static int file_sync(struct file* file) 
{
   vfs_fsync(file, 0);
   return 0;
}



/*========== input routines ==========*/



/*--- tamper input ---*/

static int kp_io_show_tamper(struct seq_file *m, void *v)
{
   struct file* filp;
   int  value = 0;
   char buffer[16] = "";
   #ifdef PCTEST
      char *path =  "/home/gerard/gpiotest/tamper/value";
   #else
      char *path = "/sys/class/gpio/gpio41/value";
   #endif

   filp = file_open(path, O_RDONLY, 0); 
   if (filp) {
      if (file_read(filp, 0, buffer, 1) == 1)
         value = (buffer[0] != '0');
      file_close(filp); 
   }

   seq_printf(m, "%d\n", value);

   return 0;
}



static int kp_io_open_tamper(struct inode *inode, struct file *file)
{
   return single_open(file, kp_io_show_tamper, NULL);
}



static const struct file_operations kp_io_fops_tamper = {
   .owner      = THIS_MODULE,
   .open       = kp_io_open_tamper,
   .read       = seq_read,
   .llseek     = seq_lseek,
   .release    = single_release,
};



/*--- bootsel input ---*/

static int kp_io_show_bootsel(struct seq_file *m, void *v)
{
   struct file* filp;
   int  value = 0;
   char buffer[16] = "";
   #ifdef PCTEST
      char *path =  "/home/gerard/gpiotest/bootsel/value";
   #else
      char *path = "/sys/class/gpio/gpio71/value";
   #endif

   filp = file_open(path, O_RDONLY, 0); 
   if (filp) {
      if (file_read(filp, 0, buffer, 1) == 1)
         value = (buffer[0] == '0');
      file_close(filp); 
   }

   seq_printf(m, "%d\n", value);

   return 0;
}



static int kp_io_open_bootsel(struct inode *inode, struct file *file)
{
   return single_open(file, kp_io_show_bootsel, NULL);
}



static const struct file_operations kp_io_fops_bootsel = {
   .owner      = THIS_MODULE,
   .open       = kp_io_open_bootsel,
   .read       = seq_read,
   .llseek     = seq_lseek,
   .release    = single_release,
};



/*--- boardrev input ---*/

static int kp_io_show_boardrev(struct seq_file *m, void *v)
{
   #define MAX_ID 6 
   struct file* filp;
   int  ii, value = 0;
   char buffer[16] = "";
   #ifdef PCTEST
      char *path[MAX_ID] = 
         {"/home/gerard/gpiotest/boardrev/value5",
          "/home/gerard/gpiotest/boardrev/value4",
          "/home/gerard/gpiotest/boardrev/value3",
          "/home/gerard/gpiotest/boardrev/value2",
          "/home/gerard/gpiotest/boardrev/value1",
          "/home/gerard/gpiotest/boardrev/value0"};
   #else
      char *path[MAX_ID] = 
         {"/sys/class/gpio/gpio73/value", 
          "/sys/class/gpio/gpio74/value", 
          "/sys/class/gpio/gpio46/value", 
          "/sys/class/gpio/gpio56/value", 
          "/sys/class/gpio/gpio55/value", 
          "/sys/class/gpio/gpio54/value"  
         };
   #endif

   for (ii = 0; ii < MAX_ID; ii++) {
      filp = file_open(path[ii], O_RDONLY, 0); 
      if (filp) {
         if (file_read(filp, 0, buffer, 1) == 1) 
            value = (value * 2) + (buffer[0] != '0');
         file_close(filp); 
      }
   }

   seq_printf(m, "%02x\n", value);

   return 0;
}



static int kp_io_open_boardrev(struct inode *inode, struct file *file)
{
   return single_open(file, kp_io_show_boardrev, NULL);
}



static const struct file_operations kp_io_fops_boardrev = {
   .owner      = THIS_MODULE,
   .open       = kp_io_open_boardrev,
   .read       = seq_read,
   .llseek     = seq_lseek,
   .release    = single_release,
};



/*========== output routines ==========*/



/*--- buzzer output ---*/

static int kp_io_show_buzzer(struct seq_file *m, void *v)
{
   seq_printf(m, "buzzer is %s\n", (buzzer == 0) ? "off" : "on");
   return 0;
}



static int kp_io_open_buzzer(struct inode *inode, struct file *file)
{
   return single_open(file, kp_io_show_buzzer, NULL);
}



static ssize_t kp_io_write_buzzer(struct file *file, const char *buf, size_t size, loff_t *ppos)
{
   struct file* filp;
   size_t len = 20;
   char mbuf[21], buffer[16];
   #ifdef PCTEST
      char *path = "/home/gerard/gpiotest/buzzer/value";
   #else
      char *path = "/sys/class/gpio/gpio2/value";
   #endif
 
   if (size < len)
      len = size;
   if (copy_from_user(mbuf, buf, len))
      return -EFAULT;
   mbuf[len] = '\0';

   buzzer = (strncmp(mbuf, "0", 1) != 0);

   filp = file_open(path, O_WRONLY, 0); 
   if (filp) {
      strcpy(buffer, buzzer ? "1" : "0");
      file_write(filp, 0, buffer, 1);
      file_sync(filp);
      file_close(filp); 
   }

   return size;
}



static const struct file_operations kp_io_fops_buzzer = {
   .owner      = THIS_MODULE,
   .open       = kp_io_open_buzzer,
   .read       = seq_read,
   .write      = kp_io_write_buzzer,
   .llseek     = seq_lseek,
   .release    = single_release,
};



/*--- statusled ---*/
struct file* filp;
static int kp_io_show_statusled(struct seq_file *m, void *v)
{
   seq_printf(m, "statusled (r-g-b) = x%02x x%02x x%02x\n", statusled_red, statusled_green, statusled_blue);
   return 0;
}



static int kp_io_open_statusled(struct inode *inode, struct file *file)
{
   return single_open(file, kp_io_show_statusled, NULL);
}



static ssize_t kp_io_write_statusled(struct file *file, const char *buf, size_t size, loff_t *ppos)
{
   size_t len = 20;
   char mbuf[21];
   #ifndef PCTEST
      struct file* filp;
      char buffer[16];
      char *pathR = "/sys/class/leds/lp5521:channel0/brightness";
      char *pathG = "/sys/class/leds/lp5521:channel1/brightness";
      char *pathB = "/sys/class/leds/lp5521:channel2/brightness";
   #endif

   if (size < len)
      len = size;

   if (copy_from_user(mbuf, buf, len))
      return -EFAULT;

   mbuf[len] = '\0';

   if (len > 6) {      // rrggbb+\n
      mbuf[6] = '\0';  // trim to rrggbb and read 'bb'
      sscanf(&mbuf[4], "%x", &statusled_blue);
      mbuf[4] = '\0';  // trim to rrgg and read 'gg'
      sscanf(&mbuf[2], "%x", &statusled_green);
      mbuf[2] = '\0';  // trim to rr and read 'rr'
      sscanf(mbuf, "%x", &statusled_red);
   } 

   #ifdef PCTEST
      (my_driver->ops->ioctl)(vc_cons[fg_console].d->port.tty, KDSETLED, (statusled_red ? 1 : 0) | (statusled_green ? 2 : 0) | (statusled_blue ? 4 : 0));
   #else
      // Set red led
      filp = file_open(pathR, O_WRONLY, 0); 
      if (filp) {
         sprintf(buffer, "%03d", statusled_red);
         file_write(filp, 0, buffer, 3);
         file_sync(filp);
         file_close(filp);
         // Set green led... 
         filp = file_open(pathG, O_WRONLY, 0); 
         if (filp) {
            sprintf(buffer, "%03d", statusled_green);
            file_write(filp, 0, buffer, 3);
            file_sync(filp);
            file_close(filp); 
            // Set blue led...
            filp = file_open(pathB, O_WRONLY, 0); 
            if (filp) {
               sprintf(buffer, "%03d", statusled_blue);
               file_write(filp, 0, buffer, 3);
               file_sync(filp);
               file_close(filp);
            }
            else
               printk(KERN_ERR "kp_io: error opening file %s\n", pathB); 
         } 
         else
            printk(KERN_ERR "kp_io: error opening file %s\n", pathG); 
      }
      else
         printk(KERN_ERR "kp_io: error opening file %s\n", pathR); 
   #endif

   return size;
}



static const struct file_operations kp_io_fops_statusled = {
   .owner      = THIS_MODULE,
   .open       = kp_io_open_statusled,
   .read       = seq_read,
   .write      = kp_io_write_statusled,
   .llseek     = seq_lseek,
   .release    = single_release,
};





/*========== init (exit) routines ==========*/



static void remove_all_proc_entries(void) 
{
   if (proc_parent_in) {
      remove_proc_entry("tamper",    proc_parent_in);
      remove_proc_entry("bootsel",   proc_parent_in);
      remove_proc_entry("boardrev",  proc_parent_in);
      remove_proc_entry("in",        proc_parent_io);
   }

   if (proc_parent_out) {
      remove_proc_entry("buzzer",    proc_parent_out);
      remove_proc_entry("statusled", proc_parent_out);
      remove_proc_entry("out",       proc_parent_io);
   }

   if (proc_parent_io) 
      remove_proc_entry("gpio", NULL);
}



static int __init kp_io_init(void)
{
   int error = 0;
   printk(KERN_INFO "kp_io: loading module\n");

   #ifdef PCTEST
      init_leds();
   #endif
   
   proc_parent_io = proc_mkdir("gpio", NULL);
   if (!proc_parent_io) 
      error++;
   else {
      proc_parent_in = proc_mkdir("in",  proc_parent_io); 
      if (!proc_parent_in) 
         error++;
      else { 
         proc_create("tamper",    0, proc_parent_in,  &kp_io_fops_tamper);
         proc_create("bootsel",   0, proc_parent_in,  &kp_io_fops_bootsel);
         proc_create("boardrev",  0, proc_parent_in,  &kp_io_fops_boardrev);

         proc_parent_out = proc_mkdir("out", proc_parent_io); 
         if (!proc_parent_out) 
            error++;
         else {
            proc_create("buzzer",    0666, proc_parent_out, &kp_io_fops_buzzer);
            proc_create("statusled", 0666, proc_parent_out, &kp_io_fops_statusled);
         }
      }
   }

   if (error) {
      printk(KERN_ERR "kp_io: loading module error\n"); 
      remove_all_proc_entries();
      #ifdef PCTEST
         exit_leds();
      #endif
   }
   return (error ? -EIO : 0);
}



static void __exit kp_io_exit(void)
{
   printk(KERN_INFO "kp_io: unloading module\n"); 
   remove_all_proc_entries();
   #ifdef PCTEST
      exit_leds();
   #endif
}



module_init(kp_io_init);
module_exit(kp_io_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gerard van de Glind, Keyprocessor B.V.");
MODULE_DESCRIPTION("Controlling the Pluto board, release 1.01");
