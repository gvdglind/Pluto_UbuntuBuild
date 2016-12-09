/*
 * userProxy
 *
 *  Created on: Dec 3, 2013
 *      Author: sebastiaan
 */

#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <linux/wait.h>
#include <asm/param.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/timer.h>
#include <linux/watchdog.h>
#include <linux/semaphore.h>

// #define _VERBOSE
// #define _VERBOSE2

#define DEF_MODULENAME	"userspace proxy"
#define MAX_TUNNELS		100
#define OUTPUTS_MINOR	135

#define UP_MODULE_VERSION	2

#define UP_CREATE_TUNNEL		0x01
#define UP_DESTROY_TUNNEL		0x03

#define MAX_TREE_DEPTH		10

struct _tunnelInit {
	int readBufferSize;
	int writeBufferSize;
	char tunnelName[150];
};

struct _pathTree {
	char * tunnelPath;
	struct proc_dir_entry* dir_entry;
};

struct _proxyTunnel {
	char tunnelName[150];
	struct _pathTree path[MAX_TREE_DEPTH];
	int pathdepth;

	struct proc_dir_entry* entry;

	uint8_t * writeBuffer;
	int writeOffset;
	int writeBufferSize;

	uint8_t * readBuffer;
	int readOffset;
	int readBufferSize;

	pid_t pid; /* Pid of process that created it */
};

struct _userProxy {
	struct _proxyTunnel *tunnels[MAX_TUNNELS];
	int tunnelCount;
	int lastWriteTunnel;
	int driverUsage;

	struct semaphore sem; /* mutual exclusion semaphore */
	wait_queue_head_t inq, outq; /* read and write queues */
};
static struct _userProxy proxy;
static int VersionTunnel = 0;

static pid_t getUserPid(void) {
	if (current) {
		return task_pid_nr(current);
	}
	return 0;
}

static struct _proxyTunnel * findTunnelWithData(void) {
	int x;
	pid_t pid = getUserPid();

	for (x = 0; x < MAX_TUNNELS; x++) {
		if (proxy.lastWriteTunnel < 0 || proxy.lastWriteTunnel >= MAX_TUNNELS)
			proxy.lastWriteTunnel = 0;

		if (proxy.tunnels[proxy.lastWriteTunnel]) {
			/* By read on 'main' fd we only return data to creator */
			if (proxy.tunnels[proxy.lastWriteTunnel]->pid == pid) {
				if (proxy.tunnels[proxy.lastWriteTunnel]->writeOffset > 0) {
					return proxy.tunnels[proxy.lastWriteTunnel];
				}
			}
		}
		proxy.lastWriteTunnel++;
	}
	return NULL;
}

static int checkTunnelWritePossibility(void) {
	int x;

	for (x = 0; x < MAX_TUNNELS; x++) {
		if (proxy.tunnels[x]->readOffset < proxy.tunnels[x]->readBufferSize)
			return x + 1;
	}
	return 0;
}

int genArg(char * buffer, int buffsize, struct _proxyTunnel *tunnel, int maxArgc, char spacer) {
	int x;
	int blankMarker = 0;
	int argc = 0;

	if (buffsize <= 0)
		return 0;

	maxArgc--;

	// initialise
	for (x = 0; x < maxArgc; x++) {
		tunnel->path[x].tunnelPath = NULL;
	}

	for (x = 0; x < buffsize; x++) {
		if ((buffer[x] != spacer) && (blankMarker == 0)) {
			if (argc >= maxArgc) {
				break;
			}
			tunnel->path[argc++].tunnelPath = buffer + x;
			blankMarker = 1;
		}
		if (buffer[x] == spacer) {
			blankMarker = 0;
			buffer[x] = 0;
		}
	}
	return argc;
}

/*
 * The proc entry displays the managed area and some counters
 */
ssize_t userproxy_tunnel_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {
	struct _proxyTunnel * tunnel = PDE_DATA(file_inode(filp));
	int len, offset = 0;

	if (tunnel == NULL ) {
		return -EFAULT;
	}

	if (f_pos) {
		offset = *f_pos;
#ifdef _VERBOSE2
		printk("userproxy_tunnel_read offset %d-%d from %s\n",offset, (int)filp->f_pos, tunnel->tunnelName);
#endif
	}

	/*do a size check*/
	len = count;
	if (offset + count > tunnel->readOffset) {
		len = tunnel->readOffset - offset;
		len--; //need one byte less
	}

	/*if size > 0 do a userspace copy*/
	if (len > 0) {
		if (copy_to_user(buf, tunnel->readBuffer + offset, len)) {
			return -EFAULT;
		}

		if (f_pos) {
			*f_pos += len;
		}

		wake_up_interruptible(&proxy.outq);
	}
	return len;
}

ssize_t userproxy_tunnel_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
	struct _proxyTunnel * tunnel = PDE_DATA(file_inode(filp));
	int len;

	if (tunnel == NULL ) {
		return -EFAULT;
	}

	/*do a size check*/
	len = count;
	if (tunnel->writeOffset + count > tunnel->writeBufferSize) {
		len = tunnel->writeBufferSize - tunnel->writeOffset;
	}
	/*if size > 0 do a userspace copy*/
	if (len > 0) {
		if (copy_from_user(tunnel->writeBuffer + tunnel->writeOffset, buf, len))
			return -EFAULT;

#ifdef _VERBOSE2
		printk("offset %d > %s\n",tunnel->writeOffset,tunnel->writeBuffer);
#endif
		tunnel->writeOffset += len;
		wake_up_interruptible(&proxy.inq); /* blocked in read() and select() */
	}
	return len;
}

struct file_operations proc_fops =
{ read: userproxy_tunnel_read, write: userproxy_tunnel_write };

void printPacket(unsigned char *packet, int packetLen) {
	int x;

	printk("PCKT len:%d :", packetLen);
	for (x = 0; x < packetLen; x++) {
		printk("%02x", packet[x]);
	}
	printk("\n");
}

int check_tree_excistance(struct _proxyTunnel *tunnel) {
	int tunnelId, p = -1;

	if (tunnel != NULL && tunnel->pathdepth >= 1) {
		for (tunnelId = 0; tunnelId < MAX_TUNNELS; tunnelId++) {
			//tunnel is open and is NOT this tunnel...
			if (proxy.tunnels[tunnelId] != NULL && proxy.tunnels[tunnelId] != tunnel) {
				for (p = 0; p < tunnel->pathdepth; p++) {

					if (strcmp(proxy.tunnels[tunnelId]->path[p].tunnelPath, tunnel->path[p].tunnelPath) == 0) {
#ifdef _VERBOSE2
						printk("match %d %s, depth=%d p=%d\n", tunnelId + 1, proxy.tunnels[tunnelId]->path[p].tunnelPath, tunnel->pathdepth, p);
#endif
						if (p + 1 == tunnel->pathdepth) {
#ifdef _VERBOSE
							printk("found occurrence at tree depth %d,tunnelId: %d, pid=%d\n", p, tunnelId + 1, proxy.tunnels[tunnelId]->pid);
#endif
							return tunnelId + 1;
						}
					} else {
						break;
					}
				}
			}
		}
	}
	//item does not exist so return 0
#ifdef _VERBOSE
	printk("could not find '%s', return 0\n", tunnel->tunnelName);
#endif
	return 0;
}

struct proc_dir_entry* buildDirectoryTree(struct _proxyTunnel *tunnel) {
	int x, p = 0;
	struct proc_dir_entry* parent = NULL;
	if (tunnel != NULL) {
		if (tunnel->pathdepth > 1) {
			for (p = 0; p < tunnel->pathdepth - 1; p++) {
				parent = NULL;
				for (x = 0; x < MAX_TUNNELS; x++) {
					//tunnel is open and is NOT this tunnel...
					if (proxy.tunnels[x] != NULL && proxy.tunnels[x] != tunnel) {
						if (strcmp(proxy.tunnels[x]->path[p].tunnelPath, tunnel->path[p].tunnelPath) == 0) {
							parent = proxy.tunnels[x]->path[p].dir_entry;
						}
					}
				}
				//directory, does not exist, so create it
				if (parent == NULL) {
#ifdef _VERBOSE
					printk("creating '%s' in dir '%s'\n", tunnel->path[p].tunnelPath, (p) ? tunnel->path[p - 1].tunnelPath : "/");
#endif
					if (p > 0) {
						parent = proc_mkdir(tunnel->path[p].tunnelPath, tunnel->path[p - 1].dir_entry); //create new directory with parent
					} else {
						parent = proc_mkdir(tunnel->path[p].tunnelPath, NULL);
					}

					if (parent == NULL) {
						printk("could not create proc directory %s in level %d\n", tunnel->path[p].tunnelPath, p);
					}
				}
				tunnel->path[p].dir_entry = parent;
			}
		}
		parent = proc_create_data(tunnel->path[p].tunnelPath, 0666, parent, &proc_fops, tunnel);
		tunnel->pid = getUserPid();
#ifdef _VERBOSE
		printk("creating endpoint '%s' for pid %d\n", tunnel->path[p].tunnelPath, tunnel->pid);
#endif
		tunnel->path[p++].dir_entry = parent;
	}
	return parent;
}

int cleanUpDirectoryTree(struct _proxyTunnel *tunnel) {
	int x, p;
	struct proc_dir_entry* parent = NULL;

	if (tunnel != NULL && tunnel->pathdepth >= 1) {
#ifdef _VERBOSE
		printk("requested cleanup of tunnel endpoint '%s' depth %d\n", tunnel->path[tunnel->pathdepth - 1].tunnelPath, tunnel->pathdepth);
#endif
		for (p = tunnel->pathdepth - 1; p >= 0; p--) {
			parent = NULL;
#ifdef _VERBOSE
			printk("cleaning up level %d : '%s'\n", p, tunnel->path[p].tunnelPath);
#endif
			for (x = 0; x < MAX_TUNNELS; x++) {
				//tunnel is open and is NOT this tunnel...
				if (proxy.tunnels[x] != NULL && proxy.tunnels[x] != tunnel) {
					/* same name */
					if (strcmp(proxy.tunnels[x]->path[p].tunnelPath, tunnel->path[p].tunnelPath) == 0) {
						/* and same parent */
						if (proxy.tunnels[x]->path[p].dir_entry == tunnel->path[p].dir_entry) {
							parent = proxy.tunnels[x]->path[p].dir_entry;
						}
					}
				}
			}
			//nobody else uses the directory, clean it up...
			if (parent == NULL) {
				if (tunnel->path[p].dir_entry != NULL) {
#ifdef _VERBOSE
					printk("removing %s from dir %s\n", tunnel->path[p].tunnelPath, tunnel->path[p - 1].tunnelPath);
#endif
					if (p > 0) {
						parent = tunnel->path[p - 1].dir_entry;
					} else {
						parent = NULL;
					}
					remove_proc_entry(tunnel->path[p].tunnelPath, parent);
				} else {
#ifdef _VERBOSE
					printk("will not remove dir entry p: %d\n", p);
#endif
				}
			}
		}
	}
	return 0;
}

int create_new_tunnel(struct _tunnelInit * tun_init, struct _proxyTunnel *tunnel) {
	int ret;
	memset(tunnel, 0, sizeof(struct _proxyTunnel));

	strlcpy(tunnel->tunnelName, tun_init->tunnelName, sizeof(tunnel->tunnelName));
	if ((tunnel->pathdepth = genArg(tunnel->tunnelName, strlen(tunnel->tunnelName), tunnel, MAX_TREE_DEPTH, '/')) <= 0) {
#ifdef _VERBOSE
		printk("Failed to parse arguments : %d\n", tunnel->pathdepth);
#endif
		return -EFAULT;
	}
#ifdef _VERBOSE
	printk("found %d arguments for directory structure %s\n", tunnel->pathdepth, tunnel->tunnelName);
#endif

	if ((ret = check_tree_excistance(tunnel)) != 0) {
		if (ret > 0) {
#ifdef _VERBOSE
			printk("opening existing tunnel! %d\n", ret);
#endif
			return ret;
		}
#ifdef _VERBOSE
		printk("tunnel could not be created, errorNo:%d\n", tunnel->pathdepth);
#endif
		return -EFAULT;
	}

	if ((tunnel->entry = buildDirectoryTree(tunnel)) != NULL) {
		if (tun_init->readBufferSize <= 0 || tun_init->writeBufferSize <= 0) {
#ifdef _VERBOSE
			printk("Error creating tunnel buffer malloc sizes too small %d || %d\n", tun_init->readBufferSize, tun_init->writeBufferSize);
#endif
			cleanUpDirectoryTree(tunnel); //cleanup tunnel tree
			return -EFAULT;
		}
		//do a malloc
		if ((tunnel->readBuffer = kmalloc(tun_init->readBufferSize + tun_init->writeBufferSize, __GFP_NORETRY | __GFP_WAIT)) == NULL) {
#ifdef _VERBOSE
			printk("Error creating tunnel buffer insufficient memory\n");
#endif
			cleanUpDirectoryTree(tunnel); //cleanup tunnel tree
			return -EFAULT;
		}
		tunnel->readBufferSize = tun_init->readBufferSize;
		tunnel->writeBuffer = tunnel->readBuffer + tunnel->readBufferSize;
		tunnel->writeBufferSize = tun_init->writeBufferSize;

#ifdef _VERBOSE
		printk("created tunnel endpoint '%s' pid %d\n", tun_init->tunnelName, tunnel->pid);
#endif
		return 0;
	}
#ifdef _VERBOSE
	else {
		printk("Error creating tunnel %s\n", tun_init->tunnelName);
	}
#endif
	return -EFAULT;
}

static int create_new_tunnel_allocation(struct _tunnelInit * tun_init) {
	int x, ret = -EFAULT;
	struct _proxyTunnel *tunnel;

	for (x = 0; x < MAX_TUNNELS; x++) {
		if (proxy.tunnels[x] == NULL) {
			if ((tunnel = kmalloc(sizeof(struct _proxyTunnel), __GFP_NORETRY | __GFP_WAIT)) != NULL) {
				if ((ret = create_new_tunnel(tun_init, tunnel)) == 0) {
#ifdef _VERBOSE
					printk("create_new_tunnel x=%d '%s'\n", x, tun_init->tunnelName);
#endif
					proxy.tunnels[x] = tunnel;
					return x + 1;
				} else {
					if (ret > 0) {
						kfree(tunnel);
						return ret;
					}
				}
				kfree(tunnel);
			} else {
#ifdef _VERBOSE
				printk("create_new_tunnel failed '%s', kmalloc\n", tun_init->tunnelName);
#endif
			}
		}
	}
#ifdef _VERBOSE
	printk("create_new_tunnel failed '%s', no slots\n", tun_init->tunnelName);
#endif
	return -ret;
}

struct _proxyTunnel * getTunnelWithId(int externalId) {
	externalId--;
	if (externalId >= 0 && externalId < MAX_TUNNELS) {
		return proxy.tunnels[externalId];
	}
	return NULL;
}

int destroy_tunnel(int tunnelId) {
	struct _proxyTunnel *tunnel;

	if (tunnelId >= 0 && tunnelId < MAX_TUNNELS) {
		if (proxy.tunnels[tunnelId] != NULL) {
			tunnel = proxy.tunnels[tunnelId];
			kfree(tunnel->readBuffer); //only readBuffer,writebuffer is a part of readBuffer
			cleanUpDirectoryTree(tunnel);
			kfree(tunnel); //free the rest
			proxy.tunnels[tunnelId] = NULL;
		}
	}
	return -EFAULT;
}

/* proxy side of the interface*/
static long userproxy_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
	struct _tunnelInit tun_init;
	long lReturn = -EFAULT;

#ifdef _VERBOSE2
	printk("received IOCTL %d\n", cmd);
#endif
	down(&proxy.sem);
	switch (cmd) {
	case UP_CREATE_TUNNEL:
		if (access_ok(VERIFY_READ, arg, sizeof(struct _tunnelInit))) {
			copy_from_user(&tun_init, (char *) arg, sizeof(tun_init));
#ifdef _VERBOSE2
			printk("requested tunnel %s create\n", tun_init.tunnelName);
#endif
			lReturn = create_new_tunnel_allocation(&tun_init);
		}
		break;

	case UP_DESTROY_TUNNEL:
#ifdef _VERBOSE2
		printk("requested tunnel %ld destruction\n", arg);
#endif
		if (arg > 0 && arg <= MAX_TUNNELS) {
			lReturn = destroy_tunnel(arg - 1);
		}
		break;

	default:
		printk("received unknow IOCTL %d\n", cmd);
		break;
	}
	up(&proxy.sem);
	return lReturn;
}

static ssize_t userproxy_read(struct file *file, char *buffer, size_t length, loff_t *off) {
	struct _proxyTunnel * tunnel;
	int len = 0, offset = -1;

	if (off) {
		offset = *off;
	}

	/*  Can't seek (pwrite) on this device  */
	/*
	 if (off != &file->f_pos){
	 printk("FileSeek error\n");
	 return -ESPIPE;
	 }*/

	if (down_interruptible(&proxy.sem)) {
		return -ERESTARTSYS;
	}

	/*find a non empty tunnel*/
	while ((tunnel = findTunnelWithData()) == NULL) {
		up(&proxy.sem); /* release the lock */
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(proxy.inq, ((tunnel = findTunnelWithData()) != NULL)))
			return -ERESTARTSYS; /* signal: tell the fs layer to handle it */

#ifdef _VERBOSE2
		printk("userproxy_read len=%d off=%d from %s\n", (int) length, offset, tunnel->tunnelName);
#endif

		/* otherwise loop, but first re-acquire the lock */
		if (down_interruptible(&proxy.sem))
			return -ERESTARTSYS;
	}
	/*do a size check*/
	len = length;
	if (tunnel->writeOffset + 1 < length) {
		len = tunnel->writeOffset - 1;
	}

#ifdef _VERBOSE2
	printk("we have tunnel with len: %d,%d\n", len, tunnel->writeOffset);
#endif

	/*if size > 0 do a userspace copy*/
	if (len > 0) {
#ifdef _VERBOSE2
		printk("tunnel %d, %s, holds %d bytes\n", (proxy.lastWriteTunnel + 1), tunnel->tunnelName, tunnel->writeOffset);
#endif

		buffer[0] = (uint8_t) proxy.lastWriteTunnel + 1;
		if (copy_to_user(buffer + 1, tunnel->writeBuffer, len))
			return -EFAULT;
		tunnel->writeOffset = 0;
		len++;
	}
	up(&proxy.sem);
	return len;
	//}
	//return 0;
	//up(&proxy.sem);
	//return -EAGAIN;
}

static int WriteTunnel(int externalId, const char *data, int length, int External) {
	int len = 0;
	struct _proxyTunnel * tunnel;

	if ((tunnel = getTunnelWithId(externalId)) != NULL) {
		len = length;
		if (length > tunnel->readBufferSize) {
			len = tunnel->readBufferSize;
		}
		/*if size > 0 do a userspace copy*/
		if (len > 0) {
			if (External) {
				if (copy_from_user(tunnel->readBuffer, data, len)) {
					len = 0;
				}
			} else {
				memcpy(tunnel->readBuffer, data, len);
			}
			if (len > 0) {
				tunnel->readOffset = len + 1;
			}
		}
	}
	return len;
}

static ssize_t userproxy_write(struct file *file, const char *data, size_t length, loff_t *off) {
	int len = 0;

	/*  Can't seek (pwrite) on this device  */
	/*
	 if (off != &file->f_pos)
	 return -ESPIPE;
	 */
	if (down_interruptible(&proxy.sem))
		return -ERESTARTSYS;

	if (length > 0) {
		if ((len = WriteTunnel(data[0], data + 1, length - 1, 1)) > 0) {
			len++;		// Compensate first 'tunnel' byte
		}
	}
	up(&proxy.sem);

	return len;
}

static unsigned int userproxy_poll(struct file *file, poll_table * wait) {
	unsigned int mask = 0;
	struct _proxyTunnel * tunnel;

	down(&proxy.sem);
	poll_wait(file, &proxy.inq, wait);
	poll_wait(file, &proxy.outq, wait);

	if ((tunnel = findTunnelWithData()) != NULL) {
		if (tunnel->writeOffset > 0) {
#ifdef _VERBOSE2
			printk("WE HAZ DATA!\n");
#endif
			mask |= POLLIN | POLLRDNORM;
		}
	}
	if (checkTunnelWritePossibility() > 0) {
		mask |= POLLOUT | POLLWRNORM;
	}
	up(&proxy.sem);
	return mask;
}

static int userproxy_open(struct inode *inode, struct file *file) {
	pid_t pid = getUserPid();

	if (down_interruptible(&proxy.sem)) {
		return -ERESTARTSYS;
	}
#ifdef _VERBOSE2
	printk( "opened from user pid %d\n", pid);
#endif
	proxy.driverUsage++;
	up(&proxy.sem);
	return 0;
}

static int userproxy_release(struct inode *inode, struct file *file) {
	int tunnelId;
	pid_t pid = getUserPid();

	down(&proxy.sem);
#ifdef _VERBOSE2
	printk( "release from user pid %d\n", pid);
#endif
	for (tunnelId = 0; tunnelId < MAX_TUNNELS; tunnelId++) {
		//tunnel is open and is NOT this tunnel...
		if (proxy.tunnels[tunnelId] != NULL) {
			if (proxy.tunnels[tunnelId]->pid == pid) {
				destroy_tunnel(tunnelId);
			}
		}
	}

	proxy.driverUsage--;
	up(&proxy.sem);
	return 0;
}

/*****************************************************************************/
/*
 *	Exported file operations structure for driver...
 */
static struct file_operations kp_io_fops =
{ owner: THIS_MODULE, read: userproxy_read, unlocked_ioctl: userproxy_ioctl, compat_ioctl: userproxy_ioctl, write: userproxy_write, poll: userproxy_poll, open: userproxy_open, release
		: userproxy_release,
};

//register misc device
static struct miscdevice kp_io_dev =
{ OUTPUTS_MINOR, "userproxy", &kp_io_fops };

static int __init userProxy_init(void) {
	int reg_status = 0;
	struct _tunnelInit tun_init;
	char szVersion[128];

	// sprintf(szVersion, "%d - %s %s", UP_MODULE_VERSION, __DATE__, __TIME__); avoid error
	sprintf(szVersion, "%d", UP_MODULE_VERSION);

	memset(&proxy, 0, sizeof(proxy));//make everything 0

	if ((reg_status = misc_register(&kp_io_dev)) < 0) {
		printk(KERN_ERR DEF_MODULENAME": devfs_register_chrdev() failed code %d\n", reg_status);
		return -1;
	}

	init_waitqueue_head(&proxy.inq);
	init_waitqueue_head(&proxy.outq);
	sema_init(&proxy.sem, 1);

	/* Create own version */
	memset(&tun_init, 0, sizeof(tun_init));
	strcpy(tun_init.tunnelName, "userProxy/version");
	tun_init.writeBufferSize = 128;
	tun_init.readBufferSize = 32;
	if ((VersionTunnel = create_new_tunnel_allocation(&tun_init)) > 0) {
		WriteTunnel(VersionTunnel, szVersion, strlen(szVersion), 0);
	}

	printk(DEF_MODULENAME" driver loaded (C)2016 Keyprocessor - v %s\n", szVersion);
	return 0;
}

static void __exit userProxy_exit(void) {
	int reg_status = 0, x;

	if (VersionTunnel) {
		destroy_tunnel(VersionTunnel - 1);
	}
	for (x = 0; x < MAX_TUNNELS; x++) {
		destroy_tunnel(x);
	}

	if ((reg_status = misc_deregister(&kp_io_dev)) < 0) {
		printk(KERN_ERR DEF_MODULENAME": devfs_unregister_chrdev() failed code %d\n", reg_status);
	}
	printk(KERN_INFO DEF_MODULENAME " driver unloaded\n");
}

MODULE_AUTHOR("Sebastiaan Pierrot");
MODULE_DESCRIPTION("Keyprocessor " DEF_MODULENAME " driver");
MODULE_LICENSE("GPL");

module_init( userProxy_init);
module_exit( userProxy_exit);

