//=============================================================================
//
//             --- CAEN SpA - Computing Systems Division ---
//
//  a1303.c
//
//  This driver controls the CAEN A1303 CaeNet boards. It supports multiple 
//  boards in the same machine.
//
//  February  2002 :   Created.
//  May       2002 :   Added version 2.2 compatibility.
//  Jan       2006 :   Added version 2.6 compatibility.
//
//=============================================================================

#define VERSION(ver,rel,seq) (((ver)<<16) | ((rel)<<8) | (seq))

static char Version[] = "1.2 Feb. 2007";

#include <linux/version.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pagemap.h>

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/delay.h>                 // udelay
#include <linux/sched.h>


#include "../include/a1303.h"


MODULE_DESCRIPTION("CAEN A1303 PCI CaeNet Board Driver");
MODULE_AUTHOR("Stefano Coluccini, s.coluccini@caen.it");
MODULE_LICENSE("GPL");


#define MAX_MINOR 	    8
#define PFX "a1303: "

//----------------------------------------------------------------------------
// Function prototypes
//----------------------------------------------------------------------------
static int a1303_open(struct inode *, struct file *);
static int a1303_release(struct inode *, struct file *);
static ssize_t a1303_read(struct file *,char *, size_t, loff_t *);
static ssize_t a1303_write(struct file *,const char *, size_t, loff_t *);
static unsigned int a1303_poll(struct file *, poll_table *);
static long a1303_ioctl(struct file *, unsigned int, 
		       unsigned long);
static int a1303_procinfo(char *, char **, off_t, int, int *,void *);

//----------------------------------------------------------------------------
// Types
//----------------------------------------------------------------------------
struct a1303_state {
	unsigned char *		baseaddr;
	unsigned long		phys;
	int			irq;
	int			minor;
	wait_queue_head_t	wait_list;
	int			timeout;
	int			rx_ready;
	int			buff_count;
	unsigned char		buff[MAX_LENGTH_FIFO];

	spinlock_t 		lock;

	unsigned int 		reads;
	unsigned int 		writes;
	unsigned int 		ioctls;

	/* we keep a1303 cards in a linked list */
	struct 			a1303_state *next;
};

// Status Vars

static int opened[MAX_MINOR + 1];

static int a1303_major = 0;

static struct a1303_state *devs;

static struct proc_dir_entry *a1303_procdir;

static struct file_operations a1303_fops = 
{
	.owner=    THIS_MODULE, 
	.open=     a1303_open,
	.release=  a1303_release,
	.read=     a1303_read,
	.write=    a1303_write,
	.poll=     a1303_poll,  
	.unlocked_ioctl=    a1303_ioctl
};

//-----------------------------------------------------------------------------
// Function   : a1303_reset
// Inputs     : struct a1303_state* s
// Outputs    : int
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static int a1303_reset(struct a1303_state* s)
{
	int i = 0;
 
	writeb(0, s->baseaddr + A1303_RESET);
	do {
		udelay(5L);
 		i++;
	} while( readb(s->baseaddr + A1303_REG) != IDLE && i != TIME_OUT );
	return ( (i == TIME_OUT) ? E_NO_A1303 : TUTTOK );
}

//-----------------------------------------------------------------------------
// Function   : a1303_led
// Inputs     : struct a1303_state* s
// Outputs    : int
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static int a1303_led(struct a1303_state* s)
{
	int i = 0;
 
	writeb(0, s->baseaddr + A1303_LED);
	do {
		udelay(5L);
 		i++;
	} while( readb(s->baseaddr + A1303_REG) != IDLE && i != TIME_OUT );
	return ( (i == TIME_OUT) ? E_NO_A1303 : TUTTOK );
}

//-----------------------------------------------------------------------------
// Function   : a1303_procinfo
// Inputs     : char *buf, char **start, off_t fpos, int lenght, int *eof, 
//              void *data
// Outputs    : int
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static int a1303_procinfo(char *buf, char **start, off_t fpos, int lenght, 
			  int *eof, void *data)
{
	char *p;
	struct a1303_state* s = devs;
	int i = 0;

	p = buf;
	p += sprintf(p,"CAEN A1303 driver %s\n\n",Version);

	while( s ) {
		p += sprintf(p,"  CAEN A1303 CaeNet Board found.");
		p += sprintf(p,"  Physical address = %08X\n",(int)s->phys);
		p += sprintf(p,"  Virtual address = %08X\n",(int)s->baseaddr);
		p += sprintf(p,"  IRQ line = %d\n",(int)s->irq);
		p += sprintf(p,"  Minor number = %d\n",(int)s->minor);
		p += sprintf(p,"  Reads = %i  Writes = %i  Ioctls = %i\n",
			     s->reads, s->writes, s->ioctls);
		p += sprintf(p,"  Board status = %08X\n", 
			     readb(s->baseaddr + A1303_REG));

		p += sprintf(p,"\n");  
		
		s = s->next;
		i++;
	}

	p += sprintf(p,"%d CAEN A1303 board(s) found.\n", i);  

	*eof = 1;
	return p - buf;
}

//-----------------------------------------------------------------------------
// Function   : register_proc
// Inputs     : void
// Outputs    : void
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static void register_proc(void)
{
	a1303_procdir = create_proc_entry("a1303", S_IFREG | S_IRUGO, 0);
	a1303_procdir->read_proc = a1303_procinfo;
}

//-----------------------------------------------------------------------------
// Function   : unregister_proc
// Inputs     : void
// Outputs    : void
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static void unregister_proc(void)
{
	 remove_proc_entry("a1303",0);
}

//-----------------------------------------------------------------------------
// Function   : a1303_poll
// Inputs     : struct file* file, poll_table* wait
// Outputs    : unsigned int
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static unsigned int a1303_poll(struct file* file, poll_table* wait)
{
	struct a1303_state *s = (struct a1303_state *)file->private_data;

	if( s->rx_ready )
		return POLLIN | POLLRDNORM;
	poll_wait(file, &s->wait_list, wait);
	if( s->rx_ready )
		return POLLIN | POLLRDNORM;
	return 0;
}

//-----------------------------------------------------------------------------
// Function   : a1303_open
// Inputs     : struct inode *inode,struct file *file
// Outputs    : int
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static int a1303_open(struct inode *inode,struct file *file)
{
	unsigned int minor = MINOR(inode->i_rdev);
	struct a1303_state *s = devs;

//	If minor is out of range, return an error 
	if (minor > MAX_MINOR) {
		return(-ENODEV);
	}

// Search for the device linked to the minor
	while (s && s->minor != minor)
		s = s->next;
	if (!s)
		return -ENODEV;

	if (!opened[minor]) {
		opened[minor] = 1;
		file->private_data = s;
#if LINUX_VERSION_CODE < VERSION(2,5,0)
		MOD_INC_USE_COUNT;
#endif		
		return(0);
	} else 
		return(-EBUSY);
}

//-----------------------------------------------------------------------------
// Function   : a1303_release
// Inputs     : struct inode *inode,struct file *file
// Outputs    : int
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static int a1303_release(struct inode *inode,struct file *file)
{
	unsigned int minor = MINOR(inode->i_rdev);

	opened[minor]--;
#if LINUX_VERSION_CODE < VERSION(2,5,0)
	MOD_DEC_USE_COUNT;
#endif	
	return 0;
}

//-----------------------------------------------------------------------------
// Function   : a1303_read
// Inputs     : struct file *file, char *buf, size_t count, loff_t *ppos
// Outputs    : ssize_t
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static ssize_t a1303_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	// unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);  
	struct a1303_state *s = (struct a1303_state *)file->private_data;
	int ret;
	s->reads++;

	//printk(KERN_INFO PFX "A1303 R-0\n");
	ret = wait_event_interruptible_timeout(s->wait_list, (s->rx_ready), s->timeout);
	if( ret == 0 ) {
               ret = -ETIMEDOUT;
		          goto err_read;
        } else if( !s->rx_ready ) {
               ret = -EINTR;
	       goto err_read;
        } else ret = 0;
	//printk(KERN_INFO PFX "A1303 R-1\n");
	s->rx_ready = 0;
	if( (ret = access_ok(VERIFY_WRITE, (void *)buf, s->buff_count)) != 1 )
		goto err_read;
	if( copy_to_user(buf, &s->buff, s->buff_count) > 0) {
		ret = -EFAULT;
		goto err_read;
	}

	ret = s->buff_count;
	s->buff_count = 0;
	//printk(KERN_INFO PFX "A1303 Readed\n");

err_read:
	return ret;
	
}

//-----------------------------------------------------------------------------
// Function   : a1303_write
// Inputs     : struct file *file, char *buf, size_t count, loff_t *ppos
// Outputs    : ssize_t
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static ssize_t a1303_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	// unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct a1303_state *s = (struct a1303_state *)file->private_data;
	int ret, i, j;
	unsigned long flags;
	
	s->writes++;
	if( (ret = access_ok(VERIFY_READ, buf, count)) != 1 ) 
  		return ret;
	
	

	if( copy_from_user(&s->buff, buf, count) > 0) {
		ret = -EFAULT;
		goto err_write;
	}
	
	spin_lock_irqsave(&s->lock, flags);

	/* Reset A1303 Module */
	if( a1303_reset(s) != 0 ) {
		printk(KERN_ERR "There are problems in Caenet Board\n");
		ret = -1;
		spin_unlock_irqrestore(&s->lock,flags);
		goto err_write;
	}
   	//printk(KERN_INFO PFX "A1303 W-1\n");
	s->rx_ready = 0;      // Added Jan 2000  

	/* Fill Transmit buffer */
	for( i = 0; i < count; i++ )
		writeb(s->buff[i], s->baseaddr+A1303_FIFO);

	/* Send data */
	writeb(0, s->baseaddr+A1303_REG); 
	//printk(KERN_INFO PFX "A1303 W-2\n");
	/* Wait until transmission is done and, at the end, reset Interrupt 1) */
	i = 0;
	while( (readb(s->baseaddr+A1303_REG) & TXEFF) && i<TIME_OUT_TX ) {
		udelay(5L);
		i++;
	}
	//printk(KERN_INFO PFX "A1303 W-3\n");
	j = readb(s->baseaddr+A1303_INTR);         // Jan 2000 - Dummy read

	ret = (i == TIME_OUT_TX) ? -ETIMEDOUT : count;
	//printk(KERN_INFO PFX "A1303 WRITE\n");
	spin_unlock_irqrestore(&s->lock,flags);
err_write:
	
	return ret;
}

//-----------------------------------------------------------------------------
// Function   : a1303_ioctl
// Inputs     : struct inode *inode,struct file *file,unsigned int cmd, unsigned long arg
// Outputs    : int
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static long a1303_ioctl(struct file *file,unsigned int cmd, unsigned long arg)
{
	// unsigned int minor = MINOR(inode->i_rdev);
	struct a1303_state *s = (struct a1303_state *)file->private_data;

	s->ioctls++;
	switch (cmd) {
		case A1303_IOCTL_RESET:
			a1303_reset(s);
		break;

/* It is the time which the read function waits before than signalling  */
/* that the addressed slave module is not present.                      */
/* The following relation holds: timeout (in sec.) = time_out / 100     */
		case A1303_IOCTL_TIMEOUT:
			s->timeout = arg;
		break;        

		case A1303_IOCTL_LED:
			a1303_led(s);
		break;        
	}
	return(0);
}

//-----------------------------------------------------------------------------
// Function   : irq_handler
// Inputs     : int irq, void *dev_id, struct pt_regs *regs
// Outputs    : void
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
// Rev 1.1
static irqreturn_t a1303_interrupt(int irq, void *dev_id)
{
	struct a1303_state *s = (struct a1303_state *)dev_id;
	unsigned char a1303_stat = 0;

	a1303_stat = readb(s->baseaddr + A1303_REG);

	if( !(a1303_stat & RXEFF) ) {                    /* A packet has come */
		//printk(KERN_INFO PFX "interrupt A1303 \n");
		while( readb(s->baseaddr+A1303_REG) & RXFEM ) {
			 /* Data available */
			s->buff[s->buff_count] = readb(s->baseaddr+A1303_FIFO);
			s->buff_count++;
		}
		writeb(0, s->baseaddr + A1303_RESET);  /* Reset Interrupt 3)  */
		s->rx_ready = 1;                 /* Flag for the read routine */
		wake_up_interruptible(&s->wait_list);
	}
//Rev 1.1	
	return IRQ_HANDLED;
	
}

//-----------------------------------------------------------------------------
// Function   : initialize_board
// Inputs     : struct pci_dev *pcidev, int index
// Outputs    : void
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static void initialize_board(struct pci_dev *pcidev, int index)
{
	struct a1303_state *s;
	
	if (pci_enable_device(pcidev))
		return;
	if (pcidev->irq == 0)
		return;
	s = kmalloc(sizeof(*s), GFP_KERNEL);
	if (!s) {
		printk(KERN_WARNING "A1303: out of memory\n");
		return;
	}

	memset(s, 0, sizeof(struct a1303_state));
	init_waitqueue_head(&s->wait_list);
	spin_lock_init(&s->lock);
	s->phys = pci_resource_start(pcidev, 2);
	if (s->phys == 0)
		return;
	if (!request_mem_region(s->phys, 16, "a1303")) {
		printk(KERN_ERR PFX "io mem %#lx-%#lx in use\n", 
		       s->phys, s->phys+16-1);
		goto err_region5;
	}
	s->irq = pcidev->irq;
	/* request irq */
	if (request_irq(s->irq, a1303_interrupt, IRQF_SHARED, "a1303", s)) {
		printk(KERN_ERR "A1303: irq %u in use\n", s->irq);
		goto err_irq;
	}
	s->baseaddr = (char *)ioremap(s->phys,16);
	printk(KERN_INFO PFX "found A1303 adapter at io %#06lx irq %u\n",
	       s->phys, s->irq);
	
	printk("  CAEN A1303 Loaded.\n"); 

	register_proc();
	
	s->minor = index;

	/* queue it for later freeing */
	s->next = devs;
	devs = s;
	return;

err_irq:
	iounmap(s->baseaddr);
	release_mem_region(s->phys, 16);

err_region5:
	kfree(s);

	return;
}


//-----------------------------------------------------------------------------
// Function   : init_a1303
// Inputs     : void
// Outputs    : int
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static int __init init_a1303(void)
{
	struct pci_dev *pcidev = NULL;
	int index = 0;
/*
  from 2.6 on pci_present is obsolete:
  From http://www.linux-m32r.org/lxr/http/source/Documentation/pci.txt?v=2.6.10 ...
' ... Since ages, you don't need to test presence
  of PCI subsystem when trying to talk to it.
  If it's not there, the list of PCI devices
  is empty and all functions for searching for
  devices just return NULL. '
*/
	
	printk(KERN_INFO "CAEN A1303 Caenet controller driver %s\n", Version);
	printk(KERN_INFO "  Copyright 1999-2002, CAEN SpA\n");

	/* register device */
	a1303_major = register_chrdev(0, "a1303", &a1303_fops);
	if ( a1303_major < 0 ) {
		printk("  Error getting Major Number for Drivers\n");
		return -ENODEV;
	}

	while (index < MAX_MINOR && (
	       (pcidev = pci_get_subsys(PCI_VENDOR_ID_PLX, 
					 PCI_DEVICE_ID_PLX_9030,
					 PCI_VENDOR_ID_PLX, 
					 PCI_SUBDEVICE_ID_CAEN_A1303, 
					 pcidev)))) { 
		initialize_board(pcidev, index);
		index++;
	}
	printk(KERN_INFO "  CAEN A1303: %d device(s) found.\n", index);

	return 0;
}

//-----------------------------------------------------------------------------
// Function   : cleanup_a1303
// Inputs     : void
// Outputs    : void
// Description: 
// Remarks    : 
// History    : 
//-----------------------------------------------------------------------------
static void __exit cleanup_a1303(void)
{
	struct a1303_state *s;

	while ((s = devs)) {
		devs = devs->next;

		free_irq(s->irq, s);

		release_mem_region(s->phys, 16);
		kfree(s);
	}
	unregister_proc();
	unregister_chrdev(a1303_major, "a1303");
	printk(KERN_INFO "CAEN A1303: unloading.\n");
}

module_init(init_a1303);
module_exit(cleanup_a1303);

