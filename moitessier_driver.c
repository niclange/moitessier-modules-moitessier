/*
    Linux device driver for the Moitessier HAT
    Copyright (C) 2018  Thomas POMS <hwsw.development@gmail.com>
    
    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

/*

    Reading from the device
    ***********************
    
    (1)
    reading from the device using the SPI device file:
    target> head -c 49 /dev/moitessier.spi
    target> cat /dev/moitessier.spi
    
    (2)
    reading from the device using the SPI to PTS bridge:
    target> sudo apt-get install socat
    target> socat -d -d pty,raw,echo=0 pty,raw,echo=0
    
    Output:
    2017/11/14 19:00:15 socat[1924] N PTY is /dev/pts/5
    2017/11/14 19:00:15 socat[1924] N PTY is /dev/pts/6
    2017/11/14 19:00:15 socat[1924] N starting data transfer loop with FDs [5,5] and [7,7]
    
    Open your navigation program (e.g OpenCPN) and define /dev/pts/6 (depending of socat output) as input interface and transfer data
    from the SPI device to the PTS device.
    target> cat /dev/moitessier.spi > /dev/pts/5
    
    (3)
    read from the device using the TTY device file:
    target> cat /dev/moitessier.tty
    
    
    Compiling the device tree
    *************************
    
    host> make -j2 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- dtbs
    host> sudo scp arch/arm/boot/dts/*.dtb pi@10.0.0.55:/media/pi/boot
    
    Required device tree entry in bcm2710-rpi-3-b.dts: 
        &spi0 {
            /delete-node/ spidev@0;
            moitessier0: moitessier@0{
        		compatible = "rooco,moitessier";
        		#address-cells = <1>;
        		#size-cells = <0>;
                reg = <0>;
        		spi-max-frequency = <12000000>;
                spi-min-frequency = <1000000>;
                status = "okay";

                boot: boot{
                    gpios = <&gpio 17 GPIO_ACTIVE_LOW>;
                    rooco,name = "IMC BOOT";
                    rooco,direction = "output";
                    status = "okay";   
                };

                reset: reset{
                    gpios = <&gpio 18 GPIO_ACTIVE_LOW>;
                    rooco,name = "IMC RESET";
                    rooco,direction = "output";
                    status = "okay";   
                };

                imc_req: imc_req{
                    gpios = <&gpio 24 GPIO_ACTIVE_LOW>, <&gpio 22 GPIO_ACTIVE_LOW>;
                    rooco,name = "IMC REQ";
                    rooco,direction = "output";
                    status = "okay";   
                };

                imc_irq: imc_irq{
                    gpios = <&gpio 23 GPIO_ACTIVE_LOW>, <&gpio 27 GPIO_ACTIVE_LOW>;
                    rooco,name = "IMC IRQ";
                    rooco,direction = "input";
                    status = "okay";   
                };
        	};
        };
        
        
        Compiling the driver
        ********************
        
        Makefile:
        obj-m += moitessier.o
        moitessier-objs := moitessier_driver.o crc.o header.o
        PWD := $(shell pwd)
        all:
        	make -C $(KDIR) M=$(PWD) clean
        	reset
        	make -C $(KDIR) M=$(PWD) modules
        clean:
        	make -C $(KDIR) M=$(PWD) clean

        
        
        host> make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- all KDIR=../kernel/linux


*/

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////// DRIVER CONFIGURATION
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define USE_TTY                         /* Enables the SPI to TTY bridge. Data can be read from either /dev/moitessier.spi or /dev/moitessier.tty. */
#define USE_THREAD                      /* Enables a kernel thread for reading data via SPI, otherwise data is read in a tasklet.
                                           Tasklet implementation not tested right now. */
//#define USE_MSG_CNT_TTY               /* a message counter will be send on the tty interface */
#define MIN_DATA_LENGTH         8       /* The minimum payload size received from the HAT. The minimum payload length of a request is 8 bytes. */
#define USE_STATISTICS                  /* collecting statistics enabled */
#define NUM_RCV_CHANNELS        2       /* number of channels per AIS receiver, this value should be 2 */
#define NUM_RCV                 2       /* number of AIS receivers, this value should be 2 */
    
#define DEVICE_NAME_SPI         "moitessier.spi"           /* the name of the device using the SPI interface --> /dev/moitessier.spi */
#define DEVICE_NAME_CTRL        "moitessier.ctrl"          /* the name of the device to control this driver --> /dev/moitessier.ctrl */
#if defined(USE_TTY)
#define DEVICE_NAME_TTY         "moitessier.tty"           /* the name of the device using the TTY interface --> /dev/moitessier.tty */
#endif /* USE_TTY */

#define SUPPORT_KEEP_ALIVE               
#define SUPPORT_SHUTDOWN                                           

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////// INCLUDES
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/kthread.h>
#include <linux/jiffies.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <asm/uaccess.h>  
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/stddef.h>
#include <linux/reboot.h>

#include <linux/version.h>

//#if !defined(KERNEL_VERSION_ALT)
#define KERNEL_VERSION_ALT(a,b,c) (((a) << 16) + ((b) << 8) + (c))
//#endif /* KERNEL_VERSION_ALT */

#if LINUX_VERSION_CODE >= KERNEL_VERSION_ALT(4, 11, 0)
#include <linux/signal.h>
#include <linux/sched/signal.h>
#endif

#if defined(USE_TTY)
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#endif /* USE_TTY */

#include "header.h"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////// DEFINES
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifdef __GNUC__
#define FUNC_NOT_USED __attribute__ ((unused))
#else
#define FUNC_NOT_USED
#endif

#define DIM(x)  (sizeof(x)/sizeof(*(x)))
#define DIM_ELEMENT(type, element) sizeof(((type *)0)->element)

/* defines the logic level for the request signals */
#define REQ_LEVEL_ACTIVE            1
#define REQ_LEVEL_INACTIVE          0

/* defines the logic level for the reset signal */
#define RESET_LEVEL_ACTIVE          0
#define RESET_LEVEL_INACTIVE        1


#define BUF_SIZE    1024                /* The buffer size used for SPI DMA transmissions. Must be equal to the size of the buffer used
                                           on the HAT. */

#define FIFO_SIZE    4096               /* Defines the size in bytes of the FIFO used to buffer received data from the HAT. The FIFO
                                           is read and reset by a read(...) operation from the user on the device file specified by DEVICE_NAME_SPI. 
                                           It must be equal or larger than BUF_SIZE. */

/* defines the debug level --> the higher the level the more kernel messages will be written */
#define LEVEL_NONE          14
#define LEVEL_DEBUG         13
#define LEVEL_INFO          12
#define LEVEL_WARNING       11
#define LEVEL_CRITICAL      10
#define LEVEL_DEBUG_TTY     1
#define LEVEL_DEBUG_SPI     0

int DEBUG_LEVEL = LEVEL_WARNING;

/* creates the file /sys/module/<FILE_NAME>/parameters/DEBUG_LEVEL */
/* The parameter DEBUG_LEVEL can be written and read during runtime. Keep in mind, that this can only be done
   as root user.
   target> su
   
   To set the debug level for example to LEVEL_CRITICAL, execute the following command:
   target> echo 10 > /sys/module/<FILE_NAME>/parameters/DEBUG_LEVEL
   
   DEBUG_LEVEL might be directly set during module loading:
   target> insmod <FILE_NAME>.ko DEBUG_LEVEL=10 
   
   Alternatively you could also change the parameters with sudo rights, but only if using tee.
   target> echo 10 | sudo tee /sys/module/<FILE_NAME>/parameters/DEBUG_LEVEL
   
   */  
module_param(DEBUG_LEVEL, int, 0664);
/* The description of the module parameters can be shown using modinfo <FILE_NAME>.ko */
MODULE_PARM_DESC(DEBUG_LEVEL, "Defines the level of debugging. \
                               \n\t\t\t14...no debug messages \
                               \n\t\t\t13...debug, info, warning, critical messages \
                               \n\t\t\t12...info, warning, critical messages \
                               \n\t\t\t11...warning, critical messages \
                               \n\t\t\t10...critical messages\n");     

/* Defines the debounce timeout for the GPIOs. This is only required, if the GPIOs are connected
   to push buttons. This is a leftover from the evaluation phase. */                              
long DEBOUNCE_MS = 0;
module_param(DEBOUNCE_MS, long, 0664);
MODULE_PARM_DESC(DEBOUNCE_MS, "GPIO input debounce time in ms");  

long KEEP_ALIVE_TIMEOUT_LONG_MS = 2000;
module_param(KEEP_ALIVE_TIMEOUT_LONG_MS, long, 0664);
MODULE_PARM_DESC(KEEP_ALIVE_TIMEOUT_LONG_MS, "Keep alive timeout (long value) in sec");  

long KEEP_ALIVE_TIMEOUT_SHORT_MS = 4000;
module_param(KEEP_ALIVE_TIMEOUT_SHORT_MS, long, 0664);
MODULE_PARM_DESC(KEEP_ALIVE_TIMEOUT_SHORT_MS, "Keep alive timeout (short value) in sec");  

int USE_REQ_FOR_RESET = 0;
module_param(USE_REQ_FOR_RESET, int, 0664);
MODULE_PARM_DESC(USE_REQ_FOR_RESET, "Set to 1, if the REQ signal should be used instead of the dedicated RESET signal to reset the HAT");  

int USE_SHUTDOWN_BUTTON = 1;
module_param(USE_SHUTDOWN_BUTTON, int, 0664);
MODULE_PARM_DESC(USE_SHUTDOWN_BUTTON, "Set to 0, if the functionality of the shutdown button should be disabled");  

int TEST_MODE_SHUTDOWN_BUTTON = 0;
module_param(TEST_MODE_SHUTDOWN_BUTTON, int, 0664);
MODULE_PARM_DESC(TEST_MODE_SHUTDOWN_BUTTON, "Set to 1, you want to test the shutdown button without actual shutdown.");  

/* If enabled, the driver checks if a keep alive message is received from the HAT periodically.
   If the keep alive is missing, the driver resets the HAT. */
int USE_KEEP_ALIVE = 1;
module_param(USE_KEEP_ALIVE, int, 0664);
MODULE_PARM_DESC(USE_KEEP_ALIVE, "Set to 0, if keep alive messages should not be proceed. Keep in mind, that the system might fail and can not detect system hangs. Use for testing purpose only.");  

long SHUTDOWN_DELAY_MS = 0;
module_param(SHUTDOWN_DELAY_MS, long, 0664);
MODULE_PARM_DESC(SHUTDOWN_DELAY_MS, "Delay (milliseconds) before shutting down the system, if the shutdown button is pressed");  

int DO_SHUTDOWN = 0;
module_param(DO_SHUTDOWN, int, 0664);
MODULE_PARM_DESC(DO_SHUTDOWN, "If set, the Moitessier HAT requests a system shutdown.");  

/* error codes */
#define ERR_NO                  0        
#define ERR_NO_DAT_AVAIL        1
#define ERR_SIG_CHANCEL         2
#define ERR_EOF                 3

/* IOCTL commands */
#define IOC_MAGIC 'N'
#define IOCTL_GET_STATISTICS        _IO(IOC_MAGIC,0)
#define IOCTL_GET_INFO              _IO(IOC_MAGIC,1)
#define IOCTL_RESET_HAT             _IO(IOC_MAGIC,2)
#define IOCTL_RESET_STATISTICS      _IO(IOC_MAGIC,3)
#define IOCTL_GNSS                  _IO(IOC_MAGIC,4)
#define IOCTL_CONFIG                _IO(IOC_MAGIC,5)
#define IOCTL_ID_EEPROM             _IO(IOC_MAGIC,6)
#define IOCTL_GNSS_MSG_CONFIG       _IO(IOC_MAGIC,7)

#define KEEP_ALIVE_TIMEOUT(x)           (jiffies + msecs_to_jiffies(x))


#define GNSS_MSG_RMC                (1 << 0)
#define GNSS_MSG_VTG                (1 << 1)
#define GNSS_MSG_GGA                (1 << 2)
#define GNSS_MSG_GSA                (1 << 3)
#define GNSS_MSG_GSV                (1 << 4)
#define GNSS_MSG_GLL                (1 << 5)
#define GNSS_MSG_TXT                (1 << 6)
#define GNSS_MSG_GSV                (1 << 7)

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////// DATA TYPE DEFINITIONS
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

struct ctrl_gpio{
    int gpio;                                   /* gpio number */
    int flags;                                  /* gpio flags */
    int irq;                                    /* irq number associated with the gpio */
    int activeState;                            /* the logic level of the active/asserted state */
    int initState;                              /* the logic level of the initial state */
    const char* name;                           /* name of the gpio */
    int nr;                                     /* consecutive internal number used by the driver */
    unsigned long jiffiesEvent;                 /* jiffies timestamp, set in the interrupt context related to the irq/gpio */
};

struct st_moitessierSpi{
	struct spi_device	    *spi;               /* the SPI device used for this driver */
    unsigned char	        txBuf[BUF_SIZE];    /* tx buffer used for SPI transmissions */
	unsigned char			rxBuf[BUF_SIZE];    /* rx buffer used for SPI receptions */
	unsigned int			speed_hz;           /* the baudrate used for the SPI */
    struct ctrl_gpio        req_gpio[2];        /* request signals (outputs) */
    struct ctrl_gpio        irq_gpio[2];        /* interrupt signals (inputs) */
    struct ctrl_gpio        boot_gpio;          /* signal to enable boot mode for the HAT */
    struct ctrl_gpio        reset_gpio;         /* signal to reset the HAT */
    spinlock_t              spinlock;           /* locks this structure */
    bool                    initialized;        /* is true if the device has been properly configured/initialized */
    bool                    opened;             /* is true if the device has been opened by a user */
};

#if defined(USE_STATISTICS)
struct st_statistics{
    uint64_t                spiCycles;              /* total number of SPI transmission/reception cycles */
    uint64_t                totalRxPayloadBytes;    /* total number of payload bytes received via the SPI interface (including NMEA and control data) */
    uint64_t                fifoOverflows;          /* incremented whenever the FIFO overflows */
    uint64_t                fifoBytesProcessed;     /* total number of bytes passed to the user via read(...) */
    uint64_t                payloadCrcErrors;       /* incremented whenever a payload with wrong CRC is received via the SPI interface */
    uint64_t                headerCrcErrors;        /* incremented whenever a header with wrong CRC is received via the SPI interface */
    uint64_t                keepAliveErrors;        /* incremented whenever a keep alive timeout occurs */
    spinlock_t              spinlock;		        /* locks this structure */
};
#endif /* USE_STATISTICS */

struct st_receiverConfig
{
    uint8_t         metaDataMask;                   /* Meta data mask of the receiver. If meta data is enabled, the AIS receiver will provide
                                                       meta information (e.g. RSSI) */
    uint32_t        afcRange;                       /* AFC range */    
    uint32_t        afcRangeDefault;                /* default AFC range */    
    uint32_t        tcxoFreq;                       /* TCXO frequency used for the AIS receiver in Hz */    
    uint32_t        channelFreq[NUM_RCV_CHANNELS];  /* channel frequencies used by the AIS receiver in Hz */    
};

struct st_info_rcv{
    struct st_receiverConfig    config;             
    uint8_t                     rng[NUM_RCV_CHANNELS];             /* PLL ranging value of the AIS receiver */
};

struct st_info_serial{
    uint32_t    h;                                  /* bits 95...64 */
    uint32_t    m;                                  /* bits 63...32 */
    uint32_t    l;                                  /* bits 31...0 */
};

struct st_simulator
{
    uint32_t mmsi[NUM_RCV_CHANNELS];                /* MMSI used for the AIS simulator */
    uint32_t enabled;                               /* 1...simulator enabled, 0...simulator disabled */
    uint32_t interval;                              /* interval used for the AIS simulator */
};

struct st_info{
    uint8_t                     mode;               /* operation mode of the HAT */
    uint8_t                     hwId[16];           /* hardware identifier of the HAT */
    uint8_t                     hwVer[8];           /* hardware version of the HAT */
    uint8_t                     bootVer[22];        /* boot loader version of the HAT --> currently not used */
    uint8_t                     appVer[22];         /* firmware/application version of the HAT */
    uint32_t                    functionality;      /* functionality available on the HAT --> currently not used */
    uint32_t                    systemErrors;       /* system errors (bit mask) detected on the HAT */
    struct st_info_serial       serial;             /* unique serial number of the HAT */
    struct st_info_rcv          rcv[NUM_RCV];       /* AIS receiver related information/configuration */
    struct st_simulator         simulator;          /* simulator related information/configuration */
    uint8_t                     wpEEPROM;           /* write protection of the ID EEPROM, 1...enabled, 0...disabled */
    uint8_t                     buttonPressed;      /* button pressed, 1...pressed, 0...not pressed */
    uint8_t                     gnssVer[32];        /**< GNSS firmware version */
    uint8_t                     gnssSysSupported;   /**< 0...not available
                                                         255...error occured
                                                         
                                                         1...GPS only
                                                         2...GLONASS only
                                                         4...GALILEO only
                                                         5...GPS + GALILEO
                                                         6...GALILEO + GLONASS
                                                         7...GPS + GLONASS + GALILEO */
    bool        valid;                              /* true if valid, else false */
    spinlock_t  spinlock;                           /* locks this structure */
};

struct st_configHAT{    
    struct st_receiverConfig    rcv[2];             /* AIS receiver configuration */
    struct st_simulator         simulator;          /* simulator configuration */
    uint8_t                     wpEEPROM;           /* write protection of the ID EEPROM, 1...enabled, 0...disabled */
    spinlock_t  spinlock;                           /* locks this structure */
};

struct st_config{
    uint8_t     gnssEnabled;                        /* 1...GNSS enabled, 0...disabled */
    uint32_t    gnssMsgEnabled;                              
    spinlock_t  spinlock;
};


struct st_fifo{
    unsigned char           data[FIFO_SIZE];        /* data memory of the FIFO */
    unsigned int            size;                   /* the size of the FIFO */
    unsigned int            cnt;                    /* number of bytes currently stored in the FIFO */
#if !defined(CONFIG_ARM64)    
    spinlock_t              spinlock;
#else    
    struct mutex	        lock;                   
#endif /* CONFIG_ARM64 */    
};

#if defined(USE_TTY)
struct st_moitessierSpi_serial {
	struct tty_struct	    *tty;		            /* pointer to the tty for this device */
	struct mutex	        lock;		            /* locks this structure */
	bool                    initialized;            /* is true if the device has been properly configured/initialized */
	bool                    opened;                 /* is true if the device has been opened by a user */
};
#endif /* USE_TTY */

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////// GLOBAL LOCAL VARIABLES
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

struct st_moitessierSpi	        *moitessier_spi = NULL;
static dev_t                    moitessier_spi_nr;
static struct cdev              *moitessier_spi_object;
static struct class             *moitessier_spi_class;
static struct device            *moitessier_spi_dev;

static dev_t                    moitessier_ctrl_nr;
static struct cdev              *moitessier_ctrl_object;
static struct class             *moitessier_ctrl_class;
static struct device            *moitessier_ctrl_dev;
#if defined(USE_STATISTICS)
static struct st_statistics     statistics;
#endif /* USE_STATISTICS */
static struct st_info           info;
static struct st_config         config;
static struct st_configHAT      configHAT;

/* default configuration of the HAT */
const struct st_configHAT configHATdefault = {
    .rcv[0].metaDataMask = 0,
    .rcv[0].afcRange = 1000,
    .rcv[0].afcRangeDefault = 0,
    .rcv[0].tcxoFreq = 13000000,
    .rcv[0].channelFreq[0] = 161975000,
    .rcv[0].channelFreq[1] = 162025000,
    .rcv[1].metaDataMask = 0,
    .rcv[1].afcRange = 1000,
    .rcv[1].afcRangeDefault = 0,
    .rcv[1].tcxoFreq = 13000000,
    .rcv[1].channelFreq[0] = 161975000,
    .rcv[1].channelFreq[1] = 162025000,
    .simulator.enabled = 0,
    .simulator.interval = 1000,
    .simulator.mmsi[0] = 807977831,
    .simulator.mmsi[1] = 807977832,
    .wpEEPROM = 1,
};

DEFINE_MUTEX(mutex_adcVal);
static DECLARE_COMPLETION(on_exit);

static wait_queue_head_t        wq_read;
static wait_queue_head_t        wq_thread;
static struct task_struct       *thread_id;

#if LINUX_VERSION_CODE >= KERNEL_VERSION_ALT(5, 9, 0)
static void irq_tasklet(struct tasklet_struct *t);
DECLARE_TASKLET(irq_tl, irq_tasklet);
#else
static void irq_tasklet(unsigned long data);
DECLARE_TASKLET(irq_tl, irq_tasklet, 0);
#endif /* LINUX_VERSION_CODE */

static atomic_t irqOccured=ATOMIC_INIT(0);

struct st_fifo rxFifo;

static atomic_t reqReceived = ATOMIC_INIT(0);
#define IS_SLAVE_REQ_RECEIVED           (atomic_read(&reqReceived) != 0)
#define SLAVE_REQ_CONFIRMED             atomic_set(&reqReceived, 0) 
#define SLAVE_REQ_RECEIVED              atomic_set(&reqReceived, 1)

static atomic_t dataAvailForUser = ATOMIC_INIT(0);
#define DATA_AVAILABLE_USER             (atomic_read(&dataAvailForUser) != 0)
#define RESET_DATA_AVAILABLE_USER       atomic_set(&dataAvailForUser, 0) 
#define SET_DATA_AVAILABLE_USER         atomic_set(&dataAvailForUser, 1)


#if defined(USE_TTY)
static struct st_moitessierSpi_serial *moitessier_serial;	
static struct class *moitessier_tty_class;
static struct device *moitessier_tty_dev;
#define MOITESSIER_TTY_MAJOR            240
//#if LINUX_VERSION_CODE >= KERNEL_VERSION_ALT(4, 15, 0)
#define MOITESSIER_TTY_MINOR_START      16
//#endif /* LINUX_VERSION_CODE */
#endif /* USE_TTY */

#if defined(SUPPORT_KEEP_ALIVE)
static atomic_t killKeepAlive = ATOMIC_INIT(0);
static struct timer_list timerKeepAlive;
spinlock_t timerKeepAlive_spinlock;
static DECLARE_COMPLETION(on_exitKeepAlive);
static struct workqueue_struct *wq = NULL;
static void keepAlive_queueFunc(struct work_struct *work);
static DECLARE_WORK(work_object, keepAlive_queueFunc);
#endif /* SUPPORT_KEEP_ALIVE */

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////// FUNCTION PROTOTYPES
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static int moitessier_transmitSpiData(struct st_moitessierSpi *dev, char* data, unsigned int len, bool devBuf);
static int moitessier_receiveSpiData(struct st_moitessierSpi *dev, size_t len);
static void moitessier_resetHAT(uint32_t ms);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/* this parameter must correlate with the used device tree, otherwise the device will not be initialized */
static const struct of_device_id moitessier_dt_ids[] = {
	{ .compatible = "rooco,moitessier" },
	{},
};
MODULE_DEVICE_TABLE(of, moitessier_dt_ids);

/* user triggered function, read from the device */
/* returns the number of bytes read, or 0 if EOF reached */
static ssize_t moitessier_read(struct file *filp, char __user *buf, size_t maxBytesToRead, loff_t *f_pos)
{
    ssize_t status = 0;
    size_t to_copy, not_copied;
    struct st_moitessierSpi *dev;

    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
        pr_info("%s - max bytes to read: %d\n", __func__, maxBytesToRead);
    
    if(!moitessier_spi)
        return -EPERM;    
    if(!moitessier_spi->initialized)
        return -EPERM;
    
    dev = filp->private_data;
    
#if !defined(USE_TTY)
   //    gpio_set_value(moitessier_spi->req_gpio[1].gpio, REQ_LEVEL_INACTIVE);
#endif /* !USE_TTY */
    		
    if(!DATA_AVAILABLE_USER)
    {
        if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
            pr_info("%s - currently no data available\n", __func__);
        
        /* no data available and device opened in non blocking mode */
        if((filp->f_flags & O_NONBLOCK))
        {
            put_user(ERR_NO_DAT_AVAIL, buf); 
            status = -EAGAIN;                                                           
            goto exit_read;
        }
    
        /* lets go for a sleep... */
        if(wait_event_interruptible(wq_read, DATA_AVAILABLE_USER))
        {
            /* interrupted by a signal while sleeping */
            put_user(ERR_SIG_CHANCEL, buf); 
    	    status = -ERESTARTSYS;                                                           
            goto exit_read;  
    	}
    }	
    
    RESET_DATA_AVAILABLE_USER;
    
#if !defined(USE_TTY)
    spin_lock_irq(&dev->spinlock);
    gpio_set_value(dev->req_gpio[1].gpio, REQ_LEVEL_INACTIVE);	
    spin_unlock_irq(&dev->spinlock);
#endif /* !USE_TTY */		

    /* On a 64 bit kernel the spin lock causes a "bad address" error when reading the SPI device file.
       Currently it is not exactly known why this issue exists at all.
       The rxFifo is not used in the interrupt context, so a mutex can be used instead anyway.
    */
#if !defined(CONFIG_ARM64)
	spin_lock_irq(&rxFifo.spinlock);
#else 	
	if (mutex_lock_interruptible(&rxFifo.lock))
    {
        return -1;
    }
#endif /* CONFIG_ARM64 */
	
	to_copy = min(rxFifo.cnt, maxBytesToRead);
	not_copied = copy_to_user(buf, rxFifo.data, to_copy);
	
#if defined(USE_STATISTICS)	
	spin_lock_irq(&statistics.spinlock);
    statistics.fifoBytesProcessed += to_copy;
    spin_unlock_irq(&statistics.spinlock);
#endif /* USE_STATISTICS */
	
	/* complete FIFO read? */
	if(to_copy != rxFifo.cnt && to_copy < rxFifo.cnt)
	{
	   memcpy(&rxFifo.data[0], &rxFifo.data[to_copy], rxFifo.cnt - to_copy);
	   rxFifo.cnt -= to_copy;
	}
	else
	{ 
	    rxFifo.cnt = 0;
	}
	
#if !defined(CONFIG_ARM64)
	spin_unlock_irq(&rxFifo.spinlock);
#else 	
	mutex_unlock(&rxFifo.lock);
#endif /* CONFIG_ARM64 */	
	
	if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	    pr_info("%s - bytes read: %d\n", __func__, to_copy);
	
	
	if(not_copied == to_copy)
		status = -EFAULT;
	else
		status = to_copy - not_copied;
		
	return status;
	
exit_read:       
    return status;     	
}

/* user triggered function, write data to the device */
/* returns the actual number of bytes written */
/* the write function is not required at the moment */
//static ssize_t moitessier_write(struct file *filp, const char __user *buf, size_t maxBytesToWrite, loff_t *f_pos)
//{
//    unsigned long not_copied;
//    ssize_t status = 0;
//    struct st_moitessierSpi *dev;
//    
//    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
//	    pr_info("%s\n", __func__);
//	
//	dev = filp->private_data;
//	
//	if (maxBytesToWrite > BUF_SIZE)
//		return -EMSGSIZE;
//
//	spin_lock_irq(&dev->spinlock);
//	not_copied = copy_from_user(dev->txBuf, buf, maxBytesToWrite);
//	if(not_copied == 0)
//		status = moitessier_transmitSpiData(dev, NULL, maxBytesToWrite, true);
//	else
//		status = -EFAULT;
//	spin_unlock_irq(&dev->spinlock);
//	
//	return maxBytesToWrite;
//}

/* user triggered function, open the device */
/* returns 0 on success or < 0 in case of an error */
static int moitessier_open(struct inode *inode, struct file *filp)
{
    struct st_moitessierSpi *dev = moitessier_spi;
    int status = -EIO;
      
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)  
	    pr_info("%s\n", __func__);
	
	if(!dev)
        return -EPERM;        
	if(!dev->initialized)
        return -EPERM;
        
	spin_lock_irq(&dev->spinlock);
	
	/* we can only open the device once */
	if(dev->opened)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
		    pr_err("%s - device already opened\n", __func__);
		status = -EIO;
		goto open_failed;
	}
	
	/* we store the device structure to this instance so we can later access it in read(...) and write(...) */
	filp->private_data = dev;

    dev->opened = true;
	
	spin_unlock_irq(&dev->spinlock);
	
	return 0;

open_failed:
    spin_unlock_irq(&dev->spinlock);
	return status;		
}

static void moitessier_resetHAT(uint32_t ms)
{
    unsigned long iflags;
    
    if(DEBUG_LEVEL >= LEVEL_DEBUG)
        pr_info("%s\n", __func__);
    
    spin_lock_irqsave(&moitessier_spi->spinlock, iflags);
    /* reset the HAT microcontroller */    
    if(USE_REQ_FOR_RESET)
    {
        gpio_set_value(moitessier_spi->req_gpio[0].gpio, REQ_LEVEL_INACTIVE);
        gpio_set_value(moitessier_spi->req_gpio[0].gpio, REQ_LEVEL_ACTIVE);
        spin_unlock_irqrestore(&moitessier_spi->spinlock, iflags);
        if(ms)
            mdelay(ms);
        spin_lock_irqsave(&moitessier_spi->spinlock, iflags);
        gpio_set_value(moitessier_spi->req_gpio[0].gpio, REQ_LEVEL_INACTIVE);
    }
    else
    {
        gpio_set_value(moitessier_spi->reset_gpio.gpio, RESET_LEVEL_INACTIVE);
        gpio_set_value(moitessier_spi->reset_gpio.gpio, RESET_LEVEL_ACTIVE);
        spin_unlock_irqrestore(&moitessier_spi->spinlock, iflags);
        if(ms)
            mdelay(ms);
        spin_lock_irqsave(&moitessier_spi->spinlock, iflags);
        gpio_set_value(moitessier_spi->reset_gpio.gpio, RESET_LEVEL_INACTIVE);
    }

    spin_unlock_irqrestore(&moitessier_spi->spinlock, iflags);
}

#if defined(USE_STATISTICS)
void moitessier_resetStatistics(struct st_statistics *stat)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG)
        pr_info("%s\n", __func__);
        
    spin_lock_irq(&stat->spinlock);
    stat->spiCycles = 0;
    stat->totalRxPayloadBytes = 0;
    stat->fifoOverflows = 0;
    stat->fifoBytesProcessed = 0;
    stat->payloadCrcErrors = 0;
    stat->headerCrcErrors = 0;
    stat->keepAliveErrors = 0;
    spin_unlock_irq(&stat->spinlock);
}
#endif /* USE_STATISTICS */

static long moitessier_ctrl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    uint8_t dat[256];
    size_t rc = 0;
    int32_t size = 0;
    
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
        pr_info("%s - cmd: %u\n", __func__, cmd);
    
    if(!moitessier_spi)
        return -EPERM;
    if(!moitessier_spi->initialized)
        return -EPERM;
    
    memset(dat, 0, sizeof(dat));
            
    switch(cmd)
    {
#if defined(USE_STATISTICS)        
        case IOCTL_GET_STATISTICS:
            spin_lock_irq(&statistics.spinlock);
            memcpy(dat, &statistics, sizeof(struct st_statistics));
            spin_unlock_irq(&statistics.spinlock);
            rc = copy_to_user((void*)arg, dat, sizeof(struct st_statistics));  
            size = sizeof(struct st_statistics);
            break;
        case IOCTL_RESET_STATISTICS:
            moitessier_resetStatistics(&statistics);
            break;
#endif /* USE_STATISTICS */                    
        case IOCTL_RESET_HAT:
            moitessier_resetHAT(10);
            break;
        case IOCTL_GET_INFO:
            spin_lock_irq(&info.spinlock);
            memcpy(dat, &info, sizeof(struct st_info));
            spin_unlock_irq(&info.spinlock);
            rc = copy_to_user((void*)arg, dat, sizeof(struct st_info));
            size = sizeof(struct st_info);
            break;
        case IOCTL_GNSS:
            rc = copy_from_user(dat, (void*)arg, sizeof(uint8_t));
            spin_lock_irq(&config.spinlock);
            config.gnssEnabled = dat[0];
            spin_unlock_irq(&config.spinlock);
            size = sizeof(uint8_t);
            break;
        case IOCTL_CONFIG:
            rc = copy_from_user(dat, (void*)arg, sizeof(struct st_configHAT) - sizeof(spinlock_t));
            spin_lock_irq(&configHAT.spinlock);
            memcpy(&configHAT, dat, sizeof(struct st_configHAT) - sizeof(spinlock_t));
            spin_unlock_irq(&configHAT.spinlock);
            size = sizeof(struct st_configHAT) - sizeof(spinlock_t);
            
            /* we need to request the current configuration */
            spin_lock_irq(&info.spinlock);
            info.valid = false;
            spin_unlock_irq(&info.spinlock);
         
            if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
            {
                pr_info("%s - receiver 1 configuration\n", __func__);
                pr_info("\t\tchannel frequency 1 [Hz]:\t %u\n", configHAT.rcv[0].channelFreq[0]);
                pr_info("\t\tchannel frequency 2 [Hz]:\t %u\n", configHAT.rcv[0].channelFreq[1]);
                pr_info("\t\tmeta data mask:\t\t\t %u\n", configHAT.rcv[0].metaDataMask);
                pr_info("\t\tAFC range [Hz]:\t\t\t %u\n", configHAT.rcv[0].afcRange);
                pr_info("\t\tTCXO frequency [Hz]:\t\t %u\n", configHAT.rcv[0].tcxoFreq);
                pr_info("%s - receiver 2 configuration\n", __func__);
                pr_info("\t\tchannel frequency 1 [Hz]:\t %u\n", configHAT.rcv[1].channelFreq[0]);
                pr_info("\t\tchannel frequency 2 [Hz]:\t %u\n", configHAT.rcv[1].channelFreq[1]);
                pr_info("\t\tmeta data mask:\t\t\t %u\n", configHAT.rcv[1].metaDataMask);
                pr_info("\t\tAFC range [Hz]:\t\t\t %u\n", configHAT.rcv[1].afcRange);
                pr_info("\t\tTCXO frequency [Hz]:\t\t %u\n", configHAT.rcv[1].tcxoFreq);  
                pr_info("%s - simulator\n", __func__);
                pr_info("\t\tenabled:\t\t\t %u\n", configHAT.simulator.enabled);  
                pr_info("\t\tinterval:\t\t\t %u\n", configHAT.simulator.interval);  
                pr_info("\t\tmmsi:\t\t\t %09u %09u\n", configHAT.simulator.mmsi[0], configHAT.simulator.mmsi[1]);  
                pr_info("%s - misc\n", __func__);
                pr_info("\t\twrite protection ID EEPROM:\t\t %u\n", configHAT.wpEEPROM);  
            }          
            break;
        case IOCTL_ID_EEPROM:
            rc = copy_from_user(dat, (void*)arg, sizeof(uint8_t));
            spin_lock_irq(&config.spinlock);
            configHAT.wpEEPROM = dat[0];
            spin_unlock_irq(&config.spinlock);
            size = sizeof(uint8_t);
            break;
        case IOCTL_GNSS_MSG_CONFIG:
            rc = copy_from_user(dat, (void*)arg, sizeof(uint8_t));
            spin_lock_irq(&config.spinlock);
            config.gnssMsgEnabled = dat[0];
            spin_unlock_irq(&config.spinlock);
            size = sizeof(uint8_t);
            if(DEBUG_LEVEL >= LEVEL_WARNING)
            {
                pr_warn("%s - GNSS message configuration = %u\n", __func__, config.gnssMsgEnabled);
                pr_warn("\t\tRMC: %s\n", (config.gnssMsgEnabled & GNSS_MSG_RMC) ? "enabled" : "disabled");
                pr_warn("\t\tVTG: %s\n", (config.gnssMsgEnabled & GNSS_MSG_VTG) ? "enabled" : "disabled");
                pr_warn("\t\tGGA: %s\n", (config.gnssMsgEnabled & GNSS_MSG_GGA) ? "enabled" : "disabled");
                pr_warn("\t\tGSA: %s\n", (config.gnssMsgEnabled & GNSS_MSG_GSA) ? "enabled" : "disabled");
                pr_warn("\t\tGSV: %s\n", (config.gnssMsgEnabled & GNSS_MSG_GSV) ? "enabled" : "disabled");
                pr_warn("\t\tGLL: %s\n", (config.gnssMsgEnabled & GNSS_MSG_GLL) ? "enabled" : "disabled");   
                pr_warn("\t\tTXT: %s\n", (config.gnssMsgEnabled & GNSS_MSG_TXT) ? "enabled" : "disabled");       
                pr_warn("\t\tGSV: %s\n", (config.gnssMsgEnabled & GNSS_MSG_GSV) ? "enabled" : "disabled");       
            }
            break;            
        default:
            if(DEBUG_LEVEL >= LEVEL_CRITICAL)
            {
                pr_err("%s: Unknown parameter.\n", __func__);
            }
            return -EPERM;
    }
    
    return (size - rc);
}


static long moitessier_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
        pr_info("%s\n", __func__);
	
	/* TODO: need to be implemented */
	return 0;
}

static long moitessier_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
        pr_info("%s\n", __func__);
	
	/* TODO: need to be implemented */
	return 0;
}

/* user triggered function, close the device */
/* returns 0 on success or < 0 on an error */
static int moitessier_release(struct inode *inode, struct file *filp)
{
    struct st_moitessierSpi *dev;
    
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	    pr_info("%s\n", __func__);

	dev = filp->private_data;
	
    spin_lock_irq(&dev->spinlock);
    
    filp->private_data = NULL;
	dev->opened = false;
	
	spin_unlock_irq(&dev->spinlock);
		
	return 0;
}

/* user triggered file operations */
static const struct file_operations moitessier_fops = {
	.owner              = THIS_MODULE,
	//.write              = moitessier_write,
	.read               = moitessier_read,
	.open               = moitessier_open,
	.release            = moitessier_release,
	.unlocked_ioctl     = moitessier_ioctl,
	.compat_ioctl       = moitessier_compat_ioctl,
};

/* user triggered file operations */
static const struct file_operations moitessier_ctrlfops = {
	.owner              = THIS_MODULE,
	.unlocked_ioctl     = moitessier_ctrl_ioctl,
};

/*
static int FUNC_NOT_USED send_byte(struct st_moitessierSpi *dev, int data)
{
	int status;
	struct spi_message msg = { };
	struct spi_transfer transfer = { };

    if(DEBUG_LEVEL >= LEVEL_DEBUG)
	    pr_info("%s\n", __func__);

	spi_message_init(&msg);
	dev->txBuf[0] = data;
	transfer.tx_buf = dev->txBuf;
	transfer.rx_buf = dev->rxBuf;
	transfer.len = 1;
	transfer.speed_hz = dev->speed_hz;
	spi_message_add_tail(&transfer, &msg);
	status = spi_sync(dev->spi, &msg);

	return status;
};
*/

static int moitessier_transmitSpiData(struct st_moitessierSpi *dev, char* data, unsigned int len, bool devBuf)
{
	int status;
	struct spi_message msg = { };
	struct spi_transfer transfer = { };

    if(DEBUG_LEVEL >= LEVEL_DEBUG)
	    pr_info("%s\n", __func__);

	spi_message_init(&msg);
	if(!devBuf)
        memcpy(dev->txBuf, data, len);
	transfer.tx_buf = dev->txBuf;
	transfer.rx_buf = dev->rxBuf;
	transfer.len = len;
	transfer.speed_hz = dev->speed_hz;
	spi_message_add_tail(&transfer, &msg);
	status = spi_sync(dev->spi, &msg);

    if (status == 0)
		status = msg.actual_length;
	return status;
};


static int FUNC_NOT_USED moitessier_receiveSpiData(struct st_moitessierSpi *dev, size_t len)
{
    int status;
	struct spi_message msg = { };
	struct spi_transfer transfer = { };
	
	if(DEBUG_LEVEL >= LEVEL_DEBUG)
	    pr_info("%s\n", __func__);
	    
	spi_message_init(&msg);
	/* set the byte that will be used for the transmit signal */
	/* we might change this, if data should be excanged and not only read */
	memset(dev->txBuf, '0', BUF_SIZE);
	
	transfer.tx_buf = dev->txBuf;
	transfer.rx_buf = dev->rxBuf;
	transfer.len = len;
	transfer.speed_hz = dev->speed_hz;
	spi_message_add_tail(&transfer, &msg);
	status = spi_sync(dev->spi, &msg);
	
	if (status == 0)
		status = msg.actual_length;
	
	return status;
}

#if defined(SUPPORT_KEEP_ALIVE)
static void keepAlive_queueFunc(struct work_struct *work)
{
    if(!USE_KEEP_ALIVE)
        return;
        
    if(!moitessier_spi)
        return;
    
    if(!moitessier_spi->initialized)
        return;
    
    if(DEBUG_LEVEL >= LEVEL_WARNING)
        pr_info("%s - reseting HAT\n", __func__);        
        
    moitessier_resetHAT(10);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION_ALT(4, 14, 0)  
static void moitessier_keepAlive(struct timer_list *t)
#else
static void moitessier_keepAlive(unsigned long dat)
#endif /* LINUX_VERSION_CODE */
{
    unsigned long iflags;
    
    if(DEBUG_LEVEL >= LEVEL_WARNING)
        pr_info("%s\n", __func__);    
    
    if(atomic_read(&killKeepAlive))
        complete(&on_exitKeepAlive);
    else
    {
        
#if defined(USE_STATISTICS)            
        spin_lock_irqsave(&statistics.spinlock, iflags);
        statistics.keepAliveErrors++;
        spin_unlock_irqrestore(&statistics.spinlock, iflags);
#endif /* USE_STATISTICS */ 
        
        spin_lock_irqsave(&timerKeepAlive_spinlock, iflags);
        //add_timer(&timerKeepAlive);        
        mod_timer(&timerKeepAlive, KEEP_ALIVE_TIMEOUT(KEEP_ALIVE_TIMEOUT_LONG_MS));
        spin_unlock_irqrestore(&timerKeepAlive_spinlock, iflags);
            
        queue_work(wq, &work_object);      
    }    
}
#endif /* SUPPORT_KEEP_ALIVE */

void moitessier_processReqData(void)
{
#if defined(SUPPORT_KEEP_ALIVE)    
    unsigned long iflags;
#endif /* SUPPORT_KEEP_ALIVE */    
    uint32_t headerPos = 0;
    uint32_t lastHeaderPos = 0;
    header_t header;
    uint32_t i = 0;
    uint32_t k = 0;
    bool dataProcessed = false;
    bool fifoFull = false;
    uint8_t *msg;
    uint32_t headerCnt = 0;
    uint16_t crc;
    bool processData = false;
    uint32_t posInTxBuf = 0;
    uint32_t posPayload = 0;
    request_t request;
	response_t response;
#if defined(USE_TTY)   
    struct st_moitessierSpi_serial *serial = NULL;
    struct tty_struct *tty = NULL;
	struct tty_port *port = NULL;	
#if defined(USE_MSG_CNT_TTY)  	
    static unsigned long cnt = 0;
    char cntBuf[100];
#endif /* USE_MSG_CNT_TTY */
#endif /* USE_TTY */	

    if(!moitessier_spi)
        return;
    
    if(!moitessier_spi->initialized)
        return;
        
#if !defined(CONFIG_ARM64)    
    spin_lock_irq(&moitessier_spi->spinlock);
#endif /* CONFIG_ARM64 */
    
    /* read data from the SPI interface */
    //moitessier_receiveSpiData(moitessier_spi, BUF_SIZE);
    
    memset(moitessier_spi->txBuf, 0, BUF_SIZE);
   
    /* create configuration command */
    HEADER_uint8_tToAsciiHex(CMD_CATEGORY_RESPONSE, &moitessier_spi->txBuf[posInTxBuf + HEADER_size()  + (offsetof(response_t, category) * 2)], NOT_NULL_TERMINATED);
    HEADER_uint8_tToAsciiHex(CMD_CONFIG, &moitessier_spi->txBuf[posInTxBuf + HEADER_size() + (offsetof(response_t, cmd) * 2)], NOT_NULL_TERMINATED);
    HEADER_uint8_tToAsciiHex(CMD_FIELD_NOT_USED, &moitessier_spi->txBuf[posInTxBuf + HEADER_size() + (offsetof(response_t, cmdSub) * 2)], NOT_NULL_TERMINATED);
    posPayload = sizeof(response_t) * 2; 
    spin_lock_irq(&configHAT.spinlock);
    
    for(i = 0; i < NUM_RCV; i++)
    {
        for(k = 0; k < NUM_RCV_CHANNELS; k++)
        {
            HEADER_uint32_tToAsciiHex(configHAT.rcv[i].channelFreq[k], &moitessier_spi->txBuf[posInTxBuf + HEADER_size() + posPayload], NOT_NULL_TERMINATED);
            posPayload += DIM_ELEMENT(struct st_receiverConfig, channelFreq[k]) * 2;
        }
        HEADER_uint8_tToAsciiHex(configHAT.rcv[i].metaDataMask, &moitessier_spi->txBuf[posInTxBuf + HEADER_size() + posPayload], NOT_NULL_TERMINATED);
        posPayload += DIM_ELEMENT(struct st_receiverConfig, metaDataMask) * 2;
        HEADER_uint32_tToAsciiHex(configHAT.rcv[i].afcRange, &moitessier_spi->txBuf[posInTxBuf + HEADER_size() + posPayload], NOT_NULL_TERMINATED);
        posPayload += DIM_ELEMENT(struct st_receiverConfig, afcRange) * 2;
        HEADER_uint32_tToAsciiHex(configHAT.rcv[i].tcxoFreq, &moitessier_spi->txBuf[posInTxBuf + HEADER_size() + posPayload], NOT_NULL_TERMINATED);
        posPayload += DIM_ELEMENT(struct st_receiverConfig, tcxoFreq) * 2;
    }
    HEADER_uint8_tToAsciiHex(configHAT.simulator.enabled, &moitessier_spi->txBuf[posInTxBuf + HEADER_size() + posPayload], NOT_NULL_TERMINATED);
    posPayload += DIM_ELEMENT(struct st_simulator, enabled) * 2;  
    HEADER_uint32_tToAsciiHex(configHAT.simulator.interval, &moitessier_spi->txBuf[posInTxBuf + HEADER_size() + posPayload], NOT_NULL_TERMINATED);
    posPayload += DIM_ELEMENT(struct st_simulator, interval) * 2;    
    for(k = 0; k < NUM_RCV_CHANNELS; k++)
    {
        HEADER_uint32_tToAsciiHex(configHAT.simulator.mmsi[k], &moitessier_spi->txBuf[posInTxBuf + HEADER_size() + posPayload], NOT_NULL_TERMINATED);
        posPayload += DIM_ELEMENT(struct st_simulator, mmsi[k]) * 2;    
    }
    HEADER_uint8_tToAsciiHex(configHAT.wpEEPROM, &moitessier_spi->txBuf[posInTxBuf + HEADER_size() + posPayload], NOT_NULL_TERMINATED);
    posPayload += DIM_ELEMENT(struct st_configHAT, wpEEPROM) * 2;  
    
    spin_unlock_irq(&configHAT.spinlock);
    
    /* fill the header for the payload with default settings */
    HEADER_setDefaults(&header);
    
    /* change the required fields in the header */
    header.source = SOURCE_SYSTEM;
    header.target = SOURCE_SYSTEM;
    header.payloadLen = posPayload;
    
    /* calculate the CRC for the payload */
    HEADER_crc(&moitessier_spi->txBuf[posInTxBuf + HEADER_size()], header.payloadLen, &crc);
    header.payloadCRC = crc;
    
    /* calculate the CRC of the header */
    HEADER_crc((uint8_t*)&header, sizeof(header_t) - sizeof(uint16_t), &crc);
    header.headerCRC = crc;
    
    /* convert the header to ASCII HEX format so it can be transmitted and write it to the appropriate transmit buffer */
    HEADER_set(&header, &moitessier_spi->txBuf[posInTxBuf], BUF_SIZE);
    
    posInTxBuf = HEADER_size() + header.payloadLen;

    /* we've not received the info of the connected HAT yet */
    if(!info.valid)
    {
        /* create the command/payload that should be transmitted to the SPI slave */
        HEADER_uint8_tToAsciiHex(CMD_CATEGORY_REQUEST, &moitessier_spi->txBuf[posInTxBuf + HEADER_size()  + (offsetof(request_t, category) * 2)], NOT_NULL_TERMINATED);
        HEADER_uint8_tToAsciiHex(CMD_INFO, &moitessier_spi->txBuf[posInTxBuf + HEADER_size() + (offsetof(request_t, cmd) * 2)], NOT_NULL_TERMINATED);
        HEADER_uint8_tToAsciiHex(CMD_INFO_SYSTEM, &moitessier_spi->txBuf[posInTxBuf + HEADER_size() + (offsetof(request_t, cmdSub) * 2)], NOT_NULL_TERMINATED);
        HEADER_uint8_tToAsciiHex(CMD_FIELD_NOT_USED, &moitessier_spi->txBuf[posInTxBuf + HEADER_size() + (offsetof(request_t, param) * 2)], NOT_NULL_TERMINATED);
        
        /* fill the header for the payload with default settings */
        HEADER_setDefaults(&header);
        
        /* change the required fields in the header */
        header.source = SOURCE_SYSTEM;
        header.target = SOURCE_SYSTEM;
        header.payloadLen = (sizeof(request_t) * 2);
        
        /* calculate the CRC for the payload */
        HEADER_crc(&moitessier_spi->txBuf[posInTxBuf + HEADER_size()], header.payloadLen, &crc);
        header.payloadCRC = crc;
        
        /* calculate the CRC of the header */
        HEADER_crc((uint8_t*)&header, sizeof(header_t) - sizeof(uint16_t), &crc);
        header.headerCRC = crc;
        
        /* convert the header to ASCII HEX format so it can be transmitted and write it to the appropriate transmit buffer */
        HEADER_set(&header, &moitessier_spi->txBuf[posInTxBuf], BUF_SIZE);      
    }
    /*
    else if(...)
    {
        you might want to add here some additional commands that should be transmitted to the SPI slave 
    }
    */
    else    
    {
        
    }
    
    /* start SPI transmission */
    moitessier_transmitSpiData(moitessier_spi, NULL, BUF_SIZE, true);
    
#if defined(USE_STATISTICS)    
    spin_lock_irq(&statistics.spinlock);
    statistics.spiCycles++;
    spin_unlock_irq(&statistics.spinlock);
#endif /* USE_STATISTICS */
    
    /* process received data */
    lastHeaderPos = 0;
    headerCnt = 0;
    do
    {
        if(lastHeaderPos >= BUF_SIZE)
            break;
            
        /* search for a header in the received data, if available process the data otherwise leave this function */
        if(HEADER_find(moitessier_spi->rxBuf, BUF_SIZE, lastHeaderPos, &headerPos))
        {
            /* error handling */
            if(DEBUG_LEVEL >= LEVEL_WARNING && !headerCnt)
                pr_err("%s - HEADER_find(...) failed - %u\n", __func__, headerCnt);
            goto moitessier_processReqData_exit;
        }
        
        headerCnt++;
        
        /* read the header data from the receiver buffer and convert the ASCII only data to the appropriate header structure */
        if(HEADER_get(&moitessier_spi->rxBuf[headerPos], BUF_SIZE, &header))
        {
            if(DEBUG_LEVEL >= LEVEL_WARNING)
                pr_err("%s - HEADER_get(...) failed\n", __func__);
            lastHeaderPos += HEADER_SOF_FIELD_LEN;
            continue;
        }
        
        /* check if the header is valid at all, if this fails we can assume data corruption and restart searching for a valid
           header */
        if(HEADER_isValid(&header))
        {
            if(DEBUG_LEVEL >= LEVEL_WARNING)
                pr_err("%s - HEADER_isValid(...) failed\n", __func__);
            lastHeaderPos += HEADER_SOF_FIELD_LEN;
            
#if defined(USE_STATISTICS)            
            spin_lock_irq(&statistics.spinlock);
            statistics.headerCrcErrors++;
            spin_unlock_irq(&statistics.spinlock);
#endif /* USE_STATISTICS */            
            continue;
        }
        
        /* check if the payload is valid, otherwise drop data and restart searching for a valid header */
        if(HEADER_payloadIsValid(&header, &moitessier_spi->rxBuf[headerPos + HEADER_size()]))
        {
            if(DEBUG_LEVEL >= LEVEL_WARNING)
                pr_err("%s - HEADER_payloadIsValid(...) failed\n", __func__);
            
            lastHeaderPos += HEADER_size();

#if defined(USE_STATISTICS)            
            spin_lock_irq(&statistics.spinlock);
            statistics.payloadCrcErrors++;
            spin_unlock_irq(&statistics.spinlock);
#endif /* USE_STATISTICS */            
            continue;
        }
        
        /* this is an optional check... we might comment it out someday */
        if(header.payloadLen < MIN_DATA_LENGTH)
        {
            if(DEBUG_LEVEL >= LEVEL_WARNING)
                pr_err("%s - received too less data bytes (%u)\n", __func__, header.payloadLen);
            
            lastHeaderPos += HEADER_size();
            continue;
        }
        
        /* check if the received message is for the system */
        /* all none system related messages are NMEA messages and start either with $ or ! */
        if(moitessier_spi->rxBuf[headerPos + HEADER_size()] != '$' && 
            moitessier_spi->rxBuf[headerPos + HEADER_size()] != '!')
        {
            request.category = HEADER_asciiHexToUint8_t(&moitessier_spi->rxBuf[headerPos + HEADER_size() + (offsetof(request_t, category) * 2)]);
        	request.cmd = HEADER_asciiHexToUint8_t(&moitessier_spi->rxBuf[headerPos + HEADER_size() + (offsetof(request_t, cmd) * 2)]);
        	request.cmdSub = HEADER_asciiHexToUint8_t(&moitessier_spi->rxBuf[headerPos + HEADER_size() + (offsetof(request_t, cmdSub) * 2)]);
        	request.param = HEADER_asciiHexToUint8_t(&moitessier_spi->rxBuf[headerPos + HEADER_size() + (offsetof(request_t, param) * 2)]);
        	
        	response.category = request.category;
        	response.cmd = request.cmd;
        	response.cmdSub = request.cmdSub;
               
            
            posPayload = 0;
            
            if(response.category == CMD_CATEGORY_RESPONSE)
            {
                if(response.cmd == CMD_INFO)
                {
                    spin_lock_irq(&info.spinlock);
                    
                    /* the response header is stored in ASCII HEX format in the receive buffer, so we need to multiply by 2 (sizeof(response_t) * 2) to 
                       get the index to the payload */ 
                    memcpy(&info.mode, (uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + sizeof(response_t) * 2 + offsetof(struct st_info, mode)], 
                            DIM_ELEMENT(struct st_info, hwId));
                    memcpy(info.hwId, (uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + sizeof(response_t) * 2 + offsetof(struct st_info, hwId)], 
                            DIM_ELEMENT(struct st_info, hwId));
                    memcpy(info.hwVer, (uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + sizeof(response_t) * 2 + offsetof(struct st_info, hwVer)], 
                            DIM_ELEMENT(struct st_info, hwVer));
                    memcpy(info.bootVer, (uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + sizeof(response_t) * 2 + offsetof(struct st_info, bootVer)], 
                            DIM_ELEMENT(struct st_info, bootVer));
                    memcpy(info.appVer, (uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + sizeof(response_t) * 2 + offsetof(struct st_info, appVer)],
                            DIM_ELEMENT(struct st_info, appVer));
                    posPayload = sizeof(response_t) * 2 + offsetof(struct st_info, appVer) + DIM_ELEMENT(struct st_info, appVer);                       
                    info.functionality = HEADER_asciiHexToUint32_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                    posPayload += DIM_ELEMENT(struct st_info, functionality) * 2;
                    info.systemErrors = HEADER_asciiHexToUint32_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                    posPayload += DIM_ELEMENT(struct st_info, systemErrors) * 2;
                    info.serial.h = HEADER_asciiHexToUint32_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                    posPayload += DIM_ELEMENT(struct st_info_serial, h) * 2;
                    info.serial.m = HEADER_asciiHexToUint32_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                    posPayload += DIM_ELEMENT(struct st_info_serial, m) * 2;
                    info.serial.l = HEADER_asciiHexToUint32_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                    posPayload += DIM_ELEMENT(struct st_info_serial, l) * 2;
                    for(i = 0; i < NUM_RCV; i++)
                    {
                        for(k = 0; k < NUM_RCV_CHANNELS; k++)
                        {
                            info.rcv[i].config.channelFreq[k] = HEADER_asciiHexToUint32_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                            posPayload += DIM_ELEMENT(struct st_receiverConfig, channelFreq[k]) * 2;
                        }
                        info.rcv[i].config.metaDataMask = HEADER_asciiHexToUint8_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                        posPayload += DIM_ELEMENT(struct st_receiverConfig, metaDataMask) * 2;
                        info.rcv[i].config.afcRange = HEADER_asciiHexToUint32_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                        posPayload += DIM_ELEMENT(struct st_receiverConfig, afcRange) * 2;
                        info.rcv[i].config.afcRangeDefault = HEADER_asciiHexToUint32_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                        posPayload += DIM_ELEMENT(struct st_receiverConfig, afcRangeDefault) * 2;
                        info.rcv[i].config.tcxoFreq = HEADER_asciiHexToUint32_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                        posPayload += DIM_ELEMENT(struct st_receiverConfig, tcxoFreq) * 2;
                        for(k = 0; k < NUM_RCV_CHANNELS; k++)
                        {
                            info.rcv[i].rng[k] = HEADER_asciiHexToUint8_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                            posPayload += DIM_ELEMENT(struct st_info_rcv, rng[k]) * 2;
                        }
                    }
                    info.simulator.enabled = HEADER_asciiHexToUint8_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                    posPayload += DIM_ELEMENT(struct st_simulator, enabled) * 2;
                    info.simulator.interval = HEADER_asciiHexToUint32_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                    posPayload += DIM_ELEMENT(struct st_simulator, interval) * 2;
                    for(k = 0; k < NUM_RCV_CHANNELS; k++)
                    {
                        info.simulator.mmsi[k] = HEADER_asciiHexToUint32_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                        posPayload += DIM_ELEMENT(struct st_simulator, mmsi[k]) * 2;
                    }
                    info.wpEEPROM = HEADER_asciiHexToUint8_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                    posPayload += DIM_ELEMENT(struct st_info, wpEEPROM) * 2;
                    
                    info.buttonPressed = 0;
                    /* since protocol version 2 we have an additional field in the info command */
                    if(header.version >= 2)
                    {
                        info.buttonPressed = HEADER_asciiHexToUint8_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                        posPayload += DIM_ELEMENT(struct st_info, buttonPressed) * 2;
                    }
                    if(header.version >= 3)
                    {
                        memcpy(info.gnssVer, (uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload],
                            DIM_ELEMENT(struct st_info, gnssVer));
                        posPayload += DIM_ELEMENT(struct st_info, gnssVer);                       
                        info.gnssSysSupported = HEADER_asciiHexToUint8_t((uint8_t*)&moitessier_spi->rxBuf[headerPos + HEADER_size() + posPayload]);
                        posPayload += DIM_ELEMENT(struct st_info, gnssSysSupported) * 2;
                    }
                    
                    
                    if(DEBUG_LEVEL >= LEVEL_DEBUG)
                    {
                        pr_info("%s: protocol version = %d\n", __func__, header.version);                    
                    }
                    
                    info.valid = true;
                    
                    spin_unlock_irq(&info.spinlock);
                }
                
#if defined(SUPPORT_KEEP_ALIVE)
                spin_lock_irqsave(&timerKeepAlive_spinlock, iflags);
                mod_timer(&timerKeepAlive, KEEP_ALIVE_TIMEOUT(KEEP_ALIVE_TIMEOUT_SHORT_MS));
                spin_unlock_irqrestore(&timerKeepAlive_spinlock, iflags);
#endif /* SUPPORT_KEEP_ALIVE */
            }
            
            lastHeaderPos += (HEADER_size() + header.payloadLen);
            continue;
        }
        
#if defined(USE_STATISTICS)        
        spin_lock_irq(&statistics.spinlock);
        statistics.totalRxPayloadBytes += header.payloadLen;
        spin_unlock_irq(&statistics.spinlock);
#endif /* USE_STATISTICS */    

#if defined(SUPPORT_KEEP_ALIVE)
        spin_lock_irqsave(&timerKeepAlive_spinlock, iflags);
        mod_timer(&timerKeepAlive, KEEP_ALIVE_TIMEOUT(KEEP_ALIVE_TIMEOUT_SHORT_MS));
        spin_unlock_irqrestore(&timerKeepAlive_spinlock, iflags);
#endif /* SUPPORT_KEEP_ALIVE */
        
        if(DEBUG_LEVEL >= LEVEL_DEBUG)
        {
            msg = kmalloc(header.payloadLen + 1, GFP_ATOMIC);
            if(msg)
            {
                memcpy(msg, &moitessier_spi->rxBuf[headerPos + HEADER_size()], header.payloadLen);
                msg[header.payloadLen] = '\0';
                pr_info("%s - %s", __func__, msg);
                kfree(msg);      
            }
        }
        
        /* GNSS data will be processed only if GNSS is enabled */
        processData = false;
        if(header.payloadLen > 3)
        {
            spin_lock_irq(&config.spinlock);
            if(moitessier_spi->rxBuf[headerPos + HEADER_size() + 0] == '$' && \
               moitessier_spi->rxBuf[headerPos + HEADER_size() + 1] == 'G' && \
               (moitessier_spi->rxBuf[headerPos + HEADER_size() + 2] == 'N' || \
                moitessier_spi->rxBuf[headerPos + HEADER_size() + 2] == 'P' ||
                moitessier_spi->rxBuf[headerPos + HEADER_size() + 2] == 'L') &&\
                !config.gnssEnabled)
            {
                
            }
            else
            {
                processData = true;
                
                if(config.gnssEnabled)
                {
                    char talkerId[3];
                    
                    memcpy(talkerId, &moitessier_spi->rxBuf[headerPos + HEADER_size() + 0], sizeof(talkerId));
                    
                    /* verify if message is GNSS related */
                    if(talkerId[0] == '$' && talkerId[1] == 'G' && (talkerId[2] == 'P' || talkerId[2] == 'N' || talkerId[2] == 'L' || talkerId[2] == 'A'))
                    {
                        char sentence[4];
                        sentence[3] = 0;
                        
                        processData = false;
                        
                        memcpy(sentence, &moitessier_spi->rxBuf[headerPos + HEADER_size() + sizeof(talkerId)], 3);
                        
                        /* verify if we should proceed the sentence */
                        if(strcmp(sentence, "RMC") == 0 && (config.gnssMsgEnabled & GNSS_MSG_RMC))
                            processData = true; 
                        if(strcmp(sentence, "VTG") == 0 && (config.gnssMsgEnabled & GNSS_MSG_VTG))
                            processData = true; 
                        if(strcmp(sentence, "GGA") == 0 && (config.gnssMsgEnabled & GNSS_MSG_GGA))
                                processData = true; 
                        if(strcmp(sentence, "GSA") == 0 && (config.gnssMsgEnabled & GNSS_MSG_GSA))
                            processData = true; 
                        if(strcmp(sentence, "GSV") == 0 && (config.gnssMsgEnabled & GNSS_MSG_GSV))
                            processData = true; 
                        if(strcmp(sentence, "GLL") == 0 && (config.gnssMsgEnabled & GNSS_MSG_GLL))
                            processData = true; 
                        if(strcmp(sentence, "TXT") == 0 && (config.gnssMsgEnabled & GNSS_MSG_TXT))
                            processData = true;  
                        if(strcmp(sentence, "GSV") == 0 && (config.gnssMsgEnabled & GNSS_MSG_GSV))
                            processData = true;                            
                    }
                }
            }
            spin_unlock_irq(&config.spinlock);
        }
        
        /* we process the data further only, if the device specified in DEVICE_NAME_SPI is opened at all, otherwise data will
           be dropped */
        if(moitessier_spi->opened)
        {
            fifoFull = false;

#if !defined(CONFIG_ARM64)            
            spin_lock_irq(&rxFifo.spinlock);
#else
            mutex_lock(&rxFifo.lock);
#endif /* CONFIG_ARM64 */
        
            /* still space left in FIFO? */
            if((rxFifo.cnt + header.payloadLen) > rxFifo.size)
            {
                if(DEBUG_LEVEL >= LEVEL_WARNING || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
                    pr_err("%s - received too many bytes (%u) to store in fifo (%u bytes left)\n", __func__, header.payloadLen, rxFifo.size - rxFifo.cnt);
                 
                fifoFull = true;
                 
#if defined(USE_STATISTICS)                 
                spin_lock_irq(&statistics.spinlock);
                statistics.fifoOverflows++;
                spin_unlock_irq(&statistics.spinlock);
#endif /* USE_STATISTICS */                
            }

            if(!fifoFull && processData)
            {
                /* append the new data to the FIFO */
                memcpy(&rxFifo.data[rxFifo.cnt], &moitessier_spi->rxBuf[headerPos + HEADER_size()], header.payloadLen); 
                rxFifo.cnt += header.payloadLen;
                dataProcessed = true;
            }
            
#if !defined(CONFIG_ARM64)            
            spin_unlock_irq(&rxFifo.spinlock);
#else
            mutex_unlock(&rxFifo.lock);
#endif /* CONFIG_ARM64 */            
            if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
            {
                pr_info("%s - FIFO cnt: %u", __func__, rxFifo.cnt);
            }
        }
        
#if defined(USE_TTY)
        serial = moitessier_serial;
                
        mutex_lock(&serial->lock);       
        if(serial)
        {
            /* we need to check if the tty/serial device is already initialized and opened, because this function will probably
               be executed earlier */
            if(serial->initialized && serial->opened && processData)
            {
                tty = serial->tty;
             	port = tty->port;
           
                for (k = 0; k < (header.payloadLen); ++k)
            	{
            		if (!tty_buffer_request_room(port, 1))
                        tty_flip_buffer_push(port);
                    tty_insert_flip_char(port, moitessier_spi->rxBuf[headerPos + HEADER_size() + k], TTY_NORMAL);
                    
            	}
            	
#if defined(USE_MSG_CNT_TTY)          	
            	cnt++;              	
            	sprintf(cntBuf, "%lu\r\n", cnt);
            	for (k = 0; k < strlen(cntBuf); ++k)
            	{
            		if (!tty_buffer_request_room(port, 1))
                        tty_flip_buffer_push(port);
                    tty_insert_flip_char(port, cntBuf[k], TTY_NORMAL);
                    
            	}
#endif /* USE_MSG_CNT_TTY */                	
        	    tty_flip_buffer_push(port);
        	}
        }
        mutex_unlock(&serial->lock);
#endif /* USE_TTY */       
        
        lastHeaderPos += (HEADER_size() + header.payloadLen);
    }while(1);
 
moitessier_processReqData_exit:    
#if defined(USE_TTY)
    gpio_set_value(moitessier_spi->req_gpio[1].gpio, REQ_LEVEL_INACTIVE);	
#endif /* USE_TTY */            
    
    if(dataProcessed)
        SET_DATA_AVAILABLE_USER;

#if !defined(CONFIG_ARM64)
    spin_unlock_irq(&moitessier_spi->spinlock);
#endif /* CONFIG_ARM64 */
    
    if(dataProcessed)
        wake_up_interruptible(&wq_read);
}

static int moitessier_thread(void *data)
{
    unsigned long iflags;
     
    allow_signal(SIGTERM);    
    
    while(kthread_should_stop() == 0)
    {       
        if(wait_event_interruptible(wq_thread, IS_SLAVE_REQ_RECEIVED))
        {
            if(DEBUG_LEVEL >= LEVEL_DEBUG)
                pr_info("%s - signal break\n", __func__);
            break;
        }
        SLAVE_REQ_CONFIRMED;
        
        if(moitessier_spi->initialized)
        {
            moitessier_processReqData();
            
#if defined(SUPPORT_SHUTDOWN)        
            if(info.buttonPressed && (USE_SHUTDOWN_BUTTON || TEST_MODE_SHUTDOWN_BUTTON))
            {
                if(DEBUG_LEVEL >= LEVEL_WARNING)
                {
                    pr_info("%s - Shutting down in %ld msec...\n", __func__, SHUTDOWN_DELAY_MS);
                }    
                
                if(SHUTDOWN_DELAY_MS)
                    msleep(SHUTDOWN_DELAY_MS);
                    
                /* reset the HAT microcontroller */    
                spin_lock_irqsave(&moitessier_spi->spinlock, iflags);
                gpio_set_value(moitessier_spi->req_gpio[0].gpio, REQ_LEVEL_INACTIVE);
                gpio_set_value(moitessier_spi->req_gpio[1].gpio, REQ_LEVEL_INACTIVE);
                
                if(USE_REQ_FOR_RESET)
                {
                    gpio_set_value(moitessier_spi->req_gpio[0].gpio, REQ_LEVEL_INACTIVE);
                    gpio_set_value(moitessier_spi->req_gpio[0].gpio, REQ_LEVEL_ACTIVE);
                    mdelay(100);
                }
                else
                {
                    gpio_set_value(moitessier_spi->reset_gpio.gpio, RESET_LEVEL_INACTIVE);
                    gpio_set_value(moitessier_spi->reset_gpio.gpio, RESET_LEVEL_ACTIVE);
                }
                
                gpio_set_value(moitessier_spi->req_gpio[0].gpio, REQ_LEVEL_INACTIVE);
                gpio_set_value(moitessier_spi->req_gpio[1].gpio, REQ_LEVEL_INACTIVE);
                
                spin_unlock_irqrestore(&moitessier_spi->spinlock, iflags);
                
                if(!TEST_MODE_SHUTDOWN_BUTTON)
                {
                    DO_SHUTDOWN = 1;   
                    //kernel_power_off(); /* we can't power off the kernel, it is not the same as sudo shutdown now */
                }
            }
#endif /* SUPPORT_SHUTDOWN */   
        }     
    }
    
    complete(&on_exit);
    return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION_ALT(5, 9, 0)
static void irq_tasklet(struct tasklet_struct *t)
#else   
static void irq_tasklet(unsigned long data)
#endif /* LINUX_VERSION_CODE */
{
    unsigned int irq = atomic_read(&irqOccured);
    unsigned int i = 0;
    
    if(!moitessier_spi)
        return;
    
    if(!moitessier_spi->initialized)
        return;
        
    spin_lock(&moitessier_spi->spinlock);    
    
    /* determine which GPIO has caused the interrupt */
    for(i = 0; i < DIM(moitessier_spi->irq_gpio); i++)
    {
        if(moitessier_spi->irq_gpio[i].irq == irq)
            break;
    }
    
    if(i <= DIM(moitessier_spi->irq_gpio))
    {
        if(jiffies < moitessier_spi->irq_gpio[i].jiffiesEvent)
            moitessier_spi->irq_gpio[i].jiffiesEvent = jiffies;
            
        if(((((unsigned long)moitessier_spi->irq_gpio[i].jiffiesEvent + (unsigned long)msecs_to_jiffies(DEBOUNCE_MS)) < (unsigned long)jiffies)) || DEBOUNCE_MS == 0)
        {
            moitessier_spi->irq_gpio[i].jiffiesEvent = jiffies;
        }
            
        if(i == 1 && moitessier_spi->spi)
        {
            gpio_set_value(moitessier_spi->req_gpio[i].gpio, REQ_LEVEL_ACTIVE);
            SLAVE_REQ_RECEIVED;
#if defined(USE_THREAD)    
            wake_up_interruptible(&wq_thread);
#else /* !USE_THREAD */                       
            spin_unlock(&moitessier_spi->spinlock); 
            moitessier_processReqData();
            spin_lock(&moitessier_spi->spinlock); 
#endif /* !USE_THREAD */                          
        }     
    }
    
    spin_unlock(&moitessier_spi->spinlock); 
}

static irqreturn_t ctrl_IrqHandler(int irq, void* data)
{
    atomic_set(&irqOccured, irq);
    
    tasklet_hi_schedule(&irq_tl);
    
    return IRQ_HANDLED;
}

static int moitessier_probe(struct spi_device *spi)
{  
    struct device_node *moitessierNode = spi->dev.of_node;
    struct device_node *reqNodes;
    struct device_node *irqNodes;
    struct device_node *bootNode;
    struct device_node *resetNode;
    int childCnt = 0;
    int i = 0;
    int size;
    int id = 0;
    int rc = 0;	
    int k = 0;	
    enum of_gpio_flags flags;
    const void *property = NULL;

/*
include/linux/of.h
*/	
    if(DEBUG_LEVEL >= LEVEL_INFO)
        pr_info("%s\n", __func__);
	  
    /* allocate driver data */
    /* this should have been already done in the init(...) function */
    if (!moitessier_spi)
	    moitessier_spi = kzalloc(sizeof(*moitessier_spi), GFP_KERNEL);
	if (!moitessier_spi)
    {
        if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	        pr_err("%s - failed to allocate memory for \"%s\"\n", __func__, DEVICE_NAME_SPI);
		return -ENOMEM;
    }
    
    moitessier_spi->initialized = false;
    moitessier_spi->opened = false;
    
	/* initialize the driver data */
	moitessier_spi->spi = spi;

    /* determine how many child nodes are available, this is for debugging only */
    childCnt = of_get_available_child_count(moitessierNode);
    if(DEBUG_LEVEL >= LEVEL_INFO)
        pr_info("%s - childCnt: %d\n", __func__, childCnt);
      
    property = of_get_property(moitessierNode, "spi-max-frequency", &size);
    moitessier_spi->speed_hz = be32_to_cpup(property);
    if(DEBUG_LEVEL >= LEVEL_INFO)
        pr_info("%s - SPI speed: %d\n", __func__, moitessier_spi->speed_hz);
        
    reqNodes = of_get_child_by_name(moitessierNode, "imc_req");
    if(reqNodes == NULL)
    {
        if(DEBUG_LEVEL >= LEVEL_CRITICAL)
            pr_err("%s - could not find node imc_req in device tree\n", __func__);
        goto free_memory;  
    }
    
    irqNodes = of_get_child_by_name(moitessierNode, "imc_irq");
    if(irqNodes == NULL)
    {
        if(DEBUG_LEVEL >= LEVEL_CRITICAL)
            pr_err("%s - could not find node imc_irq in device tree\n", __func__);
        goto free_memory;  
    }
    
    bootNode = of_get_child_by_name(moitessierNode, "boot");
    if(bootNode == NULL)
    {
        if(DEBUG_LEVEL >= LEVEL_CRITICAL)
            pr_err("%s - could not find node boot in device tree\n", __func__);
        goto free_memory;  
    }
    
    resetNode = of_get_child_by_name(moitessierNode, "reset");
    if(resetNode == NULL)
    {
        if(DEBUG_LEVEL >= LEVEL_CRITICAL)
            pr_err("%s - could not find node reset in device tree\n", __func__);
        goto free_memory;  
    }
    
    if(DEBUG_LEVEL >= LEVEL_DEBUG)
        pr_info("%s - REQ GPIO Cnt: %d\n", __func__, of_gpio_count(reqNodes));
    if(DEBUG_LEVEL >= LEVEL_DEBUG)
        pr_info("%s - IRQ GPIO Cnt: %d\n", __func__, of_gpio_count(irqNodes));
    
    /* initialize REQ signals */
    for(i = 0; i < of_gpio_count(reqNodes); i++)
    {
        moitessier_spi->req_gpio[i].gpio = of_get_gpio_flags(reqNodes, i, &flags);
		moitessier_spi->req_gpio[i].flags = (int)flags;
        moitessier_spi->req_gpio[i].name = of_get_property(reqNodes, "rooco,name", &size);
        moitessier_spi->req_gpio[i].irq = -1;
        moitessier_spi->req_gpio[i].nr = i;
        
        gpio_request(moitessier_spi->req_gpio[i].gpio, moitessier_spi->req_gpio[i].name);
		gpio_direction_output(moitessier_spi->req_gpio[i].gpio, !moitessier_spi->req_gpio[i].flags);
        gpio_export(moitessier_spi->req_gpio[i].gpio, false); 
        
        if(DEBUG_LEVEL >= LEVEL_INFO)
        { 
            pr_info("%s - ID: %u\tGPIO Nr.: %u\tGPIO Dir: %u\tGPIO IRQ: %d\tGPIO Name: %s%u\n", \
    		        __func__,                           \
    		        id,                                 \
    		        moitessier_spi->req_gpio[i].gpio,          \
    		        moitessier_spi->req_gpio[i].flags,         \
    		        moitessier_spi->req_gpio[i].irq,           \
    		        moitessier_spi->req_gpio[i].name,          \
    		        moitessier_spi->req_gpio[i].nr             \
		      );
		}
    }
    
    /* initialize BOOT signal */ 
    moitessier_spi->boot_gpio.gpio = of_get_gpio_flags(bootNode, 0, &flags);
	moitessier_spi->boot_gpio.flags = (int)flags;
    moitessier_spi->boot_gpio.name = of_get_property(bootNode, "rooco,name", &size);
    moitessier_spi->boot_gpio.irq = -1;
    
    gpio_request(moitessier_spi->boot_gpio.gpio, moitessier_spi->boot_gpio.name);
	gpio_direction_output(moitessier_spi->boot_gpio.gpio, !moitessier_spi->boot_gpio.flags);
    gpio_export(moitessier_spi->boot_gpio.gpio, false);         
    
    if(DEBUG_LEVEL >= LEVEL_INFO)
    {
        pr_info("%s - ID: %u\tGPIO Nr.: %u\tGPIO Dir: %u\tGPIO IRQ: %d\tGPIO Name: %s\n", \
    		        __func__,                           \
    		        id,                                 \
    		        moitessier_spi->boot_gpio.gpio,          \
    		        moitessier_spi->boot_gpio.flags,         \
    		        moitessier_spi->boot_gpio.irq,           \
    		        moitessier_spi->boot_gpio.name           \
    	      );
	}
	      
    /* initialize RESET signal */	      
	moitessier_spi->reset_gpio.gpio = of_get_gpio_flags(resetNode, 0, &flags);
	moitessier_spi->reset_gpio.flags = (int)flags;
    moitessier_spi->reset_gpio.name = of_get_property(resetNode, "rooco,name", &size);
    moitessier_spi->reset_gpio.irq = -1;
    gpio_request(moitessier_spi->reset_gpio.gpio, moitessier_spi->reset_gpio.name);
    gpio_export(moitessier_spi->reset_gpio.gpio, false);         
	
    if(USE_REQ_FOR_RESET)   
    {
        gpio_direction_input(moitessier_spi->reset_gpio.gpio);
        //gpio_direction_output(moitessier_spi->reset_gpio.gpio, RESET_LEVEL_INACTIVE);
    }
    else
        gpio_direction_output(moitessier_spi->reset_gpio.gpio, moitessier_spi->reset_gpio.flags);
    
    if(DEBUG_LEVEL >= LEVEL_INFO)
    { 
        pr_info("%s - ID: %u\tGPIO Nr.: %u\tGPIO Dir: %u\tGPIO IRQ: %d\tGPIO Name: %s\n", \
    		        __func__,                           \
    		        id,                                 \
    		        moitessier_spi->reset_gpio.gpio,          \
    		        moitessier_spi->reset_gpio.flags,         \
    		        moitessier_spi->reset_gpio.irq,           \
    		        moitessier_spi->reset_gpio.name           \
    	      );
    }
    
    /* initialize IRQ signals */
    for(i = 0; i < of_gpio_count(irqNodes); i++)
    {
        moitessier_spi->irq_gpio[i].gpio = of_get_gpio_flags(irqNodes, i, &flags);
		moitessier_spi->irq_gpio[i].flags = (int)flags;
        moitessier_spi->irq_gpio[i].name = of_get_property(irqNodes, "rooco,name", &size);
        moitessier_spi->irq_gpio[i].irq = gpio_to_irq(moitessier_spi->irq_gpio[i].gpio);
        moitessier_spi->irq_gpio[i].jiffiesEvent = jiffies;
        moitessier_spi->irq_gpio[i].nr = i;
        
        gpio_request(moitessier_spi->irq_gpio[i].gpio, moitessier_spi->irq_gpio[i].name);
		gpio_direction_input(moitessier_spi->irq_gpio[i].gpio);
		gpio_set_debounce(moitessier_spi->irq_gpio[i].gpio, 10000);
        gpio_export(moitessier_spi->irq_gpio[i].gpio, false);         
        
        if(DEBUG_LEVEL >= LEVEL_INFO)
        { 
            pr_info("%s - ID: %u\tGPIO Nr.: %u\tGPIO Dir: %u\tGPIO IRQ: %d\tGPIO Name: %s%u\n", \
        		        __func__,                           \
        		        id,                                 \
        		        moitessier_spi->irq_gpio[i].gpio,          \
        		        moitessier_spi->irq_gpio[i].flags,         \
        		        moitessier_spi->irq_gpio[i].irq,           \
        		        moitessier_spi->irq_gpio[i].name,          \
        		        moitessier_spi->irq_gpio[i].nr             \
    		      );
        }
		      
		if(moitessier_spi->irq_gpio[i].irq < 0)
        {
            goto free_gpios;
        }
        
        rc = request_irq(moitessier_spi->irq_gpio[i].irq, ctrl_IrqHandler, IRQF_TRIGGER_RISING/* IRQF_TRIGGER_FALLING*/, moitessier_spi->irq_gpio[i].name, (void*)moitessier_spi);
        if(rc) 
        {
            if(DEBUG_LEVEL >= LEVEL_CRITICAL)
                pr_err("%s - request_irq() failed with error = %d irq = %d\n", __func__, rc, moitessier_spi->irq_gpio[i].irq);
            goto free_irq;
        }
    }
    
    /* The HAT might have been booted prior driver loading. We reset the HAT to ensure a specified and safe state. */
    moitessier_resetHAT(1);

    moitessier_spi->initialized = true;
	return 0;
	
	/* cleaning up... */
free_irq:
    for(k = 0; k < i; k++)
    {
        disable_irq(moitessier_spi->irq_gpio[k].irq);
        free_irq(moitessier_spi->irq_gpio[k].irq, moitessier_spi);	 
    }
free_gpios:
    for(i = 0; i < of_gpio_count(reqNodes); i++)
    {
        gpio_unexport(moitessier_spi->req_gpio[k].gpio);
        gpio_free(moitessier_spi->req_gpio[k].gpio); 
    }
    
    gpio_unexport(moitessier_spi->boot_gpio.gpio);
    gpio_free(moitessier_spi->boot_gpio.gpio);         
    gpio_unexport(moitessier_spi->reset_gpio.gpio);
    gpio_free(moitessier_spi->reset_gpio.gpio); 
    
    for(k = 0; k <= i; k++)    
    {
        gpio_unexport(moitessier_spi->irq_gpio[k].gpio);
        gpio_free(moitessier_spi->irq_gpio[k].gpio); 
    }
free_memory:
    kfree(moitessier_spi);
    return -EIO;          
}

static void moitessier_remove(struct spi_device *spi)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	    pr_info("%s\n", __func__);
	return ;
}

static struct spi_driver moitessier_spi_driver = {
	.driver = {
		.name =		"Moitessier Device Driver",
		.of_match_table = of_match_ptr(moitessier_dt_ids),
	},
	.probe =	moitessier_probe,
	.remove =	moitessier_remove,
};

/* set the proper access rights for the device specified in DEVICE_NAME_SPI */
static int moitessier_spi_uevent(struct device *dev, struct kobj_uevent_env *env)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
        pr_info("%s\n", __func__);
        
    if(dev && env)    
        add_uevent_var(env, "DEVMODE=%#o", 0666);
    return 0;
}

/* set the proper access rights for the device specified in DEVICE_NAME_CTRL */
static int moitessier_ctrl_uevent(struct device *dev, struct kobj_uevent_env *env)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
        pr_info("%s\n", __func__);
        
    if(dev && env)    
        add_uevent_var(env, "DEVMODE=%#o", 0666);
    return 0;
}

#if defined(USE_TTY)
/* user triggered function, open the device */
/* returns 0 on success or < 0 on an error */
static int moitessier_tty_open(struct tty_struct *tty, struct file *file)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
	    pr_info("%s\n", __func__);

	/* initialize the pointer in case something fails */
	tty->driver_data = NULL;
	
	if (!moitessier_serial)
		return -ENODEV;

	mutex_lock(&moitessier_serial->lock);

	/* save our structure within the tty structure */
	moitessier_serial->opened = true;
	tty->driver_data = moitessier_serial;
	moitessier_serial->tty = tty;	
	
	mutex_unlock(&moitessier_serial->lock);
	return 0;
}

/* user triggered function, close the device */
static void moitessier_tty_close(struct tty_struct *tty, struct file *file)
{
    struct st_moitessierSpi_serial *serial = tty->driver_data;
    
	if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_info("%s\n", __func__);
    
    mutex_lock(&serial->lock);    
    serial->opened = false;
    mutex_unlock(&serial->lock);
}	

static void moitessier_tty_throttle(struct tty_struct *tty)
{
	if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_info("%s\n", __func__);
        
    /* TODO: need to be implemented */                
}	

static void moitessier_tty_unthrottle(struct tty_struct *tty)
{
	if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_info("%s\n", __func__);
    
    /* TODO: need to be implemented */            
}

static int moitessier_tty_putchar(struct tty_struct *tty, unsigned char ch)
{
	struct st_moitessierSpi_serial *serial = tty->driver_data;
	
    if (!serial)
		return -ENODEV;

	mutex_lock(&serial->lock);

	if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
    {
        pr_info("%s - %u", __func__, ch);
    }	     
	
	/* TODO: need to be implemented */    
    		
	mutex_unlock(&serial->lock);
	
	return 1;
}

/* user triggered function, write data to the device */
/* returns the actual number of bytes written */
static int moitessier_tty_write(struct tty_struct *tty, const unsigned char *buffer, int count)
{
	struct st_moitessierSpi_serial *serial = tty->driver_data;
	int i;
	
    if (!serial)
		return -ENODEV;

	mutex_lock(&serial->lock);

    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
    {
    	printk(KERN_DEBUG "%s - ", __func__);
    	for (i = 0; i < count; ++i)
    	{
    		printk("%02x ", buffer[i]);        
    	}
    	printk("\n");
    }
			
	mutex_unlock(&serial->lock);
	
	return count;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION_ALT(5, 13, 0)
static int moitessier_tty_write_room(struct tty_struct *tty) 
#else
static unsigned int moitessier_tty_write_room(struct tty_struct *tty)    
#endif /* LINUX_VERSION_CODE */
{
	struct st_moitessierSpi_serial *serial = tty->driver_data;
#if LINUX_VERSION_CODE < KERNEL_VERSION_ALT(5, 13, 0)	
	int room = -EINVAL;
#else
    unsigned int room = 0;
#endif /* LINUX_VERSION_CODE */	

    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_info("%s\n", __func__);

	if (!serial)
#if LINUX_VERSION_CODE < KERNEL_VERSION_ALT(5, 13, 0)	    
		return -ENODEV;
#else
        return 0;
#endif /* LINUX_VERSION_CODE */        		

	mutex_lock(&serial->lock);

	/* calculate how much room is left in the device */
	/* TODO: implement calculation */
	room = 255;

	mutex_unlock(&serial->lock);
	return room;
}

static void moitessier_tty_set_termios(struct tty_struct *tty, const  struct ktermios *old_termios)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_info("%s\n", __func__);    
        
    /* TODO: need to be implemented */
}

/* Will be called if the device specified in DEVICE_NAME_TTY is opened the first time. This is acutally executed prior open(...). */
static int moitessier_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	int retval = -ENOMEM;
    
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_info("%s\n", __func__);

    mutex_lock(&moitessier_serial->lock);
	tty->port = kmalloc(sizeof *tty->port, GFP_KERNEL);
	if (!tty->port)
		goto err;

	tty_init_termios(tty);
	driver->ttys[0] = tty;
	
	tty_port_init(tty->port);
	tty_buffer_set_limit(tty->port, 8192);
	tty_driver_kref_get(driver);
	tty->count++;	
	
	moitessier_serial->initialized = true;
    mutex_unlock(&moitessier_serial->lock);
	return 0;

err:
    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_err("%s - err\n", __func__);
	kfree(tty->port);
	return retval;
}

static struct tty_operations serial_ops = {
	.open           = moitessier_tty_open,
	.close          = moitessier_tty_close,
	.write          = moitessier_tty_write,
	.write_room     = moitessier_tty_write_room,
	.set_termios    = moitessier_tty_set_termios,
    .install        = moitessier_tty_install,
    .throttle       = moitessier_tty_throttle,
    .unthrottle     = moitessier_tty_unthrottle,
    .put_char       = moitessier_tty_putchar,
};

static char *moitessier_tty_devnode(struct device *dev, umode_t *mode)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_info("%s\n", __func__);
        
	if (!mode)
		return NULL;
	
	*mode = 0666;
	return NULL;
}

static int moitessier_tty_uevent(struct device *dev, struct kobj_uevent_env *env)
{
    if(DEBUG_LEVEL >= LEVEL_DEBUG || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
        pr_info("%s\n", __func__);
    add_uevent_var(env, "DEVMODE=%#o", 0666);
    return 0;
}

static struct tty_driver *moitessier_tty_driver;
#endif /* USE_TTY */

static int __init moitessier_init(void)
{
    int rc = 0;
    
    if(DEBUG_LEVEL >= LEVEL_INFO)
        pr_info("%s, linux version code: %d\n", __func__, LINUX_VERSION_CODE);
    
    if(DEBUG_LEVEL >= LEVEL_INFO)
        pr_info("%s\n", __func__);
        
#if defined(CONFIG_ARM64)        
    if(DEBUG_LEVEL >= LEVEL_INFO)
        pr_info("%s - running 64 bit kernel\n", __func__);        
#endif /* CONFIG_ARM64 */        

#if defined(USE_TTY)	
	moitessier_serial = NULL;
#endif /* USE_TTY */

    /* Memory allocation must be done at this point. If SPI is not enabled the probe(...) function will not
	   be called and we will get a segmentation fault later in this function when we try to access moitessier_spi. */
	moitessier_spi = kzalloc(sizeof(*moitessier_spi), GFP_KERNEL);
	if (!moitessier_spi)
    {
        if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	        pr_err("%s - failed to allocate memory for \"%s\"\n", __func__, DEVICE_NAME_SPI);
		return -ENOMEM;
    }
    moitessier_spi->initialized = false;
    	
	rxFifo.size = FIFO_SIZE;
	rxFifo.cnt = 0;
	memset(rxFifo.data, 0, rxFifo.size);	
	
	memset(&info, 0, sizeof(struct st_info));	
	
	/* reserve device number */	
	if(alloc_chrdev_region(&moitessier_spi_nr, 0, 1, DEVICE_NAME_SPI) < 0)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	        pr_err("%s - failed to reserve device number for \"%s\"\n", __func__, DEVICE_NAME_SPI);
        goto free_mem;
	}
	
	/* allocate memory for the cdev structure */
	moitessier_spi_object = cdev_alloc(); 
	if(moitessier_spi_object == NULL)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	        pr_err("%s - failed to allocate memory for device \"%s\"\n", __func__, DEVICE_NAME_SPI);
		goto free_device_number;
	}
		
    moitessier_spi_object->owner = THIS_MODULE;
	moitessier_spi_object->ops = &moitessier_fops; 
	
	/* register cdev object at the kernel */
	if(cdev_add(moitessier_spi_object, moitessier_spi_nr, 1))
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	        pr_err("%s - failed to register cdev object at kernel\n", __func__);
		goto free_cdev;		
	}
	
    /* create new device class */
	moitessier_spi_class = class_create(THIS_MODULE, DEVICE_NAME_SPI);
	if(moitessier_spi_class == NULL)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	        pr_err("%s - failed to create device \"%s\"\n", __func__, DEVICE_NAME_SPI);
		goto free_cdev;
	}
	
	/* define function to set access permissions for the device file */
    moitessier_spi_class->dev_uevent = moitessier_spi_uevent;
    		
    /* create device file */
	moitessier_spi_dev = device_create(moitessier_spi_class, NULL, moitessier_spi_nr, NULL, "%s", DEVICE_NAME_SPI);

#if defined(USE_TTY)
    /* allocate the tty driver */
#if LINUX_VERSION_CODE < KERNEL_VERSION_ALT(5, 13, 0)    
	moitessier_tty_driver = alloc_tty_driver(1);
#else 
    moitessier_tty_driver = tty_alloc_driver(1, 0);
#endif	
	if (!moitessier_tty_driver)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
	        pr_err("%s - failed to allocate memory for TTY driver\n", __func__);
	    goto free_class;
	}
	
	if(moitessier_serial == NULL)
	{
		moitessier_serial = kmalloc(sizeof(*moitessier_serial), GFP_KERNEL);
		if (!moitessier_serial)
	    {
            if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
	            pr_err("%s - failed to allocate memory\n", __func__);
	        goto free_tty;
	    }
	    moitessier_serial->initialized = false;
	    moitessier_serial->opened = false;
	    moitessier_serial->tty = NULL;
	    //moitessier_serial->tty->port = NULL;
	}
		

	/* initialize the tty driver */
	moitessier_tty_driver->owner = THIS_MODULE;
	moitessier_tty_driver->driver_name = DEVICE_NAME_TTY;
	moitessier_tty_driver->name = DEVICE_NAME_TTY;
#if LINUX_VERSION_CODE < KERNEL_VERSION_ALT(5, 0, 0)	
	moitessier_tty_driver->major = MOITESSIER_TTY_MAJOR;
#endif /* LINUX_VERSION_CODE */	
	
#if LINUX_VERSION_CODE >= KERNEL_VERSION_ALT(4, 15, 0)
	moitessier_tty_driver->minor_start = MOITESSIER_TTY_MINOR_START;    /* if not set, there will be returned a busy error during tty_register_driver(...) */
	                                                                    /* currently it is not sure since which kernel version this issue has occured */
	                                                                    /* see https://devtalk.nvidia.com/default/topic/941448/problem-with-loading-module-via-modprobe/ */
#endif /* LINUX_VERSION_CODE */	
	moitessier_tty_driver->type = TTY_DRIVER_TYPE_SYSTEM;
	moitessier_tty_driver->subtype = SYSTEM_TYPE_CONSOLE;
	moitessier_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV | TTY_DRIVER_UNNUMBERED_NODE;
	moitessier_tty_driver->init_termios = tty_std_termios;
	moitessier_tty_driver->init_termios.c_cflag = B115200 | CS8 | CREAD | HUPCL | CLOCAL;
	moitessier_tty_driver->init_termios.c_lflag &= ~ECHO;
	moitessier_tty_driver->init_termios.c_iflag |= INLCR;
	tty_set_operations(moitessier_tty_driver, &serial_ops);
	
	/* register the tty driver */
	rc = tty_register_driver(moitessier_tty_driver);
	if(rc)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
		    pr_err("%s - failed to register TTY driver. Error code: %d\n", __func__, rc);
		goto free_tty_mem;
	}
	
	moitessier_tty_dev = tty_register_device(moitessier_tty_driver, 0, NULL);
	moitessier_tty_class = moitessier_tty_dev->class;
	if(moitessier_tty_class)
    {
        /* currently settings permissions does not work, why not??? */
        /* it works for the SPI device file but not for TTY */
	    moitessier_tty_class->dev_uevent = moitessier_tty_uevent;
	    moitessier_tty_class->devnode = moitessier_tty_devnode;
	}   
	else
    {
        if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_TTY)
            pr_err("%s - failed to register TTY device\n", __func__);
        goto free_tty_driver;
    }
#endif /* USE_TTY */    
		
     /* register driver at the SPI core */
	if(spi_register_driver(&moitessier_spi_driver) < 0)	
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
	        pr_err("%s - failed to register SPI driver\n", __func__);
		//goto free_all;
		goto free_all;
	}
	
    if(alloc_chrdev_region(&moitessier_ctrl_nr, 0, 1, DEVICE_NAME_CTRL) < 0)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL)
	        pr_err("%s - failed to reserve device number for \"%s\"\n", __func__, DEVICE_NAME_CTRL);
        goto free_spi_driver;
	}
	
	/* allocate memory for the cdev structure */
	moitessier_ctrl_object = cdev_alloc(); 
	if(moitessier_ctrl_object == NULL)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL)
	        pr_err("%s - failed to allocate memory for device \"%s\"\n", __func__, DEVICE_NAME_CTRL);
		goto free_stat_device_number;
	}
		
    moitessier_ctrl_object->owner = THIS_MODULE;
	moitessier_ctrl_object->ops = &moitessier_ctrlfops; 
	
	/* register cdev object at the kernel */
	if(cdev_add(moitessier_ctrl_object, moitessier_ctrl_nr, 1))
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL)
	        pr_err("%s - failed to register cdev object at kernel\n", __func__);
		goto free_stat_cdev;		
	}
	
    /* create new device class */
	moitessier_ctrl_class = class_create(THIS_MODULE, DEVICE_NAME_CTRL);
	if(moitessier_ctrl_class == NULL)
	{
	    if(DEBUG_LEVEL >= LEVEL_CRITICAL)
	        pr_err("%s - failed to create device \"%s\"\n", __func__, DEVICE_NAME_CTRL);
		goto free_stat_cdev;
	}
	
	/* define function to set access permissions for the device file */
    moitessier_ctrl_class->dev_uevent = moitessier_ctrl_uevent;
    		
    /* create device file */
	moitessier_ctrl_dev = device_create(moitessier_ctrl_class, NULL, moitessier_ctrl_nr, NULL, "%s", DEVICE_NAME_CTRL);
	
	RESET_DATA_AVAILABLE_USER;
	SLAVE_REQ_CONFIRMED;

    spin_lock_init(&moitessier_spi->spinlock);
#if !defined(CONFIG_ARM64)    
    spin_lock_init(&rxFifo.spinlock);
#else    
    mutex_init(&rxFifo.lock);
#endif /* CONFIG_ARM64 */    
#if defined(USE_STATISTICS)    
    spin_lock_init(&statistics.spinlock);
#endif /* USE_STATISTICS */     
    spin_lock_init(&info.spinlock);
    config.gnssEnabled = 1;     /* the GNSS is enabled by default */
    config.gnssMsgEnabled = 0xFF; /* all GNSS NMEA sentences enabled */
    spin_lock_init(&config.spinlock);

    memcpy(&configHAT, &configHATdefault, sizeof(struct st_configHAT));
    spin_lock_init(&configHAT.spinlock);

#if defined(USE_TTY)    
    mutex_init(&moitessier_serial->lock);
#endif /* USE_TTY */    
    
    init_waitqueue_head(&wq_read);

#if defined(USE_STATISTICS)    
    moitessier_resetStatistics(&statistics);
#endif /* USE_STATISTICS */    
    
    info.valid = false;

    /* initialize kernel thread */
    /* the kernel thread is used for transmitting data via SPI, this only for debugging purpose */
#if defined(USE_THREAD)    
    init_waitqueue_head(&wq_thread);
    thread_id = kthread_create(moitessier_thread, NULL, "moitessier thread");
    wake_up_process(thread_id);
#endif /* USE_THREAD */    

#if defined(SUPPORT_KEEP_ALIVE)
    wq = create_workqueue("KEEP ALIVE");
    /* initialize a timer that is used to check communication to the Moitessier HAT */
    spin_lock_init(&timerKeepAlive_spinlock);
#if LINUX_VERSION_CODE >= KERNEL_VERSION_ALT(4, 14, 0)    
    timer_setup(&timerKeepAlive, moitessier_keepAlive, 0);
#else
    init_timer(&timerKeepAlive);
    timerKeepAlive.function = moitessier_keepAlive;
    timerKeepAlive.data = 0;
#endif /* LINUX_VERSION_CODE */
    timerKeepAlive.expires = KEEP_ALIVE_TIMEOUT(KEEP_ALIVE_TIMEOUT_LONG_MS);
    add_timer(&timerKeepAlive);
#endif /* SUPPORT_KEEP_ALIVE */

    if(!moitessier_spi->initialized)
    {
        if(DEBUG_LEVEL >= LEVEL_CRITICAL || DEBUG_LEVEL == LEVEL_DEBUG_SPI)
        {
            pr_err("%s - SPI seems to be disabled. Check your configuration!! SPI and I2C must be enabled to use the Moitessier HAT.\n", __func__);
        }
    }
    return 0;
    
    /* to avoid compiler errors */
    goto free_class;
    goto free_all;
    goto free_tty_driver;
    goto free_tty_mem;
    goto free_tty;	    
    goto free_spi_driver;
    goto free_stat_device_number;
	goto free_stat_class;
	goto free_stat_cdev;
	
/* cleaning up... */
free_stat_class:
    moitessier_ctrl_class->dev_uevent = NULL;
    device_destroy(moitessier_ctrl_class, moitessier_ctrl_nr);
    class_destroy(moitessier_ctrl_class);
free_stat_cdev:
    kobject_put(&moitessier_ctrl_object->kobj);
free_stat_device_number:
    unregister_chrdev_region(moitessier_ctrl_nr, 1);
free_spi_driver:
    spi_unregister_driver(&moitessier_spi_driver);  
free_all:
#if defined(USE_TTY)    
    tty_unregister_device(moitessier_tty_driver, 0);
#endif /* USE_TTY */    
free_tty_driver:
#if defined(USE_TTY)    
    tty_unregister_driver(moitessier_tty_driver);
#endif /* USE_TTY */ 
free_tty_mem:
#if defined(USE_TTY)  
    if(moitessier_serial)
    {
        kfree(moitessier_serial);
        moitessier_serial = NULL;    
	}
#endif /* USE_TTY */	
free_tty:
#if defined(USE_TTY)
#if LINUX_VERSION_CODE < KERNEL_VERSION_ALT(5, 13, 0)          
    put_tty_driver(moitessier_tty_driver);    
#else
    tty_driver_kref_put(moitessier_tty_driver);
#endif /* LINUX_VERSION_CODE */    
#endif /* USE_TTY */ 	    
free_class:
    moitessier_spi_class->dev_uevent = NULL;
    device_destroy(moitessier_spi_class, moitessier_spi_nr);
    class_destroy(moitessier_spi_class);
free_cdev:
    kobject_put(&moitessier_spi_object->kobj);
free_device_number:
    unregister_chrdev_region(moitessier_spi_nr, 1);
free_mem:
    if(moitessier_spi)
        kfree(moitessier_spi);
    rxFifo.size = 0;
	return -EIO;	
}

static void __exit moitessier_exit(void)
{
    int i = 0;
#if defined(SUPPORT_KEEP_ALIVE)    
    unsigned long iflags;
#endif /* SUPPORT_KEEP_ALIVE */    
    
    if(DEBUG_LEVEL >= LEVEL_INFO)
        pr_info("%s\n", __func__);
    
    for(i = 0; i < DIM(moitessier_spi->irq_gpio); i++)   
	{
	    disable_irq(moitessier_spi->irq_gpio[i].irq);
        free_irq(moitessier_spi->irq_gpio[i].irq, moitessier_spi);   
    }
    
    moitessier_spi->initialized = false;
    moitessier_spi->opened = false; 
    info.valid = false; 
#if defined(USE_TTY)    
    moitessier_serial->initialized = false;
    moitessier_serial->opened = false;
#endif /* USE_TTY */
    
#if defined(SUPPORT_KEEP_ALIVE)
    atomic_set(&killKeepAlive, 1);
    spin_lock_irqsave(&timerKeepAlive_spinlock, iflags);
    mod_timer(&timerKeepAlive, KEEP_ALIVE_TIMEOUT(1));
    spin_unlock_irqrestore(&timerKeepAlive_spinlock, iflags);
    wait_for_completion(&on_exitKeepAlive);
    del_timer_sync(&timerKeepAlive);
    
    if(wq)
	{
	    flush_workqueue(wq);
	    destroy_workqueue(wq);
	}
#endif /* SUPPORT_KEEP_ALIVE */    
    
    kill_pid(task_pid(thread_id), SIGTERM, 1);
    wait_for_completion(&on_exit);
    thread_id = NULL;
#if defined(USE_TTY)    
    moitessier_tty_class->dev_uevent = NULL;
    moitessier_tty_class->devnode = NULL;

    tty_unregister_device(moitessier_tty_driver, 0);
    tty_unregister_driver(moitessier_tty_driver);
#if LINUX_VERSION_CODE < KERNEL_VERSION_ALT(5, 13, 0)          
    put_tty_driver(moitessier_tty_driver);    
#else
    tty_driver_kref_put(moitessier_tty_driver);
#endif /* LINUX_VERSION_CODE */    
	if(moitessier_serial)
	{
	    if(moitessier_serial->tty)
	    {
    		if(moitessier_serial->tty->port)
    		{
    		    kfree(moitessier_serial->tty->port);
    		    moitessier_serial->tty->port = NULL;
    		}
    	}
		kfree(moitessier_serial);
		moitessier_serial = NULL;
	}
#endif /* USE_TTY */     
    
    spi_unregister_driver(&moitessier_spi_driver);
	moitessier_spi_class->dev_uevent = NULL;
	device_destroy(moitessier_spi_class, moitessier_spi_nr);
	class_destroy(moitessier_spi_class);
	cdev_del(moitessier_spi_object);
	unregister_chrdev_region(moitessier_spi_nr, 1);
	
	
	for(i = 0; i < DIM(moitessier_spi->req_gpio); i++)   
	{
	    gpio_unexport(moitessier_spi->req_gpio[i].gpio);         
        gpio_free(moitessier_spi->req_gpio[i].gpio); 
    }
    
    gpio_unexport(moitessier_spi->boot_gpio.gpio);
    gpio_free(moitessier_spi->boot_gpio.gpio); 
    
    gpio_unexport(moitessier_spi->reset_gpio.gpio);
    gpio_free(moitessier_spi->reset_gpio.gpio); 
    
    for(i = 0; i < DIM(moitessier_spi->irq_gpio); i++)   
	{
	    gpio_unexport(moitessier_spi->irq_gpio[i].gpio);            
        gpio_free(moitessier_spi->irq_gpio[i].gpio); 
    }
    
    kfree(moitessier_spi);
    
    rxFifo.size = 0;

    moitessier_ctrl_class->dev_uevent = NULL;
	device_destroy(moitessier_ctrl_class, moitessier_ctrl_nr);
	class_destroy(moitessier_ctrl_class);
	cdev_del(moitessier_ctrl_object);
	unregister_chrdev_region(moitessier_ctrl_nr, 1);
}

module_init(moitessier_init);
module_exit(moitessier_exit);
MODULE_AUTHOR("Thomas POMS, <hwsw.development@gmail.com>");
MODULE_DESCRIPTION("Moitessier HAT Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("moitessier");
