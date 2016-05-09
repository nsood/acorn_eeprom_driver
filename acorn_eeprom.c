/*
*base on at24.c - handle ACORN I2C EEPROMs
*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/mod_devicetable.h>
#include <linux/log2.h>
#include <linux/bitops.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/of_memory_accessor.h>

struct at24_data {
	struct at24_platform_data chip;
	struct memory_accessor macc;
	int use_smbus;

	/*
	 * Lock protects against activities from other Linux tasks,
	 * but not from changes by other I2C masters.
	 */
	struct mutex lock;
	struct bin_attribute bin;

	u8 *writebuf;
	unsigned write_max;
	unsigned num_addresses;

	/*
	 * Some chips tie up multiple I2C addresses; dummy devices reserve
	 * them for us, and we'll use them with SMBus calls.
	 */
	struct i2c_client *client[];
};

struct acorn_tlv{
	uint16_t	type;
	uint16_t	length;
};
/* define company id, it will be filled in higher 8 bits in tlv type */
#define ACORN_COMPANY_ID 0xac
/* define ACORN eeprom tlv type */ 
enum acorn_eeprom_type{
	ACORN_SERIAL_NUM, 	/* product serial number */ 
	ACORN_MGMT_MAC,		/* management port MAC address */
	ACORN_ETH0_MAC,		/* ETH0(data port 0) MAC address */
	ACORN_PCB_VER,		/* PCB version */
	ACORN_PD,		/* Production Date */
	ACORN_MADEIN,		/* Made In */
	ACORN_PRODUCT_TYPE,
	ACORN_PCBA_SN,
    	ACORN_BOOTFILE,
	ACORN_EEPROM_TYPE_MAX, 
};
/* define the length of each item in eeprom */
#define ACORN_TLV_SIZE			4
#define ACORN_SERIAL_NUM_LEN 		32
#define	ACORN_MAC_LEN			6
#define ACORN_PCB_VER_LEN		4
#define ACORN_PRODUCTION_DATE_LEN	16
#define ACORN_MADEIN_LEN		10
#define ACORN_PRODUCT_TYPE_LEN		10
#define ACORN_PCBA_SN_LEN		14
#define ACORN_BOOTFILE_LEN  		60
#define ACORN_EPROM_TYPE_STR_MAX 0x10

#define ACORN_SERIAL_NUM_OFFSET 	0
#define	ACORN_MGMT_MAC_OFFSET		ACORN_SERIAL_NUM_OFFSET+ACORN_TLV_SIZE+ACORN_SERIAL_NUM_LEN
#define ACORN_ETH0_MAC_OFFSET		ACORN_MGMT_MAC_OFFSET+ACORN_TLV_SIZE+ACORN_MAC_LEN
#define ACORN_PCB_VER_OFFSET		ACORN_ETH0_MAC_OFFSET+ACORN_TLV_SIZE+ACORN_MAC_LEN
#define ACORN_PRODUCTION_DATE_OFFSET	ACORN_PCB_VER_OFFSET+ACORN_TLV_SIZE+ACORN_PCB_VER_LEN
#define ACORN_MADEIN_OFFSET		ACORN_PRODUCTION_DATE_OFFSET+ACORN_TLV_SIZE+ACORN_PRODUCTION_DATE_LEN
#define ACORN_PRODUCT_TYPE_OFFSET	ACORN_MADEIN_OFFSET+ACORN_TLV_SIZE+ACORN_MADEIN_LEN
#define ACORN_PCBA_SN_OFFSET		ACORN_PRODUCT_TYPE_OFFSET+ACORN_TLV_SIZE+ACORN_PRODUCT_TYPE_LEN
#define ACORN_BOOTFILE_OFFSET  		ACORN_PCBA_SN_OFFSET+ACORN_TLV_SIZE+ACORN_PCBA_SN_LEN

static int acorn_eeprom_type2off[ACORN_EEPROM_TYPE_MAX] = {
	[ACORN_SERIAL_NUM]	=	ACORN_SERIAL_NUM_OFFSET,
	[ACORN_MGMT_MAC]	=	ACORN_MGMT_MAC_OFFSET,
	[ACORN_ETH0_MAC]	=	ACORN_ETH0_MAC_OFFSET,
	[ACORN_PCB_VER]		=	ACORN_PCB_VER_OFFSET,	
	[ACORN_PD]		=	ACORN_PRODUCTION_DATE_OFFSET,	
	[ACORN_MADEIN]		=	ACORN_MADEIN_OFFSET,
	[ACORN_PRODUCT_TYPE]	=	ACORN_PRODUCT_TYPE_OFFSET,
	[ACORN_PCBA_SN]		=	ACORN_PCBA_SN_OFFSET,
    [ACORN_BOOTFILE]    	=	ACORN_BOOTFILE_OFFSET,
};
static int acorn_eeprom_type2len[ACORN_EEPROM_TYPE_MAX] = {
	[ACORN_SERIAL_NUM]	=	ACORN_SERIAL_NUM_LEN,
	[ACORN_MGMT_MAC]	=	ACORN_MAC_LEN,
	[ACORN_ETH0_MAC]	=	ACORN_MAC_LEN,
	[ACORN_PCB_VER]		=	ACORN_PCB_VER_LEN,	
	[ACORN_PD]		=	ACORN_PRODUCTION_DATE_LEN,	
	[ACORN_MADEIN]		=	ACORN_MADEIN_LEN,
	[ACORN_PRODUCT_TYPE]	=	ACORN_PRODUCT_TYPE_LEN,
	[ACORN_PCBA_SN]		=	ACORN_PCBA_SN_LEN,
    [ACORN_BOOTFILE]    	=	ACORN_BOOTFILE_LEN,
};

static char acorn_eeprom_type2str[ACORN_EEPROM_TYPE_MAX][ACORN_EPROM_TYPE_STR_MAX] = {
    	[ACORN_SERIAL_NUM]	=	"serialnum",
    	[ACORN_MGMT_MAC]	=	"mgmtmac",
	[ACORN_ETH0_MAC]	=	"eth0mac",
	[ACORN_PCB_VER]		=	"pcbver",
	[ACORN_PD]		=	"prodate",
	[ACORN_MADEIN]		=	"madein",
	[ACORN_PRODUCT_TYPE]	=	"protype",
	[ACORN_PCBA_SN]         =   	"pcbasn",
    	[ACORN_BOOTFILE]	=   	"bootfile"
};

/*
 * This parameter is to help this driver avoid blocking other drivers out
 * of I2C for potentially troublesome amounts of time. With a 100 kHz I2C
 * clock, one 256 byte read takes about 1/43 second which is excessive;
 * but the 1/170 second it takes at 400 kHz may be quite reasonable; and
 * at 1 MHz (Fm+) a 1/430 second delay could easily be invisible.
 *
 * This value is forced to be a power of two so that writes align on pages.
 */
static unsigned io_limit = 128;
module_param(io_limit, uint, 0);
MODULE_PARM_DESC(io_limit, "Maximum bytes per I/O (default 128)");

/*
 * Specs often allow 5 msec for a page write, sometimes 20 msec;
 * it's important to recover from write timeouts.
 */
static unsigned write_timeout = 25;
module_param(write_timeout, uint, 0);
MODULE_PARM_DESC(write_timeout, "Time (in ms) to try writes (default 25)");

#define AT24_SIZE_BYTELEN 5
#define AT24_SIZE_FLAGS 8

#define AT24_BITMASK(x) (BIT(x) - 1)

/* create non-zero magic value for given eeprom parameters */
#define AT24_DEVICE_MAGIC(_len, _flags) 		\
	((1 << AT24_SIZE_FLAGS | (_flags)) 		\
	    << AT24_SIZE_BYTELEN | ilog2(_len))

static const struct i2c_device_id acorn_eeprom_ids[] = {
	{ "acorn-eeprom", AT24_DEVICE_MAGIC(65536 / 8, AT24_FLAG_ADDR16) },
	{ /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(i2c, acorn_eeprom_ids);

/*-------------------------------------------------------------------------*/

/*
 * This routine supports chips which consume multiple I2C addresses. It
 * computes the addressing information to be used for a given r/w request.
 * Assumes that sanity checks for offset happened at sysfs-layer.
 */
static struct i2c_client *at24_translate_offset(struct at24_data *at24,
		unsigned *offset)
{
	unsigned i;

	if (at24->chip.flags & AT24_FLAG_ADDR16) {
		i = *offset >> 16;
		*offset &= 0xffff;
	} else {
		i = *offset >> 8;
		*offset &= 0xff;
	}

	return at24->client[i];
}

static ssize_t at24_eeprom_read(struct at24_data *at24, char *buf,
		unsigned offset, size_t count)
{
	struct i2c_msg msg[2];
	u8 msgbuf[2];
	struct i2c_client *client;
	unsigned long timeout, read_time;
	int status, i;

	memset(msg, 0, sizeof(msg));

	/*
	 * REVISIT some multi-address chips don't rollover page reads to
	 * the next slave address, so we may need to truncate the count.
	 * Those chips might need another quirk flag.
	 *
	 * If the real hardware used four adjacent 24c02 chips and that
	 * were misconfigured as one 24c08, that would be a similar effect:
	 * one "eeprom" file not four, but larger reads would fail when
	 * they crossed certain pages.
	 */

	/*
	 * Slave address and byte offset derive from the offset. Always
	 * set the byte address; on a multi-master board, another master
	 * may have changed the chip's "current" address pointer.
	 */
	client = at24_translate_offset(at24, &offset);

	if (count > io_limit)
		count = io_limit;

	switch (at24->use_smbus) {
	case I2C_SMBUS_I2C_BLOCK_DATA:
		/* Smaller eeproms can work given some SMBus extension calls */
		if (count > I2C_SMBUS_BLOCK_MAX)
			count = I2C_SMBUS_BLOCK_MAX;
		break;
	case I2C_SMBUS_WORD_DATA:
		count = 2;
		break;
	case I2C_SMBUS_BYTE_DATA:
		count = 1;
		break;
	default:
		/*
		 * When we have a better choice than SMBus calls, use a
		 * combined I2C message. Write address; then read up to
		 * io_limit data bytes. Note that read page rollover helps us
		 * here (unlike writes). msgbuf is u8 and will cast to our
		 * needs.
		 */
		i = 0;
		if (at24->chip.flags & AT24_FLAG_ADDR16)
			msgbuf[i++] = offset >> 8;
		msgbuf[i++] = offset;

		msg[0].addr = client->addr;
		msg[0].buf = msgbuf;
		msg[0].len = i;

		msg[1].addr = client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].buf = buf;
		msg[1].len = count;
	}

	/*
	 * Reads fail if the previous write didn't complete yet. We may
	 * loop a few times until this one succeeds, waiting at least
	 * long enough for one entire page write to work.
	 */
	timeout = jiffies + msecs_to_jiffies(write_timeout);
	do {
		read_time = jiffies;
		switch (at24->use_smbus) {
		case I2C_SMBUS_I2C_BLOCK_DATA:
			status = i2c_smbus_read_i2c_block_data(client, offset,
					count, buf);
			break;
		case I2C_SMBUS_WORD_DATA:
			status = i2c_smbus_read_word_data(client, offset);
			if (status >= 0) {
				buf[0] = status & 0xff;
				buf[1] = status >> 8;
				status = count;
			}
			break;
		case I2C_SMBUS_BYTE_DATA:
			status = i2c_smbus_read_byte_data(client, offset);
			if (status >= 0) {
				buf[0] = status;
				status = count;
			}
			break;
		default:
			status = i2c_transfer(client->adapter, msg, 2);
			if (status == 2)
				status = count;
		}
		dev_dbg(&client->dev, "read %zu@%d --> %d (%ld)\n",
				count, offset, status, jiffies);

		if (status == count)
			return count;

		/* REVISIT: at HZ=100, this is sloooow */
		msleep(1);
	} while (time_before(read_time, timeout));

	return -ETIMEDOUT;
}

static ssize_t at24_read(struct at24_data *at24,
		char *buf, loff_t off, size_t count)
{
	ssize_t retval = 0;

	if (unlikely(!count))
		return count;

	/*
	 * Read data from chip, protecting against concurrent updates
	 * from this host, but not from other I2C masters.
	 */
	mutex_lock(&at24->lock);

	while (count) {
		ssize_t	status;

		status = at24_eeprom_read(at24, buf, off, count);
		if (status <= 0) {
			if (retval == 0)
				retval = status;
			break;
		}
		buf += status;
		off += status;
		count -= status;
		retval += status;
	}

	mutex_unlock(&at24->lock);

	return retval;
}

static ssize_t at24_bin_read(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	struct at24_data *at24;

	at24 = dev_get_drvdata(container_of(kobj, struct device, kobj));
	return at24_read(at24, buf, off, count);
}


/*
 * Note that if the hardware write-protect pin is pulled high, the whole
 * chip is normally write protected. But there are plenty of product
 * variants here, including OTP fuses and partial chip protect.
 *
 * We only use page mode writes; the alternative is sloooow. This routine
 * writes at most one page.
 */
static ssize_t at24_eeprom_write(struct at24_data *at24, const char *buf,
		unsigned offset, size_t count)
{
	struct i2c_client *client;
	struct i2c_msg msg;
	ssize_t status;
	unsigned long timeout, write_time;
	unsigned next_page;

	/* Get corresponding I2C address and adjust offset */
	client = at24_translate_offset(at24, &offset);

	/* write_max is at most a page */
	if (count > at24->write_max)
		count = at24->write_max;

	/* Never roll over backwards, to the start of this page */
	next_page = roundup(offset + 1, at24->chip.page_size);
	if (offset + count > next_page)
		count = next_page - offset;

	/* If we'll use I2C calls for I/O, set up the message */
	if (!at24->use_smbus) {
		int i = 0;

		msg.addr = client->addr;
		msg.flags = 0;

		/* msg.buf is u8 and casts will mask the values */
		msg.buf = at24->writebuf;
		if (at24->chip.flags & AT24_FLAG_ADDR16)
			msg.buf[i++] = offset >> 8;

		msg.buf[i++] = offset;
		memcpy(&msg.buf[i], buf, count);
		msg.len = i + count;
	}

	/*
	 * Writes fail if the previous one didn't complete yet. We may
	 * loop a few times until this one succeeds, waiting at least
	 * long enough for one entire page write to work.
	 */
	timeout = jiffies + msecs_to_jiffies(write_timeout);
	do {
		write_time = jiffies;
		if (at24->use_smbus) {
			status = i2c_smbus_write_i2c_block_data(client,
					offset, count, buf);
			if (status == 0)
				status = count;
		} else {
			status = i2c_transfer(client->adapter, &msg, 1);
			if (status == 1)
				status = count;
		}
		dev_dbg(&client->dev, "write %zu@%d --> %zd (%ld)\n",
				count, offset, status, jiffies);

		if (status == count)
			return count;

		/* REVISIT: at HZ=100, this is sloooow */
		msleep(1);
	} while (time_before(write_time, timeout));

	return -ETIMEDOUT;
}

static ssize_t at24_write(struct at24_data *at24, const char *buf, loff_t off,
			  size_t count)
{
	ssize_t retval = 0;

	if (unlikely(!count))
		return count;

	/*
	 * Write data to chip, protecting against concurrent updates
	 * from this host, but not from other I2C masters.
	 */
	mutex_lock(&at24->lock);

	while (count) {
		ssize_t	status;

		status = at24_eeprom_write(at24, buf, off, count);
		if (status <= 0) {
			if (retval == 0)
				retval = status;
			break;
		}
		buf += status;
		off += status;
		count -= status;
		retval += status;
	}

	mutex_unlock(&at24->lock);

	return retval;
}

static ssize_t at24_bin_write(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	struct at24_data *at24;

	at24 = dev_get_drvdata(container_of(kobj, struct device, kobj));
	return at24_write(at24, buf, off, count);
}

/*-------------------------------------------------------------------------*/

/*
 * This lets other kernel code access the eeprom data. For example, it
 * might hold a board's Ethernet address, or board-specific calibration
 * data generated on the manufacturing floor.
 */

static ssize_t at24_macc_read(struct memory_accessor *macc, char *buf,
			 off_t offset, size_t count)
{
	struct at24_data *at24 = container_of(macc, struct at24_data, macc);

	return at24_read(at24, buf, offset, count);
}

static ssize_t at24_macc_write(struct memory_accessor *macc, const char *buf,
			  off_t offset, size_t count)
{
	struct at24_data *at24 = container_of(macc, struct at24_data, macc);

	return at24_write(at24, buf, offset, count);
}

static size_t show_eeprom_data(struct device *dev, int type, char *buf)
{
	int i=0;
	int len=acorn_eeprom_type2len[type];
	char tmp_buf[50]={'\0'};
	struct at24_data *at24;
	at24 = dev_get_drvdata(dev);
	at24_read(at24, tmp_buf, acorn_eeprom_type2off[type]+ACORN_TLV_SIZE, len);
	for(i=0; i<len; i++) {
		if(tmp_buf[i]=='\n')
			break;
	}
	strncpy(buf, tmp_buf, i);
	return i;
}

static size_t store_eeprom_data(struct device *dev,
					int type,const char *buf,size_t count)
{
	int ret=0;
	int len = acorn_eeprom_type2len[type];
	char *tmp_buffer;
	struct at24_data *at24;
	
	if (count > len)
		count = len;
	
	tmp_buffer = kmalloc(sizeof(struct acorn_tlv) + len,0);
	((struct acorn_tlv*)tmp_buffer)->type = (ACORN_COMPANY_ID << 8) + type;
	((struct acorn_tlv*)tmp_buffer)->length = len;
	
	memcpy((struct acorn_tlv*)tmp_buffer + 1, buf, count);			

	at24 = dev_get_drvdata(dev);
	printk("store serial num:%s",buf);
	ret =  at24_write(at24, (char *)tmp_buffer, acorn_eeprom_type2off[type], count+ACORN_TLV_SIZE);
	kfree(tmp_buffer);
	
	return ret;
}

static ssize_t show_serialnum(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int ret=0;
	
	ret = show_eeprom_data(dev,ACORN_SERIAL_NUM,buf);
	return ret;
}

static ssize_t store_serialnum(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret=0;
	
	ret = store_eeprom_data(dev,ACORN_SERIAL_NUM,buf,count);
	//printk("D:ret=%d,count=%d",ret,count); 
	//normally count + sizeof(tlv) == ret
	//here must return the num bytes of input 
	//or it would print "sh: write error: Bad address"
	return count;
}

static ssize_t show_mgmtmac(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int ret=0;
	
	ret = show_eeprom_data(dev,ACORN_MGMT_MAC,buf);
	return ret;
}

static ssize_t store_mgmtmac(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret=0;
	
	ret = store_eeprom_data(dev,ACORN_MGMT_MAC,buf,count);

	return count;
}

static ssize_t show_eth0mac(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int ret=0;
	
	ret = show_eeprom_data(dev,ACORN_ETH0_MAC,buf);
	return ret;
}

static ssize_t store_eth0mac(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret=0;
	
	ret = store_eeprom_data(dev,ACORN_ETH0_MAC,buf,count);
	return count;
}

static ssize_t show_pcbver(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int ret=0;
	
	ret = show_eeprom_data(dev,ACORN_PCB_VER,buf);
	return ret;
}

static ssize_t store_pcbver(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret=0;
	
	ret = store_eeprom_data(dev,ACORN_PCB_VER,buf,count);
	return count;
}

static ssize_t show_prodate(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int ret=0;
	
	ret = show_eeprom_data(dev,ACORN_PD,buf);
	return ret;
}

static ssize_t store_prodate(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret=0;
	
	ret = store_eeprom_data(dev,ACORN_PD,buf,count);
	return count;
}

static ssize_t show_madein(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int ret=0;
	
	ret = show_eeprom_data(dev,ACORN_MADEIN,buf);
	return ret;
}

static ssize_t store_madein(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret=0;
	
	ret = store_eeprom_data(dev,ACORN_MADEIN,buf,count);
	return count;
}

static ssize_t show_protype(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int ret=0;
	
	ret = show_eeprom_data(dev,ACORN_PRODUCT_TYPE,buf);
	return ret;
}

static ssize_t store_protype(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret=0;
	
	ret = store_eeprom_data(dev,ACORN_PRODUCT_TYPE,buf,count);
	return count;
}

static ssize_t show_pcbasn(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int ret=0;
	
	ret = show_eeprom_data(dev,ACORN_PCBA_SN,buf);
	return ret;
}

static ssize_t store_pcbasn(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret=0;
	
	ret = store_eeprom_data(dev,ACORN_PCBA_SN,buf,count);
	return count;
}

static ssize_t show_bootfile(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int ret=0;
	
	ret = show_eeprom_data(dev,ACORN_BOOTFILE,buf);
	return ret;
}

static ssize_t store_bootfile(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret=0;
	
	ret = store_eeprom_data(dev,ACORN_BOOTFILE,buf,count);
	return count;
}

static DEVICE_ATTR(serialnum, 	0644, show_serialnum, 	store_serialnum);
static DEVICE_ATTR(mgmtmac, 	0644, show_mgmtmac, 	store_mgmtmac);
static DEVICE_ATTR(eth0mac, 	0644, show_eth0mac, 	store_eth0mac);
static DEVICE_ATTR(pcbver, 		0644, show_pcbver, 		store_pcbver);
static DEVICE_ATTR(prodate, 	0644, show_prodate, 	store_prodate);
static DEVICE_ATTR(madein, 		0644, show_madein, 		store_madein);
static DEVICE_ATTR(protype, 	0644, show_protype, 	store_protype);
static DEVICE_ATTR(pcbasn, 		0644, show_pcbasn, 		store_pcbasn);
static DEVICE_ATTR(bootfile, 	0644, show_bootfile, 	store_bootfile);


static const struct attribute *eeprom_attrs[] = {
	&dev_attr_serialnum.attr,
	&dev_attr_mgmtmac.attr,
	&dev_attr_eth0mac.attr,
	&dev_attr_pcbver.attr,
	&dev_attr_prodate.attr,
	&dev_attr_madein.attr,
	&dev_attr_protype.attr,
	&dev_attr_pcbasn.attr,
	&dev_attr_bootfile.attr,
	NULL,
};
struct attribute_group eeprom_attribute_group={
	.attrs = (struct attribute **) eeprom_attrs,
};

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_OF
static void at24_get_ofdata(struct i2c_client *client,
		struct at24_platform_data *chip)
{
	const __be32 *val;
	struct device_node *node = client->dev.of_node;

	if (node) {
		if (of_get_property(node, "read-only", NULL))
			chip->flags |= AT24_FLAG_READONLY;
		val = of_get_property(node, "pagesize", NULL);
		if (val)
			chip->page_size = be32_to_cpup(val);
	}
}
#else
static void at24_get_ofdata(struct i2c_client *client,
		struct at24_platform_data *chip)
{ }
#endif /* CONFIG_OF */

static int at24_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct at24_platform_data chip;
	bool writable;
	int use_smbus = 0;
	struct at24_data *at24;
	int err;
	unsigned i, num_addresses;
	kernel_ulong_t magic;

	if (client->dev.platform_data) {
		chip = *(struct at24_platform_data *)client->dev.platform_data;
	} else {
		if (!id->driver_data) {
			err = -ENODEV;
			goto err_out;
		}
		magic = id->driver_data;
		chip.byte_len = BIT(magic & AT24_BITMASK(AT24_SIZE_BYTELEN));
		magic >>= AT24_SIZE_BYTELEN;
		chip.flags = magic & AT24_BITMASK(AT24_SIZE_FLAGS);
		/*
		 * This is slow, but we can't know all eeproms, so we better
		 * play safe. Specifying custom eeprom-types via platform_data
		 * is recommended anyhow.
		 */
		chip.page_size = 1;

		/* update chipdata if OF is present */
		at24_get_ofdata(client, &chip);

		chip.setup = NULL;
		chip.context = NULL;
	}

	if (!is_power_of_2(chip.byte_len))
		dev_warn(&client->dev,
			"byte_len looks suspicious (no power of 2)!\n");
	if (!chip.page_size) {
		dev_err(&client->dev, "page_size must not be 0!\n");
		err = -EINVAL;
		goto err_out;
	}
	if (!is_power_of_2(chip.page_size))
		dev_warn(&client->dev,
			"page_size looks suspicious (no power of 2)!\n");

	/* Use I2C operations unless we're stuck with SMBus extensions. */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		if (chip.flags & AT24_FLAG_ADDR16) {
			err = -EPFNOSUPPORT;
			goto err_out;
		}
		if (i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
			use_smbus = I2C_SMBUS_I2C_BLOCK_DATA;
		} else if (i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_READ_WORD_DATA)) {
			use_smbus = I2C_SMBUS_WORD_DATA;
		} else if (i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
			use_smbus = I2C_SMBUS_BYTE_DATA;
		} else {
			err = -EPFNOSUPPORT;
			goto err_out;
		}
	}

	if (chip.flags & AT24_FLAG_TAKE8ADDR)
		num_addresses = 8;
	else
		num_addresses =	DIV_ROUND_UP(chip.byte_len,
			(chip.flags & AT24_FLAG_ADDR16) ? 65536 : 256);

	at24 = kzalloc(sizeof(struct at24_data) +
		num_addresses * sizeof(struct i2c_client *), GFP_KERNEL);
	if (!at24) {
		err = -ENOMEM;
		goto err_out;
	}

	mutex_init(&at24->lock);
	at24->use_smbus = use_smbus;
	at24->chip = chip;
	at24->num_addresses = num_addresses;

	/*
	 * Export the EEPROM bytes through sysfs, since that's convenient.
	 * By default, only root should see the data (maybe passwords etc)
	 */
	sysfs_bin_attr_init(&at24->bin);
	at24->bin.attr.name = "eeprom";
	at24->bin.attr.mode = chip.flags & AT24_FLAG_IRUGO ? S_IRUGO : S_IRUSR;
	at24->bin.read = at24_bin_read;
	at24->bin.size = chip.byte_len;

	at24->macc.read = at24_macc_read;

	writable = !(chip.flags & AT24_FLAG_READONLY);
	if (writable) {
		if (!use_smbus || i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)) {

			unsigned write_max = chip.page_size;

			at24->macc.write = at24_macc_write;

			at24->bin.write = at24_bin_write;
			at24->bin.attr.mode |= S_IWUSR;

			if (write_max > io_limit)
				write_max = io_limit;
			if (use_smbus && write_max > I2C_SMBUS_BLOCK_MAX)
				write_max = I2C_SMBUS_BLOCK_MAX;
			at24->write_max = write_max;

			/* buffer (data + address at the beginning) */
			at24->writebuf = kmalloc(write_max + 2, GFP_KERNEL);
			if (!at24->writebuf) {
				err = -ENOMEM;
				goto err_struct;
			}
		} else {
			dev_warn(&client->dev,
				"cannot write due to controller restrictions.");
		}
	}

	at24->client[0] = client;

	/* use dummy devices for multiple-address chips */
	for (i = 1; i < num_addresses; i++) {
		at24->client[i] = i2c_new_dummy(client->adapter,
					client->addr + i);
		if (!at24->client[i]) {
			dev_err(&client->dev, "address 0x%02x unavailable\n",
					client->addr + i);
			err = -EADDRINUSE;
			goto err_clients;
		}
	}

	err = sysfs_create_bin_file(&client->dev.kobj, &at24->bin);
	if (err)
		goto err_clients;

	i2c_set_clientdata(client, at24);

	err = sysfs_create_group(&client->dev.kobj, &eeprom_attribute_group);
	if (err < 0)
		goto err_group;

	dev_info(&client->dev, "%zu byte %s EEPROM, %s, %u bytes/write\n",
		at24->bin.size, client->name,
		writable ? "writable" : "read-only", at24->write_max);
	if (use_smbus == I2C_SMBUS_WORD_DATA ||
	    use_smbus == I2C_SMBUS_BYTE_DATA) {
		dev_notice(&client->dev, "Falling back to %s reads, "
			   "performance will suffer\n", use_smbus ==
			   I2C_SMBUS_WORD_DATA ? "word" : "byte");
	}

	/* export data to kernel code */
	if (chip.setup)
		chip.setup(&at24->macc, chip.context);

	if (client->dev.of_node)
		of_memory_accessor_register(&client->dev, &at24->macc);

	return 0;

err_group:
	sysfs_remove_group(&client->dev.kobj, &eeprom_attribute_group);

err_clients:
	for (i = 1; i < num_addresses; i++)
		if (at24->client[i])
			i2c_unregister_device(at24->client[i]);

	kfree(at24->writebuf);
err_struct:
	kfree(at24);
err_out:
	dev_dbg(&client->dev, "probe error %d\n", err);
	return err;
}

static int at24_remove(struct i2c_client *client)
{
	struct at24_data *at24;
	int i;

	at24 = i2c_get_clientdata(client);
	sysfs_remove_bin_file(&client->dev.kobj, &at24->bin);
	sysfs_remove_group(&client->dev.kobj, &eeprom_attribute_group);

	if (client->dev.of_node)
		of_memory_accessor_remove(&client->dev);

	for (i = 1; i < at24->num_addresses; i++)
		i2c_unregister_device(at24->client[i]);

	kfree(at24->writebuf);
	kfree(at24);
	return 0;
}

/*-------------------------------------------------------------------------*/

static struct i2c_driver acorn_eeprom_driver = {
	.driver = {
		.name = "acorn_eeprom",
		.owner = THIS_MODULE,
	},
	.probe = at24_probe,
	.remove = at24_remove,
	.id_table = acorn_eeprom_ids,
};

static int __init at24_init(void)
{
	if (!io_limit) {
		pr_err("at24: io_limit must not be 0!\n");
		return -EINVAL;
	}

	io_limit = rounddown_pow_of_two(io_limit);
	return i2c_add_driver(&acorn_eeprom_driver);
}
module_init(at24_init);

static void __exit at24_exit(void)
{
	i2c_del_driver(&acorn_eeprom_driver);
}
module_exit(at24_exit);

MODULE_DESCRIPTION("kev-u1600 eeprom driver");
MODULE_AUTHOR("Li Jian<jli@acorn-net.com>");
MODULE_LICENSE("GPL");
