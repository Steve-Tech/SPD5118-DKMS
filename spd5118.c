// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * spd5118.c - driver for Jedec 5118 compliant temperature sensors
 *
 * Copyright (c) 2023 René Rebe, ExactCODE GmbH; Germany.
 *
 * Inspired by ee1004.c and jc42.c.
 *
 * SPD5118 compliant temperature sensors are typically used on memory modules.
 */

#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/of.h>

/* Addresses to scan */
static const unsigned short normal_i2c[] = {
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, I2C_CLIENT_END };

/* SPD5118 registers. */
#define SPD5118_REG_TYPE		(0x00) /* MR0:MR1 */
#define SPD5118_REG_REVISION		(0x02) /* MR2 */
#define SPD5118_REG_VENDOR		(0x03) /* MR3:MR4 */
#define SPD5118_REG_I2C_LEGACY_MODE	(0x0B) /* MR11 */
#define SPD5118_REG_TEMP_CLR		(0x13) /* MR19 */
#define SPD5118_REG_TEMP_MAX		(0x1c) /* MR28:MR29 */
#define SPD5118_REG_TEMP_MIN		(0x1e) /* MR30:MR31 */
#define SPD5118_REG_TEMP_CRIT		(0x20) /* MR32:MR33 */
#define SPD5118_REG_TEMP_LCRIT		(0x22) /* MR34:MR35 */
#define SPD5118_REG_TEMP		(0x31) /* MR49:MR50 */
#define SPD5118_REG_TEMP_STATUS		(0x33) /* MR51 */

#define SPD5118_TEMP_STATUS_HIGH	(1 << 0)
#define SPD5118_TEMP_STATUS_LOW		(1 << 1)
#define SPD5118_TEMP_STATUS_CRIT	(1 << 2)
#define SPD5118_TEMP_STATUS_LCRIT	(1 << 3)

#define SPD5118_TEMP_CLR_HIGH		(1 << 0)
#define SPD5118_TEMP_CLR_LOW		(1 << 1)
#define SPD5118_TEMP_CLR_CRIT		(1 << 2)
#define SPD5118_TEMP_CLR_LCRIT		(1 << 3)

#define SPD5118_NUM_PAGES		8
#define SPD5118_PAGE_SIZE		128
#define SPD5118_PAGE_SHIFT		7
#define SPD5118_EEPROM_BASE		0x80
#define SPD5118_EEPROM_SIZE		(SPD5118_PAGE_SIZE * SPD5118_NUM_PAGES)

/* Temperature unit in millicelsius */
#define SPD5118_TEMP_UNIT (1000 / 4)
/* Representable temperature range in millicelsius */
#define SPD5118_TEMP_RANGE_MIN -256000
#define SPD5118_TEMP_RANGE_MAX 255750


static bool enable_temp_write;
module_param(enable_temp_write, bool, false);
MODULE_PARM_DESC(enable_temp_write, "Enable setting temperature thresholds");

static bool enable_alarm_write;
module_param(enable_alarm_write, bool, false);
MODULE_PARM_DESC(enable_alarm_write, "Enable resetting temperature alarms");


/* Each client has this additional data */
struct spd5118_data {
	struct mutex update_lock;	/* protect register access */
	int current_page;
	u16 vendor;
	u8 revision;
};

static bool spd5118_vendor_valid(u16 reg)
{
	u8 pfx = reg & 0xff;
	u8 id = reg >> 8;

	if (!__builtin_parity(pfx) || !__builtin_parity(id))
		return false;
	id &= 0x7f;
	if (id == 0 || id == 0x7f)
		return false;
	return true;
}

static int spd5118_temp_from_reg(u16 reg)
{
	reg = sign_extend32(reg >> 2, 11);
	return reg * SPD5118_TEMP_UNIT;
}

static u16 spd5118_temp_to_reg(int temp)
{
	temp = clamp_val(temp, SPD5118_TEMP_RANGE_MIN, SPD5118_TEMP_RANGE_MAX);
	return ((temp / SPD5118_TEMP_UNIT) & 0x7ff) << 2;
}

static int spd5118_read_temp(struct i2c_client *client, u32 attr, long *val)
{
	struct spd5118_data *data = i2c_get_clientdata(client);
	int reg, regval;

	switch (attr) {
	case hwmon_temp_input:
		reg = SPD5118_REG_TEMP;
		break;
	case hwmon_temp_max:
		reg = SPD5118_REG_TEMP_MAX;
		break;
	case hwmon_temp_min:
		reg = SPD5118_REG_TEMP_MIN;
		break;
	case hwmon_temp_crit:
		reg = SPD5118_REG_TEMP_CRIT;
		break;
	case hwmon_temp_lcrit:
		reg = SPD5118_REG_TEMP_LCRIT;
		break;
	default:
		return -EOPNOTSUPP;
	}

	mutex_lock(&data->update_lock);
	regval = i2c_smbus_read_word_data(client, reg);
	mutex_unlock(&data->update_lock);
	if (regval < 0)
		return regval;

	*val = spd5118_temp_from_reg(regval);
	return 0;
}

static int spd5118_write_temp(struct i2c_client *client, u32 attr, long val)
{
	struct spd5118_data *data = i2c_get_clientdata(client);
	int reg, ret;
	u16 regval;

	if (WARN_ON(!enable_temp_write))
		return -EOPNOTSUPP;

	switch (attr) {
	case hwmon_temp_max:
		reg = SPD5118_REG_TEMP_MAX;
		break;
	case hwmon_temp_min:
		reg = SPD5118_REG_TEMP_MIN;
		break;
	case hwmon_temp_crit:
		reg = SPD5118_REG_TEMP_CRIT;
		break;
	case hwmon_temp_lcrit:
		reg = SPD5118_REG_TEMP_LCRIT;
		break;
	default:
		return -EOPNOTSUPP;
	}

	regval = spd5118_temp_to_reg(val);
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, reg, regval);
	mutex_unlock(&data->update_lock);
	return ret;
}

static int spd5118_read_alarm(struct i2c_client *client, u32 attr, long *val)
{
	struct spd5118_data *data = i2c_get_clientdata(client);
	int mask, regval;

	switch (attr) {
	case hwmon_temp_max_alarm:
		mask = SPD5118_TEMP_STATUS_HIGH;
		break;
	case hwmon_temp_min_alarm:
		mask = SPD5118_TEMP_STATUS_LOW;
		break;
	case hwmon_temp_crit_alarm:
		mask = SPD5118_TEMP_STATUS_CRIT;
		break;
	case hwmon_temp_lcrit_alarm:
		mask = SPD5118_TEMP_STATUS_LCRIT;
		break;
	default:
		return -EOPNOTSUPP;
	}

	mutex_lock(&data->update_lock);
	regval = i2c_smbus_read_byte_data(client, SPD5118_REG_TEMP_STATUS);
	mutex_unlock(&data->update_lock);
	if (regval < 0)
		return regval;
	*val = !!(regval & mask);
	return 0;
}

static int spd5118_clear_alarm(struct i2c_client *client, u32 attr)
{
	struct spd5118_data *data = i2c_get_clientdata(client);
	int ret;
	u8 regval;

	if (WARN_ON(!enable_alarm_write))
		return -EOPNOTSUPP;

	switch (attr) {
	case hwmon_temp_max_alarm:
		regval = SPD5118_TEMP_CLR_HIGH;
		break;
	case hwmon_temp_min_alarm:
		regval = SPD5118_TEMP_CLR_LOW;
		break;
	case hwmon_temp_crit_alarm:
		regval = SPD5118_TEMP_CLR_CRIT;
		break;
	case hwmon_temp_lcrit_alarm:
		regval = SPD5118_TEMP_CLR_LCRIT;
		break;
	default:
		return -EOPNOTSUPP;
	}

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, SPD5118_REG_TEMP_CLR, regval);
	mutex_unlock(&data->update_lock);
	return ret;
}

static int spd5118_read(struct device *dev, enum hwmon_sensor_types type,
		     u32 attr, int channel, long *val)
{
	struct i2c_client *client = dev_get_drvdata(dev);

	if (type != hwmon_temp)
		return -EOPNOTSUPP;

	switch (attr) {
	case hwmon_temp_input:
	case hwmon_temp_max:
	case hwmon_temp_min:
	case hwmon_temp_crit:
	case hwmon_temp_lcrit:
		return spd5118_read_temp(client, attr, val);
	case hwmon_temp_max_alarm:
	case hwmon_temp_min_alarm:
	case hwmon_temp_crit_alarm:
	case hwmon_temp_lcrit_alarm:
		return spd5118_read_alarm(client, attr, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int spd5118_write(struct device *dev, enum hwmon_sensor_types type,
		      u32 attr, int channel, long val)
{
	struct i2c_client *client = dev_get_drvdata(dev);

	if (type != hwmon_temp)
		return -EOPNOTSUPP;

	switch (attr) {
	case hwmon_temp_max:
	case hwmon_temp_min:
	case hwmon_temp_crit:
	case hwmon_temp_lcrit:
		return spd5118_write_temp(client, attr, val);
	case hwmon_temp_max_alarm:
	case hwmon_temp_min_alarm:
	case hwmon_temp_crit_alarm:
	case hwmon_temp_lcrit_alarm:
		if (val)
			return -EINVAL;
		return spd5118_clear_alarm(client, attr);
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t spd5118_is_visible(const void *_data, enum hwmon_sensor_types type,
			       u32 attr, int channel)
{
	if (type != hwmon_temp)
		return 0;

	switch (attr) {
	case hwmon_temp_input:
		return 0444;
	case hwmon_temp_min:
	case hwmon_temp_max:
	case hwmon_temp_crit:
	case hwmon_temp_lcrit:
		return enable_temp_write ? 0644 : 0444;
	case hwmon_temp_min_alarm:
	case hwmon_temp_max_alarm:
	case hwmon_temp_crit_alarm:
	case hwmon_temp_lcrit_alarm:
		return enable_alarm_write ? 0644 : 0444;
	default:
		return 0;
	}
}

static ssize_t
revision_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spd5118_data *data = dev_get_drvdata(dev);
	/* From JESD300-5B
	 *   MR2 bits [5:4]: Major revision, 1..4
	 *   MR2 bits [3:1]: Minor revision, 0..8? Probably a typo.
	 */
	return sprintf(buf, "%d.%d\n", 1 + ((data->revision >> 4) & 3), (data->revision >> 1) & 7);
}

static DEVICE_ATTR_RO(revision);

static ssize_t
pmic_vendor_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spd5118_data *data = dev_get_drvdata(dev);
	u8 pfx = data->vendor & 0x7f;
	u8 id = (data->vendor >> 8) & 0x7f;
	int n = 0;

	while (pfx--) {
		n += 3;
		*buf++ = '7';
		*buf++ = 'F';
		*buf++ = ' ';
	}
	/* Low byte is number of 7F prefixes of the JEP106 ID */
	return n + sprintf(buf, "%02X\n", id);
}

static DEVICE_ATTR_RO(pmic_vendor_id);

static struct attribute *spd5118_attrs[] = {
	&dev_attr_revision.attr,
	&dev_attr_pmic_vendor_id.attr,
	NULL,
};

static int spd5118_set_current_page(struct i2c_client *client, int page)
{
	struct device *dev = &client->dev;
	struct spd5118_data *data = dev_get_drvdata(dev);
	int ret;

	if (page == data->current_page)
		return 0;

	ret = i2c_smbus_write_byte_data(client, SPD5118_REG_I2C_LEGACY_MODE, page);
	if (ret < 0) {
		dev_err(dev, "Failed to select page %d (%d)\n", page, ret);
		return ret;
	}

	dev_dbg(dev, "Selected page %d\n", page);
	data->current_page = page;

	return 0;
}

static ssize_t spd5118_eeprom_read(struct i2c_client *client, char *buf,
				  unsigned int offset, size_t count)
{
	int status, page;

	page = offset >> SPD5118_PAGE_SHIFT;
	offset &= (1 << SPD5118_PAGE_SHIFT) - 1;

	status = spd5118_set_current_page(client, page);
	if (status)
		return status;

	/* Can't cross page boundaries */
	if (offset + count > SPD5118_PAGE_SIZE)
		count = SPD5118_PAGE_SIZE - offset;

	return i2c_smbus_read_i2c_block_data_or_emulated(client, SPD5118_EEPROM_BASE + offset, count, buf);
}

static ssize_t eeprom_read(struct file *filp, struct kobject *kobj,
			   struct bin_attribute *bin_attr,
			   char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = kobj_to_i2c_client(kobj);
	struct device *dev = &client->dev;
	struct spd5118_data *data = dev_get_drvdata(dev);
	size_t requested = count;
	int ret = 0;

	mutex_lock(&data->update_lock);

	while (count) {
		ret = spd5118_eeprom_read(client, buf, off, count);
		if (ret < 0)
			goto out;

		buf += ret;
		off += ret;
		count -= ret;
	}
out:

	mutex_unlock(&data->update_lock);

	return ret < 0 ? ret : requested;
}

static BIN_ATTR_RO(eeprom, SPD5118_EEPROM_SIZE);

static struct bin_attribute *spd5118_bin_attrs[] = {
	&bin_attr_eeprom,
	NULL
};

static const struct attribute_group spd5118_attr_group = {
	.attrs = spd5118_attrs,
	.bin_attrs = spd5118_bin_attrs,
};

static const struct attribute_group *spd5118_groups[] = {
	&spd5118_attr_group,
	NULL,
};

/* Return 0 if detection is successful, -ENODEV otherwise */
static int spd5118_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	int cap, vendor;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	cap = i2c_smbus_read_word_swapped(client, SPD5118_REG_TYPE);
	if (cap != 0x5118)
		return -ENODEV;

	vendor = i2c_smbus_read_word_data(client, SPD5118_REG_VENDOR);
	if (vendor < 0 || !spd5118_vendor_valid(vendor))
		return -ENODEV;

	strscpy(info->type, "spd5118", I2C_NAME_SIZE);
	return 0;
}

static const struct hwmon_channel_info *spd5118_info[] = {
	HWMON_CHANNEL_INFO(chip,
			   HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT |
			   HWMON_T_LCRIT | HWMON_T_LCRIT_ALARM |
			   HWMON_T_MIN | HWMON_T_MIN_ALARM |
			   HWMON_T_MAX | HWMON_T_MAX_ALARM |
			   HWMON_T_CRIT | HWMON_T_CRIT_ALARM),
	NULL
};

static const struct hwmon_ops spd5118_hwmon_ops = {
	.is_visible = spd5118_is_visible,
	.read = spd5118_read,
	.write = spd5118_write,
};

static const struct hwmon_chip_info spd5118_chip_info = {
	.ops = &spd5118_hwmon_ops,
	.info = spd5118_info,
};

static int spd5118_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	unsigned int typ, revision, vendor;
	struct spd5118_data *data;

	typ = i2c_smbus_read_word_swapped(client, SPD5118_REG_TYPE);
	if (typ != 0x5118) {
		dev_dbg(dev, "Device type incorrect (%d)\n", typ);
		return -ENODEV;
	}

	revision = i2c_smbus_read_byte_data(client, SPD5118_REG_REVISION);
	if (revision < 0)
		return -ENODEV;

	vendor = i2c_smbus_read_word_data(client, SPD5118_REG_VENDOR);
	if (vendor < 0)
		return -ENODEV;

	data = devm_kzalloc(dev, sizeof(struct spd5118_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);

	mutex_init(&data->update_lock);
	data->current_page = -1;
	data->vendor = vendor;
	data->revision = revision;

	hwmon_dev = devm_hwmon_device_register_with_info(dev, "spd5118",
							 client, &spd5118_chip_info,
							 NULL);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static void spd5118_remove(struct i2c_client *client)
{
}

static const struct i2c_device_id spd5118_id[] = {
	{ "spd5118", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, spd5118_id);

#ifdef CONFIG_OF
static const struct of_device_id spd5118_of_ids[] = {
	{ .compatible = "jedec,spd5118", },
	{ }
};
MODULE_DEVICE_TABLE(of, spd5118_of_ids);
#endif

static struct i2c_driver spd5118_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "spd5118",
		.dev_groups = spd5118_groups,
		.of_match_table = of_match_ptr(spd5118_of_ids),
	},
	.probe		= spd5118_probe,
	.remove		= spd5118_remove,
	.id_table	= spd5118_id,
	.detect		= spd5118_detect,
	.address_list	= normal_i2c,
};

module_i2c_driver(spd5118_driver);

MODULE_AUTHOR("René Rebe <rene@exactcode.de>");
MODULE_DESCRIPTION("SPD 5118 driver");
MODULE_LICENSE("GPL");
