/* Bosch BMI08X inertial measurement unit driver
 *
 * Copyright (c) 2022 Meta Platforms, Inc. and its affiliates
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bmi08x_accel

#include "bmi08x.h"
#include "bmi08x_config_file.h"

#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
LOG_MODULE_REGISTER(BMI08X_ACCEL, CONFIG_SENSOR_LOG_LEVEL);

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static int bmi08x_accel_transceive_i2c(const struct device *dev, uint8_t reg, bool write,
				       void *data, size_t length)
{
	const struct bmi08x_accel_config *bmi08x = dev->config;

	if (!write) {
		return i2c_write_read_dt(&bmi08x->bus.i2c, &reg, 1, data, length);
	}
	if (length > CONFIG_BMI08X_I2C_WRITE_BURST_SIZE) {
		return -EINVAL;
	}
	uint8_t buf[1 + CONFIG_BMI08X_I2C_WRITE_BURST_SIZE];

	buf[0] = reg;
	memcpy(&buf[1], data, length);
	return i2c_write_dt(&bmi08x->bus.i2c, buf, 1 + length);
}

static int bmi08x_stream_transfer_write_i2c(const struct device *dev, uint16_t index,
					    const uint8_t *stream_data, uint16_t stream_length)
{
	uint8_t asic_msb = (uint8_t)((index / 2) >> 4);
	uint8_t asic_lsb = ((index / 2) & 0x0F);

	if (bmi08x_accel_byte_write(dev, BMI08X_ACCEL_RESERVED_5B_REG, asic_lsb) != 0) {
		LOG_ERR("Cannot write index");
		return -EIO;
	}
	if (bmi08x_accel_byte_write(dev, BMI08X_ACCEL_RESERVED_5C_REG, asic_msb) != 0) {
		LOG_ERR("Cannot write index");
		return -EIO;
	}
	if (bmi08x_accel_write(dev, BMI08X_ACCEL_FEATURE_CFG_REG, (uint8_t *)stream_data,
			       stream_length) != 0) {
		LOG_ERR("Cannot write configuration for accelerometer.");
		return -EIO;
	}
	return 0;
}

static int bmi08x_write_config_file_i2c(const struct device *dev)
{
	const uint8_t *data = bmi08x_config_file;
	uint16_t length = sizeof(bmi08x_config_file);
	uint16_t index = 0;

	while (length != 0) {
		uint16_t len1 = length;

		if (len1 > CONFIG_BMI08X_I2C_WRITE_BURST_SIZE) {
			len1 = CONFIG_BMI08X_I2C_WRITE_BURST_SIZE;
		}
		if (bmi08x_stream_transfer_write_i2c(dev, index, data, len1) != 0) {
			return -EIO;
		}
		index += len1;
		data += len1;
		length -= len1;
	}
	return 0;
}

static int bmi08x_bus_check_i2c(const union bmi08x_bus *bus)
{
	return device_is_ready(bus->i2c.bus) ? 0 : -ENODEV;
}

static const struct bmi08x_accel_bus_io bmi08x_i2c_api = {.check = bmi08x_bus_check_i2c,
							  .transceive = bmi08x_accel_transceive_i2c,
							  .write_config_file =
								  bmi08x_write_config_file_i2c};

#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int bmi08x_accel_transceive_spi(const struct device *dev, uint8_t reg, bool write,
				       void *data, size_t length)
{
	const struct bmi08x_accel_config *bmi08x = dev->config;
	const struct spi_buf tx_buf[2] = {{.buf = &reg, .len = 1}, {.buf = data, .len = length}};
	const struct spi_buf_set tx = {.buffers = tx_buf, .count = write ? 2 : 1};

	if (!write) {
		uint16_t dummy;
		const struct spi_buf rx_buf[2] = {{.buf = &dummy, .len = 2},
						  {.buf = data, .len = length}};
		const struct spi_buf_set rx = {.buffers = rx_buf, .count = 2};

		return spi_transceive_dt(&bmi08x->bus.spi, &tx, &rx);
	}

	return spi_write_dt(&bmi08x->bus.spi, &tx);
}

static int bmi08x_write_config_file_spi(const struct device *dev)
{
	if (bmi08x_accel_byte_write(dev, BMI08X_ACCEL_RESERVED_5B_REG, 0) < 0) {
		LOG_ERR("Cannot write index");
		return -EIO;
	}
	if (bmi08x_accel_byte_write(dev, BMI08X_ACCEL_RESERVED_5C_REG, 0) < 0) {
		LOG_ERR("Cannot write index");
		return -EIO;
	}
	/* write config file */
	if (bmi08x_accel_write(dev, BMI08X_ACCEL_FEATURE_CFG_REG, (uint8_t *)bmi08x_config_file,
			       sizeof(bmi08x_config_file))) {
		LOG_ERR("Cannot write configuration for accelerometer.");
		return -EIO;
	}
	return 0;
}

static int bmi08x_bus_check_spi(const union bmi08x_bus *bus)
{
	return spi_is_ready(&bus->spi) ? 0 : -ENODEV;
}

static int bmi08x_bus_init_spi(const struct device *dev)
{
	uint8_t val;
	/* do a dummy read from 0x7F to activate SPI */
	if (bmi08x_accel_byte_read(dev, 0x7F, &val) < 0) {
		LOG_ERR("Cannot read from 0x7F..");
		return -EIO;
	}

	k_usleep(100);

	return 0;
}

static const struct bmi08x_accel_bus_io bmi08x_spi_api = {.check = bmi08x_bus_check_spi,
							  .bus_init = bmi08x_bus_init_spi,
							  .transceive = bmi08x_accel_transceive_spi,
							  .write_config_file =
								  bmi08x_write_config_file_spi};

#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

static inline int bmi08x_bus_check(const struct device *dev)
{
	const struct bmi08x_accel_config *config = dev->config;

	return config->api->check(&config->bus);
}

static inline int bmi08x_bus_init(const struct device *dev)
{
	const struct bmi08x_accel_config *config = dev->config;

	/* optional, only needed to initialize SPI according to datasheet */
	if (config->api->bus_init) {
		return config->api->bus_init(dev);
	}
	return 0;
}

static int bmi08x_accel_transceive(const struct device *dev, uint8_t reg, bool write, void *data,
				   size_t length)
{
	const struct bmi08x_accel_config *config = dev->config;

	return config->api->transceive(dev, reg, write, data, length);
}

int bmi08x_accel_read(const struct device *dev, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	return bmi08x_accel_transceive(dev, reg_addr | BIT(7), false, data, len);
}

int bmi08x_accel_write(const struct device *dev, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	return bmi08x_accel_transceive(dev, reg_addr, true, data, len);
}

int bmi08x_accel_byte_read(const struct device *dev, uint8_t reg_addr, uint8_t *byte)
{
	return bmi08x_accel_transceive(dev, reg_addr | BIT(7), false, byte, 1);
}

static int bmi08x_accel_word_read(const struct device *dev, uint8_t reg_addr, uint16_t *word)
{
	if (bmi08x_accel_transceive(dev, reg_addr | BIT(7), false, word, 2) != 0) {
		return -EIO;
	}

	*word = sys_le16_to_cpu(*word);

	return 0;
}

int bmi08x_accel_byte_write(const struct device *dev, uint8_t reg_addr, uint8_t byte)
{
	return bmi08x_accel_transceive(dev, reg_addr & 0x7F, true, &byte, 1);
}

int bmi08x_accel_word_write(const struct device *dev, uint8_t reg_addr, uint16_t word)
{
	uint8_t tx_word[2] = {(uint8_t)(word & 0xff), (uint8_t)(word >> 8)};

	return bmi08x_accel_transceive(dev, reg_addr & 0x7F, true, tx_word, 2);
}

int bmi08x_accel_reg_field_update(const struct device *dev, uint8_t reg_addr, uint8_t pos,
				  uint8_t mask, uint8_t val)
{
	uint8_t old_val;

	if (bmi08x_accel_byte_read(dev, reg_addr, &old_val) < 0) {
		return -EIO;
	}

	return bmi08x_accel_byte_write(dev, reg_addr, (old_val & ~mask) | ((val << pos) & mask));
}

/*
 * Output data rate map with allowed frequencies:
 * freq = freq_int + freq_milli / 1000
 *
 * Since we don't need a finer frequency resolution than milliHz, use uint16_t
 * to save some flash.
 */
static const struct {
	uint16_t freq_int;
	uint16_t freq_milli; /* User should convert to uHz before setting the
			      * SENSOR_ATTR_SAMPLING_FREQUENCY attribute.
			      */
} bmi08x_odr_map[] = {
	{0, 0},	 {0, 780}, {1, 562}, {3, 120}, {6, 250}, {12, 500}, {25, 0},
	{50, 0}, {100, 0}, {200, 0}, {400, 0}, {800, 0}, {1600, 0}, {3200, 0},
};

static int bmi08x_freq_to_odr_val(uint16_t freq_int, uint16_t freq_milli)
{
	size_t i;

	/* An ODR of 0 Hz is not allowed */
	if (freq_int == 0U && freq_milli == 0U) {
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(bmi08x_odr_map); i++) {
		if (freq_int < bmi08x_odr_map[i].freq_int ||
		    (freq_int == bmi08x_odr_map[i].freq_int &&
		     freq_milli <= bmi08x_odr_map[i].freq_milli)) {
			return i;
		}
	}

	return -EINVAL;
}

static int bmi08x_acc_odr_set(const struct device *dev, uint16_t freq_int, uint16_t freq_milli)
{
	int odr = bmi08x_freq_to_odr_val(freq_int, freq_milli);

	if (odr < BMI08X_ACCEL_ODR_12_5_HZ) {
		return odr;
	}

	return bmi08x_accel_reg_field_update(dev, BMI08X_REG_ACCEL_CONF, 0, BMI08X_ACCEL_ODR_MASK,
					     (uint8_t)odr);
}

static const struct bmi08x_range bmi085_acc_range_map[] = {
	{2, BMI085_ACCEL_RANGE_2G},
	{4, BMI085_ACCEL_RANGE_4G},
	{8, BMI085_ACCEL_RANGE_8G},
	{16, BMI085_ACCEL_RANGE_16G},
};
#define BMI085_ACC_RANGE_MAP_SIZE ARRAY_SIZE(bmi085_acc_range_map)

static const struct bmi08x_range bmi088_acc_range_map[] = {
	{3, BMI088_ACCEL_RANGE_3G},
	{6, BMI088_ACCEL_RANGE_6G},
	{12, BMI088_ACCEL_RANGE_12G},
	{24, BMI088_ACCEL_RANGE_24G},
};
#define BMI088_ACC_RANGE_MAP_SIZE ARRAY_SIZE(bmi088_acc_range_map)

static int32_t bmi08x_range_to_reg_val(uint16_t range, const struct bmi08x_range *range_map,
				       uint16_t range_map_size)
{
	int i;

	for (i = 0; i < range_map_size; i++) {
		if (range <= range_map[i].range) {
			return range_map[i].reg_val;
		}
	}

	return -EINVAL;
}

static int32_t bmi08x_reg_val_to_range(uint8_t reg_val, const struct bmi08x_range *range_map,
				       uint16_t range_map_size)
{
	int i;

	for (i = 0; i < range_map_size; i++) {
		if (reg_val == range_map[i].reg_val) {
			return range_map[i].range;
		}
	}

	return -EINVAL;
}

static int bmi08x_acc_range_set(const struct device *dev, int32_t range)
{
	struct bmi08x_accel_data *data = dev->data;
	int32_t reg_val = -1;

	if (data->accel_chip_id == BMI085_ACCEL_CHIP_ID) {
		reg_val = bmi08x_range_to_reg_val(range, bmi085_acc_range_map,
						  BMI085_ACC_RANGE_MAP_SIZE);
	} else if (data->accel_chip_id == BMI088_ACCEL_CHIP_ID) {
		reg_val = bmi08x_range_to_reg_val(range, bmi088_acc_range_map,
						  BMI088_ACC_RANGE_MAP_SIZE);
	} else {
		return -ENXIO;
	}

	if (reg_val < 0) {
		return reg_val;
	}

	if (bmi08x_accel_byte_write(dev, BMI08X_REG_ACCEL_RANGE, reg_val & 0xff) < 0) {
		return -EIO;
	}

	data->scale = BMI08X_ACC_SCALE(range);

	return 0;
}

static int bmi08x_acc_config(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, const struct sensor_value *val)
{
	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		return bmi08x_acc_range_set(dev, sensor_ms2_to_g(val));
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return bmi08x_acc_odr_set(dev, val->val1, val->val2 / 1000);
	default:
		LOG_ERR("Accel attribute not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static int bmi08x_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
#ifdef CONFIG_PM_DEVICE
	enum pm_device_state state;

	(void)pm_device_state_get(dev, &state);
	if (state != PM_DEVICE_STATE_ACTIVE) {
		return -EBUSY;
	}
#endif

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		return bmi08x_acc_config(dev, chan, attr, val);
	default:
		LOG_ERR("attr_set() not supported on this channel.");
		return -ENOTSUP;
	}

	return 0;
}

static int bmi08x_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct bmi08x_accel_data *data = dev->data;
	size_t i;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

#ifdef CONFIG_PM_DEVICE
	enum pm_device_state state;

	(void)pm_device_state_get(dev, &state);
	if (state != PM_DEVICE_STATE_ACTIVE) {
		return -EBUSY;
	}
#endif

	pm_device_busy_set(dev);

	if (bmi08x_accel_transceive(dev, BMI08X_REG_ACCEL_X_LSB | (1 << 7), false, data->acc_sample,
				    6) < 0) {
		pm_device_busy_clear(dev);
		return -EIO;
	}

	/* convert samples to cpu endianness */
	for (i = 0; i < ARRAY_SIZE(data->acc_sample); i++) {
		data->acc_sample[i] = sys_le16_to_cpu(data->acc_sample[i]);
	}

	pm_device_busy_clear(dev);
	return 0;
}

static void bmi08x_to_fixed_point(int16_t raw_val, uint16_t scale, struct sensor_value *val)
{
	int32_t converted_val;

	/*
	 * maximum converted value we can get is: max(raw_val) * max(scale)
	 *	max(raw_val) = +/- 2^15
	 *	max(scale) = 4785
	 *	max(converted_val) = 156794880 which is less than 2^31
	 */
	converted_val = raw_val * scale;
	val->val1 = converted_val / 1000000;
	val->val2 = converted_val % 1000000;
}

static void bmi08x_channel_convert(enum sensor_channel chan, uint16_t scale, uint16_t *raw_xyz,
				   struct sensor_value *val)
{
	int i;
	uint8_t ofs_start, ofs_stop;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		ofs_start = ofs_stop = 0U;
		break;
	case SENSOR_CHAN_ACCEL_Y:
		ofs_start = ofs_stop = 1U;
		break;
	case SENSOR_CHAN_ACCEL_Z:
		ofs_start = ofs_stop = 2U;
		break;
	default:
		ofs_start = 0U;
		ofs_stop = 2U;
		break;
	}

	for (i = ofs_start; i <= ofs_stop; i++, val++) {
		bmi08x_to_fixed_point(raw_xyz[i], scale, val);
	}
}

static inline void bmi08x_acc_channel_get(const struct device *dev, enum sensor_channel chan,
					  struct sensor_value *val)
{
	struct bmi08x_accel_data *data = dev->data;

	bmi08x_channel_convert(chan, data->scale, data->acc_sample, val);
}

static int bmi08x_temp_channel_get(const struct device *dev, struct sensor_value *val)
{
	uint16_t temp_raw = 0U;
	int32_t temp_micro = 0;

	if (bmi08x_accel_word_read(dev, BMI08X_REG_TEMP_MSB, &temp_raw) < 0) {
		return -EIO;
	}

	/* the scale is 1/2^5/LSB = 31250 micro degrees */
	temp_micro = BMI08X_TEMP_OFFSET * 1000000ULL + temp_raw * 31250ULL;

	val->val1 = temp_micro / 1000000ULL;
	val->val2 = temp_micro % 1000000ULL;

	return 0;
}

static int bmi08x_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bmi08x_accel_data *data = dev->data;
	uint8_t temp_raw[2] = {0};

#ifdef CONFIG_PM_DEVICE
	enum pm_device_state state;

	(void)pm_device_state_get(dev, &state);
	if (state != PM_DEVICE_STATE_ACTIVE) {
		return -EBUSY;
	}
#endif

	switch ((int16_t)chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		bmi08x_acc_channel_get(dev, chan, val);
		return 0;
	case SENSOR_CHAN_DIE_TEMP:
		return bmi08x_temp_channel_get(dev, val);
	default:
		LOG_ERR("Channel not supported.");
		return -ENOTSUP;
	}

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int bmi08x_accel_pm_action(const struct device *dev, enum pm_device_action action)
{
	uint8_t conf_reg_val;
	uint8_t ctrl_reg_val;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		conf_reg_val = BMI08X_ACCEL_PM_ACTIVE;
		ctrl_reg_val = BMI08X_ACCEL_POWER_ENABLE;
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		conf_reg_val = BMI08X_ACCEL_PM_SUSPEND;
		ctrl_reg_val = BMI08X_ACCEL_POWER_DISABLE;
		break;
	default:
		return -ENOTSUP;
	}

	if (bmi08x_accel_byte_write(dev, BMI08X_REG_ACCEL_PWR_CONF, conf_reg_val) < 0) {
		LOG_ERR("Failed to set conf power mode");
		return -EIO;
	}
	k_msleep(BMI08X_POWER_CONFIG_DELAY);
	if (bmi08x_accel_byte_write(dev, BMI08X_REG_ACCEL_PWR_CTRL, ctrl_reg_val) < 0) {
		LOG_ERR("Failed to set ctrl power mode");
		return -EIO;
	}
	k_msleep(BMI08X_POWER_CONFIG_DELAY);

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct sensor_driver_api bmi08x_api = {
	.attr_set = bmi08x_attr_set,
#ifdef CONFIG_BMI08X_ACCEL_TRIGGER
	.trigger_set = bmi08x_trigger_set_acc,
#endif
	.sample_fetch = bmi08x_sample_fetch,
	.channel_get = bmi08x_channel_get,
};

#ifdef CONFIG_BMI08X_DATA_SYNC
static int bmi08x_apply_sync_binary_config(const struct device *dev)
{
	const struct bmi08x_accel_config *config = dev->config;

	if (bmi08x_accel_byte_write(dev, BMI08X_REG_ACCEL_PWR_CONF, BMI08X_ACCEL_PM_ACTIVE) < 0) {
		LOG_ERR("Cannot deactivate advanced power save mode.");
		return -EIO;
	}
	/* required when switching power modes */
	k_msleep(BMI08X_POWER_CONFIG_DELAY);

	/* deactivate accel, otherwise post processing can not be enabled safely */
	if (bmi08x_accel_byte_write(dev, BMI08X_REG_ACCEL_PWR_CTRL, BMI08X_ACCEL_POWER_DISABLE) <
	    0) {
		LOG_ERR("Cannot deactivate accel.");
		return -EIO;
	}
	/* required when switching power modes */
	k_msleep(BMI08X_POWER_CONFIG_DELAY);
	/* disable config loading */
	if (bmi08x_accel_byte_write(dev, BMI08X_REG_ACCEL_INIT_CTRL,
				    BMI08X_ACCEL_INIT_CTRL_DISABLE) < 0) {
		LOG_ERR("Cannot disable config loading.");
		return -EIO;
	}

	if (config->api->write_config_file(dev) != 0) {
		LOG_ERR("Cannot write configuration for accelerometer.");
		return -EIO;
	}
	k_msleep(5U);

	if (bmi08x_accel_byte_write(dev, BMI08X_REG_ACCEL_INIT_CTRL,
				    BMI08X_ACCEL_INIT_CTRL_ENABLE)) {
		LOG_ERR("Cannot write configuration for accelerometer.");
		return -EIO;
	}
	k_msleep(BMI08X_ASIC_INIT_TIME_MS);

	/* check config initialization status */
	uint8_t val;

	if (bmi08x_accel_byte_read(dev, BMI08X_REG_ACCEL_INTERNAL_STAT, &val)) {
		LOG_ERR("Cannot write configuration for accelerometer.");
		return -EIO;
	}
	if (val != 1) {
		LOG_ERR("Configuration stream error.");
		return -EIO;
	}

	/* write feature configuration */
	uint8_t fdata[8];

	if (bmi08x_accel_read(dev, BMI08X_ACCEL_FEATURE_CFG_REG, fdata, 6)) {
		LOG_ERR("Cannot read configuration for accelerometer.");
		return -EIO;
	}
	fdata[4] = config->sync_hz;
	fdata[5] = 0x00;
	if (bmi08x_accel_write(dev, BMI08X_ACCEL_FEATURE_CFG_REG, fdata, 6)) {
		LOG_ERR("Cannot write configuration for accelerometer.");
		return -EIO;
	}
	k_msleep(100U);

	if (bmi08x_accel_byte_write(dev, BMI08X_REG_ACCEL_PWR_CTRL, BMI08X_ACCEL_POWER_ENABLE) <
	    0) {
		LOG_ERR("Cannot activate accel.");
		return -EIO;
	}
	/* required when switching power modes */
	k_msleep(BMI08X_POWER_CONFIG_DELAY);

	return 0;
}
#endif

int bmi08x_accel_init(const struct device *dev)
{
	const struct bmi08x_accel_config *config = dev->config;
	struct bmi08x_accel_data *data = dev->data;
	uint8_t val = 0U;

	int status = bmi08x_bus_check(dev);

	if (status < 0) {
		LOG_ERR("Bus not ready for '%s'", dev->name);
		return status;
	}

	/* reboot the chip */
	if (bmi08x_accel_byte_write(dev, BMI08X_REG_ACCEL_SOFTRESET, BMI08X_SOFT_RESET_CMD) < 0) {
		LOG_ERR("Cannot reboot chip.");
		return -EIO;
	}

	k_msleep(BMI08X_ACCEL_SOFTRESET_DELAY_MS);

	status = bmi08x_bus_init(dev);
	if (status < 0) {
		LOG_ERR("Can't initialize bus for %s", dev->name);
		return status;
	}

	if (bmi08x_accel_byte_read(dev, BMI08X_REG_ACCEL_CHIP_ID, &val) < 0) {
		LOG_ERR("Failed to read chip id.");
		return -EIO;
	}

	if ((val != BMI085_ACCEL_CHIP_ID) && (val != BMI088_ACCEL_CHIP_ID)) {
		LOG_ERR("Unsupported chip detected (0x%02x)!", val);
		return -ENODEV;
	}
	data->accel_chip_id = val;

	/* enable power */
	if (bmi08x_accel_byte_write(dev, BMI08X_REG_ACCEL_PWR_CONF, BMI08X_ACCEL_PM_ACTIVE) < 0) {
		LOG_ERR("Failed to set conf power mode");
		return -EIO;
	}
	k_msleep(BMI08X_POWER_CONFIG_DELAY);
	if (bmi08x_accel_byte_write(dev, BMI08X_REG_ACCEL_PWR_CTRL, BMI08X_ACCEL_POWER_ENABLE) <
	    0) {
		LOG_ERR("Failed to set ctrl power mode");
		return -EIO;
	}
	k_msleep(BMI08X_POWER_CONFIG_DELAY);

#ifdef CONFIG_BMI08X_DATA_SYNC
	status = bmi08x_apply_sync_binary_config(dev);
	if (status < 0) {
		return status;
	}
#endif

	/* set accelerometer default range */
	status = bmi08x_acc_range_set(dev, config->accel_fs);
	if (status < 0) {
		LOG_ERR("Cannot set default range for accelerometer.");
		return status;
	}

	/* set accelerometer default odr */
	status = bmi08x_accel_reg_field_update(dev, BMI08X_REG_ACCEL_CONF, 0, BMI08X_ACCEL_ODR_MASK,
					       config->accel_hz);
	if (status < 0) {
		LOG_ERR("Failed to set accel's default ODR.");
		return -EIO;
	}

#ifdef CONFIG_BMI08X_ACCEL_TRIGGER
	if (bmi08x_acc_trigger_mode_init(dev) < 0) {
		LOG_ERR("Cannot set up trigger mode.");
		return -EINVAL;
	}
#endif

	return 0;
}

#define BMI08X_CONFIG_SPI(inst)                                                                    \
	.bus.spi = SPI_DT_SPEC_INST_GET(                                                           \
		inst, SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8), 2),

#define BMI08X_CONFIG_I2C(inst) .bus.i2c = I2C_DT_SPEC_INST_GET(inst),

#define BMI08X_CREATE_INST(inst)                                                                   \
                                                                                                   \
	static struct bmi08x_accel_data bmi08x_drv_##inst;                                         \
                                                                                                   \
	static const struct bmi08x_accel_config bmi08x_config_##inst = {                           \
		COND_CODE_1(DT_INST_ON_BUS(inst, spi), (BMI08X_CONFIG_SPI(inst)),                  \
			    (BMI08X_CONFIG_I2C(inst)))                                             \
			.api = COND_CODE_1(DT_INST_ON_BUS(inst, spi), (&bmi08x_spi_api),           \
					   (&bmi08x_i2c_api)),                                     \
		.int_gpio = GPIO_DT_SPEC_INST_GET(inst, int_gpios),                                \
		.int1_map = DT_INST_PROP(inst, int1_map_io),                                       \
		.int2_map = DT_INST_PROP(inst, int2_map_io),                                       \
		.int1_io_conf = DT_INST_PROP(inst, int1_io_conf),                                  \
		.int2_io_conf = DT_INST_PROP(inst, int2_io_conf),                                  \
		.accel_hz = DT_INST_ENUM_IDX(inst, accel_hz) + 5,                                  \
		.accel_fs = DT_INST_ENUM_IDX(inst, accel_fs) / 2,                                  \
		IF_ENABLED(CONFIG_BMI08X_DATA_SYNC,                                                \
			   (.sync_hz = DT_INST_ENUM_IDX(inst, sync_hz) + 1,))};                   \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, bmi08x_accel_pm_action);                                    \
	DEVICE_DT_INST_DEFINE(inst, bmi08x_accel_init, PM_DEVICE_DT_INST_GET(inst),                \
			      &bmi08x_drv_##inst, &bmi08x_config_##inst, POST_KERNEL,              \
			      CONFIG_SENSOR_INIT_PRIORITY, &bmi08x_api);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(BMI08X_CREATE_INST)
