/* Bosch BMI08X inertial measurement unit driver, trigger implementation
 *
 * Copyright (c) 2022 Meta Platforms, Inc. and its affiliates
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#define DT_DRV_COMPAT bosch_bmi08x_gyro
#include "bmi08x.h"

#include <logging/log.h>
LOG_MODULE_DECLARE(BMI08X_GYRO, CONFIG_SENSOR_LOG_LEVEL);

static void bmi08x_handle_drdy_gyr(const struct device *dev)
{
	struct bmi08x_gyro_data *bmi08x = dev->data;
	struct sensor_trigger drdy_trigger = {
		.type = SENSOR_TRIG_DATA_READY,
	};

#ifdef CONFIG_PM_DEVICE
	enum pm_device_state state;

	(void)pm_device_state_get(dev, &state);
	if (state != PM_DEVICE_STATE_ACTIVE) {
		return;
	}
#endif

	if (bmi08x->handler_drdy_gyr) {
		bmi08x->handler_drdy_gyr(dev, &drdy_trigger);
	}
}

static void bmi08x_handle_interrupts_gyr(void *arg)
{
	const struct device *dev = (device_ptr_t)arg;

	bmi08x_handle_drdy_gyr(dev);
}

#ifdef CONFIG_BMI08X_GYRO_TRIGGER_OWN_THREAD
static void bmi08x_gyr_thread_main(void *arg1, void *unused1, void *unused2)
{
	k_thread_name_set(NULL, "bmi08x_gyr_trig");

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	const struct device *dev = (device_ptr_t)arg1;
	struct bmi08x_gyro_data *bmi08x = dev->data;

	while (1) {
		k_sem_take(&bmi08x->sem, K_FOREVER);
		bmi08x_handle_interrupts_gyr((void *)dev);
	}
}
#endif

#ifdef CONFIG_BMI08X_GYRO_TRIGGER_GLOBAL_THREAD
static void bmi08x_gyr_work_handler(struct k_work *work)
{
	struct bmi08x_gyro_data *bmi08x = CONTAINER_OF(work, struct bmi08x_gyro_data, work);

	bmi08x_handle_interrupts_gyr((void *)bmi08x->dev);
}
#endif

static void bmi08x_gyr_gpio_callback(const struct device *port, struct gpio_callback *cb,
				     uint32_t pin)
{
	struct bmi08x_gyro_data *bmi08x = CONTAINER_OF(cb, struct bmi08x_gyro_data, gpio_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pin);

#if defined(CONFIG_BMI08X_GYRO_TRIGGER_OWN_THREAD)
	k_sem_give(&bmi08x->sem);
#elif defined(CONFIG_BMI08X_GYRO_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&bmi08x->work);
#endif
}

int bmi08x_trigger_set_gyr(const struct device *dev, const struct sensor_trigger *trig,
			   sensor_trigger_handler_t handler)
{
	struct bmi08x_gyro_data *bmi08x = dev->data;

	if ((trig->chan == SENSOR_CHAN_GYRO_XYZ) && (trig->type == SENSOR_TRIG_DATA_READY)) {
		bmi08x->handler_drdy_gyr = handler;
		return 0;
	}

	return -ENOTSUP;
}

int bmi08x_gyr_trigger_mode_init(const struct device *dev)
{
	struct bmi08x_gyro_data *bmi08x = dev->data;
	const struct bmi08x_gyro_config *cfg = dev->config;

	if (!device_is_ready(cfg->int_gpio.port)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}

#if defined(CONFIG_BMI08X_GYRO_TRIGGER_OWN_THREAD)
	k_sem_init(&bmi08x->sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&bmi08x->thread, bmi08x->thread_stack,
			CONFIG_BMI08X_GYRO_THREAD_STACK_SIZE, bmi08x_gyr_thread_main, (void *)dev,
			NULL, NULL, K_PRIO_COOP(CONFIG_BMI08X_GYRO_THREAD_PRIORITY), 0, K_NO_WAIT);
#elif defined(CONFIG_BMI08X_GYRO_TRIGGER_GLOBAL_THREAD)
	bmi08x->work.handler = bmi08x_gyr_work_handler;
	bmi08x->dev = dev;
#endif

	gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);

	gpio_init_callback(&bmi08x->gpio_cb, bmi08x_gyr_gpio_callback, BIT(cfg->int_gpio.pin));

	int ret = gpio_add_callback(bmi08x->gpio, &bmi08x->gpio_cb);

	if (ret < 0) {
		return ret;
	}
	gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);

	return 0;
}
