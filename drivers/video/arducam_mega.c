/**
 * Copyright (c) 2023 Arducam Technology Co., Ltd. <www.arducam.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT arducam_mega
#include <zephyr/drivers/video-controls.h>
//#include <zephyr/drivers/video/arducam_mega.h>
#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include "video_ctrls.h"
#include "video_device.h"

LOG_MODULE_REGISTER(mega_camera,/*CONFIG_VIDEO_LOG_LEVEL*/LOG_LEVEL_DBG);


#define ARDUCHIP_FIFO   0x04 /* FIFO and I2C control */
#define ARDUCHIP_FIFO_2 0x07 /* FIFO and I2C control */

#define FIFO_CLEAR_ID_MASK 0x01
#define FIFO_START_MASK    0x02

#define ARDUCHIP_TRIG 0x44 /* Trigger source */
#define VSYNC_MASK    0x01
#define SHUTTER_MASK  0x02
#define CAP_DONE_MASK 0x04

#define FIFO_SIZE1 0x45 /* Camera write FIFO size[7:0] for burst to read */
#define FIFO_SIZE2 0x46 /* Camera write FIFO size[15:8] */
#define FIFO_SIZE3 0x47 /* Camera write FIFO size[18:16] */

#define BURST_FIFO_READ  0x3C /* Burst FIFO read operation */
#define SINGLE_FIFO_READ 0x3D /* Single FIFO read operation */

/* DSP register bank FF=0x00*/
#define CAM_REG_POWER_CONTROL                 0X02
#define CAM_REG_SENSOR_RESET                  0X07
#define CAM_REG_FORMAT                        0X20
#define CAM_REG_CAPTURE_RESOLUTION            0X21
#define CAM_REG_BRIGHTNESS_CONTROL            0X22
#define CAM_REG_CONTRAST_CONTROL              0X23
#define CAM_REG_SATURATION_CONTROL            0X24
#define CAM_REG_EV_CONTROL                    0X25
#define CAM_REG_WHITEBALANCE_CONTROL          0X26
#define CAM_REG_COLOR_EFFECT_CONTROL          0X27
#define CAM_REG_SHARPNESS_CONTROL             0X28
#define CAM_REG_AUTO_FOCUS_CONTROL            0X29
#define CAM_REG_IMAGE_QUALITY                 0x2A
#define CAM_REG_EXPOSURE_GAIN_WHITEBAL_ENABLE 0X30
#define CAM_REG_MANUAL_GAIN_BIT_9_8           0X31
#define CAM_REG_MANUAL_GAIN_BIT_7_0           0X32
#define CAM_REG_MANUAL_EXPOSURE_BIT_19_16     0X33
#define CAM_REG_MANUAL_EXPOSURE_BIT_15_8      0X34
#define CAM_REG_MANUAL_EXPOSURE_BIT_7_0       0X35
#define CAM_REG_BURST_FIFO_READ_OPERATION     0X3C
#define CAM_REG_SINGLE_FIFO_READ_OPERATION    0X3D
#define CAM_REG_SENSOR_ID                     0x40
#define CAM_REG_YEAR_SDK                      0x41
#define CAM_REG_MONTH_SDK                     0x42
#define CAM_REG_DAY_SDK                       0x43
#define CAM_REG_SENSOR_STATE                  0x44
#define CAM_REG_FPGA_VERSION_NUMBER           0x49
#define CAM_REG_DEBUG_DEVICE_ADDRESS          0X0A
#define CAM_REG_DEBUG_REGISTER_HIGH           0X0B
#define CAM_REG_DEBUG_REGISTER_LOW            0X0C
#define CAM_REG_DEBUG_REGISTER_VALUE          0X0D

#define SENSOR_STATE_IDLE   (1 << 1)
#define SENSOR_RESET_ENABLE (1 << 6)

#define CTR_WHITEBALANCE 0X02
#define CTR_EXPOSURE     0X01
#define CTR_GAIN         0X00

#define AC_STACK_SIZE 4096
#define AC_PRIORITY 5



K_THREAD_STACK_DEFINE(ac_stack_area, AC_STACK_SIZE);

#define VIDEO_CID_ARDUCAM_EV        (VIDEO_CID_PRIVATE_BASE + 1)
#define VIDEO_CID_ARDUCAM_INFO      (VIDEO_CID_PRIVATE_BASE + 2)
#define VIDEO_CID_ARDUCAM_RESET     (VIDEO_CID_PRIVATE_BASE + 3)
#define VIDEO_CID_ARDUCAM_LOWPOWER  (VIDEO_CID_PRIVATE_BASE + 4)

/**
 * @enum MEGA_CONTRAST_LEVEL
 * @brief Configure camera contrast level
 */
enum MEGA_CONTRAST_LEVEL {
	MEGA_CONTRAST_LEVEL_NEGATIVE_3 = -3, /**<Level -3 */
	MEGA_CONTRAST_LEVEL_NEGATIVE_2 = -2, /**<Level -2 */
	MEGA_CONTRAST_LEVEL_NEGATIVE_1 = -1, /**<Level -1 */
	MEGA_CONTRAST_LEVEL_DEFAULT = 0,    /**<Level Default*/
	MEGA_CONTRAST_LEVEL_1 = 1,          /**<Level +1 */
	MEGA_CONTRAST_LEVEL_2 = 2,          /**<Level +2 */
	MEGA_CONTRAST_LEVEL_3 = 3           /**<Level +3 */
};

/**
 * @enum MEGA_EV_LEVEL
 * @brief Configure camera EV level
 */
enum MEGA_EV_LEVEL {
	MEGA_EV_LEVEL_NEGATIVE_3 = -3, /**<Level -3 */
	MEGA_EV_LEVEL_NEGATIVE_2 = -2, /**<Level -2 */
	MEGA_EV_LEVEL_NEGATIVE_1 = -1, /**<Level -1 */
	MEGA_EV_LEVEL_DEFAULT = 0,    /**<Level Default*/
	MEGA_EV_LEVEL_1 = 1,          /**<Level +1 */
	MEGA_EV_LEVEL_2 = 2,          /**<Level +2 */
	MEGA_EV_LEVEL_3 = 3,          /**<Level +3 */
};

/**
 * @enum MEGA_SATURATION_LEVEL
 * @brief Configure camera saturation  level
 */
enum MEGA_SATURATION_LEVEL {
	MEGA_SATURATION_LEVEL_NEGATIVE_3 = -3, /**<Level -3 */
	MEGA_SATURATION_LEVEL_NEGATIVE_2 = -2, /**<Level -2 */
	MEGA_SATURATION_LEVEL_NEGATIVE_1 = -1, /**<Level -1 */
	MEGA_SATURATION_LEVEL_DEFAULT = 0,    /**<Level Default*/
	MEGA_SATURATION_LEVEL_1 = 1,          /**<Level +1 */
	MEGA_SATURATION_LEVEL_2 = 2,          /**<Level +2 */
	MEGA_SATURATION_LEVEL_3 = 3,          /**<Level +3 */
};

/**
 * @enum MEGA_BRIGHTNESS_LEVEL
 * @brief Configure camera brightness level
 */
enum MEGA_BRIGHTNESS_LEVEL {
	MEGA_BRIGHTNESS_LEVEL_NEGATIVE_4 = -4, /**<Level -4 */
	MEGA_BRIGHTNESS_LEVEL_NEGATIVE_3 = -3, /**<Level -3 */
	MEGA_BRIGHTNESS_LEVEL_NEGATIVE_2 = -2, /**<Level -2 */
	MEGA_BRIGHTNESS_LEVEL_NEGATIVE_1 = -1, /**<Level -1 */
	MEGA_BRIGHTNESS_LEVEL_DEFAULT = 0,    /**<Level Default*/
	MEGA_BRIGHTNESS_LEVEL_1 = 1,          /**<Level +1 */
	MEGA_BRIGHTNESS_LEVEL_2 = 2,          /**<Level +2 */
	MEGA_BRIGHTNESS_LEVEL_3 = 3,          /**<Level +3 */
	MEGA_BRIGHTNESS_LEVEL_4 = 4,          /**<Level +4 */
};

/**
 * @enum MEGA_SHARPNESS_LEVEL
 * @brief Configure camera Sharpness level
 */
enum MEGA_SHARPNESS_LEVEL {
	MEGA_SHARPNESS_LEVEL_AUTO = 0, /**<Sharpness Auto */
	MEGA_SHARPNESS_LEVEL_1,        /**<Sharpness Level 1 */
	MEGA_SHARPNESS_LEVEL_2,        /**<Sharpness Level 2 */
	MEGA_SHARPNESS_LEVEL_3,        /**<Sharpness Level 3 */
	MEGA_SHARPNESS_LEVEL_4,        /**<Sharpness Level 4 */
	MEGA_SHARPNESS_LEVEL_5,        /**<Sharpness Level 5 */
	MEGA_SHARPNESS_LEVEL_6,        /**<Sharpness Level 6 */
	MEGA_SHARPNESS_LEVEL_7,        /**<Sharpness Level 7 */
	MEGA_SHARPNESS_LEVEL_8,        /**<Sharpness Level 8 */
};

/**
 * @enum MEGA_COLOR_FX
 * @brief Configure special effects
 */
enum MEGA_COLOR_FX {
	MEGA_COLOR_FX_NONE = 0,      /**< no effect   */
	MEGA_COLOR_FX_BLUEISH,       /**< cool light   */
	MEGA_COLOR_FX_REDISH,        /**< warm   */
	MEGA_COLOR_FX_BW,            /**< Black/white   */
	MEGA_COLOR_FX_SEPIA,         /**<Sepia   */
	MEGA_COLOR_FX_NEGATIVE,      /**<positive/negative inversion  */
	MEGA_COLOR_FX_GRASS_GREEN,   /**<Grass green */
	MEGA_COLOR_FX_OVER_EXPOSURE, /**<Over exposure*/
	MEGA_COLOR_FX_SOLARIZE,      /**< Solarize   */
};

/**
 * @enum MEGA_WHITE_BALANCE
 * @brief Configure white balance mode
 */
enum MEGA_WHITE_BALANCE {
	MEGA_WHITE_BALANCE_MODE_DEFAULT = 0, /**< Auto */
	MEGA_WHITE_BALANCE_MODE_SUNNY,       /**< Sunny */
	MEGA_WHITE_BALANCE_MODE_OFFICE,      /**< Office */
	MEGA_WHITE_BALANCE_MODE_CLOUDY,      /**< Cloudy*/
	MEGA_WHITE_BALANCE_MODE_HOME,        /**< Home */
};

/**
 * @enum MEGA_IMAGE_QUALITY
 * @brief Configure JPEG image quality
 */
enum MEGA_IMAGE_QUALITY {
	HIGH_QUALITY = 0,
	DEFAULT_QUALITY = 1,
	LOW_QUALITY = 2,
};

enum {
	ARDUCAM_SENSOR_5MP_1 = 0x81,
	ARDUCAM_SENSOR_3MP_1 = 0x82,
	ARDUCAM_SENSOR_5MP_2 = 0x83, /* 2592x1936 */
	ARDUCAM_SENSOR_3MP_2 = 0x84,
};

/**
 * @enum MEGA_PIXELFORMAT
 * @brief Configure camera pixel format
 */
enum MEGA_PIXELFORMAT {
	MEGA_PIXELFORMAT_JPG = 0X01,
	MEGA_PIXELFORMAT_RGB565 = 0X02,
	MEGA_PIXELFORMAT_YUV = 0X03,
};

/**
 * @enum MEGA_RESOLUTION
 * @brief Configure camera resolution
 */
enum MEGA_RESOLUTION {
	MEGA_RESOLUTION_QQVGA = 0x00,   /**<160x120 */
	MEGA_RESOLUTION_QVGA = 0x01,    /**<320x240*/
	MEGA_RESOLUTION_VGA = 0x02,     /**<640x480*/
	MEGA_RESOLUTION_SVGA = 0x03,    /**<800x600*/
	MEGA_RESOLUTION_HD = 0x04,      /**<1280x720*/
	MEGA_RESOLUTION_SXGAM = 0x05,   /**<1280x960*/
	MEGA_RESOLUTION_UXGA = 0x06,    /**<1600x1200*/
	MEGA_RESOLUTION_FHD = 0x07,     /**<1920x1080*/
	MEGA_RESOLUTION_QXGA = 0x08,    /**<2048x1536*/
	MEGA_RESOLUTION_WQXGA2 = 0x09,  /**<2592x1944*/
	MEGA_RESOLUTION_96X96 = 0x0a,   /**<96x96*/
	MEGA_RESOLUTION_128X128 = 0x0b, /**<128x128*/
	MEGA_RESOLUTION_320X320 = 0x0c, /**<320x320*/
	MEGA_RESOLUTION_12 = 0x0d,      /**<Reserve*/
	MEGA_RESOLUTION_13 = 0x0e,      /**<Reserve*/
	MEGA_RESOLUTION_14 = 0x0f,      /**<Reserve*/
	MEGA_RESOLUTION_15 = 0x10,      /**<Reserve*/
	MEGA_RESOLUTION_NONE,
};

/**
 * @enum MEGA_RESOLUTION
 * @brief MEGA camera state
 */
enum MEGA_STATE {
	MEGA_STATE_INIT,
	MEGA_STATE_CAPTURE
};

/**
 * @struct arducam_mega_info
 * @brief Some information about mega camera
 */
struct arducam_mega_info {
	int support_resolution;
	int support_special_effects;
	unsigned long exposure_value_max;
	unsigned int exposure_value_min;
	unsigned int gain_value_max;
	unsigned int gain_value_min;
	unsigned char enable_focus;
	unsigned char enable_sharpness;
	unsigned char device_address;
	unsigned char camera_id;
};

/**
 * @struct mega_sdk_data
 * @brief Basic information of the camera firmware
 */
struct mega_sdk_data {
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t version;
};

struct arducam_mega_config {
	struct spi_dt_spec bus;
};

struct arducam_mega_ctrls {

	/* exposure & auto-exposure */
	struct {
		struct video_ctrl auto_exposure;
		struct video_ctrl exposure;
	};
	/* auto-white balance & balance */
	struct {
		struct video_ctrl auto_white_balance;
		struct video_ctrl white_balance;
	};

	/* auto-gain & gain */
	struct {
		struct video_ctrl auto_gain;
		struct video_ctrl gain;
	};

	struct video_ctrl jpeg;
	struct video_ctrl brightness;
	struct video_ctrl contrast;
	struct video_ctrl saturation;
	struct video_ctrl sharpness;
	struct video_ctrl low_power;
	struct video_ctrl color_fx;
	struct video_ctrl ev;
};


struct arducam_mega_data {
	const struct device *dev;
	struct arducam_mega_ctrls ctrls;
	struct video_format fmt;
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	struct k_work_delayable work;
	struct k_work_q work_q;
	struct k_poll_signal *signal;
	struct arducam_mega_info *info;
	struct mega_sdk_data ver;
	uint8_t fifo_first_read;
	uint32_t fifo_length;
	uint32_t capture_retry;
	enum MEGA_STATE state;
	uint8_t stream_on;
};

static struct arducam_mega_info mega_infos[] = {{
							.support_resolution = 7894,
							.support_special_effects = 63,
							.exposure_value_max = 30000,
							.exposure_value_min = 1,
							.gain_value_max = 1023,
							.gain_value_min = 1,
							.enable_focus = 1,
							.enable_sharpness = 0,
							.device_address = 0x78,
						},
						{
							.support_resolution = 7638,
							.support_special_effects = 319,
							.exposure_value_max = 30000,
							.exposure_value_min = 1,
							.gain_value_max = 1023,
							.gain_value_min = 1,
							.enable_focus = 0,
							.enable_sharpness = 1,
							.device_address = 0x78,
						}};

#define ARDUCAM_MEGA_VIDEO_FORMAT_CAP(width, height, format)                                       \
	{                                                                                          \
		.pixelformat = (format), .width_min = (width), .width_max = (width),               \
		.height_min = (height), .height_max = (height), .width_step = 0, .height_step = 0  \
	}

static struct video_format_cap fmts[] = {
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(96, 96, VIDEO_PIX_FMT_RGB565),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(128, 128, VIDEO_PIX_FMT_RGB565),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(320, 240, VIDEO_PIX_FMT_RGB565),
	/*ARDUCAM_MEGA_VIDEO_FORMAT_CAP(320, 320, VIDEO_PIX_FMT_RGB565),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(640, 480, VIDEO_PIX_FMT_RGB565),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1280, 720, VIDEO_PIX_FMT_RGB565),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1600, 1200, VIDEO_PIX_FMT_RGB565),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1920, 1080, VIDEO_PIX_FMT_RGB565),*/
	{0},
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(96, 96, VIDEO_PIX_FMT_JPEG),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(128, 128, VIDEO_PIX_FMT_JPEG),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(320, 240, VIDEO_PIX_FMT_JPEG),
	/*ARDUCAM_MEGA_VIDEO_FORMAT_CAP(320, 320, VIDEO_PIX_FMT_JPEG),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(640, 480, VIDEO_PIX_FMT_JPEG),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1280, 720, VIDEO_PIX_FMT_JPEG),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1600, 1200, VIDEO_PIX_FMT_JPEG),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1920, 1080, VIDEO_PIX_FMT_JPEG),*/
	{0},
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(96, 96, VIDEO_PIX_FMT_YUYV),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(128, 128, VIDEO_PIX_FMT_YUYV),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(320, 240, VIDEO_PIX_FMT_YUYV),
	/*ARDUCAM_MEGA_VIDEO_FORMAT_CAP(320, 320, VIDEO_PIX_FMT_YUYV),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(640, 480, VIDEO_PIX_FMT_YUYV),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1280, 720, VIDEO_PIX_FMT_YUYV),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1600, 1200, VIDEO_PIX_FMT_YUYV),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1920, 1080, VIDEO_PIX_FMT_YUYV),*/
	{0},
	{0},
};

#define SUPPORT_RESOLUTION_NUM 3

static uint8_t support_resolution[SUPPORT_RESOLUTION_NUM] = {
	MEGA_RESOLUTION_96X96,   MEGA_RESOLUTION_128X128, MEGA_RESOLUTION_QVGA,
	/*MEGA_RESOLUTION_320X320, MEGA_RESOLUTION_VGA,     MEGA_RESOLUTION_HD,
	MEGA_RESOLUTION_UXGA,    MEGA_RESOLUTION_FHD,     MEGA_RESOLUTION_NONE,*/
};

static int arducam_mega_write_reg(const struct spi_dt_spec *spec, uint8_t reg_addr, uint8_t value)
{
	uint8_t tries = 3;

	reg_addr |= 0x80;

	struct spi_buf tx_buf[2] = {
		{.buf = &reg_addr, .len = 1},
		{.buf = &value, .len = 1},
	};

	struct spi_buf_set tx_bufs = {.buffers = tx_buf, .count = 2};

	while (tries--) {
		if (!spi_write_dt(spec, &tx_bufs)) {
			return 0;
		}
		/* If writing failed wait 5ms before next attempt */
		k_msleep(5);
	}
	LOG_ERR("failed to write 0x%x to 0x%x", value, reg_addr);

	return -1;
}

static int arducam_mega_read_reg(const struct spi_dt_spec *spec, uint8_t reg_addr)
{
	uint8_t tries = 3;
	uint8_t value;
	uint8_t ret;

	reg_addr &= 0x7F;

	struct spi_buf tx_buf[] = {
		{.buf = &reg_addr, .len = 1},
	};

	struct spi_buf_set tx_bufs = {.buffers = tx_buf, .count = 1};

	struct spi_buf rx_buf[] = {
		{.buf = &value, .len = 1},
		{.buf = &value, .len = 1},
		{.buf = &value, .len = 1},
	};

	struct spi_buf_set rx_bufs = {.buffers = rx_buf, .count = 3};

	while (tries--) {
		ret = spi_transceive_dt(spec, &tx_bufs, &rx_bufs);
		if (!ret) {
			return value;
		}

		/* If reading failed wait 5ms before next attempt */
		k_msleep(5);
	}
	LOG_ERR("failed to read 0x%x register", reg_addr);

	return -1;
}

static int arducam_mega_read_block(const struct spi_dt_spec *spec, uint8_t *img_buff,
				   uint32_t img_len, uint8_t first)
{
	uint8_t cmd_fifo_read[] = {BURST_FIFO_READ, 0x00};
	uint8_t buf_len = first == 0 ? 1 : 2;

	struct spi_buf tx_buf[] = {
		{.buf = cmd_fifo_read, .len = buf_len},
	};

	struct spi_buf_set tx_bufs = {.buffers = tx_buf, .count = 1};

	struct spi_buf rx_buf[2] = {
		{.buf = cmd_fifo_read, .len = buf_len},
		{.buf = img_buff, .len = img_len},
	};
	struct spi_buf_set rx_bufs = {.buffers = rx_buf, .count = 2};

	return spi_transceive_dt(spec, &tx_bufs, &rx_bufs);
}

static int arducam_mega_await_bus_idle(const struct spi_dt_spec *spec, uint8_t tries)
{
	while ((arducam_mega_read_reg(spec, CAM_REG_SENSOR_STATE) & 0x03) != SENSOR_STATE_IDLE) {
		if (tries-- == 0) {
			return -1;
		}
		k_msleep(2);
	}

	return 0;
}

static int arducam_mega_set_brightness(const struct device *dev, enum MEGA_BRIGHTNESS_LEVEL level)
{
	int ret = 0;
	const struct arducam_mega_config *cfg = dev->config;
	uint8_t values[] = {0,1,-1,2,-2,3,-3,4,-4};
	
	uint8_t idx;
	for (idx = 0; idx<ARRAY_SIZE(values);++idx) {
		if (values[idx] == level)
		{
			break;
		}
	}

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 3);

	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_BRIGHTNESS_CONTROL, idx);

	if (ret == -1) {
		LOG_ERR("Failed to set brightness level %d", level);
	}

	return ret;
}

static int arducam_mega_set_saturation(const struct device *dev, enum MEGA_SATURATION_LEVEL level)
{
	int ret = 0;
	const struct arducam_mega_config *cfg = dev->config;
	uint8_t values[] = {0,1,-1,2,-2,3,-3};
	
	uint8_t idx;
	for (idx = 0; idx<ARRAY_SIZE(values);++idx) {
		if (values[idx] == level)
		{
			break;
		}
	}

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 3);

	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_SATURATION_CONTROL, idx);

	if (ret == -1) {
		LOG_ERR("Failed to set saturation level %d", level);
	}

	return ret;
}

static int arducam_mega_set_contrast(const struct device *dev, enum MEGA_CONTRAST_LEVEL level)
{
	int ret = 0;
	const struct arducam_mega_config *cfg = dev->config;
	uint8_t values[] = {0,1,-1,2,-2,3,-3};
	
	uint8_t idx;
	for (idx = 0; idx<ARRAY_SIZE(values);++idx) {
		if (values[idx] == level)
		{
			break;
		}
	}

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 3);

	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_CONTRAST_CONTROL, idx);

	if (ret == -1) {
		LOG_ERR("Failed to set contrast level %d", level);
	}

	return ret;
}

static int arducam_mega_set_EV(const struct device *dev, enum MEGA_EV_LEVEL level)
{
	int ret = 0;
	const struct arducam_mega_config *cfg = dev->config;
	uint8_t values[] = {0,1,-1,2,-2,3,-3};
	
	uint8_t idx;
	for (idx = 0; idx<ARRAY_SIZE(values);++idx) {
		if (values[idx] == level)
		{
			break;
		}
	}

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 3);

	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_EV_CONTROL, idx);

	if (ret == -1) {
		LOG_ERR("Failed to set contrast level %d", level);
	}

	return ret;
}

static int arducam_mega_set_sharpness(const struct device *dev, enum MEGA_SHARPNESS_LEVEL level)
{
	int ret = 0;
	struct arducam_mega_data *drv_data = dev->data;
	struct arducam_mega_info *drv_info = drv_data->info;
	const struct arducam_mega_config *cfg = dev->config;

	if (!drv_info->enable_sharpness) {
		LOG_ERR("This device does not support set sharpness.");
		return -1;
	}

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 3);

	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_SHARPNESS_CONTROL, level);

	if (ret == -1) {
		LOG_ERR("Failed to set sharpness level %d", level);
	}

	return ret;
}

static int arducam_mega_set_special_effects(const struct device *dev, enum MEGA_COLOR_FX effect)
{
	int ret = 0;
	const struct arducam_mega_config *cfg = dev->config;

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 3);

	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_COLOR_EFFECT_CONTROL, effect);

	if (ret == -1) {
		LOG_ERR("Failed to set special effects %d", effect);
	}

	return ret;
}

static int arducam_mega_set_output_format(const struct device *dev, int output_format)
{
	int ret = 0;
	const struct arducam_mega_config *cfg = dev->config;

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 3);

	if (output_format == VIDEO_PIX_FMT_JPEG) {
		/* Set output to JPEG compression */
		ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_FORMAT, MEGA_PIXELFORMAT_JPG);
	} else if (output_format == VIDEO_PIX_FMT_RGB565) {
		/* Set output to RGB565 */
		ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_FORMAT, MEGA_PIXELFORMAT_RGB565);
	} else if (output_format == VIDEO_PIX_FMT_YUYV) {
		/* Set output to YUV422 */
		ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_FORMAT, MEGA_PIXELFORMAT_YUV);
	} else {
		LOG_ERR("Image format not supported");
		return -ENOTSUP;
	}

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 30);

	return ret;
}

static int arducam_mega_set_JPEG_quality(const struct device *dev, enum MEGA_IMAGE_QUALITY qc)
{
	int ret = 0;
	const struct arducam_mega_config *cfg = dev->config;
	struct arducam_mega_data *drv_data = dev->data;

	LOG_DBG("%s: %d", __func__, qc);
	if (drv_data->fmt.pixelformat == VIDEO_PIX_FMT_JPEG) {
		ret |= arducam_mega_await_bus_idle(&cfg->bus, 3);
		/* Write QC register */
		ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_IMAGE_QUALITY, qc);
	} else {
		LOG_ERR("Image format not support setting the quality");
		return -ENOTSUP;
	}

	return ret;
}

static int arducam_mega_set_white_bal_enable(const struct device *dev, int enable)
{
	int ret = 0;
	uint8_t reg = 0;
	const struct arducam_mega_config *cfg = dev->config;

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 3);

	if (enable) {
		reg |= 0x80;
	}
	reg |= CTR_WHITEBALANCE;
	/* Update register to enable/disable automatic white balance*/
	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_EXPOSURE_GAIN_WHITEBAL_ENABLE, reg);

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 10);

	return ret;
}

static int arducam_mega_set_white_bal(const struct device *dev, enum MEGA_EV_LEVEL level)
{
	int ret = 0;
	const struct arducam_mega_config *cfg = dev->config;

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 3);

	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_WHITEBALANCE_CONTROL, level);

	if (ret == -1) {
		LOG_ERR("Failed to set contrast level %d", level);
	}

	return ret;
}

static int arducam_mega_set_gain_enable(const struct device *dev, int enable)
{
	int ret = 0;
	uint8_t reg = 0;
	const struct arducam_mega_config *cfg = dev->config;

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 3);

	if (enable) {
		reg |= 0x80;
	}
	reg |= CTR_GAIN;
	/* Update register to enable/disable automatic gain*/
	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_EXPOSURE_GAIN_WHITEBAL_ENABLE, reg);

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 10);

	return ret;
}

static int arducam_mega_set_lowpower_enable(const struct device *dev, int enable)
{
	int ret = 0;
	const struct arducam_mega_config *cfg = dev->config;
	const struct arducam_mega_data *drv_data = dev->data;
	const struct arducam_mega_info *drv_info = drv_data->info;

	if (drv_info->camera_id == ARDUCAM_SENSOR_5MP_2 ||
	    drv_info->camera_id == ARDUCAM_SENSOR_3MP_2) {
		enable = !enable;
	}

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 3);
	if (enable) {
		ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_POWER_CONTROL, 0x07);
	} else {
		ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_POWER_CONTROL, 0x05);
	}
	return ret;
}

static int arducam_mega_set_gain(const struct device *dev, uint16_t value)
{
	int ret = 0;
	const struct arducam_mega_config *cfg = dev->config;

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 3);
	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_MANUAL_GAIN_BIT_9_8, (value >> 8) & 0xff);
	ret |= arducam_mega_await_bus_idle(&cfg->bus, 10);
	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_MANUAL_GAIN_BIT_7_0, value & 0xff);
	ret |= arducam_mega_await_bus_idle(&cfg->bus, 10);

	return ret;
}

static int arducam_mega_set_exposure_enable(const struct device *dev, int enable)
{
	int ret = 0;
	const struct arducam_mega_config *cfg = dev->config;

	uint8_t reg = 0;

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 3);

	if (enable) {
		reg |= 0x80;
	}
	reg |= CTR_EXPOSURE;
	/* Enable/disable automatic exposure control */
	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_EXPOSURE_GAIN_WHITEBAL_ENABLE, reg);

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 10);
	return ret;
}

static int arducam_mega_set_exposure(const struct device *dev, uint32_t value)
{
	int ret = 0;
	const struct arducam_mega_config *cfg = dev->config;

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 3);
	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_MANUAL_EXPOSURE_BIT_19_16,
				      (value >> 16) & 0xff);
	ret |= arducam_mega_await_bus_idle(&cfg->bus, 10);
	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_MANUAL_EXPOSURE_BIT_15_8,
				      (value >> 8) & 0xff);
	ret |= arducam_mega_await_bus_idle(&cfg->bus, 10);
	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_MANUAL_EXPOSURE_BIT_7_0, value & 0xff);
	ret |= arducam_mega_await_bus_idle(&cfg->bus, 10);
	return ret;
}

static int arducam_mega_set_resolution(const struct device *dev, enum MEGA_RESOLUTION resolution)
{
	int ret = 0;
	const struct arducam_mega_config *cfg = dev->config;

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 3);

	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_CAPTURE_RESOLUTION, resolution);

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 10);

	return ret;
}

static int arducam_mega_check_connection(const struct device *dev)
{
	int ret = 0;
	uint8_t cam_id;
	const struct arducam_mega_config *cfg = dev->config;
	struct arducam_mega_data *drv_data = dev->data;

	ret |= arducam_mega_await_bus_idle(&cfg->bus, 255);
	cam_id = arducam_mega_read_reg(&cfg->bus, CAM_REG_SENSOR_ID);

	if (!(cam_id & 0x87)) {
		LOG_ERR("arducam mega not detected, 0x%x\n", cam_id);
		return -ENODEV;
	}

	LOG_INF("detect camera id 0x%x, ret = %d\n", cam_id, ret);

	switch (cam_id) {
	case ARDUCAM_SENSOR_5MP_1: /* 5MP-1 */
		/*
		fmts[8] = (struct video_format_cap)ARDUCAM_MEGA_VIDEO_FORMAT_CAP(
			2592, 1944, VIDEO_PIX_FMT_RGB565);
		fmts[17] = (struct video_format_cap)ARDUCAM_MEGA_VIDEO_FORMAT_CAP(
			2592, 1944, VIDEO_PIX_FMT_JPEG);
		fmts[26] = (struct video_format_cap)ARDUCAM_MEGA_VIDEO_FORMAT_CAP(
			2592, 1944, VIDEO_PIX_FMT_YUYV);
		support_resolution[8] = MEGA_RESOLUTION_WQXGA2;*/
		drv_data->info = &mega_infos[0];
		break;
	case ARDUCAM_SENSOR_3MP_1: /* 3MP-1 */
	/*
		fmts[8] = (struct video_format_cap)ARDUCAM_MEGA_VIDEO_FORMAT_CAP(
			2048, 1536, VIDEO_PIX_FMT_RGB565);
		fmts[17] = (struct video_format_cap)ARDUCAM_MEGA_VIDEO_FORMAT_CAP(
			2048, 1536, VIDEO_PIX_FMT_JPEG);
		fmts[26] = (struct video_format_cap)ARDUCAM_MEGA_VIDEO_FORMAT_CAP(
			2048, 1536, VIDEO_PIX_FMT_YUYV);
		support_resolution[8] = MEGA_RESOLUTION_QXGA;*/
		drv_data->info = &mega_infos[1];
		break;
	case ARDUCAM_SENSOR_5MP_2: /* 5MP-2 */
	/*
		fmts[8] = (struct video_format_cap)ARDUCAM_MEGA_VIDEO_FORMAT_CAP(
			2592, 1936, VIDEO_PIX_FMT_RGB565);
		fmts[17] = (struct video_format_cap)ARDUCAM_MEGA_VIDEO_FORMAT_CAP(
			2592, 1936, VIDEO_PIX_FMT_JPEG);
		fmts[26] = (struct video_format_cap)ARDUCAM_MEGA_VIDEO_FORMAT_CAP(
			2592, 1936, VIDEO_PIX_FMT_YUYV);
		support_resolution[8] = MEGA_RESOLUTION_WQXGA2;*/
		break;
		drv_data->info = &mega_infos[0];
		break;
	case ARDUCAM_SENSOR_3MP_2: /* 3MP-2 */
	/*
		fmts[8] = (struct video_format_cap)ARDUCAM_MEGA_VIDEO_FORMAT_CAP(
			2048, 1536, VIDEO_PIX_FMT_RGB565);
		fmts[17] = (struct video_format_cap)ARDUCAM_MEGA_VIDEO_FORMAT_CAP(
			2048, 1536, VIDEO_PIX_FMT_JPEG);
		fmts[26] = (struct video_format_cap)ARDUCAM_MEGA_VIDEO_FORMAT_CAP(
			2048, 1536, VIDEO_PIX_FMT_YUYV);
		support_resolution[8] = MEGA_RESOLUTION_QXGA;*/
		break;
		drv_data->info = &mega_infos[1];
		break;
	default:
		return -ENODEV;
	}
	drv_data->info->camera_id = cam_id;

	return ret;
}

static int arducam_mega_set_fmt(const struct device *dev,
				struct video_format *fmt)
{
	struct arducam_mega_data *drv_data = dev->data;
	uint16_t width, height;
	int ret = 0;
	int i = 0;

	/* We only support RGB565, JPEG, and YUYV pixel formats */
	if (fmt->pixelformat != VIDEO_PIX_FMT_RGB565 && fmt->pixelformat != VIDEO_PIX_FMT_JPEG &&
	    fmt->pixelformat != VIDEO_PIX_FMT_YUYV) {
		LOG_ERR("Arducam Mega camera only supports RGB565, JPG, and YUYV pixel formats!");
		return -ENOTSUP;
	}

	width = fmt->width;
	height = fmt->height;

	if (!memcmp(&drv_data->fmt, fmt, sizeof(drv_data->fmt))) {
		/* nothing to do */
		return 0;
	}

	/* Check if camera is capable of handling given format */
	while (fmts[i].pixelformat) {
		if (fmts[i].width_min == width && fmts[i].height_min == height &&
		    fmts[i].pixelformat == fmt->pixelformat) {
			/* Set output format */
			ret |= arducam_mega_set_output_format(dev, fmt->pixelformat);
			/* Set window size */
			ret |= arducam_mega_set_resolution(
				dev, support_resolution[i % SUPPORT_RESOLUTION_NUM]);
			if (!ret) {
				drv_data->fmt = *fmt;
				drv_data->fmt.pitch = drv_data->fmt.width * 2;
			}
			return ret;
		}
		i++;
	}
	/* Camera is not capable of handling given format */
	LOG_ERR("Image resolution not supported\n");
	return -ENOTSUP;
}

static int arducam_mega_get_fmt(const struct device *dev,
				struct video_format *fmt)
{
	struct arducam_mega_data *drv_data = dev->data;

	*fmt = drv_data->fmt;

	return 0;
}

static int arducam_mega_stream_start(const struct device *dev)
{
	struct arducam_mega_data *drv_data = dev->data;

	if (drv_data->stream_on) {
		return 0;
	}

	drv_data->stream_on = 1;
	drv_data->fifo_length = 0;

	k_work_schedule_for_queue(&drv_data->work_q,&drv_data->work, K_MSEC(30));

	return 0;
}

static int arducam_mega_stream_stop(const struct device *dev)
{
	struct arducam_mega_data *drv_data = dev->data;
	struct k_work_sync work_sync = {0};
	drv_data->stream_on = 0;
	k_work_cancel_delayable_sync(&drv_data->work, &work_sync);
	return 0;
}

static int arducam_mega_flush(const struct device *dev, bool cancel)
{
	struct arducam_mega_data *drv_data = dev->data;
	struct video_buffer *vbuf;

	/* Clear fifo cache */
	while (!k_fifo_is_empty(&drv_data->fifo_out)) {
		vbuf = k_fifo_get(&drv_data->fifo_out, K_USEC(10));
		if (vbuf != NULL) {
			k_fifo_put(&drv_data->fifo_in, vbuf);
		}
	}
	return 0;
}

static int arducam_mega_soft_reset(const struct device *dev)
{
	int ret = 0;
	const struct arducam_mega_config *cfg = dev->config;
	struct arducam_mega_data *drv_data = dev->data;

	if (drv_data->stream_on) {
		arducam_mega_stream_stop(dev);
	}
	/* Initiate system reset */
	ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_SENSOR_RESET, SENSOR_RESET_ENABLE);
	k_msleep(1000);

	return ret;
}

static int arducam_mega_capture(const struct device *dev, uint32_t *length)
{
	const struct arducam_mega_config *cfg = dev->config;
	struct arducam_mega_data *drv_data = dev->data;
	int trigger_reg;
	switch (drv_data->state)
	{
	case MEGA_STATE_INIT:
		arducam_mega_write_reg(&cfg->bus, ARDUCHIP_FIFO, FIFO_CLEAR_ID_MASK);
		arducam_mega_write_reg(&cfg->bus, ARDUCHIP_FIFO, FIFO_START_MASK);
		drv_data->capture_retry=0;
		drv_data->state = MEGA_STATE_CAPTURE;
		break;
	case MEGA_STATE_CAPTURE:
		trigger_reg = arducam_mega_read_reg(&cfg->bus, ARDUCHIP_TRIG);
		if((trigger_reg & CAP_DONE_MASK) == CAP_DONE_MASK) {
			drv_data->fifo_length = arducam_mega_read_reg(&cfg->bus, FIFO_SIZE1);
			drv_data->fifo_length |= (arducam_mega_read_reg(&cfg->bus, FIFO_SIZE2) << 8);
			drv_data->fifo_length |= (arducam_mega_read_reg(&cfg->bus, FIFO_SIZE3) << 16);
			drv_data->fifo_first_read = 1;
			*length = drv_data->fifo_length;
			drv_data->state = MEGA_STATE_INIT;
		}else
		{
			++drv_data->capture_retry;
			if(drv_data->capture_retry > 2) {
				drv_data->state = MEGA_STATE_INIT;
				LOG_WRN("reached max capture retries");
			}
		}
	default:
		break;
	}
	return 0;
}

static int arducam_mega_fifo_read(const struct device *dev, struct video_buffer *buf)
{
	int ret;
	int32_t rlen;
	const struct arducam_mega_config *cfg = dev->config;
	struct arducam_mega_data *drv_data = dev->data;

	rlen = buf->size > drv_data->fifo_length ? drv_data->fifo_length : buf->size;

	LOG_DBG("read fifo :%u. - fifo_length %u", buf->size, drv_data->fifo_length);

	ret = arducam_mega_read_block(&cfg->bus, buf->buffer, rlen, drv_data->fifo_first_read);

	if (ret == 0) {
		drv_data->fifo_length -= rlen;
		buf->bytesused = rlen;
		if (drv_data->fifo_first_read) {
			drv_data->fifo_first_read = 0;
		}
	}

	return ret;
}

static void arducam_mega_worker(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct arducam_mega_data *drv_data =
		CONTAINER_OF(dwork, struct arducam_mega_data, work);
	uint32_t f_timestamp = 0; 
	uint32_t f_length = 0;
	struct video_buffer *vbuf;

	k_work_reschedule_for_queue(&drv_data->work_q,&drv_data->work, K_MSEC(30));

	

	if (drv_data->fifo_length == 0) {
		arducam_mega_capture(drv_data->dev, &f_length);
		f_timestamp = k_uptime_get_32();
	}
	if(f_length > 0){
		vbuf = k_fifo_get(&drv_data->fifo_in, K_FOREVER);
		if (vbuf == NULL) {
			return;
		}
		arducam_mega_fifo_read(drv_data->dev, vbuf);
		vbuf->timestamp = f_timestamp;
		k_fifo_put(&drv_data->fifo_out, vbuf);
		if (IS_ENABLED(CONFIG_POLL) && drv_data->signal) {
			k_poll_signal_raise(drv_data->signal, VIDEO_BUF_DONE);
		}
	}
	k_yield();
	
#if 0 //Attention
	if (drv_data->fifo_length == 0) {
		vbuf->flags = VIDEO_BUF_EOF;
	} else {
		vbuf->flags = VIDEO_BUF_FRAG;
		k_work_submit_to_queue(&ac_work_q, &drv_data->buf_work);
	}

	vbuf->timestamp = f_timestamp;
	//Attention
	//vbuf->bytesframe = f_length;
	k_fifo_put(&drv_data->fifo_out, vbuf);
	if (IS_ENABLED(CONFIG_POLL) && drv_data->signal) {
		k_poll_signal_raise(drv_data->signal, VIDEO_BUF_DONE);
	}

	k_yield();
	#endif 
}

static int arducam_mega_enqueue(const struct device *dev,
				struct video_buffer *vbuf)
{
	struct arducam_mega_data *data = dev->data;

	k_fifo_put(&data->fifo_in, vbuf);

	LOG_DBG("enqueue buffer %p", (void*)vbuf->buffer);

	return 0;
}

static int arducam_mega_dequeue(const struct device *dev,
				struct video_buffer **vbuf, k_timeout_t timeout)
{
	struct arducam_mega_data *data = dev->data;

	*vbuf = k_fifo_get(&data->fifo_out, timeout);

	LOG_DBG("dequeue buffer %p", (void*)(*vbuf)->buffer);

	if (*vbuf == NULL) {
		return -EAGAIN;
	}

	return 0;
}

static int arducam_mega_get_caps(const struct device *dev,
				 struct video_caps *caps)
{
	caps->min_vbuf_count = 0;
	caps->min_line_count = LINE_COUNT_HEIGHT;
	caps->max_line_count = LINE_COUNT_HEIGHT;
	caps->format_caps = fmts;
	return 0;
}

static int arducam_mega_set_ctrl(const struct device *dev, uint32_t cid)
{
	int ret = 0;
	struct arducam_mega_data *data = dev->data;
	struct arducam_mega_ctrls *ctrls = &data->ctrls;

	switch (cid) {
	case VIDEO_CID_EXPOSURE_AUTO:
		ret = arducam_mega_set_exposure_enable(dev, ctrls->auto_exposure.val);
		break;
	case VIDEO_CID_EXPOSURE:
		ret = arducam_mega_set_exposure(dev, ctrls->exposure.val);
		break;
	case VIDEO_CID_AUTOGAIN:
		ret = arducam_mega_set_gain_enable(dev, ctrls->auto_gain.val);
		break;
	case VIDEO_CID_GAIN:
		ret = arducam_mega_set_gain(dev, ctrls->gain.val);
		break;
	case VIDEO_CID_BRIGHTNESS:
		ret = arducam_mega_set_brightness(dev, (enum MEGA_BRIGHTNESS_LEVEL)ctrls->brightness.val);
		break;
	case VIDEO_CID_SATURATION:
		ret = arducam_mega_set_saturation(dev, (enum MEGA_SATURATION_LEVEL)ctrls->saturation.val);
		break;
	case VIDEO_CID_AUTO_WHITE_BALANCE:
		ret = arducam_mega_set_white_bal_enable(dev, ctrls->auto_white_balance.val);
		break;
	case VIDEO_CID_WHITE_BALANCE_TEMPERATURE:
		ret = arducam_mega_set_white_bal(dev, (enum MEGA_WHITE_BALANCE )ctrls->auto_white_balance.val);
		break;
	case VIDEO_CID_CONTRAST:
		ret = arducam_mega_set_contrast(dev, (enum MEGA_CONTRAST_LEVEL )ctrls->contrast.val);
		break;
	case VIDEO_CID_JPEG_COMPRESSION_QUALITY:
		ret = arducam_mega_set_JPEG_quality(dev, (enum MEGA_IMAGE_QUALITY )ctrls->jpeg.val);
		break;
	case VIDEO_CID_ARDUCAM_EV:
		ret = arducam_mega_set_EV(dev, (enum MEGA_EV_LEVEL)ctrls->ev.val);
		break;
	case VIDEO_CID_SHARPNESS:
		ret = arducam_mega_set_sharpness(dev, (enum MEGA_SHARPNESS_LEVEL )ctrls->sharpness.val);
		break;
	case VIDEO_CID_COLORFX:
		ret = arducam_mega_set_special_effects(dev, (enum MEGA_COLOR_FX)ctrls->color_fx.val);
		break;	
	case VIDEO_CID_ARDUCAM_RESET:
		ret = arducam_mega_soft_reset(dev);
		ret = arducam_mega_check_connection(dev);
		break;
	case VIDEO_CID_ARDUCAM_LOWPOWER:
		ret = arducam_mega_set_lowpower_enable(dev, ctrls->low_power.val);
		break;
	default:
		return -ENOTSUP;
	}
	return ret;
}

int arducam_mega_get_info(const struct device *dev, struct arducam_mega_info *info)
{
	struct arducam_mega_data *drv_data = dev->data;

	*info = (*drv_data->info);

	return 0;
}

static int arducam_mega_set_stream(const struct device *dev, bool enable, enum video_buf_type type)
{
	if (enable) {
		return arducam_mega_stream_start(dev);
	} else {
		return arducam_mega_stream_stop(dev);
	}
}

#ifdef CONFIG_POLL
static int arducam_mega_set_signal(const struct device *dev, struct k_poll_signal *sig)
{
	struct arducam_mega_data *data = dev->data;

	if (data->signal && sig != NULL) {
		return -EALREADY;
	}

	data->signal = sig;

	return 0;
}
#endif


static int arducam_mega_get_volatile_ctrl(const struct device *dev, unsigned int cid)
{
	int ret = 0;
	uint32_t *value = &ret;

	switch (cid) {
	case VIDEO_CID_ARDUCAM_INFO:
		ret |= arducam_mega_get_info(dev, (struct arducam_mega_info *)value);
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}

static DEVICE_API(video, arducam_mega_driver_api) = {
	.set_format = arducam_mega_set_fmt,
	.get_format = arducam_mega_get_fmt,
	.set_stream = arducam_mega_set_stream,
	.get_caps = arducam_mega_get_caps,
	.flush = arducam_mega_flush,
	.set_ctrl = arducam_mega_set_ctrl,
	.get_volatile_ctrl = arducam_mega_get_volatile_ctrl,
#ifdef CONFIG_POLL
	.set_signal = arducam_mega_set_signal,
#endif
	.enqueue = arducam_mega_enqueue,
	.dequeue = arducam_mega_dequeue,
};



static int arducam_mega_init_controls(const struct device *dev)
{
	int ret;
	struct arducam_mega_data *drv_data = dev->data;
	struct arducam_mega_ctrls *ctrls = &drv_data->ctrls;

	ret = video_init_ctrl(&ctrls->auto_gain, dev, VIDEO_CID_AUTOGAIN,
			(struct video_ctrl_range){.min = 0, 
			.max = 1, .step = 1, 
			.def = 0});
	if (ret) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->gain, dev, VIDEO_CID_GAIN,
			(struct video_ctrl_range){.min = 0, 
			.max = (1<<10) - 1 , .step = 1, 
			.def = 0});
	if (ret) {
		return ret;
	}

	video_auto_cluster_ctrl(&ctrls->auto_gain,2,false);

		ret = video_init_ctrl(&ctrls->auto_exposure, dev, VIDEO_CID_EXPOSURE_AUTO,
			(struct video_ctrl_range){.min = 0, 
			.max = 1, .step = 1, 
			.def = 0});
	if (ret) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->exposure, dev, VIDEO_CID_EXPOSURE,
			(struct video_ctrl_range){.min = 0, 
			.max = (1<<20) - 1 , .step = 1, 
			.def = 0});
	if (ret) {
		return ret;
	}

	video_auto_cluster_ctrl(&ctrls->auto_exposure,2,false);


	ret = video_init_ctrl(&ctrls->auto_white_balance, dev, VIDEO_CID_AUTO_WHITE_BALANCE,
		(struct video_ctrl_range){.min = 0, 
		.max = 1, .step = 1, 
		.def = 0});
	if (ret) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->white_balance, dev, VIDEO_CID_WHITE_BALANCE_TEMPERATURE,
			(struct video_ctrl_range){.min = MEGA_WHITE_BALANCE_MODE_DEFAULT, 
			.max = MEGA_WHITE_BALANCE_MODE_HOME, .step = 1, 
			.def = MEGA_WHITE_BALANCE_MODE_DEFAULT});
	if (ret) {
		return ret;
	}

	video_auto_cluster_ctrl(&ctrls->auto_white_balance,2,false);



	ret = video_init_ctrl(&ctrls->brightness, dev, VIDEO_CID_BRIGHTNESS,
			(struct video_ctrl_range){.min = MEGA_BRIGHTNESS_LEVEL_NEGATIVE_4, 
			.max = MEGA_BRIGHTNESS_LEVEL_4, .step = 1, 
			.def = MEGA_BRIGHTNESS_LEVEL_DEFAULT});
	if (ret) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->contrast, dev, VIDEO_CID_CONTRAST,
			(struct video_ctrl_range){.min = MEGA_CONTRAST_LEVEL_NEGATIVE_3, 
					.max = MEGA_CONTRAST_LEVEL_3, .step = 1, 
					.def = MEGA_CONTRAST_LEVEL_DEFAULT});
	if (ret) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->saturation, dev, VIDEO_CID_SATURATION,
		(struct video_ctrl_range){.min = MEGA_SATURATION_LEVEL_NEGATIVE_3, 
			.max = MEGA_SATURATION_LEVEL_3, .step = 1, .def = MEGA_SATURATION_LEVEL_DEFAULT});
	if (ret) {
		return ret;
	}

	ret = video_init_ctrl(
		&ctrls->jpeg, dev, VIDEO_CID_JPEG_COMPRESSION_QUALITY,
		(struct video_ctrl_range){.min = HIGH_QUALITY, .max = LOW_QUALITY, .step = 1, .def = DEFAULT_QUALITY});
	if (ret) {
		return ret;
	}

	ret = video_init_ctrl(
		&ctrls->sharpness, dev, VIDEO_CID_SHARPNESS,
		(struct video_ctrl_range){.min = MEGA_SHARPNESS_LEVEL_AUTO, .max = MEGA_SHARPNESS_LEVEL_8, .step = 1, .def = MEGA_SHARPNESS_LEVEL_AUTO});
	if (ret) {
		return ret;
	}

	return ret;
}


static int arducam_mega_init(const struct device *dev)
{
	const struct arducam_mega_config *cfg = dev->config;
	struct arducam_mega_data *drv_data = dev->data;

	struct video_format fmt;
	int ret = 0;

	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("%s: device is not ready", cfg->bus.bus->name);
		return -ENODEV;
	}

	drv_data->dev = dev;
	k_fifo_init(&drv_data->fifo_in);
	k_fifo_init(&drv_data->fifo_out);
	k_work_init_delayable(&drv_data->work, arducam_mega_worker);
	k_work_queue_init(&drv_data->work_q);
	k_work_queue_start(&drv_data->work_q, ac_stack_area,
		K_THREAD_STACK_SIZEOF(ac_stack_area), AC_PRIORITY,
		NULL);

	arducam_mega_soft_reset(dev);
	ret = arducam_mega_check_connection(dev);

	if (ret) {
		LOG_ERR("arducam mega camera not connection.\n");
		return ret;
	}

	drv_data->state = MEGA_STATE_INIT;
	drv_data->capture_retry = 0;
	drv_data->ver.year = arducam_mega_read_reg(&cfg->bus, CAM_REG_YEAR_SDK) & 0x3F;
	drv_data->ver.month = arducam_mega_read_reg(&cfg->bus, CAM_REG_MONTH_SDK) & 0x0F;
	drv_data->ver.day = arducam_mega_read_reg(&cfg->bus, CAM_REG_DAY_SDK) & 0x1F;
	drv_data->ver.version =
		arducam_mega_read_reg(&cfg->bus, CAM_REG_FPGA_VERSION_NUMBER) & 0xfF;

	LOG_INF("arducam mega ver: %d-%d-%d \t %x", drv_data->ver.year, drv_data->ver.month,
		drv_data->ver.day, drv_data->ver.version);

	/* set default/init format 96x96 RGB565 */
	fmt.pixelformat = VIDEO_PIX_FMT_RGB565;
	fmt.width = 96;
	fmt.height = 96;
	fmt.pitch = 96 * video_bits_per_pixel(VIDEO_PIX_FMT_RGB565) / BITS_PER_BYTE;;
	ret = arducam_mega_set_fmt(dev, &fmt);
	if (ret) {
		LOG_ERR("Unable to configure default format");
		return -EIO;
	}

	return arducam_mega_init_controls(dev);
}

#define ARDUCAM_MEGA_INIT(inst)                                                                    \
	static const struct arducam_mega_config arducam_mega_cfg_##inst = {                        \
		.bus = SPI_DT_SPEC_INST_GET(inst,                                                  \
					    SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |                 \
						    SPI_CS_ACTIVE_HIGH | SPI_LINES_SINGLE |        \
						    SPI_LOCK_ON,                                   \
					    1),                                                    \
	};                                                                                         \
                                                                                                   \
	static struct arducam_mega_data arducam_mega_data_##inst;                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &arducam_mega_init, NULL, &arducam_mega_data_##inst,           \
			      &arducam_mega_cfg_##inst, POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY,   \
			      &arducam_mega_driver_api); \
	VIDEO_DEVICE_DEFINE(arducam_mega_##inst, DEVICE_DT_INST_GET(inst), NULL); 

DT_INST_FOREACH_STATUS_OKAY(ARDUCAM_MEGA_INIT)
 
