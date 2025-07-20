/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT himax_hm01b0
#include <hardware/pio.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "video_ctrls.h"
#include "video_device.h"


LOG_MODULE_REGISTER(hm01b0, CONFIG_VIDEO_LOG_LEVEL);
#define HM01B0_STACK_SIZE 1024
#define HM01B0_PRIORITY 5
K_THREAD_STACK_DEFINE(hm01b0_stack_area, HM01B0_STACK_SIZE);
#define MAX_FRAME_RATE 10
#define MIN_FRAME_RATE 1
#define HM01B0_ID 0x01B0
#define HM01B0_START_BIT (0)
#define HM01B0_STOP_BIT (1)
#define HM01B0_RX_BIT (2)
#define HM01B0_RELOAD_BIT (3)
#define HM01B0_RECONFIGURE_BIT (4)

enum hm01b0_resolution {
  RESOLUTION_160x120 = 0,
  RESOLUTION_320x240,
  RESOLUTION_320x320
};

enum hm01b0_reg {
  REG_ID = 0x0000,
  REG_STS = 0x0100,
  REG_RESET = 0x0103,
  REG_GRP_PARAM_HOLD = 0x0104,
  REG_INTEGRATION_H = 0x0202,
  REG_FRAME_LENGTH_LINES = 0x0340,
  REG_LINE_LENGTH_PCLK = 0x0342,
  REG_WIDTH = 0x0383,
  REG_HEIGHT = 0x0387,
  REG_BINNING_MODE = 0x0390,
  REG_QVGA_WIN_EN = 0x3010,
  REG_BIT_CONTROL = 0x3059,
  REG_OSC_CLOCK_DIV = 0x3060
};

struct hm01b0_reg_value {
  enum hm01b0_reg address;
  size_t value_size; // 1 or 2 bytes
  union {
    uint16_t word;
    uint8_t byte;
  } value;
};

const struct hm01b0_reg_value hm01b0_reg_values[RESOLUTION_320x320 + 1][6] =
    {[RESOLUTION_160x120] =
         {{REG_WIDTH, sizeof(uint8_t), .value.byte = 0x3},
          {REG_HEIGHT, sizeof(uint8_t), .value.byte = 0x3},
          {REG_BINNING_MODE, sizeof(uint8_t), .value.byte = 0x3},
          {REG_QVGA_WIN_EN, sizeof(uint8_t), .value.byte = 0x1},
          {REG_FRAME_LENGTH_LINES, sizeof(uint16_t), .value.word = 0x80},
          {REG_LINE_LENGTH_PCLK, sizeof(uint16_t), .value.word = 0xD7}},
     [RESOLUTION_320x240] =
         {{REG_WIDTH, sizeof(uint8_t), .value.byte = 0x1},
          {REG_HEIGHT, sizeof(uint8_t), .value.byte = 0x1},
          {REG_BINNING_MODE, sizeof(uint8_t), .value.byte = 0x0},
          {REG_QVGA_WIN_EN, sizeof(uint8_t), .value.byte = 0x1},
          {REG_FRAME_LENGTH_LINES, sizeof(uint16_t), .value.word = 0x104},
          {REG_LINE_LENGTH_PCLK, sizeof(uint16_t), .value.word = 0x178}

         },
     [RESOLUTION_320x320] = {
         {REG_WIDTH, sizeof(uint8_t), .value.byte = 0x1},
         {REG_HEIGHT, sizeof(uint8_t), .value.byte = 0x1},
         {REG_BINNING_MODE, sizeof(uint8_t), .value.byte = 0x0},
         {REG_QVGA_WIN_EN, sizeof(uint8_t), .value.byte = 0x0},
         {REG_FRAME_LENGTH_LINES, sizeof(uint16_t), .value.word = 0x158},
         {REG_LINE_LENGTH_PCLK, sizeof(uint16_t), .value.word = 0x178}}};

struct hm01b0_ctrls {
  struct video_ctrl integration;
};

struct hm01b0_data {
  const struct device *dev;
  PIO pio;
  size_t pio_sm;
  int pio_program_offset;
  pio_sm_config pio_sm_config;
  pio_program_t pio_program;
  struct hm01b0_ctrls ctrls;
  struct video_format fmt;
  struct k_fifo fifo_in;
  struct k_fifo fifo_out;
  struct k_work_delayable work;
  struct k_work_q work_q;
  struct k_poll_signal *sig;
  struct video_buffer *vbuf;
  enum hm01b0_resolution resolution;
  int dma_channel;
  int pattern;
  unsigned int frame_rate;
  unsigned int pclk_per_px;
  unsigned int border_px;
  uint8_t ctrl_val;
  atomic_t stream_evt;
  bool stream_on;
};

struct hm01b0_config {
  const struct device *piodev;
  const struct pinctrl_dev_config *pin_cfg;
  struct gpio_dt_spec vsync_gpio;
  struct gpio_dt_spec hsync_gpio;
  struct gpio_dt_spec pclk_gpio;
  struct gpio_dt_spec data_gpio;
  struct i2c_dt_spec i2c_dev;
  const struct device *dma_dev;
  const uint32_t data_bits;
};

#define HM01B0_VIDEO_FORMAT_CAP(width, height, format)                         \
  {.pixelformat = (format),                                                    \
   .width_min = (width),                                                       \
   .width_max = (width),                                                       \
   .height_min = (height),                                                     \
   .height_max = (height),                                                     \
   .width_step = 0,                                                            \
   .height_step = 0}

static const struct video_format_cap hm01b0_fmts[] = {
    HM01B0_VIDEO_FORMAT_CAP(160, 120, VIDEO_PIX_FMT_GREY),
    HM01B0_VIDEO_FORMAT_CAP(320, 240, VIDEO_PIX_FMT_GREY),
    HM01B0_VIDEO_FORMAT_CAP(320, 320, VIDEO_PIX_FMT_GREY),
    {0}};

#ifdef CONFIG_POLL
static int hm01b0_set_signal(const struct device *dev,
                             struct k_poll_signal *sig) {
  struct hm01b0_data *data = dev->data;

  if (data->sig && sig != NULL) {
    return -EALREADY;
  }
  data->sig = sig;
  return 0;
}
#endif

static int hm01b0_write_reg8(const struct device *dev, enum hm01b0_reg reg,
                             uint8_t val) {
  struct hm01b0_config *config = (struct hm01b0_config *)dev->config;
  uint8_t data[3];
  uint16_t address = (uint16_t)reg;
  address = sys_cpu_to_be16(address);
  memcpy(data, &address, sizeof(address));
  data[2] = val;
  return i2c_write_dt(&config->i2c_dev, (const uint8_t *)data, sizeof(data));
}

static int hm01b0_read_reg8(const struct device *dev, enum hm01b0_reg reg,
                            uint8_t *val) {
  struct hm01b0_config *config = (struct hm01b0_config *)dev->config;
  uint16_t address = (uint16_t)reg;
  address = sys_cpu_to_be16(reg);
  return i2c_write_read_dt(&config->i2c_dev, &address, sizeof(address), val,
                           sizeof(*val));
}

static int hm01b0_write_reg16(const struct device *dev, enum hm01b0_reg reg,
                              uint16_t val) {
  struct hm01b0_config *config = (struct hm01b0_config *)dev->config;
  uint16_t address = (uint16_t)reg;
  const uint16_t data[2] = {sys_cpu_to_be16(address), sys_cpu_to_be16(val)};
  return i2c_write_dt(&config->i2c_dev, (const uint8_t *)data, sizeof(data));
}

static int hm01b0_read_reg16(const struct device *dev, enum hm01b0_reg reg,
                             uint16_t *val) {
  struct hm01b0_config *config = (struct hm01b0_config *)dev->config;
  uint16_t address = (uint16_t)reg;
  address = sys_cpu_to_be16(reg);
  int res = i2c_write_read_dt(&config->i2c_dev, &address, sizeof(address), val,
                              sizeof(*val));
  *val = sys_be16_to_cpu(*val);
  return res;
}

static int hm01b0_config_pio(const struct device *dev) {
  struct hm01b0_config *config = (struct hm01b0_config *)dev->config;
  struct hm01b0_data *data = (struct hm01b0_data *)dev->data;
  if (data->pio_program_offset >= 0) {
    pio_remove_program(data->pio, &data->pio_program, data->pio_program_offset);
    data->pio_program_offset = -1;
  }
  uint16_t pio_program_instructions[] = {
      /* 0 */ pio_encode_pull(false, true),
      /* 1 */ pio_encode_wait_gpio(false, config->vsync_gpio.pin),
      /* 2 */ pio_encode_wait_gpio(true, config->vsync_gpio.pin),
      /* 3 */ pio_encode_set(pio_y, data->border_px - 1),
      /* 4 */ pio_encode_wait_gpio(true, config->hsync_gpio.pin), // border
                                                                  // pixel y
      /* 5 */ pio_encode_wait_gpio(false, config->hsync_gpio.pin),
      /* 6 */ pio_encode_jmp_y_dec(4),
      /* .wrap_target */
      /* 7 */ pio_encode_mov(pio_x, pio_osr),
      /* 8 */ pio_encode_wait_gpio(true, config->hsync_gpio.pin),
      /* 9 */ pio_encode_set(pio_y, data->border_px * data->pclk_per_px - 1),
      /* 10 */ pio_encode_wait_gpio(true, config->pclk_gpio.pin), // border
                                                                  // pixel x
      /* 11 */ pio_encode_wait_gpio(false, config->pclk_gpio.pin),
      /* 12 */ pio_encode_jmp_y_dec(10),
      /* 13 */ pio_encode_wait_gpio(true, config->pclk_gpio.pin),
      /* 14 */ pio_encode_in(pio_pins, config->data_bits),
      /* 15 */ pio_encode_wait_gpio(false, config->pclk_gpio.pin),
      /* 16 */ pio_encode_jmp_x_dec(13),
      /* 17 */ pio_encode_wait_gpio(false, config->hsync_gpio.pin),
      /* .wrap */
  };

  data->pio_program.instructions = pio_program_instructions;
  data->pio_program.length =
      sizeof(pio_program_instructions) / sizeof(pio_program_instructions[0]);
  data->pio_program.origin = -1;

  data->pio_program_offset = pio_add_program(data->pio, &data->pio_program);
  if (data->pio_program_offset >= 0) {
    data->pio_sm_config = pio_get_default_sm_config();
    sm_config_set_in_pins(&data->pio_sm_config, config->data_gpio.pin);
    sm_config_set_in_shift(&data->pio_sm_config, true, true, data->pclk_per_px);
    sm_config_set_wrap(&data->pio_sm_config, data->pio_program_offset + 7,
                       data->pio_program_offset + data->pio_program.length - 1);
    pio_sm_set_consecutive_pindirs(data->pio, data->pio_sm,
                                   config->data_gpio.pin, 1, false);
  }
  return data->pio_program_offset >= 0 ? 0 : data->pio_program_offset;
}

static void hm01b0_dma_callback(const struct device *dev, void *user_data,
                                uint32_t channel, int status) {
  struct hm01b0_data *data = (struct hm01b0_data *)user_data;

  if (status >= 0) {
    atomic_set_bit(&data->stream_evt, HM01B0_RX_BIT);
    k_work_reschedule_for_queue(&data->work_q, &data->work, K_NO_WAIT);
  }
}

static int hm01b0_stop_capture(const struct device *dev) {
  struct hm01b0_data *data = (struct hm01b0_data *)dev->data;
  struct hm01b0_config *config = (struct hm01b0_config *)dev->config;
  LOG_DBG("hm01b0 capture stop");
  pio_sm_set_enabled(data->pio, data->pio_sm, false);
  dma_stop(config->dma_dev, data->dma_channel);
  dma_release_channel(config->dma_dev, data->dma_channel);
  /* MODE_SELECT */
  return hm01b0_write_reg8(data->dev, REG_STS, 0x00);
}

static int hm01b0_disable_capture(const struct device *dev) {
  struct hm01b0_data *data = (struct hm01b0_data *)dev->data;
  LOG_DBG("hm01b0 capture disable");
  pio_sm_set_enabled(data->pio, data->pio_sm, false);
  /* MODE_SELECT */
  return hm01b0_write_reg8(data->dev, REG_STS, 0x00);
}

static int hm01b0_reload_capture(const struct device *dev) {
  struct hm01b0_data *data = (struct hm01b0_data *)dev->data;
  struct hm01b0_config *config = (struct hm01b0_config *)dev->config;
  int ret = -ENOMEM;
  struct video_buffer *vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT);
  LOG_DBG("hm01b0 capture reload");
  if (vbuf != NULL) {
    data->vbuf = vbuf;
    uint8_t *src_addr = (uint8_t *)&data->pio->rxf[data->pio_sm];
    pio_sm_init(data->pio, data->pio_sm, data->pio_program_offset,
                &data->pio_sm_config);
    dma_reload(config->dma_dev, data->dma_channel, (uint32_t)&src_addr[3],
               (uint32_t)vbuf->buffer, data->fmt.width * data->fmt.height);
    dma_start(config->dma_dev, data->dma_channel);
    pio_sm_set_enabled(data->pio, data->pio_sm, true);
    pio_sm_put_blocking(data->pio, data->pio_sm,
                        data->fmt.width * data->pclk_per_px - 1);
    /* MODE_SELECT */
    ret = hm01b0_write_reg8(dev, REG_STS, 0x01);
  }
  return ret;
}

static int hm01b0_start_capture(const struct device *dev) {
  struct hm01b0_data *data = (struct hm01b0_data *)dev->data;
  struct hm01b0_config *config = (struct hm01b0_config *)dev->config;
  struct video_buffer *vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT);
  int ret = -ENOMEM;
  LOG_DBG("hm01b0 capture start");
  if (vbuf != NULL) {
    data->vbuf = vbuf;
    data->dma_channel = dma_request_channel(config->dma_dev, NULL);
    if (data->dma_channel >= 0) {
      pio_sm_init(data->pio, data->pio_sm, data->pio_program_offset,
                  &data->pio_sm_config);
      uint8_t *src_addr = (uint8_t *)&data->pio->rxf[data->pio_sm];
      struct dma_block_config block_cfg = {
          .source_address = (uint32_t)&src_addr[3],
          .dest_address = (uint32_t)(vbuf->buffer),
          .source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
          .dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,
          .block_size = data->fmt.width * data->fmt.height};

      struct dma_config dma_cfg = {
          .dma_slot = ~pio_get_dreq(data->pio, data->pio_sm, false),
          .channel_direction = PERIPHERAL_TO_MEMORY,
          .block_count = 1,
          .head_block = &block_cfg,
          .source_data_size = sizeof(uint8_t),
          .dest_data_size = sizeof(uint8_t),
          .source_burst_length = block_cfg.block_size,
          .dma_callback = hm01b0_dma_callback,
          .user_data = data};
      ret = dma_config(config->dma_dev, data->dma_channel, &dma_cfg);
      if (ret == 0) {
        dma_start(config->dma_dev, data->dma_channel);
        pio_sm_set_enabled(data->pio, data->pio_sm, true);
        pio_sm_put_blocking(data->pio, data->pio_sm,
                            data->fmt.width * data->pclk_per_px - 1);
        /* MODE_SELECT */
        hm01b0_write_reg8(dev, REG_STS, 0x01);

      } else {
        LOG_ERR("Failed to configure DMA channel %d", data->dma_channel);
        dma_release_channel(config->dma_dev, data->dma_channel);
      }
    } else {
      LOG_ERR("Failed to request DMA channel");
      ret = data->dma_channel; // Return the error code from dma_request_channel
    }
  } else {
    LOG_ERR("Failed to get video buffer from FIFO");
  }
  return ret;
}

static int hm01b0_apply_configuration(const struct device *dev,
                                      enum hm01b0_resolution resolution) {
  struct hm01b0_data *data = (struct hm01b0_data *)dev->data;
  int ret;
  for (int i = 0; i < ARRAY_SIZE(hm01b0_reg_values[resolution]); i++) {
    const struct hm01b0_reg_value *reg_val = &hm01b0_reg_values[resolution][i];
    if (reg_val->value_size == sizeof(uint8_t)) {
      ret = hm01b0_write_reg8(dev, reg_val->address, reg_val->value.byte);
    } else {
      ret = hm01b0_write_reg16(dev, reg_val->address, reg_val->value.word);
    }
    if (ret < 0) {
      LOG_ERR("Failed to write config list register 0x%04x", reg_val->address);
      return ret;
    }
  }
  /* REG_BIT_CONTROL */
  ret = hm01b0_write_reg8(dev, REG_BIT_CONTROL, data->ctrl_val);
  if (ret == 0) {
    /* OSC_CLK_DIV */
    ret = hm01b0_write_reg8(dev, REG_OSC_CLOCK_DIV, 0x08);
    if (ret == 0) {
      /* INTEGRATION_H */
      ret = hm01b0_write_reg16(dev, REG_INTEGRATION_H,
                               hm01b0_reg_values[resolution][5].value.word / 2);
      if (ret == 0) {
        /* GRP_PARAM_HOLD */
        hm01b0_write_reg8(dev, REG_GRP_PARAM_HOLD, 0x01);
        if (ret == 0) {
          ret = hm01b0_config_pio(dev);
        }
      }
    }
  }
  if (ret != 0) {
    LOG_ERR("Error applying new configuration (%d)", ret);
  }
  return ret;
}

static int hm01b0_get_caps(const struct device *dev, struct video_caps *caps) {
  caps->min_vbuf_count = 0;
  caps->min_line_count = LINE_COUNT_HEIGHT;
  caps->max_line_count = LINE_COUNT_HEIGHT;
  caps->format_caps = hm01b0_fmts;
  return 0;
}

static int hm01b0_set_fmt(const struct device *dev, struct video_format *fmt) {
  struct hm01b0_data *drv_data = (struct hm01b0_data *)dev->data;
  uint16_t width, height;
  int ret = 0;
  int i = 0;
  LOG_INF("HM01B0 set_fmt: %d x %d, fmt: %s", fmt->width, fmt->height,
          VIDEO_FOURCC_TO_STR(fmt->pixelformat));

  /* We only support GREY pixel formats */
  if (fmt->pixelformat != VIDEO_PIX_FMT_GREY) {
    LOG_ERR("HM01B0 camera only supports GREY pixel formats!");
    return -ENOTSUP;
  }

  width = fmt->width;
  height = fmt->height;

  if (!memcmp(&drv_data->fmt, fmt, sizeof(drv_data->fmt))) {
    return 0;
  }

  /* Check if camera is capable of handling given format */
  while (hm01b0_fmts[i].pixelformat) {
    if (hm01b0_fmts[i].width_min == width &&
        hm01b0_fmts[i].height_min == height &&
        hm01b0_fmts[i].pixelformat == fmt->pixelformat) {
      drv_data->resolution = (enum hm01b0_resolution)i;
      atomic_set_bit(&drv_data->stream_evt, HM01B0_RECONFIGURE_BIT);
      drv_data->fmt = *fmt;
      k_work_reschedule_for_queue(&drv_data->work_q, &drv_data->work,
                                  K_MSEC(500));
      return ret;
    }
    ++i;
  }
  /* Camera is not capable of handling given format */
  LOG_ERR("Image resolution not supported\n");
  return -ENOTSUP;
}

static int hm01b0_get_fmt(const struct device *dev, struct video_format *fmt) {
  struct hm01b0_data *data = dev->data;
  *fmt = data->fmt;
  LOG_INF("HM01B0 get_fmt: %d x %d, fmt: %s", fmt->width, fmt->height,
          VIDEO_FOURCC_TO_STR(fmt->pixelformat));
  return 0;
}

static int hm01b0_set_ctrl(const struct device *dev, uint32_t id) { return 0; }

static int hm01b0_set_stream(const struct device *dev, bool enable,
                             enum video_buf_type type) {
  struct hm01b0_data *data = (struct hm01b0_data *)dev->data;

  if (enable && !data->stream_on) {
    atomic_set_bit(&data->stream_evt, HM01B0_START_BIT);
    k_work_reschedule_for_queue(&data->work_q, &data->work, K_MSEC(100));
  } else if (!enable && data->stream_on) {
    atomic_set_bit(&data->stream_evt, HM01B0_STOP_BIT);
    k_work_reschedule_for_queue(&data->work_q, &data->work, K_NO_WAIT);
  }
  return 0;
}

static int hm01b0_enqueue(const struct device *dev, struct video_buffer *vbuf) {
  struct hm01b0_data *data = dev->data;

  k_fifo_put(&data->fifo_in, vbuf);

  return 0;
}

static int hm01b0_dequeue(const struct device *dev, struct video_buffer **vbuf,
                          k_timeout_t timeout) {
  struct hm01b0_data *data = dev->data;

  *vbuf = k_fifo_get(&data->fifo_out, timeout);
  if (*vbuf == NULL) {
    return -EAGAIN;
  }

  return 0;
}

static int hm01b0_flush(const struct device *dev, bool cancel) {
  struct hm01b0_data *data = dev->data;
  struct video_buffer *vbuf;

  if (!cancel) {
    /* wait for all buffer to be processed */
    do {
      k_sleep(K_MSEC(1));
    } while (!k_fifo_is_empty(&data->fifo_in));
  } else {
    while ((vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT))) {
      k_fifo_put(&data->fifo_out, vbuf);
      if (IS_ENABLED(CONFIG_POLL) && data->sig) {
        k_poll_signal_raise(data->sig, VIDEO_BUF_ABORTED);
      }
    }
  }
  return 0;
}
static void hm01b0_worker(struct k_work *work) {
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct hm01b0_data *data;
  data = CONTAINER_OF(dwork, struct hm01b0_data, work);
  struct hm01b0_config *config = (struct hm01b0_config *)data->dev->config;
  bool reload = false;
  int ret = 0;

  if (atomic_test_and_clear_bit(&data->stream_evt, HM01B0_START_BIT)) {
    ret = hm01b0_start_capture(data->dev);
    if (ret == 0) {
      data->stream_on = true;
    } else {
      LOG_ERR("Failed to start hm01b0 stream (%d)", ret);
    }
  }

  if (atomic_test_and_clear_bit(&data->stream_evt, HM01B0_RECONFIGURE_BIT)) {
    if (data->stream_on) {
      ret = hm01b0_disable_capture(data->dev);
      dma_stop(config->dma_dev, data->dma_channel);
      reload = true;
      if (ret != 0) {
        LOG_ERR("Failed to disable hm01b0 stream (%d)", ret);
      }
    }
    if (ret == 0) {
      ret = hm01b0_apply_configuration(data->dev, data->resolution);
      if (ret != 0) {
        LOG_ERR("Failed to reconfigure hm01b0 (%d)", ret);
      }
    }
  }

  if (atomic_test_and_clear_bit(&data->stream_evt, HM01B0_STOP_BIT)) {
    data->stream_on = false;
    ret = hm01b0_stop_capture(data->dev);
    if (ret != 0) {
      LOG_ERR("Failed to stop hm01b0 stream (%d)", ret);
    }
  }

  if (data->stream_on) {
    if (atomic_test_and_clear_bit(&data->stream_evt, HM01B0_RELOAD_BIT)) {
      ret = hm01b0_reload_capture(data->dev);
      if (ret != 0) {
        LOG_WRN("Warning reloading hm01b0 stream (%d)", ret);
        reload = true;
      }
    }
  }

  if (atomic_test_and_clear_bit(&data->stream_evt, HM01B0_RX_BIT)) {
    if (hm01b0_disable_capture(data->dev) == 0) {
      data->vbuf->bytesused = data->fmt.height * data->fmt.width;
      data->vbuf->timestamp = k_uptime_get_32();
      k_fifo_put(&data->fifo_out, data->vbuf);
      if (IS_ENABLED(CONFIG_POLL) && data->sig) {
        k_poll_signal_raise(data->sig, VIDEO_BUF_DONE);
      }
      atomic_set_bit(&data->stream_evt, HM01B0_RELOAD_BIT);
      k_work_reschedule_for_queue(&data->work_q, &data->work,
                                  K_MSEC(1000  / data->frame_rate));
    }
  }

  if (reload) {
    atomic_set_bit(&data->stream_evt, HM01B0_RELOAD_BIT);
    k_work_reschedule_for_queue(&data->work_q, &data->work,
                                K_MSEC(1000  / data->frame_rate));
  }
  k_yield();
}

static int hm01b0_soft_reset(const struct device *dev) {
  int ret = hm01b0_write_reg8(dev, REG_RESET, 0x01);
  uint8_t val = 0xff;
  if (ret == 0) {
    for (int retries = 0; retries < 10; retries++) {
      ret = hm01b0_read_reg8(dev, REG_STS, &val);
      if (ret != 0 || val == 0x0) {
        break;
      }
      k_msleep(100);
    }
  }
  if (ret != 0) {
    LOG_ERR("Soft reset error (%d)", ret);
  }
  return ret;
}

static int hm01b0_set_frmival(const struct device *dev,
                              struct video_frmival *frmival) {
  struct hm01b0_data *data = dev->data;

  data->frame_rate =
      CLAMP(DIV_ROUND_CLOSEST(frmival->denominator, frmival->numerator),
            MIN_FRAME_RATE, MAX_FRAME_RATE);
  frmival->numerator = 1;
  frmival->denominator = data->frame_rate;

  return 0;
}

static int hm01b0_get_frmival(const struct device *dev,
                              struct video_frmival *frmival) {
  struct hm01b0_data *data = dev->data;

  frmival->numerator = 1;
  frmival->denominator = data->frame_rate;

  return 0;
}

static int hm01b0_enum_frmival(const struct device *dev, struct video_frmival_enum *fie)
{
	size_t idx;
	int ret;

	if (fie->index >= 1) {
		return -ERANGE;
	}

	ret = video_format_caps_index(hm01b0_fmts, fie->format, &idx);
	if (ret < 0) {
		LOG_ERR("Unsupported pixel format or resolution");
		return ret;
	}

	fie->type = VIDEO_FRMIVAL_TYPE_STEPWISE;
	fie->stepwise.min.numerator = 1;
	fie->stepwise.min.denominator = MIN_FRAME_RATE;
	fie->stepwise.max.numerator = 1;
	fie->stepwise.max.denominator = MAX_FRAME_RATE;
	/* The frame interval step size is the minimum resolution of K_MSEC(), which is 1ms */
	fie->stepwise.step.numerator = 1;
	fie->stepwise.step.denominator = 1000;

	return 0;
}


static DEVICE_API(video,
                  hm01b0_driver_api) = {.set_format = hm01b0_set_fmt,
                                        .get_format = hm01b0_get_fmt,
                                        .set_stream = hm01b0_set_stream,
                                        .get_caps = hm01b0_get_caps,
                                        .enqueue = hm01b0_enqueue,
                                        .dequeue = hm01b0_dequeue,
                                        .set_ctrl = hm01b0_set_ctrl,
                                        .set_frmival = hm01b0_set_frmival,
                                        .get_frmival = hm01b0_get_frmival,
                                        .enum_frmival = hm01b0_enum_frmival,
#ifdef CONFIG_POLL
                                        .set_signal = hm01b0_set_signal,
#endif
                                        .flush = hm01b0_flush};

static int hm01b0_init_controls(const struct device *dev) { return 0; }

static bool hm01b0_check_connection(const struct device *dev) {
  uint16_t model_id;
  int ret = hm01b0_read_reg16(dev, REG_ID, &model_id);
  bool is_connected = (ret == 0 && model_id == HM01B0_ID);
  if (!is_connected) {
    LOG_ERR("Model ID mismatch: expected 0x%04x, got 0x%04x, ret (%d)",
            HM01B0_ID, model_id, ret);
  }
  return is_connected;
}

static int hm01b0_init(const struct device *dev) {
  struct hm01b0_data *data = (struct hm01b0_data *)dev->data;
  struct hm01b0_config *config = (struct hm01b0_config *)dev->config;
  data->dev = dev;
  data->pio = pio_rpi_pico_get_pio(config->piodev);
  int ret = pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT);
  if (ret) {
    LOG_ERR("Failed to apply pinctrl state");
    return ret;
  }

  if (pio_rpi_pico_allocate_sm(config->piodev, &data->pio_sm) != 0) {
    LOG_ERR("Failed to allocate PIO state machine");
    return -ENODEV;
  }

  if (config->data_bits == 8) {
    data->ctrl_val = 0x02;
    data->pclk_per_px = 1;
  } else if (config->data_bits == 4) {
    data->ctrl_val = 0x42;
    data->pclk_per_px = 2;
  } else if (config->data_bits == 1) {
    data->ctrl_val = 0x22;
    data->pclk_per_px = 8;
  } else {
    LOG_ERR("Invalid data bits!");
    return -ENODEV;
  }

  if (!hm01b0_check_connection(dev)) {
    LOG_ERR("%s is not ready", dev->name);
    return -ENODEV;
  }

  if (hm01b0_soft_reset(dev)) {
    LOG_ERR("error sof reset");
    return -ENODEV;
  }
  /* 2 pixels for borders */
  data->border_px = 2;
  data->pio_program_offset = -1;

  struct video_format fmt = {
      .pixelformat = VIDEO_PIX_FMT_GREY,
      .width = 160,
      .height = 120,
      .type = VIDEO_BUF_TYPE_OUTPUT,
      .pitch = 160 * video_bits_per_pixel(VIDEO_PIX_FMT_GREY) / BITS_PER_BYTE};

  if (hm01b0_set_fmt(dev, &fmt)) {
    LOG_ERR("error setting video format");
    return -ENODEV;
  }

  k_fifo_init(&data->fifo_in);
  k_fifo_init(&data->fifo_out);
  data->frame_rate = MAX_FRAME_RATE;
  data->stream_on = false;
  k_work_init_delayable(&data->work, hm01b0_worker);
  k_work_queue_init(&data->work_q);
  k_work_queue_start(&data->work_q, hm01b0_stack_area,
                     K_THREAD_STACK_SIZEOF(hm01b0_stack_area), HM01B0_PRIORITY,
                     NULL);
#ifdef HM01B0_LIB
  const static struct lib_hm01b0_config hm01b0_config = {

#ifdef SPARKFUN_MICROMOD
      .vsync_pin = 25,
      .hsync_pin = 28,
      .pclk_pin = 11,
      .data_pin_base = 16, // Base data pin
      .data_bits = 8, // The SparkFun MicroMod ML Carrier Board has all 8 data
                      // pins connected
      .pio = pio0,
      .pio_sm = 0,
      .reset_pin = 24,
      .mclk_pin = 10,
#else
      .vsync_pin = 6,
      .hsync_pin = 7,
      .pclk_pin = 8,
      .data_pin_base = 9,
      .data_bits = 1,
      .pio = pio0,
      .pio_sm = 0,
      .reset_pin = -1, // Not connected
      .mclk_pin = -1,  // Not connected
#endif

      .width = 160,
      .height = 120,
  };

  lib_hm01b0_init(&hm01b0_config, &data->pio_program_offset,
                  &data->pio_sm_config);
  struct video_format fmt = {
      .pixelformat = VIDEO_PIX_FMT_GREY,
      .width = 160,
      .height = 120,
      .type = VIDEO_BUF_TYPE_OUTPUT,
      .pitch = 120 * video_bits_per_pixel(VIDEO_PIX_FMT_GREY) / BITS_PER_BYTE};
  data->fmt = fmt;
  data->pio = hm01b0_config.pio;
  data->pio_sm = hm01b0_config.pio_sm;
  data->pclk_per_px = 8;
  data->border_px = 2; // 2 pixels for borders
  data->resolution = RESOLUTION_160x120;
  data->ctrl_val = 0x02; // 8 bits per pixel
  data->pattern = 0;

#endif
  return hm01b0_init_controls(dev);
}

#define HM01B0_INIT(inst)                                                      \
  PINCTRL_DT_INST_DEFINE(inst);                                                \
  static struct hm01b0_config hm01b0_config_##inst = {                         \
      .piodev = DEVICE_DT_GET(DT_INST_PARENT(inst)),                           \
      .pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                         \
      .vsync_gpio = GPIO_DT_SPEC_INST_GET(inst, vsync_gpios),                  \
      .hsync_gpio = GPIO_DT_SPEC_INST_GET(inst, hsync_gpios),                  \
      .pclk_gpio = GPIO_DT_SPEC_INST_GET(inst, pclk_gpios),                    \
      .data_gpio = GPIO_DT_SPEC_INST_GET(inst, data_gpios),                    \
      .i2c_dev = {DEVICE_DT_GET(DT_PROP(DT_DRV_INST(inst), i2c_bus)),          \
                  DT_PROP(DT_DRV_INST(inst), i2c_address)},                    \
      .dma_dev = DEVICE_DT_GET(DT_PROP(DT_DRV_INST(inst), dma)),               \
      .data_bits = 1 /* Use only 1 pin for data */                             \
  };                                                                           \
  static struct hm01b0_data hm01b0_data_##inst;                                \
  DEVICE_DT_INST_DEFINE(inst, &hm01b0_init, NULL, &hm01b0_data_##inst,         \
                        &hm01b0_config_##inst, POST_KERNEL,                    \
                        CONFIG_VIDEO_INIT_PRIORITY, &hm01b0_driver_api);       \
  VIDEO_DEVICE_DEFINE(hm01b0_##inst, DEVICE_DT_INST_GET(inst), NULL);

DT_INST_FOREACH_STATUS_OKAY(HM01B0_INIT)
