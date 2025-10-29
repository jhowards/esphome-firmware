#pragma once

#ifdef USE_ESP32

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <array>

#include "esphome/components/i2s_audio/i2s_audio.h"
#include "esphome/components/microphone/microphone.h"
#include "esphome/core/component.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

namespace esphome {
namespace i2s_audio {

class Sat1Microphone : public I2SAudioIn, public microphone::Microphone, public Component {
 public:
  void setup() override;
  void dump_config() override {this->dump_i2s_settings();}
  void start();
  void stop();

  void loop() override;

  void set_correct_dc_offset(bool correct_dc_offset) { this->correct_dc_offset_ = correct_dc_offset; }

 protected:
  /// @brief Starts the I2S driver. Updates the ``audio_stream_info_`` member variable with the current setttings.
  /// @return True if succesful, false otherwise
  bool start_driver_();

  /// @brief Stops the I2S driver.
  bool stop_driver_();

  /// @brief Attempts to correct a microphone DC offset; e.g., a microphones silent level is offset from 0. Applies a
  /// correction offset that is updated using an exponential moving average for all samples away from 0.
  /// @param data
  void fix_dc_offset_(std::vector<uint8_t> &data);

  size_t read_(uint8_t *buf, size_t len, TickType_t ticks_to_wait);

  /// @brief Sets the Microphone ``audio_stream_info_`` member variable to the configured I2S settings.
  void configure_stream_settings_();
  void reset_filters_();
  size_t decimate_and_filter_audio_(int32_t *samples, size_t frames, uint8_t channels);

  static void mic_task(void *params);

  static constexpr uint8_t MAX_AUDIO_CHANNELS = 2;
  static constexpr uint8_t DECIMATION_FACTOR = 3;

  std::array<std::array<int32_t, DECIMATION_FACTOR>, MAX_AUDIO_CHANNELS> decimator_buffer_{};
  uint8_t decimator_phase_{0};
  uint8_t decimator_valid_samples_{0};

  SemaphoreHandle_t active_listeners_semaphore_{nullptr};
  EventGroupHandle_t event_group_{nullptr};
  TaskHandle_t task_handle_{nullptr};
  bool correct_dc_offset_;
  int32_t dc_offset_{0};
};


}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
