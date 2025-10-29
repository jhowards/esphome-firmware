#pragma once

#ifdef USE_ESP32

#include "../i2s_audio.h"

#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/FreeRTOS.h>

#include "esphome/components/audio/audio.h"
#include "esphome/components/speaker/speaker.h"

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"
#include "esphome/components/audio/ring_buffer.h"

namespace esphome {
namespace i2s_audio {

class I2SAudioSpeaker : public I2SAudioOut, public speaker::Speaker, public Component {
 public:
  float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }

  void setup() override;
  void loop() override;
  void dump_config() override {this->dump_i2s_settings();}

  void set_buffer_duration(uint32_t buffer_duration_ms) { this->buffer_duration_ms_ = buffer_duration_ms; }
  void set_timeout(uint32_t ms) { this->timeout_ = ms; }

  void start() override;
  void stop() override;
  void finish() override;

  void set_pause_state(bool pause_state) override { this->pause_state_ = pause_state; }
  bool get_pause_state() const override { return this->pause_state_; }

  /// @brief Plays the provided audio data.
  /// Starts the speaker task, if necessary. Writes the audio data to the ring buffer.
  /// @param data Audio data in the format set by the parent speaker classes ``set_audio_stream_info`` method.
  /// @param length The length of the audio data in bytes.
  /// @param ticks_to_wait The FreeRTOS ticks to wait before writing as much data as possible to the ring buffer.
  /// @return The number of bytes that were actually written to the ring buffer.
  size_t play(const uint8_t *data, size_t length, TickType_t ticks_to_wait, bool write_partial = false) override;
  size_t play(const uint8_t *data, size_t length) override { return play(data, length, 0); }
  
  
  /// @brief Inserts silence by delaying audio readout and filling the DMA buffer with zeros.
  /// This function writes zeros to the DMA buffer instead of audio data for a specified duration.
  /// @param length_ms Duration of silence to insert, in milliseconds.
  /// @return Total number of zero frames that are currently in the queue.
  size_t play_silence(size_t length_ms) override;
 
  bool has_buffered_data() const override;

  /// @brief Sets the volume of the speaker. Uses the speaker's configured audio dac component. If unavailble, it is
  /// implemented as a software volume control. Overrides the default setter to convert the floating point volume to a
  /// Q15 fixed-point factor.
  /// @param volume between 0.0 and 1.0
  void set_volume(float volume) override;

  /// @brief Mutes or unmute the speaker. Uses the speaker's configured audio dac component. If unavailble, it is
  /// implemented as a software volume control. Overrides the default setter to convert the floating point volume to a
  /// Q15 fixed-point factor.
  /// @param mute_state true for muting, false for unmuting
  void set_mute_state(bool mute_state) override;
  
  int64_t get_playout_time( int64_t self_buffer_us ) const override;
  bool update_buffer_states(int32_t bytes_transfered ) override;

 protected:
  /// @brief Function for the FreeRTOS task handling audio output.
  /// After receiving the COMMAND_START signal, allocates space for the buffers, starts the I2S driver, and reads
  /// audio from the ring buffer and writes audio to the I2S port. Stops immmiately after receiving the COMMAND_STOP
  /// signal and stops only after the ring buffer is empty after receiving the COMMAND_STOP_GRACEFULLY signal. Stops if
  /// the ring buffer hasn't read data for more than timeout_ milliseconds. When stopping, it deallocates the buffers,
  /// stops the I2S driver, unlocks the I2S port, and deletes the task. It communicates the state and any errors via
  /// event_group_.
  /// @param params I2SAudioSpeaker component
  static void speaker_task(void *params);

  /// @brief Sends a stop command to the speaker task via event_group_.
  /// @param wait_on_empty If false, sends the COMMAND_STOP signal. If true, sends the COMMAND_STOP_GRACEFULLY signal.
  void stop_(bool wait_on_empty);

  /// @brief Sets the corresponding ERR_ESP event group bits.
  /// @param err esp_err_t error code.
  /// @return True if an ERR_ESP bit is set and false if err == ESP_OK
  bool send_esp_err_to_event_group_(esp_err_t err);

  /// @brief Allocates the data buffer and ring buffer
  /// @param data_buffer_size Number of bytes to allocate for the data buffer.
  /// @param ring_buffer_size Number of bytes to allocate for the ring buffer.
  /// @return ESP_ERR_NO_MEM if either buffer fails to allocate
  ///         ESP_OK if successful
  esp_err_t allocate_buffers_(size_t data_buffer_size, size_t ring_buffer_size);

  /// @brief Starts the ESP32 I2S driver.
  /// Attempts to lock the I2S port, starts the I2S driver using the passed in stream information, and sets the data out
  /// pin. If it fails, it will unlock the I2S port and uninstall the driver, if necessary.
  /// @param audio_stream_info Stream information for the I2S driver.
  /// @return ESP_ERR_NOT_ALLOWED if the I2S port can't play the incoming audio stream.
  ///         ESP_ERR_INVALID_STATE if the I2S port is already locked.
  ///         ESP_ERR_INVALID_ARG if nstalling the driver or setting the data outpin fails due to a parameter error.
  ///         ESP_ERR_NO_MEM if the driver fails to install due to a memory allocation error.
  ///         ESP_FAIL if setting the data out pin fails due to an IO error ESP_OK if successful
  esp_err_t start_i2s_driver_(audio::AudioStreamInfo &audio_stream_info);
  
  /// @brief Deletes the speaker's task.
  /// Deallocates the data_buffer_ and audio_ring_buffer_, if necessary, and deletes the task. Should only be called by
  /// the speaker_task itself.
  /// @param buffer_size The allocated size of the data_buffer_.
  void delete_task_(size_t buffer_size);
  
#ifndef USE_I2S_LEGACY
  /// @brief Callback function used to send playback timestamps the to the speaker task.
  /// @param handle (i2s_chan_handle_t)
  /// @param event (i2s_event_data_t)
  /// @param user_ctx (void*) User context pointer that the callback accesses
  /// @return True if a higher priority task was interrupted
  static bool i2s_on_sent_cb(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx);
#endif

  TaskHandle_t speaker_task_handle_{nullptr};
  EventGroupHandle_t event_group_{nullptr};

  uint8_t *data_buffer_;
  std::shared_ptr<audio::RingBuffer> audio_ring_buffer_;

  uint32_t buffer_duration_ms_;

  optional<uint32_t> timeout_;

  bool task_created_{false};
  bool pause_state_{false};

  int16_t q15_volume_factor_{INT16_MAX};
  
  int64_t last_dma_write_{0};
  size_t padded_zero_frames_{0};
  size_t bytes_in_ringbuffer_{0};
  size_t in_write_buffer_{0};
  SemaphoreHandle_t lock_;
  QueueHandle_t i2s_sent_time_queue_;
};

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
