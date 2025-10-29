#include "i2s_audio_speaker.h"

#ifdef USE_ESP32

#include "esphome/components/audio/audio.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "esp_timer.h"

namespace esphome {
namespace i2s_audio {

static const size_t TASK_STACK_SIZE = 4096;
static const ssize_t TASK_PRIORITY = 17;

static const char *const TAG = "i2s_audio.speaker";

enum SpeakerEventGroupBits : uint32_t {
  COMMAND_START = (1 << 0),            // starts the speaker task
  COMMAND_STOP = (1 << 1),             // stops the speaker task
  COMMAND_STOP_GRACEFULLY = (1 << 2),  // Stops the speaker task once all data has been written
  STATE_STARTING = (1 << 10),
  STATE_RUNNING = (1 << 11),
  STATE_STOPPING = (1 << 12),
  STATE_STOPPED = (1 << 13),
  ERR_TASK_FAILED_TO_START = (1 << 14),
  ERR_ESP_INVALID_STATE = (1 << 15),
  ERR_ESP_NOT_SUPPORTED = (1 << 16),
  ERR_ESP_INVALID_ARG = (1 << 17),
  ERR_ESP_INVALID_SIZE = (1 << 18),
  ERR_ESP_NO_MEM = (1 << 19),
  ERR_ESP_FAIL = (1 << 20),
  ALL_ERR_ESP_BITS = ERR_ESP_INVALID_STATE | ERR_ESP_NOT_SUPPORTED | ERR_ESP_INVALID_ARG | ERR_ESP_INVALID_SIZE |
                     ERR_ESP_NO_MEM | ERR_ESP_FAIL,
  ALL_BITS = 0x00FFFFFF,  // All valid FreeRTOS event group bits
};

// Translates a SpeakerEventGroupBits ERR_ESP bit to the coressponding esp_err_t
static esp_err_t err_bit_to_esp_err(uint32_t bit) {
  switch (bit) {
    case SpeakerEventGroupBits::ERR_ESP_INVALID_STATE:
      return ESP_ERR_INVALID_STATE;
    case SpeakerEventGroupBits::ERR_ESP_INVALID_ARG:
      return ESP_ERR_INVALID_ARG;
    case SpeakerEventGroupBits::ERR_ESP_INVALID_SIZE:
      return ESP_ERR_INVALID_SIZE;
    case SpeakerEventGroupBits::ERR_ESP_NO_MEM:
      return ESP_ERR_NO_MEM;
    case SpeakerEventGroupBits::ERR_ESP_NOT_SUPPORTED:
      return ESP_ERR_NOT_SUPPORTED;
    default:
      return ESP_FAIL;
  }
}

/// @brief Multiplies the input array of Q15 numbers by a Q15 constant factor
///
/// Based on `dsps_mulc_s16_ansi` from the esp-dsp library:
/// https://github.com/espressif/esp-dsp/blob/master/modules/math/mulc/fixed/dsps_mulc_s16_ansi.c
/// (accessed on 2024-09-30).
/// @param input Array of Q15 numbers
/// @param output Array of Q15 numbers
/// @param len Length of array
/// @param c Q15 constant factor
static void q15_multiplication(const int16_t *input, int16_t *output, size_t len, int16_t c) {
  for (int i = 0; i < len; i++) {
    int32_t acc = (int32_t) input[i] * (int32_t) c;
    output[i] = (int16_t) (acc >> 15);
  }
}

// Lists the Q15 fixed point scaling factor for volume reduction.
// Has 100 values representing silence and a reduction [49, 48.5, ... 0.5, 0] dB.
// dB to PCM scaling factor formula: floating_point_scale_factor = 2^(-db/6.014)
// float to Q15 fixed point formula: q15_scale_factor = floating_point_scale_factor * 2^(15)
static const std::vector<int16_t> Q15_VOLUME_SCALING_FACTORS = {
    0,     116,   122,   130,   137,   146,   154,   163,   173,   183,   194,   206,   218,   231,   244,
    259,   274,   291,   308,   326,   345,   366,   388,   411,   435,   461,   488,   517,   548,   580,
    615,   651,   690,   731,   774,   820,   868,   920,   974,   1032,  1094,  1158,  1227,  1300,  1377,
    1459,  1545,  1637,  1734,  1837,  1946,  2061,  2184,  2313,  2450,  2596,  2750,  2913,  3085,  3269,
    3462,  3668,  3885,  4116,  4360,  4619,  4893,  5183,  5490,  5816,  6161,  6527,  6914,  7324,  7758,
    8218,  8706,  9222,  9770,  10349, 10963, 11613, 12302, 13032, 13805, 14624, 15491, 16410, 17384, 18415,
    19508, 20665, 21891, 23189, 24565, 26022, 27566, 29201, 30933, 32767};


void I2SAudioSpeaker::setup() {
  ESP_LOGCONFIG(TAG, "Setting up I2S Audio Speaker...");

    this->event_group_ = xEventGroupCreate();

  if (this->event_group_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create event group");
    this->mark_failed();
    return;
  }

  this->lock_ = xSemaphoreCreateMutex();
}

void I2SAudioSpeaker::loop() {
  uint32_t event_group_bits = xEventGroupGetBits(this->event_group_);

  if (event_group_bits & SpeakerEventGroupBits::STATE_STARTING) {
    ESP_LOGD(TAG, "Starting Speaker");
    this->state_ = speaker::STATE_STARTING;
    xEventGroupClearBits(this->event_group_, SpeakerEventGroupBits::STATE_STARTING);
  }
  if (event_group_bits & SpeakerEventGroupBits::STATE_RUNNING) {
    ESP_LOGD(TAG, "Started Speaker");
    this->state_ = speaker::STATE_RUNNING;
    xEventGroupClearBits(this->event_group_, SpeakerEventGroupBits::STATE_RUNNING);
    this->status_clear_warning();
    this->status_clear_error();
  }
  if (event_group_bits & SpeakerEventGroupBits::STATE_STOPPING) {
    ESP_LOGD(TAG, "Stopping Speaker");
    this->state_ = speaker::STATE_STOPPING;
    xEventGroupClearBits(this->event_group_, SpeakerEventGroupBits::STATE_STOPPING);
  }
  if (event_group_bits & SpeakerEventGroupBits::STATE_STOPPED) {
    if (!this->task_created_) {
      ESP_LOGD(TAG, "Stopped Speaker");
      this->state_ = speaker::STATE_STOPPED;
      xEventGroupClearBits(this->event_group_, SpeakerEventGroupBits::ALL_BITS);
      this->speaker_task_handle_ = nullptr;
    }
  }

  if (event_group_bits & SpeakerEventGroupBits::ERR_TASK_FAILED_TO_START) {
    this->status_set_error("Failed to start speaker task");
    xEventGroupClearBits(this->event_group_, SpeakerEventGroupBits::ERR_TASK_FAILED_TO_START);
  }

  if (event_group_bits & SpeakerEventGroupBits::ALL_ERR_ESP_BITS) {
    uint32_t error_bits = event_group_bits & SpeakerEventGroupBits::ALL_ERR_ESP_BITS;
    ESP_LOGW(TAG, "Error writing to I2S: %s", esp_err_to_name(err_bit_to_esp_err(error_bits)));
    this->status_set_warning();
  }

  if (event_group_bits & SpeakerEventGroupBits::ERR_ESP_NOT_SUPPORTED) {
    this->status_set_error("Failed to adjust I2S bus to match the incoming audio");
    ESP_LOGE(TAG,
             "Incompatible audio format: sample rate = %" PRIu32 ", channels = %" PRIu8 ", bits per sample = %" PRIu8,
             this->audio_stream_info_.get_sample_rate(), this->audio_stream_info_.get_channels(),
             this->audio_stream_info_.get_bits_per_sample());
}

  xEventGroupClearBits(this->event_group_, ALL_ERR_ESP_BITS);
}

void I2SAudioSpeaker::set_volume(float volume) {
  this->volume_ = volume;
#ifdef USE_AUDIO_DAC
  if (this->audio_dac_ != nullptr) {
    if (volume > 0.0) {
      this->audio_dac_->set_mute_off();
    }
    this->audio_dac_->set_volume(volume);
  } else
#endif
  {
    // Fallback to software volume control by using a Q15 fixed point scaling factor
    ssize_t decibel_index = remap<ssize_t, float>(volume, 0.0f, 1.0f, 0, Q15_VOLUME_SCALING_FACTORS.size() - 1);
    this->q15_volume_factor_ = Q15_VOLUME_SCALING_FACTORS[decibel_index];
  }
}

void I2SAudioSpeaker::set_mute_state(bool mute_state) {
  this->mute_state_ = mute_state;
#ifdef USE_AUDIO_DAC
  if (this->audio_dac_) {
    if (mute_state) {
      this->audio_dac_->set_mute_on();
    } else {
      this->audio_dac_->set_mute_off();
  }
  } else
#endif
  {
    if (mute_state) {
      // Fallback to software volume control and scale by 0
      this->q15_volume_factor_ = 0;
    } else {
      // Revert to previous volume when unmuting
      this->set_volume(this->volume_);
    }
  }
}

size_t I2SAudioSpeaker::play(const uint8_t *data, size_t length, TickType_t ticks_to_wait, bool write_partial) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Cannot play audio, speaker failed to setup");
    return 0;
  }
  if (this->state_ != speaker::STATE_RUNNING && this->state_ != speaker::STATE_STARTING) {
    this->start();
  }

  if ((this->state_ != speaker::STATE_RUNNING)) { 
    // Unable to write data to a running speaker, so delay the max amount of time so it can get ready
    vTaskDelay(ticks_to_wait);
    ticks_to_wait = 0;
  }

  size_t bytes_written = 0;
  auto rb = this->audio_ring_buffer_;
  if (!rb) return 0;
  
  bytes_written = rb->write_without_replacement((void *) data, length, ticks_to_wait, write_partial);
  return bytes_written;
}

bool I2SAudioSpeaker::update_buffer_states(int32_t bytes_transfered) {
  if (xSemaphoreTake(this->lock_, pdMS_TO_TICKS(10))) {
    this->bytes_in_ringbuffer_ += bytes_transfered;
    xSemaphoreGive(this->lock_);
    return true;
  }
  return false;
}

bool I2SAudioSpeaker::has_buffered_data() const {
  if (this->audio_ring_buffer_ != nullptr) {
    return this->audio_ring_buffer_->available() > 0;
  }
  return false;
}

void I2SAudioSpeaker::speaker_task(void *params) {
  I2SAudioSpeaker *this_speaker = (I2SAudioSpeaker *) params;
  this_speaker->task_created_ = true;

  uint32_t event_group_bits =
      xEventGroupWaitBits(this_speaker->event_group_,
                          SpeakerEventGroupBits::COMMAND_START | SpeakerEventGroupBits::COMMAND_STOP |
                              SpeakerEventGroupBits::COMMAND_STOP_GRACEFULLY,  // Bit message to read
                          pdTRUE,                                              // Clear the bits on exit
                          pdFALSE,                                             // Don't wait for all the bits,
                          portMAX_DELAY);                                      // Block indefinitely until a bit is set

  if (event_group_bits & (SpeakerEventGroupBits::COMMAND_STOP | SpeakerEventGroupBits::COMMAND_STOP_GRACEFULLY)) {
    // Received a stop signal before the task was requested to start
    this_speaker->delete_task_(0);
  }

  xEventGroupSetBits(this_speaker->event_group_, SpeakerEventGroupBits::STATE_STARTING);

  audio::AudioStreamInfo audio_stream_info = this_speaker->audio_stream_info_;

  const size_t dma_buffer_size_bytes = this_speaker->get_dma_buffer_size_bytes();
  const uint8_t dma_buffers_count = this_speaker->get_dma_buffer_count();
  const size_t dma_buffer_duration_ms = this_speaker->get_dma_buffer_size_ms();
  const size_t task_delay_ms = dma_buffer_duration_ms * dma_buffers_count / 2;
  
  const uint8_t expand_factor = this_speaker->i2s_bits_per_sample() / audio_stream_info.get_bits_per_sample();
  const size_t read_buffer_size = dma_buffer_size_bytes * dma_buffers_count / expand_factor;
  
  // Ensure ring buffer duration is at least the duration of all DMA buffers
  const size_t duration_settings_bytes = audio_stream_info.ms_to_bytes(this_speaker->buffer_duration_ms_);
  const size_t ring_buffer_size = std::max(duration_settings_bytes, read_buffer_size);

  if (this_speaker->i2s_sent_time_queue_ == nullptr) {
    this_speaker->i2s_sent_time_queue_ = xQueueCreate(dma_buffers_count+1, sizeof(int64_t));
  }


#ifndef USE_I2S_LEGACY
  uint8_t* scaling_buffer = nullptr;
  if (expand_factor > 1) {
    // Allocate a scaling buffer to convert the audio data to the required bits per sample
    // The size of the scaling buffer is the same as the DMA buffer size, but with the expanded bits per sample
    scaling_buffer = new uint8_t[dma_buffer_size_bytes];
  } 
#endif
  
  if (this_speaker->send_esp_err_to_event_group_(this_speaker->allocate_buffers_(read_buffer_size, ring_buffer_size))) {
    // Failed to allocate buffers
    xEventGroupSetBits(this_speaker->event_group_, SpeakerEventGroupBits::ERR_ESP_NO_MEM);
    this_speaker->delete_task_(read_buffer_size);
  }

  if (!this_speaker->send_esp_err_to_event_group_(this_speaker->start_i2s_driver_(audio_stream_info))) {
    xEventGroupSetBits(this_speaker->event_group_, SpeakerEventGroupBits::STATE_RUNNING);

    bool stop_gracefully = false;
    uint32_t last_data_received_time = millis();

    this_speaker->last_dma_write_ = 0;
    
    // Keep looping if paused, there is no timeout configured, or data was received more recently than the configured
    // timeout
    while (this_speaker->pause_state_ || !this_speaker->timeout_.has_value() ||
           (millis() - last_data_received_time) <= this_speaker->timeout_.value()) {
      event_group_bits = xEventGroupGetBits(this_speaker->event_group_);

      if (event_group_bits & SpeakerEventGroupBits::COMMAND_STOP) {
        xEventGroupClearBits(this_speaker->event_group_, SpeakerEventGroupBits::COMMAND_STOP);
        break;
      }
      if (event_group_bits & SpeakerEventGroupBits::COMMAND_STOP_GRACEFULLY) {
        xEventGroupClearBits(this_speaker->event_group_, SpeakerEventGroupBits::COMMAND_STOP_GRACEFULLY);
        stop_gracefully = true;
      }
      if (this_speaker->audio_stream_info_ != audio_stream_info) {
        // Audio stream info changed, stop the speaker task so it will restart with the proper settings.
        break;
      }

      if (this_speaker->pause_state_) {
        // Pause state is accessed atomically, so thread safe
        // Delay so the task can yields, then skip transferring audio data
        delay(task_delay_ms);
        continue;
      }
      
      // we always write full dma-buffer-sized chunks to the I2S port
      // write zeros if not enough data is available
      std::memset(this_speaker->data_buffer_, 0, read_buffer_size);
      //fill up all dma-buffers with zeros, gives a defined playout time
      if( this_speaker->last_dma_write_ == 0 ){
        size_t bytes_written = 0;
        if( expand_factor == 1) {
            i2s_channel_write(this_speaker->parent_->get_tx_handle(), this_speaker->data_buffer_,
                          read_buffer_size, &bytes_written, 0);
        } else if (expand_factor == 2) {
          std::memset(scaling_buffer, 0, dma_buffer_size_bytes);
          for (uint32_t i = 0; i < dma_buffers_count; ++i) {
            esp_err_t err = i2s_channel_write(
              this_speaker->parent_->get_tx_handle(), 
              scaling_buffer,
              dma_buffer_size_bytes, 
              &bytes_written, 
              0
            );
            if( bytes_written != dma_buffer_size_bytes){
              break;
            }
          }  
        }
        this_speaker->last_dma_write_ = esp_timer_get_time();
      }

      size_t delay_bytes = 0;
      size_t bytes_read = 0;
      if(xSemaphoreTake( this_speaker->lock_, pdMS_TO_TICKS(10))) {            
        delay_bytes = audio_stream_info.frames_to_bytes(this_speaker->padded_zero_frames_);
        if( read_buffer_size > delay_bytes ){
            size_t to_read = read_buffer_size - delay_bytes; 
        
            bytes_read = this_speaker->audio_ring_buffer_->read(
              (void *) this_speaker->data_buffer_, 
              to_read,
              0
            );
            
            this_speaker->bytes_in_ringbuffer_ -= bytes_read;
            this_speaker->padded_zero_frames_  -= audio_stream_info.bytes_to_frames(delay_bytes);
            this_speaker->in_write_buffer_ = read_buffer_size;  
          
        } else {
          // don't read any bytes, write zeros to all DMA buffers instead
          delay_bytes = read_buffer_size;
          this_speaker->padded_zero_frames_ -= std::min( this_speaker->padded_zero_frames_, (size_t) audio_stream_info.bytes_to_frames(delay_bytes));
          this_speaker->in_write_buffer_ = read_buffer_size;
        }
        xSemaphoreGive(this_speaker->lock_);
      } else {
        continue;
      }
      
      if ( bytes_read > 0 && (audio_stream_info.get_bits_per_sample() == 16) && (this_speaker->q15_volume_factor_ < INT16_MAX)) {
        // Scale samples by the volume factor in place
        q15_multiplication((int16_t *) this_speaker->data_buffer_, (int16_t *) this_speaker->data_buffer_,
                            bytes_read / sizeof(int16_t), this_speaker->q15_volume_factor_);
      }

      if( bytes_read > 0 ){
        last_data_received_time = millis();
      }
      
      // Write the audio data to a single DMA buffer at a time,
      // limits the required size of the scaling buffer to the size of one DMA buffer
      for (uint32_t i = 0; i < dma_buffers_count; ++i) {
        const size_t bytes_to_write = read_buffer_size / dma_buffers_count;
        size_t bytes_written = 0;

#ifdef USE_I2S_LEGACY
        if ( expand_factor == 1) {
            i2s_write(this_speaker->parent_->get_port(), this_speaker->data_buffer_ + i * bytes_to_write,
                      bytes_to_write, &bytes_written, pdMS_TO_TICKS(dma_buffer_duration_ms * 5));
        } else if ( expand_factor > 1) {
            i2s_write_expand(this_speaker->parent_->get_port(),
                             this_speaker->data_buffer_ + i * bytes_to_write, bytes_to_write,
                           audio_stream_info.get_bits_per_sample(), this_speaker->bits_per_sample_, &bytes_written,
                             pdMS_TO_TICKS(dma_buffer_duration_ms * 5));
        }
#else
        if( expand_factor == 1) {
          i2s_channel_write(this_speaker->parent_->get_tx_handle(), this_speaker->data_buffer_ + i * bytes_to_write,
                          bytes_to_write, &bytes_written, pdMS_TO_TICKS(dma_buffer_duration_ms * 5));
        } else if (expand_factor == 2) {
          int32_t *output = reinterpret_cast<int32_t*>(scaling_buffer);
          int16_t *input  = reinterpret_cast<int16_t*>(this_speaker->data_buffer_ + i * bytes_to_write);
          size_t  samples = bytes_to_write / sizeof(int16_t);
          for (size_t s = 0; s < samples; ++s) {
             output[s] = static_cast<int32_t>(input[s]) << 16;
          }
          esp_err_t err = i2s_channel_write(
            this_speaker->parent_->get_tx_handle(), 
            scaling_buffer,
            samples * sizeof(int32_t), 
            &bytes_written, 
            pdMS_TO_TICKS(dma_buffer_duration_ms * 5)
          );
          bytes_written /= 2;
        }
#endif
        
        if (bytes_written != bytes_to_write) {
          xEventGroupSetBits(this_speaker->event_group_, SpeakerEventGroupBits::ERR_ESP_INVALID_SIZE);
        }
      }
      
      int64_t write_timestamp;
      while (xQueueReceive(this_speaker->i2s_sent_time_queue_, &write_timestamp, 0)){}
      
      if(xSemaphoreTake( this_speaker->lock_, pdMS_TO_TICKS(10))) {  
        this_speaker->in_write_buffer_ = 0;
        this_speaker->last_dma_write_ = write_timestamp + dma_buffer_duration_ms * 1000;
        xSemaphoreGive(this_speaker->lock_);
      }

      // No data received
      if ( bytes_read == 0 && stop_gracefully) {
        break;
      }
    }
    
    this_speaker->last_dma_write_ = 0;
    xEventGroupSetBits(this_speaker->event_group_, SpeakerEventGroupBits::STATE_STOPPING);

    this_speaker->stop_i2s_channel_();
#ifndef USE_I2S_LEGACY
    if(scaling_buffer != nullptr)
    {
      // Deallocate the scaling buffer if it was allocated
      delete[] scaling_buffer;
    }    
#endif
  }
  this_speaker->delete_task_(read_buffer_size);
}

void I2SAudioSpeaker::start() {
  if (!this->is_ready() || this->is_failed() || this->status_has_error())
    return;
  if ((this->state_ == speaker::STATE_STARTING) || (this->state_ == speaker::STATE_RUNNING))
    return;

  if (!this->task_created_ && (this->speaker_task_handle_ == nullptr)) {
    xTaskCreate(I2SAudioSpeaker::speaker_task, "speaker_task", TASK_STACK_SIZE, (void *) this, TASK_PRIORITY,
                &this->speaker_task_handle_);

  if (this->speaker_task_handle_ != nullptr) {
    xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::COMMAND_START);
  } else {
    xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_TASK_FAILED_TO_START);
    }
  }
}

size_t I2SAudioSpeaker::play_silence(size_t length_ms){
  audio::AudioStreamInfo audio_stream_info = this->audio_stream_info_;
  if (xSemaphoreTake( this->lock_, pdMS_TO_TICKS(10))){
    this->padded_zero_frames_ += audio_stream_info.ms_to_frames(length_ms);
    xSemaphoreGive(this->lock_);
  }
  return this->padded_zero_frames_;
}


int64_t I2SAudioSpeaker::get_playout_time( int64_t add_buffer_us ) const {
  if( this->audio_ring_buffer_ == nullptr ){
    return 0;
  }
  audio::AudioStreamInfo audio_stream_info = this->audio_stream_info_;
  const size_t dma_buffer_duration_us = this->get_dma_buffer_size_ms() * 1000;
  const uint8_t dma_buffers_count = this->get_dma_buffer_count();
  int64_t playout_at = 0;
  uint32_t pending_frames = 0;
  if (xSemaphoreTake(this->lock_, pdMS_TO_TICKS(10))){
      playout_at  = this->last_dma_write_ + static_cast<int64_t>(dma_buffers_count) * dma_buffer_duration_us;
      playout_at += add_buffer_us;  
      
      pending_frames  = this->audio_stream_info_.bytes_to_frames(this->bytes_in_ringbuffer_ + this->in_write_buffer_);
      pending_frames += this->padded_zero_frames_;
      playout_at += this->audio_stream_info_.frames_to_microseconds(pending_frames);
      
      //printf( "pending-ringbuffer: %d\n", this->audio_stream_info_.bytes_to_ms(this->bytes_in_ringbuffer_));
      //printf( "pending-zeros: %d\n", this->audio_stream_info_.frames_to_microseconds(this->padded_zero_frames_) / 1000);
      //printf( "dma-buffer: %d (time_delta: %d)\n", in_dma_buffer, time_delta);
      //printf( "Speaker: returning %d us\n" , playout_at );
      xSemaphoreGive(this->lock_);
  }
  
  return playout_at;
}




void I2SAudioSpeaker::stop() { this->stop_(false); }

void I2SAudioSpeaker::finish() { this->stop_(true); }

void I2SAudioSpeaker::stop_(bool wait_on_empty) {
  if (this->is_failed())
    return;
  if (this->state_ == speaker::STATE_STOPPED)
    return;

  if (wait_on_empty) {
    xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::COMMAND_STOP_GRACEFULLY);
  } else {
    xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::COMMAND_STOP);
  }
}

bool I2SAudioSpeaker::send_esp_err_to_event_group_(esp_err_t err) {
  switch (err) {
    case ESP_OK:
      return false;
    case ESP_ERR_INVALID_STATE:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_INVALID_STATE);
      return true;
    case ESP_ERR_INVALID_ARG:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_INVALID_ARG);
      return true;
    case ESP_ERR_INVALID_SIZE:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_INVALID_SIZE);
      return true;
    case ESP_ERR_NO_MEM:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_NO_MEM);
      return true;
    case ESP_ERR_NOT_SUPPORTED:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_NOT_SUPPORTED);
      return true;
    default:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_FAIL);
      return true;
  }
}

esp_err_t I2SAudioSpeaker::allocate_buffers_(size_t data_buffer_size, size_t ring_buffer_size) {
  if (this->data_buffer_ == nullptr) {
    // Allocate data buffer for temporarily storing audio from the ring buffer before writing to the I2S bus
    ExternalRAMAllocator<uint8_t> allocator(ExternalRAMAllocator<uint8_t>::ALLOW_FAILURE);
    this->data_buffer_ = allocator.allocate(data_buffer_size);
  }

  if (this->data_buffer_ == nullptr) {
    return ESP_ERR_NO_MEM;
  }

  if (this->audio_ring_buffer_.use_count() == 0) {
    // Allocate ring buffer. Uses a shared_ptr to ensure it isn't improperly deallocated.
    this->audio_ring_buffer_ = audio::RingBuffer::create(ring_buffer_size);
  }

  if (this->audio_ring_buffer_ == nullptr) {
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}


#ifndef USE_I2S_LEGACY
bool IRAM_ATTR I2SAudioSpeaker::i2s_on_sent_cb(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx) {
  int64_t now = esp_timer_get_time();

  BaseType_t need_yield1 = pdFALSE;
  BaseType_t need_yield2 = pdFALSE;
  BaseType_t need_yield3 = pdFALSE;

  I2SAudioSpeaker *this_speaker = (I2SAudioSpeaker *) user_ctx;

  if (xQueueIsQueueFullFromISR(this_speaker->i2s_sent_time_queue_)) {
    // Queue is full, so discard the oldest event
    int64_t dummy;
    xQueueReceiveFromISR(this_speaker->i2s_sent_time_queue_, &dummy, &need_yield1);
  }

  xQueueSendToBackFromISR(this_speaker->i2s_sent_time_queue_, &now, &need_yield3);

  return need_yield1 | need_yield2 | need_yield3;
}
#endif


esp_err_t I2SAudioSpeaker::start_i2s_driver_(audio::AudioStreamInfo &audio_stream_info) {
  if (this->has_fixed_i2s_rate() && (this->sample_rate_ != audio_stream_info.get_sample_rate())) {  // NOLINT
    // Can't reconfigure I2S bus, so the sample rate must match the configured value
    return ESP_ERR_NOT_SUPPORTED;
  }
  if ( this->has_fixed_i2s_bitdepth() && 
       this->i2s_bits_per_sample() != audio_stream_info.get_bits_per_sample() &&
       this->i2s_bits_per_sample() != 2 * audio_stream_info.get_bits_per_sample()
     ) {  // NOLINT
    // Can't reconfigure I2S bus, and bit depth must match the configured value
    return ESP_ERR_NOT_SUPPORTED;
  }
#if USE_I2S_LEGACY
  if( !this->start_i2s_channel_() ){
    return ESP_ERR_INVALID_STATE;
  }
#else
  const i2s_event_callbacks_t callbacks = {
                .on_sent = i2s_on_sent_cb,
    };  
if( !this->start_i2s_channel_(callbacks) ){
    return ESP_ERR_INVALID_STATE;
  }
#endif  
  return ESP_OK;
}

void I2SAudioSpeaker::delete_task_(size_t buffer_size) {
  this->audio_ring_buffer_.reset();  // Releases ownership of the shared_ptr
  this->bytes_in_ringbuffer_ = 0;
  
  if (this->data_buffer_ != nullptr) {
    ExternalRAMAllocator<uint8_t> allocator;
    allocator.deallocate(this->data_buffer_, buffer_size);
    this->data_buffer_ = nullptr;
  }

  xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::STATE_STOPPED);

  this->task_created_ = false;
  vTaskDelete(nullptr);
}

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
