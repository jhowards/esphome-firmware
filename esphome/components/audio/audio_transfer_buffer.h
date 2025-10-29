#pragma once

#ifdef USE_ESP32
#include "chunked_ring_buffer.h"

#include "esphome/core/defines.h"
#include "ring_buffer.h"

#ifdef USE_SPEAKER
#include "esphome/components/speaker/speaker.h"
#endif

#include "esp_err.h"

#include <freertos/FreeRTOS.h>

namespace esphome {
namespace audio {

class AudioTransferBuffer {
  /*
   * @brief Class that facilitates tranferring data between a buffer and an audio source or sink.
   * The transfer buffer is a typical C array that temporarily holds data for processing in other audio components.
   * Both sink and source transfer buffers can use a ring buffer as the sink/source.
   *   - The ring buffer is stored in a shared_ptr, so destroying the transfer buffer object will release ownership.
   */
 public:
  /// @brief Destructor that deallocates the transfer buffer
  ~AudioTransferBuffer();

  /// @brief Returns a pointer to the start of the transfer buffer where available() bytes of exisiting data can be read
  uint8_t *get_buffer_start() const { return this->data_start_; }

  /// @brief Returns a pointer to the end of the transfer buffer where free() bytes of new data can be written
  uint8_t *get_buffer_end() const { return this->data_start_ + this->buffer_length_; }

  /// @brief Updates the internal state of the transfer buffer. This should be called after reading data
  /// @param bytes The number of bytes consumed/read
  void decrease_buffer_length(size_t bytes);

  /// @brief Updates the internal state of the transfer buffer. This should be called after writing data
  /// @param bytes The number of bytes written
  void increase_buffer_length(size_t bytes);

  /// @brief Returns the transfer buffer's currently available bytes to read
  size_t available() const { return this->buffer_length_; }

  /// @brief Returns the transfer buffers allocated bytes
  size_t capacity() const { return this->buffer_size_; }

  /// @brief Returns the transfer buffer's currrently free bytes available to write
  size_t free() const;

  /// @brief Clears data in the transfer buffer and, if possible, the source/sink.
  virtual void clear_buffered_data();

  /// @brief Tests if there is any data in the transfer buffer or the source/sink.
  /// @return True if there is data, false otherwise.
  virtual bool has_buffered_data() const;

  bool reallocate(size_t new_buffer_size);

 protected:
  /// @brief Allocates the transfer buffer in external memory, if available.
  /// @param buffer_size The number of bytes to allocate
  /// @return True is successful, false otherwise.
  bool allocate_buffer_(size_t buffer_size);

  /// @brief Deallocates the buffer and resets the class variables.
  void deallocate_buffer_();

  // A possible source or sink for the transfer buffer
  std::shared_ptr<RingBuffer> ring_buffer_;

  uint8_t *buffer_{nullptr};
  uint8_t *data_start_{nullptr};

  size_t buffer_size_{0};
  size_t buffer_length_{0};
};

class AudioSinkTransferBuffer : public AudioTransferBuffer {
  /*
   * @brief A class that implements a transfer buffer for audio sinks.
   * Supports writing processed data in the transfer buffer to a ring buffer or a speaker component.
   */
 public:
  /// @brief Creates a new sink transfer buffer.
  /// @param buffer_size Size of the transfer buffer in bytes.
  /// @return unique_ptr if successfully allocated, nullptr otherwise
  static std::unique_ptr<AudioSinkTransferBuffer> create(size_t buffer_size);

  /// @brief Writes any available data in the transfer buffer to the sink.
  /// @param ticks_to_wait FreeRTOS ticks to block while waiting for the sink to have enough space
  /// @param post_shift If true, all remaining data is moved to the start of the buffer after transferring to the sink.
  ///                   Defaults to true.
  /// @return Number of bytes written
  size_t transfer_data_to_sink(TickType_t ticks_to_wait, bool post_shift = true, bool write_partial = false);

  /// @brief Adds a ring buffer as the transfer buffer's sink.
  /// @param ring_buffer weak_ptr to the allocated ring buffer
  void set_sink(const std::weak_ptr<RingBuffer> &ring_buffer) { this->ring_buffer_ = ring_buffer.lock(); }

#ifdef USE_SPEAKER
  /// @brief Adds a speaker as the transfer buffer's sink.
  /// @param speaker Pointer to the speaker component
  void set_sink(speaker::Speaker *speaker) { this->speaker_ = speaker; }
#endif

  void clear_buffered_data() override;

  bool has_buffered_data() const override;

 protected:
#ifdef USE_SPEAKER
  speaker::Speaker *speaker_{nullptr};
#endif
};

class AudioSourceTransferBuffer : public AudioTransferBuffer {
  /*
   * @brief A class that implements a transfer buffer for audio sources.
   * Supports reading audio data from a ring buffer into the transfer buffer for processing.
   */
 public:
  /// @brief Creates a new source transfer buffer.
  /// @param buffer_size Size of the transfer buffer in bytes.
  /// @return unique_ptr if successfully allocated, nullptr otherwise
  static std::unique_ptr<AudioSourceTransferBuffer> create(size_t buffer_size);

  /// @brief Reads any available data from the sink into the transfer buffer.
  /// @param ticks_to_wait FreeRTOS ticks to block while waiting for the source to have enough data
  /// @param pre_shift If true, any unwritten data is moved to the start of the buffer before transferring from the
  ///                  source. Defaults to true.
  /// @return Number of bytes read
  size_t transfer_data_from_source(TickType_t ticks_to_wait, bool pre_shift = true);

  /// @brief Adds a ring buffer as the transfer buffer's source.
  /// @param ring_buffer weak_ptr to the allocated ring buffer
  void set_source(const std::weak_ptr<RingBuffer> &ring_buffer) { this->ring_buffer_ = ring_buffer.lock(); };
};


class TimedAudioSourceTransferBuffer : public AudioSourceTransferBuffer {
  /*
   * @brief A class that implements a transfer buffer for audio sources.
   * Supports reading audio data from a ring buffer into the transfer buffer for processing.
   */
 public:
  TimedAudioSourceTransferBuffer() = default;
  
  /// @brief Creates a new source transfer buffer.
  /// @param buffer_size Size of the transfer buffer in bytes.
  /// @return unique_ptr if successfully allocated, nullptr otherwise
  static std::unique_ptr<TimedAudioSourceTransferBuffer> create(size_t buffer_size);

  /// @brief Reads any available data from the sink into the transfer buffer.
  /// @param ticks_to_wait FreeRTOS ticks to block while waiting for the source to have enough data
  /// @param pre_shift If true, any unwritten data is moved to the start of the buffer before transferring from the
  ///                  source. Defaults to true.
  /// @return Number of bytes read
  size_t transfer_data_from_source(TickType_t ticks_to_wait, bool pre_shift = true);

  /// @brief Adds a ring buffer as the transfer buffer's source.
  /// @param ring_buffer weak_ptr to the allocated ring buffer
  void set_source(const std::weak_ptr<TimedRingBuffer> &ring_buffer) { this->ring_buffer_ = ring_buffer.lock(); };

  bool has_buffered_data() const override;
  
  tv_t get_current_time_stamp() const { return this->current_time_stamp_; }
  void set_current_time_stamp(tv_t time_stamp) { this->current_time_stamp_ = time_stamp; }

  protected:
  std::shared_ptr<TimedRingBuffer> ring_buffer_;
  tv_t current_time_stamp_{0, 0};  // Current timestamp in seconds and microseconds
};

class TimedAudioSinkTransferBuffer : public AudioSinkTransferBuffer {
  /*
   * @brief A class that implements a transfer buffer for audio sinks.
   * Supports writing processed data in the transfer buffer to a ring buffer or a speaker component.
   */
 public:
  TimedAudioSinkTransferBuffer() = default;

  /// @brief Creates a new sink transfer buffer.
  /// @param buffer_size Size of the transfer buffer in bytes.
  /// @return unique_ptr if successfully allocated, nullptr otherwise
  static std::unique_ptr<TimedAudioSinkTransferBuffer> create(size_t buffer_size);

  /// @brief Writes any available data in the transfer buffer to the sink.
  /// @param ticks_to_wait FreeRTOS ticks to block while waiting for the sink to have enough space
  /// @param post_shift If true, all remaining data is moved to the start of the buffer after transferring to the sink.
  ///                   Defaults to true.
  /// @return Number of bytes written
  esp_err_t transfer_data_to_sink(TickType_t ticks_to_wait, uint32_t &skip_next_frames, bool post_shift = true);




  bool has_buffered_data() const override;
  tv_t get_current_time_stamp() const { return this->current_time_stamp_; }
    void set_current_time_stamp(tv_t time_stamp) { 
    this->current_time_stamp_ = time_stamp; 
  }

  
protected:
  tv_t current_time_stamp_{0, 0};
  uint32_t last_adjustment_at_{0};
};



}  // namespace audio
}  // namespace esphome

#endif
