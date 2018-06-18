//  Copyright [2018] <Alexander Hurd>"

#include "SoapySidekiq.hpp"
#include <SoapySDR/Formats.hpp>
#include <cstring>  // memcpy

std::vector<std::string> SoapySidekiq::getStreamFormats(const int direction, const size_t channel) const {
  std::vector<std::string> formats;
  formats.push_back(SOAPY_SDR_CS16);
  formats.push_back(SOAPY_SDR_CF32);
  return formats;
}

std::string SoapySidekiq::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {
  fullScale = 32767;
  return "CS16";
}

SoapySDR::ArgInfoList SoapySidekiq::getStreamArgsInfo(const int direction, const size_t channel) const {
  SoapySDR::ArgInfoList streamArgs;

  SoapySDR::ArgInfo bufflenArg;
  bufflenArg.key = "bufflen";
  bufflenArg.value = std::to_string(DEFAULT_BUFFER_LENGTH);
  bufflenArg.name = "Buffer Size";
  bufflenArg.description = "Number of IQ samples per buffer.";
  bufflenArg.units = "samples";
  bufflenArg.type = SoapySDR::ArgInfo::INT;

  streamArgs.push_back(bufflenArg);

  SoapySDR::ArgInfo buffersArg;
  buffersArg.key = "buffers";
  buffersArg.value = std::to_string(DEFAULT_NUM_BUFFERS);
  buffersArg.name = "Ring buffers";
  buffersArg.description = "Number of buffers in the ring.";
  buffersArg.units = "buffers";
  buffersArg.type = SoapySDR::ArgInfo::INT;

  streamArgs.push_back(buffersArg);

  return streamArgs;
}

/*******************************************************************
 * Sidekiq receive thread
 ******************************************************************/

void SoapySidekiq::rx_receive_operation(void) {
  SoapySDR_log(SOAPY_SDR_INFO, "Starting RX Sidekiq Thread");

  /* set rx source as iq data */
  if (skiq_write_rx_data_src(card, rx_hdl, skiq_data_src_iq) < 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_write_rx_data_src (card %d)", card);
  }

  /* set a modest rx timeout */
  if (skiq_set_rx_transfer_timeout(card, 100000) < 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_set_rx_transfer_timeout (card %d)", card);
  }

  /* start rx streaming */
  if (skiq_start_rx_streaming(card, rx_hdl) < 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_start_rx_streaming (card %d)", card);
  }

  //  skiq receive params
  skiq_rx_block_t *p_rx_block;
  uint32_t len;

  //  loop until stream is deactivated
  while (rx_running) {
    //  check for overflow
    if (_buf_count == numBuffers) {
      SoapySDR_log(SOAPY_SDR_WARNING, "Detected overflow Event in RX Sidekiq Thread");
      _overflowEvent = true;
    }

    /*  blocking skiq_receive */
    if (skiq_receive(card, &rx_hdl, &p_rx_block, &len) == skiq_rx_status_success) {
      //  number of i and q samples
      uint32_t num_samples = (len - SKIQ_RX_HEADER_SIZE_IN_BYTES) / sizeof(int16_t);

      //  buffer space required
      uint32_t space_req = num_samples * elementsPerSample * shortsPerWord;

      //  buf mutex
      {
        std::lock_guard<std::mutex> lock(_buf_mutex);
        // check if we need to move on to next buffer in ring
        if ((_buffs[_buf_tail].size() + space_req) >= (bufferLength)) {
          SoapySDR_logf(SOAPY_SDR_TRACE, "Rotating Buffer Ring %d", _buf_count.load());

          //  increment the tail pointer and buffer count
          _buf_tail = (_buf_tail + 1) % numBuffers;
          _buf_count++;

          // notify readStream()
          _buf_cond.notify_one();
        }

        //  get current fill buffer
        auto &buff = _buffs[_buf_tail];
        buff.resize(buff.size() + space_req);

        //  copy into the buffer queue
        unsigned int i = 0;

        if (useShort) { //  int16_t
          int16_t *dptr = buff.data();
          dptr += (buff.size() - space_req);
          for (i = 0; i < (num_samples / elementsPerSample); i++) {
            if (!iq_swap) {
              *dptr++ = p_rx_block->data[i * 2 + 1]; // I
              *dptr++ = p_rx_block->data[i * 2]; // Q
            } else {
              *dptr++ = p_rx_block->data[i * 2]; // Q
              *dptr++ = p_rx_block->data[i * 2 + 1]; // I
            }
          }
        } else { //  float
          float *dptr = (float *) buff.data();
          dptr += ((buff.size() - space_req) / shortsPerWord);
          for (i = 0; i < (num_samples / elementsPerSample); i++) {
            if (!iq_swap) {
              *dptr++ = (float) p_rx_block->data[i * 2 + 1] / 32768.0f; // I
              *dptr++ = (float) p_rx_block->data[i * 2] / 32768.0f; // Q
            } else {
              *dptr++ = (float) p_rx_block->data[i * 2] / 32768.0f; // Q
              *dptr++ = (float) p_rx_block->data[i * 2 + 1] / 32768.0f; // I
            }
          }
        }
      }
    } else {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_receive (card %d)", card);
    }
  }
  SoapySDR_log(SOAPY_SDR_INFO, "Exiting RX Sidekiq Thread");
}

/*******************************************************************
 * Stream API
 ******************************************************************/

SoapySDR::Stream *SoapySidekiq::setupStream(const int direction,
                                            const std::string &format,
                                            const std::vector<size_t> &channels,
                                            const SoapySDR::Kwargs &args) {
  //  check the channel configuration
  if (channels.size() > 1 || (channels.size() > 0 && channels.at(0) != 0)) {
    throw std::runtime_error("setupStream invalid channel selection");
  }

  //  check the format
  if (format == "CS16") {
    useShort = true;
    shortsPerWord = 1;
    bufferLength = bufferElems * elementsPerSample * shortsPerWord;
    SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16.");
  } else if (format == "CF32") {
    useShort = false;
    shortsPerWord = sizeof(float) / sizeof(short);
    bufferLength =
        bufferElems * elementsPerSample * shortsPerWord;  // allocate enough space for floats instead of shorts
    SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32.");
  } else {
    throw std::runtime_error(
        "setupStream invalid format '" + format
            + "' -- Only CS16 or CF32 is supported by SoapySidekiq module.");
  }

  bufferLength = DEFAULT_BUFFER_LENGTH;
  if (args.count("bufflen") != 0) {
    try {
      int bufferLength_in = std::stoi(args.at("bufflen"));
      if (bufferLength_in > 0) {
        bufferLength = bufferLength_in;
      }
    }
    catch (const std::invalid_argument &) {}
  }
  SoapySDR_logf(SOAPY_SDR_DEBUG, "Sidekiq Using buffer length %d", bufferLength);

  numBuffers = DEFAULT_NUM_BUFFERS;
  if (args.count("buffers") != 0) {
    try {
      int numBuffers_in = std::stoi(args.at("buffers"));
      if (numBuffers_in > 0) {
        numBuffers = numBuffers_in;
      }
    }
    catch (const std::invalid_argument &) {}
  }
  SoapySDR_logf(SOAPY_SDR_DEBUG, "Sidekiq Using %d buffers", numBuffers);

  //  buff mutext
  {
    std::lock_guard<std::mutex> lock(_buf_mutex);

    //  clear async fifo counts
    _buf_tail = 0;
    _buf_count = 0;
    _buf_head = 0;

    //  allocate buffers
    _buffs.resize(numBuffers);
    for (auto &buff : _buffs) buff.reserve(bufferLength);
    for (auto &buff : _buffs) buff.clear();
  }
  return (SoapySDR::Stream *) this;
}

void SoapySidekiq::closeStream(SoapySDR::Stream *stream) {
  this->deactivateStream(stream, 0, 0);
  _buffs.clear();
}

size_t SoapySidekiq::getStreamMTU(SoapySDR::Stream *stream) const {
  return bufferLength;
}

int SoapySidekiq::activateStream(SoapySDR::Stream *stream,
                                 const int flags,
                                 const long long timeNs,
                                 const size_t numElems) {
  if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
  resetBuffer = true;
  bufferedElems = 0;

  //  start the receive thread
  if (!_rx_receive_thread.joinable()) {
    _rx_receive_thread = std::thread(&SoapySidekiq::rx_receive_operation, this);
  }

  return 0;
}

int SoapySidekiq::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs) {
  if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;

  // stop receive thread
  if (_rx_receive_thread.joinable()) {
    rx_running = false;
    _rx_receive_thread.join();
  }

  /* stop rx streaming */
  if (skiq_stop_rx_streaming(card, rx_hdl) < 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_stop_rx_streaming (card %d)", card);
  }

  return 0;
}

int SoapySidekiq::readStream(SoapySDR::Stream *stream,
                             void *const *buffs,
                             const size_t numElems,
                             int &flags,
                             long long &timeNs,
                             const long timeoutUs) {
  //  drop remainder buffer on reset
  if (resetBuffer && bufferedElems != 0) {
    bufferedElems = 0;
    this->releaseReadBuffer(stream, _currentHandle);
  }

  //  this is the user's buffer for channel 0
  void *buff0 = buffs[0];

  //  are elements left in the buffer? if not, do a new read.
  if (bufferedElems == 0) {
    int ret = this->acquireReadBuffer(stream, _currentHandle, (const void **) &_currentBuff, flags, timeNs,
                                      timeoutUs);
    if (ret < 0) return ret;
    bufferedElems = ret;
  }

  size_t returnedElems = std::min(bufferedElems, numElems);

  // copy into user's buff0
  if (useShort) {
    std::memcpy(buff0, _currentBuff, returnedElems * elementsPerSample * sizeof(int16_t));
  } else {
    std::memcpy(buff0, (float *) _currentBuff, returnedElems * 2 * sizeof(float));
  }

  //  bump variables for next call into readStream
  bufferedElems -= returnedElems;

  // scope lock here to update _currentBuff position
  {
    std::lock_guard<std::mutex> lock(_buf_mutex);
    _currentBuff += returnedElems * elementsPerSample * shortsPerWord;
  }

  //  return number of elements written to buff0
  if (bufferedElems != 0)
    flags |= SOAPY_SDR_MORE_FRAGMENTS;
  else
    this->releaseReadBuffer(stream, _currentHandle);
  return returnedElems;
}

int SoapySidekiq::writeStream(SoapySDR::Stream *stream,
                              const void *const *buffs,
                              const size_t numElems,
                              int &flags,
                              const long long timeNs,
                              const long timeoutUs) {
return -1;
}

/*******************************************************************
 * Direct buffer access API
 ******************************************************************/

size_t SoapySidekiq::getNumDirectAccessBuffers(SoapySDR::Stream *stream) {
  std::lock_guard<std::mutex> lock(_buf_mutex);
  return _buffs.size();
}

int SoapySidekiq::getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs) {
  std::lock_guard<std::mutex> lock(_buf_mutex);
  buffs[0] = (void *) _buffs[handle].data();
  return 0;
}

int SoapySidekiq::acquireReadBuffer(SoapySDR::Stream *stream,
                                    size_t &handle,
                                    const void **buffs,
                                    int &flags,
                                    long long &timeNs,
                                    const long timeoutUs) {
  std::unique_lock<std::mutex> lock(_buf_mutex);

  // reset is issued by various settings
  // overflow set in the rx thread
  if (resetBuffer || _overflowEvent) {
    SoapySDR_log(SOAPY_SDR_INFO, "Resetting all RX Buffers");
    // drain all buffers from the fifo
    _buf_tail = 0;
    _buf_head = 0;
    _buf_count = 0;
    for (auto &buff : _buffs) buff.clear();
    _overflowEvent = false;
    if (resetBuffer) {
      resetBuffer = false;
    } else {
      SoapySDR_log(SOAPY_SDR_SSI, "O");
      return SOAPY_SDR_OVERFLOW;
    }
  }

  // wait for a buffer to become available
  if (_buf_count == 0) {
    _buf_cond.wait_for(lock, std::chrono::microseconds(timeoutUs));
    if (_buf_count == 0) {
      SoapySDR_logf(SOAPY_SDR_WARNING, "Read Timeout occured after %d ms", timeoutUs);
      return SOAPY_SDR_TIMEOUT;
    }
  }

  // extract handle and buffer
  handle = _buf_head;
  buffs[0] = (void *) _buffs[handle].data();
  flags = 0;

  _buf_head = (_buf_head + 1) % numBuffers;

  // return number available
  return static_cast<int>(_buffs[handle].size() / (elementsPerSample * shortsPerWord));
}

void SoapySidekiq::releaseReadBuffer(SoapySDR::Stream *stream, const size_t handle) {
  SoapySDR_logf(SOAPY_SDR_TRACE, "Release Read Buffer %d", handle);
  std::lock_guard<std::mutex> lock(_buf_mutex);
  _buffs[handle].clear();
  _buf_count--;
}

int SoapySidekiq::acquireWriteBuffer(SoapySDR::Stream *stream,
                                     size_t &handle,
                                     void **buffs,
                                     const long timeoutUs) {

  return -1;

}

void SoapySidekiq::releaseWriteBuffer(SoapySDR::Stream *stream,
                                      const size_t handle,
                                      const size_t numElems,
                                      int &flags,
                                      const long long timeNs) {

}