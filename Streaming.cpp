//  Copyright [2018] <Alexander Hurd>"

#include "SoapySidekiq.hpp"
#include <SoapySDR/Formats.hpp>
#include <cstring>  // memcpy
#include <sidekiq_types.h>
#include <unistd.h>

std::vector<std::string> SoapySidekiq::getStreamFormats(const int direction, const size_t channel) const {
  SoapySDR_logf(SOAPY_SDR_TRACE, "getStreamFormats");
  std::vector<std::string> formats;
  formats.push_back(SOAPY_SDR_CS16);
  formats.push_back(SOAPY_SDR_CF32);
  return formats;
}

std::string SoapySidekiq::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {
  fullScale = 32767;

  SoapySDR_logf(SOAPY_SDR_TRACE, "getNativeStreamFormat");
  return "CS16";
}

SoapySDR::ArgInfoList SoapySidekiq::getStreamArgsInfo(const int direction, const size_t channel) const {
  SoapySDR::ArgInfoList streamArgs;

  SoapySDR::ArgInfo bufflenArg;
  bufflenArg.key = "bufflen";

  SoapySDR_logf(SOAPY_SDR_TRACE, "getStreamArgsInfo");

  if (direction == SOAPY_SDR_RX) {
    bufflenArg.value = std::to_string(rx_block_size_in_words);
  }else {
    bufflenArg.value = std::to_string(DEFAULT_TX_BUFFER_LENGTH);
  }

  bufflenArg.name = "Buffer Sample Count";
  bufflenArg.description = "Number of IQ samples per buffer.";
  bufflenArg.units = "samples";
  bufflenArg.type = SoapySDR::ArgInfo::INT;

  streamArgs.push_back(bufflenArg);
  

  SoapySDR::ArgInfo buffersArg;
  buffersArg.key = "buffers";
  buffersArg.value = std::to_string(DEFAULT_NUM_BUFFERS);
  buffersArg.name = "Ring Buffers";
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
  int status = 0;

  SoapySDR_log(SOAPY_SDR_INFO, "Starting RX Sidekiq Thread");

//  status = skiq_write_rx_data_src(card, rx_hdl, skiq_data_src_counter);
  /* set rx source as iq data */
  status = skiq_write_iq_order_mode(card, skiq_iq_order_iq) ;
  if (status != 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_write_rx_data_src (card %d) status %d", card, status);
    throw std::runtime_error(" skiq_write_rx_data_src error");
  }

  /* set a modest rx timeout */
  status = skiq_set_rx_transfer_timeout(card, 100000) ;
  if (status != 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_set_rx_transfer_timeout (card %d) status %d", card, status);
    throw std::runtime_error(" skiq_set_rx_transfer_timeout ( error");
  }

  /* start rx streaming */
  status = skiq_start_rx_streaming(card, rx_hdl) ;
  if (skiq_start_rx_streaming(card, rx_hdl) != 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_start_rx_streaming (card %d) status %d", card, status);
    throw std::runtime_error(" skiq_start_rx_streaming error");
  }

  //  skiq receive params
  skiq_rx_block_t *tmp_p_rx_block;
  uint32_t len;

  // metadata
  uint64_t overload = 0;
  skiq_rx_hdl_t rcvd_hdl;

  //  loop until stream is deactivated
  while (rx_running) {
    //  check for overflow
    if (rxReadIndex == ((rxWriteIndex + 1) % DEFAULT_NUM_BUFFERS) || overload) {
      SoapySDR_log(SOAPY_SDR_WARNING, "Detected overflow Event in RX Sidekiq Thread");
      SoapySDR_logf(SOAPY_SDR_DEBUG, "rxReadIndex %d, rxWriteIndex %d, overload %d\n", 
                                     rxReadIndex , rxWriteIndex, overload);
      rxWriteIndex = rxReadIndex;   
    }
    else {
      /*  blocking skiq_receive 
       *  put the block into a ring buffer so readStream can read it out at 
       *  a different pace */ 
      status = skiq_receive(card, &rcvd_hdl, &tmp_p_rx_block, &len); 
      if (status == skiq_rx_status_success) {
        if (rcvd_hdl == rx_hdl) {
          if (len != rx_block_size_in_bytes) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "received length %d is not the correct block size %d\n", 
                                            len, rx_block_size_in_bytes);
            throw std::runtime_error("skiq_receive length error");
          }

          // get overload out of metadata
          overload = tmp_p_rx_block->overload;

          //copy the out of the rx_buffer and into the ring buffers
          memcpy(p_rx_block[rxWriteIndex], tmp_p_rx_block, len);

          rxWriteIndex = (rxWriteIndex + 1) % DEFAULT_NUM_BUFFERS;

        }
      }
      else {
        SoapySDR_logf(SOAPY_SDR_FATAL, "Failure: skiq_receive (card %d) status %d", card, status);
        throw std::runtime_error("skiq_receive error");
      }
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

  int status = 0;
  skiq_rx_stream_mode_t stream_mode = skiq_rx_stream_mode_balanced;
  SoapySDR_logf(SOAPY_SDR_TRACE, "setupStream");

  rx_block_size_in_words = 0;

  //  check the channel configuration
  if (channels.size() > 1 || (channels.size() > 0 && channels.at(0) != 0)) {
    throw std::runtime_error("setupStream invalid channel selection");
  }



  if (direction == SOAPY_SDR_RX) {
    status = skiq_write_rx_stream_mode( card, stream_mode );
    if( status != 0 )
    {
      SoapySDR_logf(SOAPY_SDR_ERROR,"Error: unable to set RX stream mode with status %d\n", status);
    }
    status = skiq_read_rx_block_size(card, stream_mode);
    if (status < 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR," failed to read RX block size status %d\n", rx_block_size_in_bytes);
      throw std::runtime_error("skiq_read_rx_block_size");
  }
  rx_block_size_in_bytes = status;
  rx_payload_size_in_bytes = status - SKIQ_RX_HEADER_SIZE_IN_BYTES;
  rx_payload_size_in_words = rx_payload_size_in_bytes / 4;

  // allocate the ring buffers
  for (int i = 0; i < DEFAULT_NUM_BUFFERS; i++) {
    p_rx_block[i] = (skiq_rx_block_t *) malloc(rx_block_size_in_bytes);
    if (p_rx_block[i] == NULL) {
       SoapySDR_log(SOAPY_SDR_ERROR," malloc failed to allocate memory ");
       throw std::runtime_error("malloc failed to allocate memory");
    }


    memset(p_rx_block[i], 0, rx_block_size_in_bytes);
  }
  rxWriteIndex = 0;
  rxReadIndex = 0;

  //  check the format
  if (format == "CS16") {
    rx_block_size_in_words = rx_block_size_in_bytes / 4;
    SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16\n");

  } else if (format == "CF32") {
    rx_block_size_in_words = rx_block_size_in_bytes / 8;
    SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32\n");
  } else {
    throw std::runtime_error(
        "setupStream invalid format '" + format
            + "' -- Only CS16 or CF32 is supported by SoapySidekiq module.");
  }

  return RX_STREAM;

  } else if (direction == SOAPY_SDR_TX) {
    //  check the channel configuration
    if (channels.size() > 1 || (channels.size() > 0 && channels.at(0) != 0)) {
      throw std::runtime_error("setupStream invalid channel selection");
    }

    //  check the format
    if (format == "CS16") {
      SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16\n"); 
    } else if (format == "CF32") {
      SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32\n");
    } else {
      throw std::runtime_error(
          "setupStream invalid format '" + format
              + "' -- Only CS16 or CF32 is supported by SoapySidekiq module.");
    }

    // Allocate buffers
    for (int i = 0; i < DEFAULT_NUM_BUFFERS; i++){
        p_tx_block[i] = skiq_tx_block_allocate(DEFAULT_TX_BUFFER_LENGTH);
    }
    currTXBuffIndex = 0;
    return TX_STREAM;
  } else {
    throw std::runtime_error("Invalid direction");
  }
}

void SoapySidekiq::closeStream(SoapySDR::Stream *stream) {

  SoapySDR_logf(SOAPY_SDR_TRACE, "closeStream");
  this->deactivateStream(stream, 0, 0);

  if (stream == RX_STREAM) {
      for (int i = 0; i < DEFAULT_NUM_BUFFERS; i++) {
          free(p_rx_block[i]);
      }
  }
  else if (stream == TX_STREAM) {
      for (int i = 0; i < DEFAULT_NUM_BUFFERS; i++){
          skiq_tx_block_free(p_tx_block[i]);
      }
  }
}

size_t SoapySidekiq::getStreamMTU(SoapySDR::Stream *stream) const {

  SoapySDR_logf(SOAPY_SDR_TRACE, "getStremMTU");

  if (stream == RX_STREAM) {
    return rx_block_size_in_words;
  }
  else if (stream == TX_STREAM){
    return DEFAULT_TX_BUFFER_LENGTH;
  } else {
    return SOAPY_SDR_NOT_SUPPORTED;
  }   
  
  return 0;
}

int SoapySidekiq::activateStream(SoapySDR::Stream *stream,
                                 const int flags,
                                 const long long timeNs,
                                 const size_t numElems) {

  int status = 0;
  
  SoapySDR_logf(SOAPY_SDR_TRACE, "activateStream");
  if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;

  if (stream == RX_STREAM) {
    p_rx_block_index = 0;

    //  start the receive thread
    if (!_rx_receive_thread.joinable()) {
      SoapySDR_logf(SOAPY_SDR_DEBUG, "Start RX");
      rx_running = true;
      _rx_receive_thread = std::thread(&SoapySidekiq::rx_receive_operation, this);
    }

  } else if (stream == TX_STREAM) {
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Start TX");
    tx_underruns = 0;

    //  tx block size
    status = skiq_write_tx_block_size(card, tx_hdl, DEFAULT_TX_BUFFER_LENGTH); 
    if (status != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_write_tx_block_size (card %d) status %d", 
                                      card, status);
      throw std::runtime_error("skiq_write_tx_block_size error");
    }

    //  tx data flow mode
    status = skiq_write_tx_data_flow_mode(card, tx_hdl, skiq_tx_immediate_data_flow_mode); 
    if (status != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_write_tx_data_flow_mode (card %d) status %d", 
                                      card, status);
      throw std::runtime_error("skiq_write_tx_data_flow_mode error");
    }

    // transfer mode (sync for now)
    status = skiq_write_tx_transfer_mode(card, tx_hdl, skiq_tx_transfer_mode_sync); 
    if (status != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_tx_transfer_mode (card %d) status %d", 
                                      card, status);
      throw std::runtime_error("skiq_tx_transfer_mode error");
    }
    
    /* start tx streaming */
    status = skiq_start_tx_streaming(card, tx_hdl) ;
    if (status != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_start_tx_streaming (card %d) status %d", 
                                      card, status);
      throw std::runtime_error("skiq_start_tx_streaming error");
    }
  }

  return 0;
}

int SoapySidekiq::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs) {
  int status = 0;
  SoapySDR_logf(SOAPY_SDR_TRACE, "deactivateStream");

  if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;

  if (stream == RX_STREAM && rx_running == true) {
    // stop receive thread
    rx_running = false;

    /* stop rx streaming */
    status = skiq_stop_rx_streaming(card, rx_hdl); 
    if (status != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_stop_rx_streaming (card %d) handle %d, status %d", card, rx_hdl, status);
    }

    /* wait till the rx thread is done */
    if (_rx_receive_thread.joinable()) {
      _rx_receive_thread.join();
    }
  } else if (stream == TX_STREAM) {
    /* stop tx streaming */
    status = skiq_stop_tx_streaming(card, tx_hdl) ;
    if (status != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_stop_tx_streaming (card %d)", card);
    }
  }

  return 0;
}

int SoapySidekiq::readStream(SoapySDR::Stream *stream,
                             void *const *buffs,
                             const size_t numElems,
                             int &flags,
                             long long &timeNs,
                             const long timeoutUs) {
  long waitTime = timeoutUs;
  SoapySDR_logf(SOAPY_SDR_TRACE, "readStream");

  if (stream != RX_STREAM) {
    return SOAPY_SDR_NOT_SUPPORTED;
  }

  // if the user didn't give a waittime then wait a LONG time
  if (waitTime == 0) {
    waitTime = SLEEP_1SEC;
  }

  // see if we have any receive buffers to give
  while ((rxReadIndex == rxWriteIndex) && (waitTime > 0)) {
    // wait
    usleep(DEFAULT_SLEEP_US);
    waitTime -= DEFAULT_SLEEP_US;
    p_rx_block_index = 0;
  }

  if (waitTime <= 0) {
    SoapySDR_log(SOAPY_SDR_DEBUG, "readStream timed out");
    return SOAPY_SDR_TIMEOUT;
  }

  // The numElems may be smaller or bigger than our receive buffer length
  // if smaller we will need to send multiple responses for one receive buffer
  // if bigger we will have to wait for multiple receive buffers to be received before we return
  size_t numElemsLeft = numElems;
  char *buff_ptr = (char *)buffs[0];

  // we need to handle the case where we left in the middle of a ring buffer
  // the p_rx_block_index is != 0 if that is the case
  uint32_t words_left_in_block = rx_payload_size_in_words - (p_rx_block_index / 4) ;
  char *ringbuffer_ptr = (char *)p_rx_block[rxReadIndex]->data + p_rx_block_index;

  // determine if we have more words in the ring buffer block than we need (or equal) 
  while (numElemsLeft >= words_left_in_block) {
//#define debug
#ifdef debug
    SoapySDR_logf(SOAPY_SDR_DEBUG, "1 p_rx_block_index %d, numElemsLeft %d, words_left_in_block %d", 
                                    p_rx_block_index, numElemsLeft, words_left_in_block);
    SoapySDR_logf(SOAPY_SDR_DEBUG, "rxReadIndex %d, rxWriteIndex %d", rxReadIndex, rxWriteIndex);
    SoapySDR_logf(SOAPY_SDR_DEBUG, "buff_ptr %p, ringbuffer_ptr %p", buff_ptr, ringbuffer_ptr);

#endif

    uint32_t bytes_left_in_block = words_left_in_block * 4;

    // copy in the amount of data we have in the ring block
    memcpy(buff_ptr, ringbuffer_ptr, bytes_left_in_block); 

#ifdef debug
    int16_t *temp_ptr = (int16_t *)buff_ptr;
    for (int i=0; i < 10; i++) {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "%d, ", *temp_ptr);
        temp_ptr++;
    }
#endif

    buff_ptr = buff_ptr + bytes_left_in_block;
    numElemsLeft -= words_left_in_block;

    // move to the next buffer in the ring
    rxReadIndex = (rxReadIndex + 1) % DEFAULT_NUM_BUFFERS;
    p_rx_block_index = 0;

    if (numElemsLeft == 0) {
      break;
    }
   
    // if no more data in ring buffer wait 
    while ((rxReadIndex == rxWriteIndex) && (waitTime > 0)) {
      usleep(DEFAULT_SLEEP_US);
      waitTime -= DEFAULT_SLEEP_US;
    }
    if (waitTime <= 0) {
      SoapySDR_log(SOAPY_SDR_DEBUG, "readStream timed out");
      return SOAPY_SDR_TIMEOUT;
    }
    words_left_in_block = rx_payload_size_in_words;
    ringbuffer_ptr = (char *)p_rx_block[rxReadIndex]->data + p_rx_block_index;
  } 

  // if we are here then there is at least one ring buffer available 
  // and we can send it without waiting 
  if (numElemsLeft < words_left_in_block && numElemsLeft != 0) {
#ifdef debug
    SoapySDR_logf(SOAPY_SDR_DEBUG, "2 numElemsLeft %d, words_left_in_block %d", numElemsLeft, words_left_in_block);
    SoapySDR_logf(SOAPY_SDR_DEBUG, "rxReadIndex %d, rxWriteIndex %d", rxReadIndex, rxWriteIndex);
    SoapySDR_logf(SOAPY_SDR_DEBUG, "buff_ptr %p, ringbuffer_ptr %p", buff_ptr, ringbuffer_ptr);
#endif

    uint32_t numElemsLeft_in_bytes = numElemsLeft * 4;

    // copy in the numElems amount of words
    memcpy(buff_ptr, ringbuffer_ptr, numElemsLeft_in_bytes); 
    
#ifdef debug
    int16_t *temp_ptr = (int16_t *)ringbuffer_ptr;
    for (int i=0; i < 10; i++) {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "%d, ", *temp_ptr);
        temp_ptr++;
    }
#endif
    // keep this around for the next call  
    p_rx_block_index += numElemsLeft_in_bytes ;
  }

  // if we are here then we have put NumElems into the buffer
  return numElems;
}

int SoapySidekiq::writeStream(SoapySDR::Stream *stream,
                              const void *const *buffs,
                              const size_t numElems,
                              int &flags,
                              const long long timeNs,
                              const long timeoutUs) {

  SoapySDR_logf(SOAPY_SDR_TRACE, "writeStream");
  if (stream != TX_STREAM) {
    return SOAPY_SDR_NOT_SUPPORTED;
  }

  const uint32_t block_size_in_words = DEFAULT_TX_BUFFER_LENGTH;


  size_t data_bytes = numElems * 2 * sizeof(int16_t);
  uint32_t num_blocks = (data_bytes / (block_size_in_words * 4));
  if ((data_bytes % (block_size_in_words * 4)) != 0) {
    num_blocks++;
  }

  uint32_t i;
  uint32_t errors=0;
  char     *data_ptr;

  for (i = 0; i < num_blocks; i++) {
    data_ptr = (char *)(buffs[0]) + (i * block_size_in_words * 4);

//    printf("passed in pointer %p, calculated pointer %p\n", buffs[0], data_ptr);
    memcpy(p_tx_block[currTXBuffIndex]->data, data_ptr, (block_size_in_words * 4));

    if (skiq_transmit(card, tx_hdl, p_tx_block[currTXBuffIndex], NULL) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_transmit (card %d)", card);
    }
    currTXBuffIndex = (currTXBuffIndex + 1) % DEFAULT_NUM_BUFFERS;
  }

  skiq_read_tx_num_underruns(card, tx_hdl, &errors);

  if (errors != tx_underruns){
      printf("underruns %d\n", errors);
      tx_underruns = errors;
  }

  return numElems;
}

/*******************************************************************
 * Direct buffer access API
 ******************************************************************/

size_t SoapySidekiq::getNumDirectAccessBuffers(SoapySDR::Stream *stream) {
  SoapySDR_logf(SOAPY_SDR_TRACE, "getNumDirectAccessBuffers");
  return 0;
}

int SoapySidekiq::getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs) {
  SoapySDR_logf(SOAPY_SDR_TRACE, "getDirectAccessBufferAddrs");
  return 0;
}

int SoapySidekiq::acquireReadBuffer(SoapySDR::Stream *stream,
                                    size_t &handle,
                                    const void **buffs,
                                    int &flags,
                                    long long &timeNs,
                                    const long timeoutUs) {
    SoapySDR_logf(SOAPY_SDR_TRACE, "acquireReadBuffer");
    return SOAPY_SDR_NOT_SUPPORTED;

}

void SoapySidekiq::releaseReadBuffer(SoapySDR::Stream *stream, const size_t handle) {
  if (stream != RX_STREAM) {
    throw std::runtime_error("Invalid stream");
  }
  SoapySDR_logf(SOAPY_SDR_TRACE, "Release Read Buffer %d", handle);
}

int SoapySidekiq::acquireWriteBuffer(SoapySDR::Stream *stream,
                                     size_t &handle,
                                     void **buffs,
                                     const long timeoutUs) {
  SoapySDR_logf(SOAPY_SDR_TRACE, "acquireWriteBuffer");
  return SOAPY_SDR_NOT_SUPPORTED;
}

void SoapySidekiq::releaseWriteBuffer(SoapySDR::Stream *stream,
                                      const size_t handle,
                                      const size_t numElems,
                                      int &flags,
                                      const long long timeNs) {
  SoapySDR_logf(SOAPY_SDR_TRACE, "releaseWriteBuffer");
}
