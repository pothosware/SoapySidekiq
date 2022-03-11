//  Copyright [2018] <Alexander Hurd>"

#pragma once

#include <sidekiq_api.h>

#include <stdexcept>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <string>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Types.hpp>

#define DEFAULT_NUM_BUFFERS       (30)
#define DEFAULT_ELEMS_PER_SAMPLE  (2)
#define DEFAULT_TX_BUFFER_LENGTH  (8188)
#define DEFAULT_SLEEP_US          (100)
#define SLEEP_1SEC                (1 * 1000000)

class SoapySidekiq : public SoapySDR::Device {
 public:
  SoapySidekiq(const SoapySDR::Kwargs &args);

  ~SoapySidekiq(void);

  /*******************************************************************
   * Identification API
   ******************************************************************/

  std::string getDriverKey(void) const;

  std::string getHardwareKey(void) const;

  SoapySDR::Kwargs getHardwareInfo(void) const;

  /*******************************************************************
   * Channels API
   ******************************************************************/

  size_t getNumChannels(const int) const;

  /*******************************************************************
   * Stream API
   ******************************************************************/

  std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;

  std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const;

  SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const;

  SoapySDR::Stream *setupStream(const int direction, const std::string &format, const std::vector<size_t> &channels =
  std::vector<size_t>(), const SoapySDR::Kwargs &args = SoapySDR::Kwargs());

  void closeStream(SoapySDR::Stream *stream);

  size_t getStreamMTU(SoapySDR::Stream *stream) const;

  int activateStream(SoapySDR::Stream *stream,
                     const int flags = 0,
                     const long long timeNs = 0,
                     const size_t numElems = 0);

  int deactivateStream(SoapySDR::Stream *stream, const int flags = 0, const long long timeNs = 0);

  int readStream(SoapySDR::Stream *stream,
                 void *const *buffs,
                 const size_t numElems,
                 int &flags,
                 long long &timeNs,
                 const long timeoutUs = 100000);

  int writeStream(SoapySDR::Stream *stream,
                  const void *const *buffs,
                  const size_t numElems,
                  int &flags,
                  const long long timeNs = 0,
                  const long timeoutUs = 100000);

  /*******************************************************************
   * Direct buffer access API
   ******************************************************************/

  size_t getNumDirectAccessBuffers(SoapySDR::Stream *stream);

  int getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs);

  int acquireReadBuffer(SoapySDR::Stream *stream,
                        size_t &handle,
                        const void **buffs,
                        int &flags,
                        long long &timeNs,
                        const long timeoutUs = 100000);

  void releaseReadBuffer(SoapySDR::Stream *stream, const size_t handle);

  int acquireWriteBuffer(SoapySDR::Stream *stream,
                         size_t &handle,
                         void **buffs,
                         const long timeoutUs = 100000);

  void releaseWriteBuffer(SoapySDR::Stream *stream,
                          const size_t handle,
                          const size_t numElems,
                          int &flags,
                          const long long timeNs = 0);

  /*******************************************************************
   * Antenna API
   ******************************************************************/

  std::vector<std::string> listAntennas(const int direction, const size_t channel) const;

  void setAntenna(const int direction, const size_t channel, const std::string &name);

  std::string getAntenna(const int direction, const size_t channel) const;

  /*******************************************************************
   * Frontend corrections API
   ******************************************************************/

  bool hasDCOffsetMode(const int direction, const size_t channel) const;

  void setDCOffsetMode(const int direction, const size_t channel, const bool automatic);

  bool getDCOffsetMode(const int direction, const size_t channel) const;

  /*******************************************************************
   * Gain API
   ******************************************************************/

  std::vector<std::string> listGains(const int direction, const size_t channel) const;

  bool hasGainMode(const int direction, const size_t channel) const;

  void setGainMode(const int direction, const size_t channel, const bool automatic);

  bool getGainMode(const int direction, const size_t channel) const;

  void setGain(const int direction, const size_t channel, const double value);

  double getGain(const int direction, const size_t channel) const;

  SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string &name) const;

  /*******************************************************************
   * Frequency API
   ******************************************************************/

  void setFrequency(const int direction,
                    const size_t channel,
                    const std::string &name,
                    const double frequency,
                    const SoapySDR::Kwargs &args = SoapySDR::Kwargs());

  double getFrequency(const int direction, const size_t channel, const std::string &name) const;

  std::vector<std::string> listFrequencies(const int direction, const size_t channel) const;

  SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel, const std::string &name) const;

  SoapySDR::ArgInfoList getFrequencyArgsInfo(const int direction, const size_t channel) const;

  /*******************************************************************
   * Sample Rate API
   ******************************************************************/

  void setSampleRate(const int direction, const size_t channel, const double rate);

  double getSampleRate(const int direction, const size_t channel) const;

  std::vector<double> listSampleRates(const int direction, const size_t channel) const;

  void setBandwidth(const int direction, const size_t channel, const double bw);

  double getBandwidth(const int direction, const size_t channel) const;

  std::vector<double> listBandwidths(const int direction, const size_t channel) const;

  /*******************************************************************
   * Attenuation API
   ******************************************************************/

  //  TODO

  /*******************************************************************
   * Sensor API
   ******************************************************************/
  std::vector<std::string> listSensors(void) const;
  SoapySDR::ArgInfo getSensorInfo(const std::string &key) const;
  std::string readSensor(const std::string &key) const;
  std::vector<std::string> listSensors(const int direction, const size_t channel) const;
  std::string readSensor(const int direction, const size_t channel, const std::string &key) const;

  /*******************************************************************
   * Settings API
   ******************************************************************/

  SoapySDR::ArgInfoList getSettingInfo(void) const;

  void writeSetting(const std::string &key, const std::string &value);

  std::string readSetting(const std::string &key) const;

 private:

  SoapySDR::Stream* const TX_STREAM = (SoapySDR::Stream*) 0x1;
  SoapySDR::Stream* const RX_STREAM = (SoapySDR::Stream*) 0x2;

  //  sidekiq card
  uint8_t card;

  //  sidekiq hdl
  skiq_rx_hdl_t rx_hdl;
  skiq_tx_hdl_t tx_hdl;

  //  rx
  uint64_t rx_center_frequency;
  uint32_t rx_sample_rate, rx_bandwidth;
  uint32_t rx_block_size_in_words;
  uint32_t rx_block_size_in_bytes;

  //  tx
  uint64_t tx_center_frequency;
  uint32_t tx_sample_rate, tx_bandwidth;
  uint32_t tx_underruns;

  //  setting
  bool iq_swap;

  // RX buffer
  skiq_rx_block_t *p_rx_block[DEFAULT_NUM_BUFFERS];
  uint32_t rxReadIndex;
  uint32_t rxWriteIndex;
  uint32_t p_rx_block_index;

  // TX buffer
  skiq_tx_block_t *p_tx_block[DEFAULT_NUM_BUFFERS];
  uint32_t currTXBuffIndex;

 public:
  //  receive thread
  std::thread _rx_receive_thread;
  void rx_receive_operation(void);

  static std::vector<SoapySDR::Kwargs> sidekiq_devices;
  static bool rx_running;
};
