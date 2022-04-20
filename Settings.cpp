//  Copyright [2018] <Alexander Hurd>"

#include "SoapySidekiq.hpp"

std::vector<SoapySDR::Kwargs> SoapySidekiq::sidekiq_devices;
bool SoapySidekiq::rx_running;

SoapySidekiq::SoapySidekiq(const SoapySDR::Kwargs &args) {
  //  rx defaults
  rx_sample_rate = 2048000;
  rx_bandwidth = 2048000;
  rx_center_frequency = 100000000;

  //  tx defaults
  tx_sample_rate = 2048000;
  tx_bandwidth = 2048000;
  tx_center_frequency = 100000000;

  iq_swap = false;

  //  this may change later according to format
  rx_running = false;

  if (args.count("card") != 0) {
    try {
      card = std::stoi(args.at("card"));
    }
    catch (const std::invalid_argument &) {
    }

    SoapySDR_logf(SOAPY_SDR_DEBUG, "Found Sidekiq Device using device index parameter 'card' = %d", card);
  }

  //  Handle (TODO add to args)
  rx_hdl = skiq_rx_hdl_A1;
  tx_hdl = skiq_tx_hdl_A1;

  skiq_xport_type_t type = skiq_xport_type_auto;
  skiq_xport_init_level_t level = skiq_xport_init_level_full;

  SoapySDR_logf(SOAPY_SDR_DEBUG, "Sidekiq opening card %d", card);

  /* init sidekiq */
  if (skiq_init(type, level, &card, 1) != 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_init (card %d)", card);
  }
  /* set iq order to iq instead of qi */
  if (skiq_write_iq_order_mode(card, skiq_iq_order_iq) != 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: setting iq order (card %d)", card);
  }
  rx_block_size_in_words = 0;
}

SoapySidekiq::~SoapySidekiq(void) {
  if (skiq_exit() != 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_exit", card);
  }
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapySidekiq::getDriverKey(void) const {
  return "Sidekiq";
}

std::string SoapySidekiq::getHardwareKey(void) const {
  return "Sidekiq";
}

SoapySDR::Kwargs SoapySidekiq::getHardwareInfo(void) const {
  //  key/value pairs for any useful information
  //  this also gets printed in --probe
  SoapySDR::Kwargs args;

  args["origin"] = "https://github.com/pothosware/SoapySidekiq";
  args["card"] = std::to_string(card);

  return args;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapySidekiq::getNumChannels(const int dir) const {
  return 1;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapySidekiq::listAntennas(const int direction, const size_t channel) const {
  std::vector<std::string> antennas;
  antennas.push_back("RX");
  antennas.push_back("TX");
  return antennas;
}

void SoapySidekiq::setAntenna(const int direction, const size_t channel, const std::string &name) {
  SoapySDR::Device::setAntenna(direction, channel, name);
}

std::string SoapySidekiq::getAntenna(const int direction, const size_t channel) const {
  return SoapySDR::Device::getAntenna(direction, channel);
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapySidekiq::hasDCOffsetMode(const int direction, const size_t channel) const {
  return direction == SOAPY_SDR_RX;
}

void SoapySidekiq::setDCOffsetMode(const int direction, const size_t channel, const bool automatic) {
  if (direction == SOAPY_SDR_RX) {
    if (skiq_write_rx_dc_offset_corr(card, rx_hdl, automatic) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_write_rx_dc_offset_corr (card %d, enable %d)", card, automatic);
    }
  }
}

bool SoapySidekiq::getDCOffsetMode(const int direction, const size_t channel) const {
  bool enable;
  if (direction == SOAPY_SDR_RX) {
    if (skiq_read_rx_dc_offset_corr(card, rx_hdl, &enable) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_read_rx_dc_offset_corr (card %d)", card);
    }
    return enable;
  }

  return SoapySDR::Device::getDCOffsetMode(direction, channel);
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapySidekiq::listGains(const int direction, const size_t channel) const {
  //  list available gain elements,
  std::vector<std::string> results;
  return results;
}

bool SoapySidekiq::hasGainMode(const int direction, const size_t channel) const {
  return true;
}

void SoapySidekiq::setGainMode(const int direction, const size_t channel, const bool automatic) {
  if (direction == SOAPY_SDR_RX) {
    SoapySDR_logf(SOAPY_SDR_DEBUG,
                  "Setting Sidekiq RX Gain Mode: %s",
                  automatic ? "skiq_rx_gain_auto" : "skiq_rx_gain_manual");
    skiq_rx_gain_t mode = automatic ? skiq_rx_gain_auto : skiq_rx_gain_manual;
    if (skiq_write_rx_gain_mode(card, rx_hdl, mode) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_write_rx_gain_mode (card %d, mode %d)", card, mode);
    }
  }
}

bool SoapySidekiq::getGainMode(const int direction, const size_t channel) const {
  if (direction == SOAPY_SDR_RX) {
    skiq_rx_gain_t p_gain_mode;
    if (skiq_read_rx_gain_mode(card, rx_hdl, &p_gain_mode) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_read_rx_gain_mode (card %d)", card);
    }
    return p_gain_mode == skiq_rx_gain_auto;
  }

  return SoapySDR::Device::getGainMode(direction, channel);
}

void SoapySidekiq::setGain(const int direction, const size_t channel, const double value) {
    int status;

  if (direction == SOAPY_SDR_RX) {
    uint16_t gain = (uint16_t)(abs(value)); 
    status = skiq_write_rx_gain(card, rx_hdl, gain) ; 
    if (status != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_write_rx_gain (card %d, value %d) status %d", card, gain, status);
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting rx gain: %d", gain);
  }

  /* For TX gain is attenuation, someone may send that gain as negative or positive 
   * Assume it is attenuation and take the abs() of the number */
  if (direction == SOAPY_SDR_TX) {
    uint16_t attenuation = (uint16_t)(abs(value)); 
    if (skiq_write_tx_attenuation(card, tx_hdl, attenuation) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_write_tx_gain (card %d, value %d)", card, attenuation);
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting tx attenuation: %d", attenuation);
  }
}

double SoapySidekiq::getGain(const int direction, const size_t channel) const {
  if (direction == SOAPY_SDR_RX) {
    uint8_t gain_index;
    if (skiq_read_rx_gain(card, rx_hdl, &gain_index) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_read_rx_gain (card %d)", card);
    }
    return static_cast<double>(gain_index);
  }

  return SoapySDR::Device::getGain(direction, channel);
}

SoapySDR::Range SoapySidekiq::getGainRange(const int direction, const size_t channel, const std::string &name) const {
  if (direction == SOAPY_SDR_RX) {
    uint8_t gain_index_min;
    uint8_t gain_index_max;
    if (skiq_read_rx_gain_index_range(card, rx_hdl, &gain_index_min, &gain_index_max) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_read_rx_gain_index_range (card %d)", card);
    }
    return SoapySDR::Range(gain_index_max, gain_index_max);
  }

  return SoapySDR::Device::getGainRange(direction, channel, name);
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapySidekiq::setFrequency(const int direction,
                                const size_t channel,
                                const std::string &name,
                                const double frequency,
                                const SoapySDR::Kwargs &args) {
  if (direction == SOAPY_SDR_RX && name == "RF") {
    rx_center_frequency = (uint64_t) frequency;
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting rx center freq: %ld", rx_center_frequency);
    if (skiq_write_rx_LO_freq(card, rx_hdl, rx_center_frequency) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR,
                    "Failure: skiq_write_rx_LO_freq (card %d, frequency %d)",
                    card,
                    rx_center_frequency);
    }
  }

  if (direction == SOAPY_SDR_TX && name == "RF") {
    tx_center_frequency = (uint64_t) frequency;
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting tx center freq: %d", tx_center_frequency);
    if (skiq_write_tx_LO_freq(card, tx_hdl, tx_center_frequency) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR,
                    "Failure: skiq_write_tx_LO_freq (card %d, frequency %d)",
                    card,
                    tx_center_frequency);
    }
  }
}

double SoapySidekiq::getFrequency(const int direction, const size_t channel, const std::string &name) const {
  if (direction == SOAPY_SDR_RX && name == "RF") {
    uint64_t freq;
    double tuned_freq;
    if (skiq_read_rx_LO_freq(card, rx_hdl, &freq, &tuned_freq) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_read_rx_LO_freq (card %d)", card);
    }
    return static_cast<double>(freq);
  }

  if (direction == SOAPY_SDR_TX && name == "RF") {
    uint64_t freq;
    double tuned_freq;
    if (skiq_read_tx_LO_freq(card, tx_hdl, &freq, &tuned_freq) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_read_tx_LO_freq (card %d)", card);
    }
    return static_cast<double>(freq);
  }

  return 0;
}

std::vector<std::string> SoapySidekiq::listFrequencies(const int direction, const size_t channel) const {
  std::vector<std::string> names;
  names.push_back("RF");
  return names;
}

SoapySDR::RangeList SoapySidekiq::getFrequencyRange(const int direction,
                                                    const size_t channel,
                                                    const std::string &name) const {
  SoapySDR::RangeList results;
  uint64_t max;
  uint64_t min;

  if (direction == SOAPY_SDR_RX && name == "RF") {
    if (skiq_read_rx_LO_freq_range(card, &max, &min) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_read_rx_LO_freq_range (card %d)", card);
    }
    results.push_back(SoapySDR::Range(min, max));
  }

  if (direction == SOAPY_SDR_TX && name == "RF") {
    if (skiq_read_tx_LO_freq_range(card, &max, &min) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_read_tx_LO_freq_range (card %d)", card);
    }
    results.push_back(SoapySDR::Range(min, max));
  }

  return results;
}

SoapySDR::ArgInfoList SoapySidekiq::getFrequencyArgsInfo(const int direction, const size_t channel) const {
  SoapySDR::ArgInfoList freqArgs;

  // TODO: frequency arguments

  return freqArgs;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapySidekiq::setSampleRate(const int direction, const size_t channel, const double rate) {
  if (direction == SOAPY_SDR_RX) {
    rx_sample_rate = (uint32_t) rate;
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting rx sample rate: %d", rx_sample_rate);
    if (skiq_write_rx_sample_rate_and_bandwidth(card, rx_hdl, rx_sample_rate, rx_bandwidth) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR,
                    "Failure: skiq_write_rx_sample_rate_and_bandwidth (card %d, sample_rate %d, bandwidth %d)",
                    card,
                    rx_sample_rate,
                    rx_bandwidth);
    }
  }

  if (direction == SOAPY_SDR_TX) {
    tx_sample_rate = (uint32_t) rate;
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting tx sample rate: %d", tx_sample_rate);
    if (skiq_write_tx_sample_rate_and_bandwidth(card, tx_hdl, tx_sample_rate, tx_bandwidth) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR,
                    "Failure: skiq_write_tx_sample_rate_and_bandwidth (card %d, sample_rate %d, bandwidth %d)",
                    card,
                    tx_sample_rate,
                    tx_bandwidth);
    }
  }
}

double SoapySidekiq::getSampleRate(const int direction, const size_t channel) const {
  uint32_t rate;
  double actual_rate;
  if (direction == SOAPY_SDR_RX) {
    if (skiq_read_rx_sample_rate(card, rx_hdl, &rate, &actual_rate) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_read_rx_sample_rate (card %d)", card);
    }
    return static_cast<double>(rate);
  }

  if (direction == SOAPY_SDR_TX) {
    if (skiq_read_tx_sample_rate(card, tx_hdl, &rate, &actual_rate) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_read_tx_sample_rate (card %d)", card);
    }
    return static_cast<double>(rate);
  }

  return SoapySDR::Device::getSampleRate(direction, channel);
}

std::vector<double> SoapySidekiq::listSampleRates(const int direction, const size_t channel) const {
  std::vector<double> results;

  uint32_t min_sample_rate;
  if (skiq_read_min_sample_rate(card, &min_sample_rate) != 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_read_min_sample_rate (card %d)", card);
  }
  uint32_t max_sample_rate;
  if (skiq_read_max_sample_rate(card, &max_sample_rate) != 0) {
    SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_read_min_sample_rate (card %d)", card);
  }

  //  iterate through all sample rates
  uint32_t sample_rate = min_sample_rate;
  while (sample_rate <= max_sample_rate) {
    results.push_back(sample_rate);
    sample_rate += 250000;
  }

  return results;
}

void SoapySidekiq::setBandwidth(const int direction, const size_t channel, const double bw) {
  if (direction == SOAPY_SDR_RX) {
    rx_bandwidth = (uint32_t) bw;
    if (skiq_write_rx_sample_rate_and_bandwidth(card, rx_hdl, rx_sample_rate, rx_bandwidth) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR,
                    "Failure: skiq_write_rx_sample_rate_and_bandwidth (card %d, sample_rate %d, bandwidth %d)",
                    card,
                    rx_sample_rate,
                    rx_bandwidth);
    }
  }

  if (direction == SOAPY_SDR_TX) {
    tx_bandwidth = (uint32_t) bw;
    if (skiq_write_tx_sample_rate_and_bandwidth(card, tx_hdl, tx_sample_rate, tx_bandwidth) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR,
                    "Failure: skiq_write_tx_sample_rate_and_bandwidth (card %d, sample_rate %d, bandwidth %d)",
                    card,
                    tx_sample_rate,
                    tx_bandwidth);
    }
  }
}

double SoapySidekiq::getBandwidth(const int direction, const size_t channel) const {
  uint32_t rate;
  double actual_rate;
  uint32_t bandwidth;
  uint32_t actual_bandwidth;
  if (direction == SOAPY_SDR_RX) {
    if (skiq_read_rx_sample_rate_and_bandwidth(card, rx_hdl, &rate, &actual_rate, &bandwidth, &actual_bandwidth) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_read_rx_sample_rate_and_bandwidth (card %d)", card);
    }
  }

  if (direction == SOAPY_SDR_TX) {
    if (skiq_read_tx_sample_rate_and_bandwidth(card, tx_hdl, &rate, &actual_rate, &bandwidth, &actual_bandwidth) != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_read_tx_sample_rate_and_bandwidth (card %d)", card);
    }
  }

  return bandwidth;
}

std::vector<double> SoapySidekiq::listBandwidths(const int direction, const size_t channel) const {
  std::vector<double> bandwidths;
  bandwidths.push_back(200000);
  bandwidths.push_back(300000);
  bandwidths.push_back(600000);
  bandwidths.push_back(1536000);
  bandwidths.push_back(5000000);
  bandwidths.push_back(6000000);
  bandwidths.push_back(7000000);
  bandwidths.push_back(8000000);
  return bandwidths;
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapySidekiq::getSettingInfo(void) const {
  SoapySDR::ArgInfoList setArgs;

  SoapySDR::ArgInfo iqSwapArg;

  iqSwapArg.key = "iq_swap";
  iqSwapArg.value = "false";
  iqSwapArg.name = "I/Q Swap";
  iqSwapArg.description = "I/Q Swap Mode";
  iqSwapArg.type = SoapySDR::ArgInfo::BOOL;

  setArgs.push_back(iqSwapArg);

  return setArgs;
}

void SoapySidekiq::writeSetting(const std::string &key, const std::string &value) {
  if (key == "iq_swap") {
    iq_swap = ((value == "true") ? true : false);
    SoapySDR_logf(SOAPY_SDR_DEBUG, "I/Q swap: %s", iq_swap ? "true" : "false");
  }
}

std::string SoapySidekiq::readSetting(const std::string &key) const {
  if (key == "iq_swap") {
    return iq_swap ? "true" : "false";
  }

  SoapySDR_logf(SOAPY_SDR_WARNING, "Unknown setting '%s'", key.c_str());
  return "";
}
