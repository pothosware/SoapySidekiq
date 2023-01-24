//  Copyright [2018] <Alexander Hurd>"

#include "SoapySidekiq.hpp"

std::vector<std::string> SoapySidekiq::listSensors(void) const {
  std::vector<std::string> sensors;
  SoapySDR_logf(SOAPY_SDR_TRACE, "listSensors");

  sensors.push_back("temperature");
  sensors.push_back("acceleration");

  return sensors;
}

SoapySDR::ArgInfo SoapySidekiq::getSensorInfo(const std::string &key) const {
  SoapySDR_logf(SOAPY_SDR_TRACE, "getSensorInfo");

  return SoapySDR::Device::getSensorInfo(key);
}

std::string SoapySidekiq::readSensor(const std::string &key) const {
  int status = 0;
  SoapySDR_logf(SOAPY_SDR_TRACE, "readSensor");

  if (key.compare("temperature")) {
    int8_t temp = 0;
    status = skiq_read_temp(card, &temp) ;
    if (status  != 0) 
    {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_read_temp (card %i), status %d", card, status);
    }
    else {
      SoapySDR_logf(SOAPY_SDR_DEBUG, "Temp is %d", temp);
    }

    return std::to_string(temp);
  }
  bool supported = false;

  if (key.compare("acceleration")) {
    status = skiq_is_accel_supported(card, &supported);
    if (status  != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq-is_accel_supported (card %i), status %d", card, status);
    }

    if (!supported) {
      SoapySDR_logf(SOAPY_SDR_WARNING, "Acceleration not supported by card %i, status %d", card, status);
      return "{}";
    }

    /* enable accel for the card */
    status = skiq_write_accel_state(card, 1);
    if (status != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_write_accel_state (card %i), status %d", card, status);
      return "{}";
    }

    int16_t x_data = 0;
    int16_t y_data = 0;
    int16_t z_data = 0;
    status = skiq_read_accel(card, &x_data, &y_data, &z_data);
    if (status != 0) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_read_accel (card %i), status %d", card, status);
      return "{}";
    }

    /* disable accel */
    status = skiq_write_accel_state(card, 0);
    if (status != 0) { 
      SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_write_accel_state (card %i), status %d", card, status);
      return "{}";
    };
    std::stringstream ss;
    ss << "{\"x\":" << x_data << " \"y\":" << y_data << " \"z\":" << z_data << "}";  //  json format
    return ss.str();
  }

  return SoapySDR::Device::readSensor(key);
}

std::vector<std::string> SoapySidekiq::listSensors(const int direction, const size_t channel) const {
  SoapySDR_logf(SOAPY_SDR_TRACE, "listSensors2");
  return SoapySDR::Device::listSensors(direction, channel);
}
std::string SoapySidekiq::readSensor(const int direction, const size_t channel, const std::string &key) const {
  SoapySDR_logf(SOAPY_SDR_TRACE, "read_sensor2");
  return SoapySDR::Device::readSensor(direction, channel, key);
}
