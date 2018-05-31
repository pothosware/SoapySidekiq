//  Copyright [2018] <Alexander Hurd>"

#include "SoapySidekiq.hpp"

std::vector<std::string> SoapySidekiq::listSensors(void) const {
  std::vector<std::string> sensors;
  sensors.push_back("temperature");
  sensors.push_back("acceleration");
  return sensors;
}

SoapySDR::ArgInfo SoapySidekiq::getSensorInfo(const std::string &key) const {
  return SoapySDR::Device::getSensorInfo(key);
}

std::string SoapySidekiq::readSensor(const std::string &key) const {
  if (key.compare("temperature")) {
    int8_t temp = 0;
    skiq_read_temp(card, &temp);
    return std::to_string(temp);
  }
  bool supported = false;
  if (key.compare("acceleration")) {
    if (!skiq_is_accel_supported(card, &supported)) {
      SoapySDR_logf(SOAPY_SDR_WARNING, "Acceleration not supported by card %i", card);
      return "{}";
    }
    skiq_write_accel_state(card, 1);  //  enable
    int16_t x_data = 0;
    int16_t y_data = 0;
    int16_t z_data = 0;
    skiq_read_accel(card, &x_data, &y_data, &z_data);
    skiq_write_accel_state(card, 0);  //  disable
    std::stringstream ss;
    ss << "{\"x\":" << x_data << " \"y\":" << y_data << " \"z\":" << z_data << "}";  //  json format
    return ss.str();
  }

  return SoapySDR::Device::readSensor(key);
}

std::vector<std::string> SoapySidekiq::listSensors(const int direction, const size_t channel) const {
  return SoapySDR::Device::listSensors(direction, channel);
}
std::string SoapySidekiq::readSensor(const int direction, const size_t channel, const std::string &key) const {
  return SoapySDR::Device::readSensor(direction, channel, key);
}
