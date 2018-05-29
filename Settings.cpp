
#include "SoapySidekiq.hpp"

std::vector<SoapySDR::Kwargs> SoapySidekiq::sidekiq_devices;

SoapySidekiq::SoapySidekiq(const SoapySDR::Kwargs &args)
{
    rx_sample_rate = 2048000;
    rx_center_frequency = 100000000;
    tx_sample_rate = 2048000;
    tx_center_frequency = 100000000;


    numBuffers = DEFAULT_NUM_BUFFERS;
    bufferLength = DEFAULT_BUFFER_LENGTH;

    iqSwap = false;
    agcMode = false;

    bufferedElems = 0;
    resetBuffer = false;

    if (args.count("card") != 0)
    {
        try
        {
            card = std::stoi(args.at("card"));
        }
        catch (const std::invalid_argument &)
        {
        }

        SoapySDR_logf(SOAPY_SDR_DEBUG, "Found Sidekiq Device using device index parameter 'card' = %d", card);
    }

    SoapySDR_logf(SOAPY_SDR_DEBUG, "Sidekiq opening card %d", card);

    skiq_xport_type_t type = skiq_xport_type_auto;
    skiq_xport_init_level_t level = skiq_xport_init_level_full;

    /* initialize libsidekiq for card numbers 0 and 1 */
    skiq_init(type, level, &card, 1);

}

SoapySidekiq::~SoapySidekiq(void)
{
    //cleanup device handles
    skiq_exit();
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapySidekiq::getDriverKey(void) const
{
    return "Sidekiq";
}

std::string SoapySidekiq::getHardwareKey(void) const
{
    return "Sidekiq";
}

SoapySDR::Kwargs SoapySidekiq::getHardwareInfo(void) const
{
    //key/value pairs for any useful information
    //this also gets printed in --probe
    SoapySDR::Kwargs args;

    args["origin"] = "https://github.com/hurdad/SoapySidekiq";
    args["card"] = std::to_string(card);

    return args;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapySidekiq::getNumChannels(const int dir) const
{
    return 1;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapySidekiq::listAntennas(const int direction, const size_t channel) const
{
    std::vector<std::string> antennas;
    antennas.push_back("RX");
    antennas.push_back("TX");
    return antennas;
}

void SoapySidekiq::setAntenna(const int direction, const size_t channel, const std::string &name)
{
    SoapySDR::Device::setAntenna(direction, channel, name);
}

std::string SoapySidekiq::getAntenna(const int direction, const size_t channel) const
{
    return SoapySDR::Device::getAntenna(direction, channel);
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapySidekiq::hasDCOffsetMode(const int direction, const size_t channel) const
{
    return direction == SOAPY_SDR_RX;
}

bool SoapySidekiq::hasFrequencyCorrection(const int direction, const size_t channel) const
{
    return false;
}

void SoapySidekiq::setFrequencyCorrection(const int direction, const size_t channel, const double value)
{
    SoapySDR::Device::setFrequencyCorrection(direction, channel, value);
}

double SoapySidekiq::getFrequencyCorrection(const int direction, const size_t channel) const
{
    return SoapySDR::Device::getFrequencyCorrection(direction,channel);
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapySidekiq::listGains(const int direction, const size_t channel) const
{
    //list available gain elements,
    std::vector<std::string> results;

    return results;
}

bool SoapySidekiq::hasGainMode(const int direction, const size_t channel) const
{
    return true;
}

void SoapySidekiq::setGainMode(const int direction, const size_t channel, const bool automatic)
{
    agcMode = automatic;
    if (direction == SOAPY_SDR_RX) {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting Sidekiq RX Gain Mode: %s", automatic ? "skiq_rx_gain_auto" : "skiq_rx_gain_manual");
        skiq_rx_gain_t mode = automatic ? skiq_rx_gain_auto : skiq_rx_gain_manual;
        skiq_write_rx_gain_mode(card, rx_hdl, mode);
    }
}

bool SoapySidekiq::getGainMode(const int direction, const size_t channel) const
{
    if (direction == SOAPY_SDR_RX) {
        skiq_rx_gain_t p_gain_mode;
        skiq_read_rx_gain_mode(card, rx_hdl, &p_gain_mode);
        return p_gain_mode == skiq_rx_gain_auto;
    }

    return  SoapySDR::Device::getGainMode(direction, channel);
}

void SoapySidekiq::setGain(const int direction, const size_t channel, const double value)
{
    //set the overall gain by distributing it across available gain elements
    //OR delete this function to use SoapySDR's default gain distribution algorithm...
    SoapySDR::Device::setGain(direction, channel, value);
}

void SoapySidekiq::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
    if (direction == SOAPY_SDR_RX)
    {
        skiq_write_rx_gain(card, rx_hdl, value);
    }
}

double SoapySidekiq::getGain(const int direction, const size_t channel, const std::string &name) const
{
    if (direction == SOAPY_SDR_RX)
    {
        uint8_t gain_index;
        skiq_read_rx_gain(card, rx_hdl, &gain_index);
        return double(gain_index);
    }

    return SoapySDR::Device::getGain(direction, channel, name);
}

SoapySDR::Range SoapySidekiq::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    if (direction == SOAPY_SDR_RX)
    {
        uint8_t gain_index_min;
        uint8_t gain_index_max;
        skiq_read_rx_gain_index_range(card, rx_hdl, &gain_index_min, &gain_index_max);
        return SoapySDR::Range(gain_index_max, gain_index_max);
    }

    return SoapySDR::Device::getGainRange(direction, channel, name);

}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapySidekiq::setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args)
{
    if (direction == SOAPY_SDR_RX && name == "RF"){
        rx_center_frequency = (uint64_t) frequency;
        resetBuffer = true;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting rx center freq: %d", rx_center_frequency);
        skiq_write_rx_LO_freq(card, rx_hdl, rx_center_frequency);
    }

    if (direction == SOAPY_SDR_TX && name == "RF"){
        tx_center_frequency = (uint64_t) frequency;
        resetBuffer = true;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting tx center freq: %d", tx_center_frequency);
        skiq_write_tx_LO_freq(card, tx_hdl, tx_center_frequency);
    }
}

double SoapySidekiq::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
    if (direction == SOAPY_SDR_RX && name == "RF") {
        uint64_t freq;
        double tuned_freq;
        skiq_read_rx_LO_freq(card, rx_hdl,&freq, &tuned_freq);
        return double(freq);
    }

    if (direction == SOAPY_SDR_TX && name == "RF"){
        uint64_t freq;
        double tuned_freq;
        skiq_read_tx_LO_freq(card, tx_hdl, &freq, &tuned_freq);
        return double(freq);
    }

    return 0;
}

std::vector<std::string> SoapySidekiq::listFrequencies(const int direction, const size_t channel) const
{
    std::vector<std::string> names;
    names.push_back("RF");
    names.push_back("CORR");
    return names;
}

SoapySDR::RangeList SoapySidekiq::getFrequencyRange(
        const int direction,
        const size_t channel,
        const std::string &name) const
{
    SoapySDR::RangeList results;
    uint64_t max;
    uint64_t min;

    if (direction == SOAPY_SDR_RX && name == "RF")
    {
        skiq_read_rx_LO_freq_range(card, &max, &min);
        results.push_back(SoapySDR::Range(min, max));
    }

    if (direction == SOAPY_SDR_TX && name == "RF")
    {
        skiq_read_tx_LO_freq_range(card, &max, &min);
        results.push_back(SoapySDR::Range(min, max));
    }

    return results;
}

SoapySDR::ArgInfoList SoapySidekiq::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
    SoapySDR::ArgInfoList freqArgs;

    // TODO: frequency arguments

    return freqArgs;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapySidekiq::setSampleRate(const int direction, const size_t channel, const double rate) {

    if (direction == SOAPY_SDR_RX)
    {
        rx_sample_rate = rate;
        resetBuffer = true;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting rx sample rate: %d", rx_sample_rate);
        skiq_write_rx_sample_rate_and_bandwidth(card, rx_hdl, rx_sample_rate, rx_bandwidth);
    }

    if (direction == SOAPY_SDR_TX)
    {
        tx_sample_rate = rate;
        resetBuffer = true;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting tx sample rate: %d", tx_sample_rate);
        skiq_write_tx_sample_rate_and_bandwidth(card, tx_hdl, tx_sample_rate, tx_bandwidth);
    }
}

double SoapySidekiq::getSampleRate(const int direction, const size_t channel) const
{
    uint32_t rate;
    double actual_rate;
    if (direction == SOAPY_SDR_RX)
    {
        skiq_read_rx_sample_rate(card, rx_hdl, &rate, &actual_rate);
        return double(rate);
    }

    if (direction == SOAPY_SDR_TX)
    {
        skiq_read_tx_sample_rate(card, tx_hdl, &rate, &actual_rate);
        return double(rate);
    }

    return SoapySDR::Device::getSampleRate(direction, channel);
}

std::vector<double> SoapySidekiq::listSampleRates(const int direction, const size_t channel) const
{
    std::vector<double> results;

    uint32_t min_sample_rate;
    skiq_read_min_sample_rate(card, &min_sample_rate);

    uint32_t max_sample_rate;
    skiq_read_max_sample_rate(card, &max_sample_rate);

    results.push_back(min_sample_rate);
    results.push_back(max_sample_rate);

    return results;
}

void SoapySidekiq::setBandwidth(const int direction, const size_t channel, const double bw)
{
    if (direction == SOAPY_SDR_RX)
    {
        rx_bandwidth = bw;
        skiq_write_rx_sample_rate_and_bandwidth(card, rx_hdl, rx_sample_rate, rx_bandwidth);
    }

    if (direction == SOAPY_SDR_TX)
    {
        tx_bandwidth = bw;
        skiq_write_tx_sample_rate_and_bandwidth(card, tx_hdl, tx_sample_rate, tx_bandwidth);
    }
}

double SoapySidekiq::getBandwidth(const int direction, const size_t channel) const
{
    uint32_t rate;
    double actual_rate;
    uint32_t bandwidth;
    uint32_t actual_bandwidth;
    if (direction == SOAPY_SDR_RX)
    {
        skiq_read_rx_sample_rate_and_bandwidth(card, rx_hdl, &rate, &actual_rate, &bandwidth, &actual_bandwidth);
    }

    if (direction == SOAPY_SDR_TX)
    {
        skiq_read_tx_sample_rate_and_bandwidth(card, tx_hdl, &rate, &actual_rate, &bandwidth, &actual_bandwidth);
    }

    return bandwidth;
}

std::vector<double> SoapySidekiq::listBandwidths(const int direction, const size_t channel) const
{
    std::vector<double> results;

    return results;
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapySidekiq::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList setArgs;

    SoapySDR::ArgInfo iqSwapArg;

    iqSwapArg.key = "iq_swap";
    iqSwapArg.value = "false";
    iqSwapArg.name = "I/Q Swap";
    iqSwapArg.description = "Sidekiq I/Q Swap Mode";
    iqSwapArg.type = SoapySDR::ArgInfo::BOOL;

    setArgs.push_back(iqSwapArg);

    SoapySDR_logf(SOAPY_SDR_DEBUG, "SETARGS?");

    return setArgs;
}

void SoapySidekiq::writeSetting(const std::string &key, const std::string &value)
{

    if (key == "iq_swap")
    {
        iqSwap = (value=="true");
        SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-SDR I/Q swap: %s", iqSwap ? "true" : "false");
    }

}

std::string SoapySidekiq::readSetting(const std::string &key) const
{
    if (key == "iq_swap") {
        return iqSwap ? "true" : "false";
    }

    SoapySDR_logf(SOAPY_SDR_WARNING, "Unknown setting '%s'", key.c_str());
    return SoapySDR::Device::readSetting(key);
}
