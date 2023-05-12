//  Copyright [2018] <Alexander Hurd>"

#include "SoapySidekiq.hpp"

std::vector<SoapySDR::Kwargs> SoapySidekiq::sidekiq_devices;
bool                          SoapySidekiq::rx_running;

// Constructor
SoapySidekiq::SoapySidekiq(const SoapySDR::Kwargs &args)
{
    int status = 0;
    uint8_t channels = 0;

    /* We need to set some default parameters in case the user does not */

    SoapySDR::setLogLevel(SOAPY_SDR_TRACE);

    //  rx defaults
    rx_sample_rate      = 2048000;
    rx_bandwidth        = (uint32_t)(rx_sample_rate * 0.8);
    rx_center_frequency = 100000000;

    //  tx defaults
    tx_sample_rate      = 2048000;
    tx_bandwidth        = (uint32_t)(tx_sample_rate * 0.8);
    tx_center_frequency = 100000000;

    useShort  = true;
    iq_swap   = false;
    counter   = false;
    debug_ctr = 0;
    card      = 0;

    //  this may change later according to format
    rx_running = false;

    if (args.count("card") != 0)
    {
        try
        {

            card = std::stoi(args.at("card"));
            serial = args.at("serial");
        }
        catch (const std::invalid_argument &)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Requested card (%d), not found", std::stoi(args.at("card")));
        }

        SoapySDR_logf( SOAPY_SDR_DEBUG, "Found Sidekiq Device 'card' = %d, 'serial' = %s", card, serial.c_str());
    }
    else
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "No cards found");
    }



    rx_hdl = skiq_rx_hdl_A1;
    tx_hdl = skiq_tx_hdl_A1;

    skiq_xport_type_t       type  = skiq_xport_type_auto;
    skiq_xport_init_level_t level = skiq_xport_init_level_full;

    SoapySDR_logf(SOAPY_SDR_DEBUG, "Sidekiq opening card %d", card);

    /* init sidekiq */
    status = skiq_init(type, level, &card, 1);
    if (status != 0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR,
                      "Failure: skiq_init (card %d), status %d", card, status);
    }
    /* set iq order to iq instead of qi */
    status = skiq_write_iq_order_mode(card, skiq_iq_order_iq);
    if (status != 0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR,
                      "Failure: setting iq order (card %d), status %d", card,
                      status);
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting iq mode");

    rx_block_size_in_words = 0;

    status = skiq_read_num_rx_chans(card, &channels);
    if (status != 0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR,
                      "failure reading rx number of channels (card %d), status %d", card, status);
    }
    num_rx_channels = channels;


    status = skiq_read_num_tx_chans(card, &channels);
    if (status != 0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR,
                      "failure reading tx number of channels (card %d), status %d", card, status);
    }
    num_tx_channels = channels;
    uint8_t tmp_resolution = 0;

    /* Every card can have a different iq resolution.  Some are 12 bits and some are 14 and some 16
     * So to validate the data we need to know when to wrap the value
     */
    status = skiq_read_rx_iq_resolution( card, &tmp_resolution );
    if (status != 0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "Error: failed to read iq_resolution %d (status = %d)", 
                card, status);
    }

    this->resolution = tmp_resolution;
    this->max_value = (double) ((1 << (tmp_resolution-1))-1);
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Info: resolution %d" " bits, max ADC value %d", 
                  this->resolution, (uint64_t) this->max_value); 

}

SoapySidekiq::~SoapySidekiq(void)
{

    SoapySDR_logf(SOAPY_SDR_INFO, "In destructor", card);
    if (skiq_exit() != 0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "Failure: skiq_exit", card);
    }
    SoapySDR_logf(SOAPY_SDR_INFO, "leaving destructor", card);
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapySidekiq::getDriverKey(void) const
{
    SoapySDR_logf(SOAPY_SDR_TRACE, "getDriverKey");
    return "Sidekiq";
}

std::string SoapySidekiq::getHardwareKey(void) const
{
    int status = 0;
    skiq_param_t param;
    std::string part_str = "none";
    skiq_part_t part;

    SoapySDR_logf(SOAPY_SDR_TRACE, "getHardwareKey");
    // query card name and return

    status = skiq_read_parameters(card, &param);
    if (status != 0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR,
                      "Failure: card %d, skiq_read_parameters status is %d", 
                      card, status);
    }

    part = param.card_param.part_type; 
    part_str = skiq_part_string(part);

    SoapySDR_logf(SOAPY_SDR_DEBUG, "Part type is %s", part_str.c_str());

    return part_str;
}

SoapySDR::Kwargs SoapySidekiq::getHardwareInfo(void) const
{
    //  key/value pairs for any useful information
    //  this also gets printed in --probe
    SoapySDR::Kwargs args;
    SoapySDR_logf(SOAPY_SDR_TRACE, "getHardwareInfo");

    args["origin"] = "https://github.com/pothosware/SoapySidekiq";
    args["card"]   = std::to_string(card);
    args["serial"]   = serial;
    args["rx_channels"]   = std::to_string(num_rx_channels);
    args["tx_channels"]   = std::to_string(num_tx_channels);

    return args;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapySidekiq::getNumChannels(const int dir) const
{
    SoapySDR_logf(SOAPY_SDR_TRACE, "getNumChannels, direction = %d", dir);

    if (dir == SOAPY_SDR_RX)
    {
        return num_rx_channels;
    }
    else if (dir == SOAPY_SDR_TX)
    {
        return num_tx_channels;
    }
    else
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "Failure getNumChannels called for an Invalid Direction %d", dir);
    }

    return -1;
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapySidekiq::hasDCOffsetMode(const int    direction,
                                   const size_t channel) const
{
    int status = 0;
    uint32_t mask = 0; 

    SoapySDR_logf(SOAPY_SDR_TRACE, "hasDCOffsetMode direction = %d", direction);

    
    if (direction == SOAPY_SDR_RX)
    {
        skiq_rx_hdl_t hdl = static_cast<skiq_rx_hdl_t>(channel);
        status = skiq_read_rx_cal_types_avail(card, hdl, &mask);
        if (status != 0)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                          "Failure: skiq_read_rx_rx_cal_types_avail (card %d, "
                          "mask %d), status %d",
                          card, mask, status);
        }
        
        if ((mask & skiq_rx_cal_type_dc_offset)== skiq_rx_cal_type_dc_offset)
        {
            SoapySDR_logf(SOAPY_SDR_DEBUG, "Channel %d, has DC Offest Correction %d", channel, mask);
            return true;
        }
        else
        {
            SoapySDR_logf(SOAPY_SDR_DEBUG, "Channel %d, does not have DC Offest Correction %d", channel, mask);
            return false;
        }
    }
    else
    {
        //TODO
    }

    return false;
}

void SoapySidekiq::setDCOffsetMode(const int direction, const size_t channel,
                                   const bool automatic)
{
    int status = 0;
    SoapySDR_logf(SOAPY_SDR_TRACE, "setDCOffsetMode, direction = %d", direction);

    if (direction == SOAPY_SDR_RX)
    {
        skiq_rx_hdl_t hdl = static_cast<skiq_rx_hdl_t>(channel);
        status = skiq_write_rx_dc_offset_corr(card, hdl, automatic);
        if (status != 0)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                          "Failure: skiq_write_rx_dc_offset_corr (card %d, "
                          "enable %d), status %d",
                          card, automatic, status);
        }
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting DC Offest Correction to %d", automatic);
    }
    else
    {
        //TODO
    }
}

bool SoapySidekiq::getDCOffsetMode(const int    direction,
                                   const size_t channel) const
{
    bool enable = false;
    int  status = 0;
    SoapySDR_logf(SOAPY_SDR_TRACE, "getDCOffsetMode");

    if (direction == SOAPY_SDR_RX)
    {
        skiq_rx_hdl_t hdl = static_cast<skiq_rx_hdl_t>(channel);
        status = skiq_read_rx_dc_offset_corr(card, hdl, &enable);
        if (status != 0)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                          "Failure: skiq_read_rx_dc_offset_corr (card %d, "
                          "enable %d), status %d",
                          card, enable, status);
        }
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Channel %d, get DC Offest Correction %d", channel, enable);
    }
    else
    {
        //TODO
    }

    return enable;

}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapySidekiq::listAntennas(const int    direction,
                                                    const size_t channel) const
{
    std::vector<std::string> antennas;
    antennas.push_back("");
    //  antennas.push_back("TX");
    return antennas;
}

void SoapySidekiq::setAntenna(const int direction, const size_t channel,
                              const std::string &name)
{
    SoapySDR::Device::setAntenna(direction, channel, name);
}

std::string SoapySidekiq::getAntenna(const int    direction,
                                     const size_t channel) const
{
    return SoapySDR::Device::getAntenna(direction, channel);
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapySidekiq::listGains(const int    direction,
                                                 const size_t channel) const
{
    SoapySDR_logf(SOAPY_SDR_TRACE, "listGains");
    //  list available gain elements,
    std::vector<std::string> results;
    return results;
}

bool SoapySidekiq::hasGainMode(const int direction, const size_t channel) const
{
    SoapySDR_logf(SOAPY_SDR_TRACE, "hasGainMode");
    return true;
}

void SoapySidekiq::setGainMode(const int direction, const size_t channel,
                               const bool automatic)
{
    int status = 0;

    SoapySDR_logf(SOAPY_SDR_TRACE, "setGainMode");

    if (direction == SOAPY_SDR_RX)
    {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting Sidekiq RX Gain Mode: %s",
                      automatic ? "skiq_rx_gain_auto" : "skiq_rx_gain_manual");
        skiq_rx_gain_t mode =
            automatic ? skiq_rx_gain_auto : skiq_rx_gain_manual;
        status = skiq_write_rx_gain_mode(card, rx_hdl, mode);
        if (status != 0)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                          "Failure: skiq_write_rx_gain_mode (card %d, mode "
                          "%d), status %d",
                          card, mode, status);
        }
    }
}

bool SoapySidekiq::getGainMode(const int direction, const size_t channel) const
{
    int status = 0;
    SoapySDR_logf(SOAPY_SDR_TRACE, "getGainMode");

    if (direction == SOAPY_SDR_RX)
    {
        skiq_rx_gain_t p_gain_mode;
        status = skiq_read_rx_gain_mode(card, rx_hdl, &p_gain_mode);
        if (status != 0)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                          "Failure: skiq_read_rx_gain_mode (card %d), status",
                          card, status);
        }
        return p_gain_mode == skiq_rx_gain_auto;
    }

    return SoapySDR::Device::getGainMode(direction, channel);
}

void SoapySidekiq::setGain(const int direction, const size_t channel,
                           const double value)
{
    int status;

    SoapySDR_logf(SOAPY_SDR_TRACE, "setGain");

    if (direction == SOAPY_SDR_RX)
    {
        /* if gain mode is automatic, we should leave */
        if (!getGainMode(direction, channel))
        {
            uint16_t gain = (uint16_t)(abs(value));
            status        = skiq_write_rx_gain(card, rx_hdl, gain);
            if (status != 0)
            {
                SoapySDR_logf(
                    SOAPY_SDR_ERROR,
                    "Failure: skiq_write_rx_gain (card %d, value %d) status %d",
                    card, gain, status);
            }
            SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting rx gain: %d", gain);
        }
        else
        {
            SoapySDR_log(SOAPY_SDR_WARNING,
                         "Attempt to set gain even though it"
                         " was already set to automatic gain mode");
        }
    }

    /* For TX gain is attenuation, someone may send that gain as negative or
     * positive Assume it is attenuation and take the abs() of the number */
    if (direction == SOAPY_SDR_TX)
    {
        uint16_t attenuation = (uint16_t)(abs(value));
        status = skiq_write_tx_attenuation(card, tx_hdl, attenuation);
        if (status != 0)
        {
            SoapySDR_logf(
                SOAPY_SDR_ERROR,
                "Failure: skiq_write_tx_gain (card %d, value %d), status %d",
                card, attenuation, status);
        }
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting tx attenuation: %d",
                      attenuation);
    }
}

double SoapySidekiq::getGain(const int direction, const size_t channel) const
{
    int status = 0;

    SoapySDR_logf(SOAPY_SDR_TRACE, "getGain");

    if (direction == SOAPY_SDR_RX)
    {
        uint8_t gain_index;
        status = skiq_read_rx_gain(card, rx_hdl, &gain_index);
        if (status != 0)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                          "Failure: skiq_read_rx_gain (card %d), status %d",
                          card, status);
        }
        return static_cast<double>(gain_index);
    }

    return SoapySDR::Device::getGain(direction, channel);
}

SoapySDR::Range SoapySidekiq::getGainRange(const int    direction,
                                           const size_t channel) const
{
    int status = 0;

    SoapySDR_log(SOAPY_SDR_TRACE, "getGainRange");

    if (direction == SOAPY_SDR_RX)
    {
        uint8_t gain_index_min;
        uint8_t gain_index_max;

        status = skiq_read_rx_gain_index_range(card, rx_hdl, &gain_index_min,
                                               &gain_index_max);
        if (status != 0)
        {
            SoapySDR_logf(
                SOAPY_SDR_ERROR,
                "Failure: skiq_read_rx_gain_index_range (card %d), status %d",
                card);
            return SoapySDR::Device::getGainRange(direction, channel);
        }

        //TODO convert from gain index to dB

        return SoapySDR::Range(gain_index_min, gain_index_max);
    }

    return SoapySDR::Device::getGainRange(direction, channel);
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapySidekiq::setFrequency(const int direction, const size_t channel,
                                const std::string &name, const double frequency,
                                const SoapySDR::Kwargs &args)
{
    int status = 0;

    SoapySDR_log(SOAPY_SDR_TRACE, "setFrequency");
    if (direction == SOAPY_SDR_RX && name == "RF")
    {
        rx_center_frequency = (uint64_t)frequency;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting rx center freq: %ld",
                      rx_center_frequency);
        status = skiq_write_rx_LO_freq(card, rx_hdl, rx_center_frequency);
        if (status != 0)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                          "Failure: skiq_write_rx_LO_freq (card %d, frequency "
                          "%d), status %d",
                          card, rx_center_frequency, status);
        }
    }

    if (direction == SOAPY_SDR_TX && name == "RF")
    {
        tx_center_frequency = (uint64_t)frequency;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting tx center freq: %d",
                      tx_center_frequency);
        status = skiq_write_tx_LO_freq(card, tx_hdl, tx_center_frequency);
        if (status != 0)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                          "Failure: skiq_write_tx_LO_freq (card %d, frequency "
                          "%d), status %d",
                          card, tx_center_frequency, status);
        }
    }
}

double SoapySidekiq::getFrequency(const int direction, const size_t channel,
                                  const std::string &name) const
{
    int status = 0;

    SoapySDR_log(SOAPY_SDR_TRACE, "getFrequency");
    if (direction == SOAPY_SDR_RX && name == "RF")
    {
        uint64_t freq;
        double   tuned_freq;
        status = skiq_read_rx_LO_freq(card, rx_hdl, &freq, &tuned_freq);
        if (status != 0)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                          "Failure: skiq_read_rx_LO_freq (card %d), status %d",
                          card, status);
        }
        return static_cast<double>(freq);
    }

    if (direction == SOAPY_SDR_TX && name == "RF")
    {
        uint64_t freq;
        double   tuned_freq;
        status = skiq_read_tx_LO_freq(card, tx_hdl, &freq, &tuned_freq);
        if (status != 0)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                          "Failure: skiq_read_tx_LO_freq (card %d), status %d",
                          card, status);
        }
        return static_cast<double>(freq);
    }

    return 0;
}

std::vector<std::string> SoapySidekiq::listFrequencies(
    const int direction, const size_t channel) const
{
    std::vector<std::string> names;
    names.push_back("RF");
    return names;
}

SoapySDR::RangeList SoapySidekiq::getFrequencyRange(
    const int direction, const size_t channel, const std::string &name) const
{
    SoapySDR::RangeList results;
    SoapySDR_log(SOAPY_SDR_TRACE, "getFrequencyRange");
    uint64_t max;
    uint64_t min;
    int      status = 0;

    SoapySDR_log(SOAPY_SDR_TRACE, "getFrequencyRange");
    if (direction == SOAPY_SDR_RX && name == "RF")
    {
        status = skiq_read_rx_LO_freq_range(card, &max, &min);
        if (status != 0)
        {
            SoapySDR_logf(
                SOAPY_SDR_ERROR,
                "Failure: skiq_read_rx_LO_freq_range (card %d), status %d",
                card, status);
        }
        results.push_back(SoapySDR::Range(min, max));
    }

    if (direction == SOAPY_SDR_TX && name == "RF")
    {
        status = skiq_read_tx_LO_freq_range(card, &max, &min);
        if (status != 0)
        {
            SoapySDR_logf(
                SOAPY_SDR_ERROR,
                "Failure: skiq_read_tx_LO_freq_range (card %d), status %d",
                card, status);
        }
        results.push_back(SoapySDR::Range(min, max));
    }

    return results;
}

SoapySDR::ArgInfoList SoapySidekiq::getFrequencyArgsInfo(
    const int direction, const size_t channel) const
{
    SoapySDR::ArgInfoList freqArgs;
    SoapySDR_log(SOAPY_SDR_TRACE, "getFrequencyArgsInfo");

    // TODO: frequency arguments

    return freqArgs;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapySidekiq::setSampleRate(const int direction, const size_t channel,
                                 const double rate)
{
    int status = 0;

    SoapySDR_log(SOAPY_SDR_TRACE, "setSampleRate");
    if (direction == SOAPY_SDR_RX)
    {
        rx_sample_rate = (uint32_t)rate;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting rx sample rate: %d",
                      rx_sample_rate);
        status = skiq_write_rx_sample_rate_and_bandwidth(
            card, rx_hdl, rx_sample_rate, rx_bandwidth);
        if (status != 0)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                          "Failure: skiq_write_rx_sample_rate_and_bandwidth "
                          "(card %d, sample_rate %d, bandwidth %d, status %d)",
                          card, rx_sample_rate, rx_bandwidth, status);
        }
    }

    if (direction == SOAPY_SDR_TX)
    {
        tx_sample_rate = (uint32_t)rate;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting tx sample rate: %d",
                      tx_sample_rate);
        status = skiq_write_tx_sample_rate_and_bandwidth(
            card, tx_hdl, tx_sample_rate, tx_bandwidth);
        if (status != 0)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                          "Failure: skiq_write_tx_sample_rate_and_bandwidth "
                          "(card %d, sample_rate %d, bandwidth %d, status %d)",
                          card, tx_sample_rate, tx_bandwidth, status);
        }
    }
}

double SoapySidekiq::getSampleRate(const int    direction,
                                   const size_t channel) const
{
    uint32_t rate;
    double   actual_rate;
    int      status = 0;

    SoapySDR_log(SOAPY_SDR_TRACE, "getSampleRate");
    if (direction == SOAPY_SDR_RX)
    {
        status = skiq_read_rx_sample_rate(card, rx_hdl, &rate, &actual_rate);
        if (status != 0)
        {
            SoapySDR_logf(
                SOAPY_SDR_ERROR,
                "Failure: skiq_read_rx_sample_rate (card %d), status %d", card,
                status);
        }
        return static_cast<double>(rate);
    }

    if (direction == SOAPY_SDR_TX)
    {
        status = skiq_read_tx_sample_rate(card, tx_hdl, &rate, &actual_rate);
        if (status != 0)
        {
            SoapySDR_logf(
                SOAPY_SDR_ERROR,
                "Failure: skiq_read_tx_sample_rate (card %d), status %d", card,
                status);
        }
        return static_cast<double>(rate);
    }

    return SoapySDR::Device::getSampleRate(direction, channel);
}

std::vector<double> SoapySidekiq::listSampleRates(const int    direction,
                                                  const size_t channel) const
{
    std::vector<double> results;
    int                 status = 0;

    SoapySDR_log(SOAPY_SDR_TRACE, "listSampleRates");
    uint32_t min_sample_rate;
    status = skiq_read_min_sample_rate(card, &min_sample_rate);
    if (status != 0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR,
                      "Failure: skiq_read_min_sample_rate (card %d), status %d",
                      card, status);
    }
    uint32_t max_sample_rate;
    status = skiq_read_max_sample_rate(card, &max_sample_rate);
    if (status != 0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR,
                      "Failure: skiq_read_min_sample_rate (card %d), status %d",
                      card, status);
    }

    //  iterate through all sample rates
    uint32_t sample_rate = min_sample_rate;
    while (sample_rate <= max_sample_rate)
    {
        results.push_back(sample_rate);
        sample_rate += 250000;
    }

    return results;
}

SoapySDR::RangeList SoapySidekiq::getSampleRateRange(const int    direction,
                                                     const size_t channel) const
{
    int                 status = 0;
    uint32_t            min_sample_rate;
    uint32_t            max_sample_rate;
    SoapySDR::RangeList ranges;

    SoapySDR_log(SOAPY_SDR_TRACE, "getSampleRateRange");
    status = skiq_read_min_sample_rate(card, &min_sample_rate);
    if (status != 0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR,
                      "Failure: skiq_read_min_sample_rate (card %d), status %d",
                      card, status);
    }
    status = skiq_read_max_sample_rate(card, &max_sample_rate);
    if (status != 0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR,
                      "Failure: skiq_read_min_sample_rate (card %d), status %d",
                      card, status);
    }
    ranges.push_back(SoapySDR::Range(min_sample_rate, max_sample_rate));

    return ranges;
}

void SoapySidekiq::setBandwidth(const int direction, const size_t channel,
                                const double bw)
{
    int status = 0;

    SoapySDR_logf(SOAPY_SDR_TRACE, "setBandwidth");
    if (direction == SOAPY_SDR_RX)
    {
        rx_bandwidth = (uint32_t)bw;
        status       = skiq_write_rx_sample_rate_and_bandwidth(
            card, rx_hdl, rx_sample_rate, rx_bandwidth);
        if (status != 0)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                          "Failure: skiq_write_rx_sample_rate_and_bandwidth "
                          "(card %d, sample_rate %d, bandwidth %d, status %d)",
                          card, rx_sample_rate, rx_bandwidth, status);
        }
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting rx bandwidth to %f", bw);

    if (direction == SOAPY_SDR_TX)
    {
        tx_bandwidth = (uint32_t)bw;
        status       = skiq_write_tx_sample_rate_and_bandwidth(
            card, tx_hdl, tx_sample_rate, tx_bandwidth);
        if (status != 0)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                          "Failure: skiq_write_tx_sample_rate_and_bandwidth "
                          "(card %d, sample_rate %d, bandwidth %d, status %d)",
                          card, tx_sample_rate, tx_bandwidth, status);
        }
    }
}

double SoapySidekiq::getBandwidth(const int    direction,
                                  const size_t channel) const
{
    uint32_t rate;
    double   actual_rate;
    uint32_t bandwidth;
    uint32_t actual_bandwidth;

    SoapySDR_log(SOAPY_SDR_TRACE, "getBandwidth");
    if (direction == SOAPY_SDR_RX)
    {
        if (skiq_read_rx_sample_rate_and_bandwidth(card, rx_hdl, &rate,
                                                   &actual_rate, &bandwidth,
                                                   &actual_bandwidth) != 0)
        {
            SoapySDR_logf(
                SOAPY_SDR_ERROR,
                "Failure: skiq_read_rx_sample_rate_and_bandwidth (card %d)",
                card);
        }
    }

    if (direction == SOAPY_SDR_TX)
    {
        if (skiq_read_tx_sample_rate_and_bandwidth(card, tx_hdl, &rate,
                                                   &actual_rate, &bandwidth,
                                                   &actual_bandwidth) != 0)
        {
            SoapySDR_logf(
                SOAPY_SDR_ERROR,
                "Failure: skiq_read_tx_sample_rate_and_bandwidth (card %d)",
                card);
        }
    }

    return bandwidth;
}

std::vector<double> SoapySidekiq::listBandwidths(const int    direction,
                                                 const size_t channel) const
{
    std::vector<double> bandwidths;
    SoapySDR_log(SOAPY_SDR_TRACE, "getBandwidth");
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

SoapySDR::ArgInfoList SoapySidekiq::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList setArgs;
    SoapySDR_logf(SOAPY_SDR_TRACE, "getSettingInfo");

    SoapySDR::ArgInfo settingArg;

    settingArg.key         = "iq_swap";
    settingArg.value       = "false";
    settingArg.name        = "I/Q Swap";
    settingArg.description = "I/Q Swap Mode";
    settingArg.type        = SoapySDR::ArgInfo::BOOL;
    setArgs.push_back(settingArg);

    settingArg.key         = "counter";
    settingArg.value       = "false";
    settingArg.name        = "counter";
    settingArg.description = "RF counter mode";
    settingArg.type        = SoapySDR::ArgInfo::BOOL;
    setArgs.push_back(settingArg);

    settingArg.key         = "log";
    settingArg.value       = "false";
    settingArg.name        = "log";
    settingArg.description = "Set the Log Level";
    settingArg.type        = SoapySDR::ArgInfo::BOOL;
    setArgs.push_back(settingArg);
    return setArgs;
}

void SoapySidekiq::writeSetting(const std::string &key,
                                const std::string &value)
{
    int status = 0;

    SoapySDR_logf(SOAPY_SDR_TRACE, "writeSetting");

    if (key == "log")
    {
        if (value == "trace")
        {
            SoapySDR::setLogLevel(SOAPY_SDR_TRACE);
            SoapySDR_logf(SOAPY_SDR_DEBUG, "Set Log Level to TRACE on card %d",
                          card);
        }
        else if (value == "debug")
        {
            SoapySDR::setLogLevel(SOAPY_SDR_DEBUG);
            SoapySDR_logf(SOAPY_SDR_DEBUG, "Set Log Level to DEBUG on card %d",
                          card);
        }
        else
        {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Invalid log level received %s",
                          value.c_str());
        }
    }
    else if (key == "iq_swap")
    {
        iq_swap = ((value == "true") ? true : false);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "I/Q swap: %s",
                      iq_swap ? "true" : "false");
    }
    else if (key == "counter")
    {
        if (value == "true")
        {
            status =
                skiq_write_rx_data_src(card, rx_hdl, skiq_data_src_counter);
            if (status != 0)
            {
                SoapySDR_logf(
                    SOAPY_SDR_ERROR,
                    "Failure in writing data src to counter status is %d",
                    status);
            }
            else
            {
                SoapySDR_log(SOAPY_SDR_DEBUG, "Set rx src to counter mode ");
                counter = true;
            }
        }
        else
        {
            status = skiq_write_rx_data_src(card, rx_hdl, skiq_data_src_iq);
            if (status != 0)
            {
                SoapySDR_logf(SOAPY_SDR_ERROR,
                              "Failure in writing data src to iq status is %d",
                              status);
            }
            else
            {
                SoapySDR_log(SOAPY_SDR_DEBUG, "Set rx src to normal mode ");
                counter = false;
            }
        }
    }
    else
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "Invalid key %s ", key.c_str());
    }
}

std::string SoapySidekiq::readSetting(const std::string &key) const
{
    SoapySDR_logf(SOAPY_SDR_TRACE, "readSetting");
    if (key == "iq_swap")
    {
        return iq_swap ? "true" : "false";
    }
    else if (key == "counter")
    {
        return counter ? "true" : "false";
    }
    else
    {
        SoapySDR_logf(SOAPY_SDR_WARNING, "Unknown setting '%s'", key.c_str());
    }
    return "";
}

/*******************************************************************
 * Time API
 ******************************************************************/

std::vector<std::string> SoapySidekiq::listTimeSources(void) const
{
    std::vector<std::string> result;
    SoapySDR_logf(SOAPY_SDR_TRACE, "listTimeSources");

    result.push_back("System");
    result.push_back("RF");

    return result;
}

void SoapySidekiq::setTimeSource(const std::string &source)
{
    SoapySDR_logf(SOAPY_SDR_TRACE, "setTimeSource, source is %s", source.c_str());

    timeSource = source;
    return;
}

std::string SoapySidekiq::getTimeSource(void) const
{
    SoapySDR_logf(SOAPY_SDR_TRACE, "getTimeSource");
    return timeSource;
}

bool SoapySidekiq::hasHardwareTime(const std::string &) const
{
    SoapySDR_logf(SOAPY_SDR_TRACE, "hasHardwareTime");
    return true;
}

long long SoapySidekiq::getHardwareTime(const std::string &) const
{
    //TODO
    SoapySDR_logf(SOAPY_SDR_TRACE, "getHardwareTime");
    return 0;
}

void SoapySidekiq::setHardwareTime(const long long timeNs, const std::string &what)
{
    //TODO
    SoapySDR_logf(SOAPY_SDR_TRACE, "setHardwareTime");
    if (what == "CMD") this->setCommandTime(timeNs, what);
}

void SoapySidekiq::setCommandTime(const long long, const std::string &)
{
    //TODO
    SoapySDR_logf(SOAPY_SDR_TRACE, "setCommandTime");
    return;
}
