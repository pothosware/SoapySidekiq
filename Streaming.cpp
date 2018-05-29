
#include "SoapySidekiq.hpp"
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>
#include <algorithm> //min
#include <climits> //SHRT_MAX
#include <cstring> // memcpy


std::vector<std::string> SoapySidekiq::getStreamFormats(const int direction, const size_t channel) const {
    std::vector<std::string> formats;
    formats.push_back(SOAPY_SDR_CS16);
    return formats;
}

std::string SoapySidekiq::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {
    return SoapySDR::Device::getNativeStreamFormat(direction, channel, fullScale);
}

SoapySDR::ArgInfoList SoapySidekiq::getStreamArgsInfo(const int direction, const size_t channel) const {

    SoapySDR::ArgInfoList streamArgs;

    SoapySDR::ArgInfo bufflenArg;
    bufflenArg.key = "bufflen";
    bufflenArg.value = std::to_string(DEFAULT_BUFFER_LENGTH);
    bufflenArg.name = "Buffer Size";
    bufflenArg.description = "Number of bytes per buffer, multiples of 512 only.";
    bufflenArg.units = "bytes";
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

    SoapySDR::ArgInfo asyncbuffsArg;
    asyncbuffsArg.key = "asyncBuffs";
    asyncbuffsArg.value = "0";
    asyncbuffsArg.name = "Async buffers";
    asyncbuffsArg.description = "Number of async usb buffers (advanced).";
    asyncbuffsArg.units = "buffers";
    asyncbuffsArg.type = SoapySDR::ArgInfo::INT;

    streamArgs.push_back(asyncbuffsArg);

    return streamArgs;
}

/*******************************************************************
 * Async thread work
 ******************************************************************/
/*
static void _rx_callback(unsigned char *buf, uint32_t len, void *ctx)
{
    //printf("_rx_callback\n");
    SoapySidekiq *self = (SoapySidekiq *)ctx;
    self->rx_callback(buf, len);
}

void SoapySidekiq::rx_async_operation(void)
{
    //printf("rx_async_operation\n");
    Sidekiq_read_async(dev, &_rx_callback, this, asyncBuffs, bufferLength);
    //printf("rx_async_operation done!\n");
}

void SoapySidekiq::rx_callback(unsigned char *buf, uint32_t len)
{
    //printf("_rx_callback %d _buf_head=%d, numBuffers=%d\n", len, _buf_head, _buf_tail);

    //overflow condition: the caller is not reading fast enough
    if (_buf_count == numBuffers)
    {
        _overflowEvent = true;
        return;
    }

    //copy into the buffer queue
    auto &buff = _buffs[_buf_tail];
    buff.resize(len);
    std::memcpy(buff.data(), buf, len);

    //increment the tail pointer
    _buf_tail = (_buf_tail + 1) % numBuffers;

    //increment buffers available under lock
    //to avoid race in acquireReadBuffer wait
    {
        std::lock_guard<std::mutex> lock(_buf_mutex);
        _buf_count++;

    }

    //notify readStream()
    _buf_cond.notify_one();
}
*/
/*******************************************************************
 * Stream API
 ******************************************************************/

SoapySDR::Stream *SoapySidekiq::setupStream(const int direction, const std::string &format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args)
{

    //check the channel configuration
    if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0))
    {
        throw std::runtime_error("setupStream invalid channel selection");
    }

    //check the format
    if (format == SOAPY_SDR_CS16)
    {
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16.");
    }
    else
    {
        throw std::runtime_error(
                "setupStream invalid format '" + format
                + "' -- Only CS16 is supported by SoapySidekiq module.");
    }


    bufferLength = DEFAULT_BUFFER_LENGTH;
    if (args.count("bufflen") != 0)
    {
        try
        {
            int bufferLength_in = std::stoi(args.at("bufflen"));
            if (bufferLength_in > 0)
            {
                bufferLength = bufferLength_in;
            }
        }
        catch (const std::invalid_argument &){}
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-SDR Using buffer length %d", bufferLength);

    numBuffers = DEFAULT_NUM_BUFFERS;
    if (args.count("buffers") != 0)
    {
        try
        {
            int numBuffers_in = std::stoi(args.at("buffers"));
            if (numBuffers_in > 0)
            {
                numBuffers = numBuffers_in;
            }
        }
        catch (const std::invalid_argument &){}
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-SDR Using %d buffers", numBuffers);


    //clear async fifo counts
    _buf_tail = 0;
    _buf_count = 0;
    _buf_head = 0;

    //allocate buffers
    _buffs.resize(numBuffers);
    for (auto &buff : _buffs) buff.reserve(bufferLength);
    for (auto &buff : _buffs) buff.resize(bufferLength);

    return (SoapySDR::Stream *) this;
}

void SoapySidekiq::closeStream(SoapySDR::Stream *stream)
{
    this->deactivateStream(stream, 0, 0);
    _buffs.clear();
}

size_t SoapySidekiq::getStreamMTU(SoapySDR::Stream *stream) const
{
    return bufferLength / BYTES_PER_SAMPLE;
}

int SoapySidekiq::activateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs, const size_t numElems)
{
    if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
    resetBuffer = true;
    bufferedElems = 0;

    /* begin streaming on the Rx interface */
    skiq_start_rx_streaming(card, rx_hdl);

    return 0;
}

int SoapySidekiq::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs)
{
    if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;

    skiq_stop_rx_streaming(card, rx_hdl);

    return 0;
}

int SoapySidekiq::readStream(SoapySDR::Stream *stream, void * const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs)
{
    //drop remainder buffer on reset
    if (resetBuffer and bufferedElems != 0)
    {
        bufferedElems = 0;
        this->releaseReadBuffer(stream, _currentHandle);
    }

    //this is the user's buffer for channel 0
    void *buff0 = buffs[0];

    //are elements left in the buffer? if not, do a new read.
    if (bufferedElems == 0)
    {
        int ret = this->acquireReadBuffer(stream, _currentHandle, (const void **)&_currentBuff, flags, timeNs, timeoutUs);
        if (ret < 0) return ret;
        bufferedElems = ret;
    }

    size_t returnedElems = std::min(bufferedElems, numElems);

    //convert into user's buff0

    int8_t *itarget = (int8_t *) buff0;
    if (iqSwap)
    {
        for (size_t i = 0; i < returnedElems; i++)
        {
            itarget[i * 2] = _currentBuff[i * 2 + 1]-128;
            itarget[i * 2 + 1] = _currentBuff[i * 2]-128;
        }
    }
    else
    {
        for (size_t i = 0; i < returnedElems; i++)
        {
            itarget[i * 2] = _currentBuff[i * 2]-128;
            itarget[i * 2 + 1] = _currentBuff[i * 2 + 1]-128;
        }
    }


    //bump variables for next call into readStream
    bufferedElems -= returnedElems;
    _currentBuff += returnedElems*BYTES_PER_SAMPLE;

    //return number of elements written to buff0
    if (bufferedElems != 0) flags |= SOAPY_SDR_MORE_FRAGMENTS;
    else this->releaseReadBuffer(stream, _currentHandle);
    return returnedElems;
}

/*******************************************************************
 * Direct buffer access API
 ******************************************************************/

size_t SoapySidekiq::getNumDirectAccessBuffers(SoapySDR::Stream *stream)
{
    return _buffs.size();
}

int SoapySidekiq::getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs)
{
    buffs[0] = (void *)_buffs[handle].data();
    return 0;
}

int SoapySidekiq::acquireReadBuffer(SoapySDR::Stream *stream, size_t &handle, const void **buffs, int &flags, long long &timeNs, const long timeoutUs)
{
    //reset is issued by various settings
    //to drain old data out of the queue
    if (resetBuffer)
    {
        //drain all buffers from the fifo
        _buf_head = (_buf_head + _buf_count.exchange(0)) % numBuffers;
        resetBuffer = false;
        _overflowEvent = false;
    }

    //handle overflow from the rx callback thread
    if (_overflowEvent)
    {
        //drain the old buffers from the fifo
        _buf_head = (_buf_head + _buf_count.exchange(0)) % numBuffers;
        _overflowEvent = false;
        SoapySDR::log(SOAPY_SDR_SSI, "O");
        return SOAPY_SDR_OVERFLOW;
    }

    //wait for a buffer to become available
    if (_buf_count == 0)
    {
        std::unique_lock <std::mutex> lock(_buf_mutex);
        _buf_cond.wait_for(lock, std::chrono::microseconds(timeoutUs), [this]{return _buf_count != 0;});
        if (_buf_count == 0) return SOAPY_SDR_TIMEOUT;
    }

    //extract handle and buffer
    handle = _buf_head;
    _buf_head = (_buf_head + 1) % numBuffers;
    buffs[0] = (void *)_buffs[handle].data();
    flags = 0;

    //return number available
    return _buffs[handle].size() / BYTES_PER_SAMPLE;
}

void SoapySidekiq::releaseReadBuffer(SoapySDR::Stream *stream, const size_t handle)
{
    //TODO this wont handle out of order releases
    _buf_count--;
}