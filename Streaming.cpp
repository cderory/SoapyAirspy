/*
 * The MIT License (MIT)
 * 
 * Copyright (c) 2015 Charles J. Cliffe

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "SoapyAirspy.hpp"
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/ConverterRegistry.hpp>
#include <algorithm> //min
#include <climits> //SHRT_MAX
#include <cstring> // memcpy
#include <chrono>
#include <cstdint>

#define SOAPY_NATIVE_FORMAT SOAPY_SDR_CS16

std::vector<std::string> SoapyAirspy::getStreamFormats(const int direction, const size_t channel) const {

    std::vector<std::string> formats;

    if(direction != SOAPY_SDR_RX) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "SoapyAirspy::getStreamFormats(%d, %d) - direction must be RX", direction, channel);
        return formats;
    }

    for (const auto &target : SoapySDR::ConverterRegistry::listTargetFormats(SOAPY_NATIVE_FORMAT)) {
        formats.push_back(target);
    }

    return formats;
}

std::string SoapyAirspy::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {
    // TODO maybe use constant? 
    fullScale = 32767;
    return SOAPY_SDR_CS16;
}

SoapySDR::ArgInfoList SoapyAirspy::getStreamArgsInfo(const int direction,
                                                     const size_t channel) const {
    SoapySDR::ArgInfoList streamArgs;
    return streamArgs;
}

/*******************************************************************
 * Async thread work
 ******************************************************************/

static int rx_callback_(airspy_transfer *transfer)
{
    SoapyAirspy *self = (SoapyAirspy *)transfer->ctx;
    return self->rx_callback(transfer);
}

int SoapyAirspy::rx_callback(airspy_transfer *transfer)
{

    const uint32_t timeout_us = 500000;

    // SoapySDR::logf(SOAPY_SDR_INFO, "rx_callback %ld %d",
    //                transfer->sample_count, transfer->sample_type);

    const auto written = ringbuffer_.write_at_least<std::complex<int16_t>>
        (transfer->sample_count,
         std::chrono::microseconds(timeout_us),
         [&](std::complex<int16_t>* begin, [[maybe_unused]] const uint32_t available) {
             // Copy samples to ringbufer

             std::memcpy(begin,
                         transfer->samples,
                         transfer->sample_count * sizeof(std::complex<int16_t>));


             return transfer->sample_count;
         });

    if(written < 0) {
        SoapySDR::logf(SOAPY_SDR_INFO, "SoapyAirspy::rx_callback: ringbuffer write timeout");
        return 0;
    }

    return 0; // anything else is an error.
}


/*******************************************************************
 * Stream API
 ******************************************************************/

SoapySDR::Stream *SoapyAirspy::setupStream(const int direction,
                                           const std::string &format,
                                           const std::vector<size_t> &channels,
                                           const SoapySDR::Kwargs &args) {
    int ret;

    if(direction != SOAPY_SDR_RX) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "SoapyAirspy::setupStream(%d, %s, %d, %d) - direction must be RX", direction, format.c_str(), channels.size(), args.size());
        return nullptr;
    }

    // Check the channel configuration
    if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0)) {
        throw std::runtime_error("setupStream invalid channel selection");
    }

    std::vector<std::string> sources = SoapySDR::ConverterRegistry::listSourceFormats(format);

    if (std::find(sources.begin(), sources.end(), SOAPY_NATIVE_FORMAT) == sources.end()) {
        throw std::runtime_error(
                "setupStream invalid format '" + format + "'.");
    }

    // Find converter functinon
    converterFunction_ = SoapySDR::ConverterRegistry::getFunction(SOAPY_NATIVE_FORMAT, format, SoapySDR::ConverterRegistry::GENERIC);

    ret = airspy_set_sample_type(dev_, AIRSPY_SAMPLE_INT16_IQ);
    if(ret != AIRSPY_SUCCESS) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "airspy_set_sample_type() failed: %s (%d)", airspy_error_name((airspy_error)ret), ret);
        return nullptr;
    }

    SoapySDR::logf(SOAPY_SDR_INFO, "setupStream: format=%s", format.c_str());

    return (SoapySDR::Stream *) this;
}

void SoapyAirspy::closeStream(SoapySDR::Stream *stream)
{
    //TODO
    ringbuffer_.clear();
}

size_t SoapyAirspy::getStreamMTU(SoapySDR::Stream *stream) const
{
    // TODO
    return 65536;
}

int SoapyAirspy::activateStream(SoapySDR::Stream *stream,
                                const int flags,
                                const long long timeNs,
                                const size_t numElems)
{
    int ret;

    if (flags != 0) { return SOAPY_SDR_NOT_SUPPORTED; }

    ringbuffer_.clear();

    ret = airspy_start_rx(dev_, rx_callback_, this);
    if(ret != AIRSPY_SUCCESS) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "airspy_start_rx() failed: %d", ret);
        return SOAPY_SDR_STREAM_ERROR;
    }

    return 0;
}

int SoapyAirspy::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs)
{

    int ret;

    SoapySDR::logf(SOAPY_SDR_DEBUG, "deactivateStream: flags=%d, timeNs=%lld", flags, timeNs);

    // No flags supported
    if (flags != 0) { return SOAPY_SDR_NOT_SUPPORTED; }

    ret = airspy_stop_rx(dev_);

    if (ret != AIRSPY_SUCCESS) {
        return SOAPY_SDR_STREAM_ERROR;
    }

    return 0;
}

int SoapyAirspy::readStream(SoapySDR::Stream *stream,
                            void * const *buffs,
                            const size_t numElems,
                            int &flags,
                            long long &timeNs,
                            const long timeoutUs) {

    if(flags != 0) { return SOAPY_SDR_NOT_SUPPORTED; }

    const auto to_convert = std::min(numElems, getStreamMTU(stream));

    SoapySDR::logf(SOAPY_SDR_DEBUG, "readStream: numElems=%d, timeoutUs=%ld, topcopy=%ld", numElems, timeoutUs, to_convert);

    const auto converted = ringbuffer_.read_at_least<std::complex<int16_t>>
        (to_convert,
         std::chrono::microseconds(timeoutUs),
         [&](const std::complex<int16_t>* begin, [[maybe_unused]] const uint32_t available) {
             // Convert samples to output buffer
             converterFunction_(begin,
                                buffs[0],
                                to_convert,
                                1.0);

             // Consume from ringbuffer
             return to_convert;
         });

    if(converted < 0) {
        SoapySDR::logf(SOAPY_SDR_DEBUG, "readStream: ringbuffer read timeout");
        return SOAPY_SDR_TIMEOUT;
    }

    // TODO
    timeNs = 0;

    return converted;
}
