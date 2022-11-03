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

SoapyAirspy::SoapyAirspy(const SoapySDR::Kwargs &args)
    : dev_(nullptr),
      ringbuffer_(21) // fits 8 buffers
{

    int ret;

    std::stringstream serialstr;
    serialstr.str("");

    if (args.count("serial") != 0)
    {
        try {
            serial_ = std::stoull(args.at("serial"), nullptr, 16);
        } catch (const std::invalid_argument &) {
            throw std::runtime_error("serial is not a hex number");
        } catch (const std::out_of_range &) {
            throw std::runtime_error("serial value of out range");
        }
        serialstr << std::hex << serial_;

        ret = airspy_open_sn(&dev_, serial_);
        if (ret != AIRSPY_SUCCESS) {
            // TODO: add ret to error message
            throw std::runtime_error("Unable to open AirSpy device with serial " + serialstr.str());
        }
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Found AirSpy device: serial = %16Lx", serial_);
    }
    else
    {
        ret = airspy_open(&dev_);
        if (ret != AIRSPY_SUCCESS) {
            throw std::runtime_error("Unable to open AirSpy device");
        }
    }

    //apply arguments to settings when they match
    for (const auto &info : this->getSettingInfo())
    {
        const auto it = args.find(info.key);
        if (it != args.end()) this->writeSetting(it->first, it->second);
    }
}

SoapyAirspy::~SoapyAirspy(void)
{
    if(dev_ != nullptr) {
        airspy_close(dev_);
    }
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyAirspy::getDriverKey(void) const
{
    return "Airspy";
}

std::string SoapyAirspy::getHardwareKey(void) const
{
    return "Airspy";
}

SoapySDR::Kwargs SoapyAirspy::getHardwareInfo(void) const
{
    //key/value pairs for any useful information
    //this also gets printed in --probe
    SoapySDR::Kwargs args;

    std::stringstream serialstr;
    serialstr.str("");
    serialstr << std::hex << serial_;
    args["serial"] = serialstr.str();

    return args;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyAirspy::getNumChannels(const int dir) const
{
    return (dir == SOAPY_SDR_RX) ? 1 : 0;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyAirspy::listAntennas(const int direction, const size_t channel) const
{
    std::vector<std::string> antennas;

    antennas.push_back("RX");

    return antennas;
}

void SoapyAirspy::setAntenna(const int direction, const size_t channel, const std::string &name)
{
    // TODO
}

std::string SoapyAirspy::getAntenna(const int direction, const size_t channel) const
{
    return "RX";
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyAirspy::hasDCOffsetMode(const int direction, const size_t channel) const
{
    return false;
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyAirspy::listGains(const int direction, const size_t channel) const
{
    //list available gain elements,
    //the functions below have a "name" parameter
    std::vector<std::string> results;

    results.push_back("LNA");
    results.push_back("MIX");
    results.push_back("VGA");

    return results;
}

bool SoapyAirspy::hasGainMode(const int direction, const size_t channel) const
{
    return true;
}

void SoapyAirspy::setGainMode(const int direction, const size_t channel, const bool automatic)
{
    int ret;

    if(direction != SOAPY_SDR_RX) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "setGainMode: invalid direction");
    }

    SoapySDR_logf(SOAPY_SDR_DEBUG, "setGainMode(%d)", automatic);

    if(agcMode_ != automatic) {
        // LNA
        ret = airspy_set_lna_agc(dev_, automatic);
        if(ret != AIRSPY_SUCCESS) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "airspy_set_lna_agc() failed: %d", ret);
            return;
        }
        // Mixer
        ret = airspy_set_mixer_agc(dev_, automatic);
        if(ret != AIRSPY_SUCCESS) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "airspy_set_mixer_agc() failed: %d", ret);
            return;
        }
        // Store
        agcMode_ = automatic;
    }
}

bool SoapyAirspy::getGainMode(const int direction, const size_t channel) const
{
    return agcMode_;
}

void SoapyAirspy::setGain(const int direction, const size_t channel, const double value)
{
    //set the overall gain by distributing it across available gain elements
    //OR delete this function to use SoapySDR's default gain distribution algorithm...
    SoapySDR::Device::setGain(direction, channel, value);
}

void SoapyAirspy::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{

    int ret;

    // TODO: fix casts
    if (name == "LNA")
    {
        uint8_t lnaGain = static_cast<uint8_t>(value);
        ret = airspy_set_lna_gain(dev_, lnaGain);
        if(ret != AIRSPY_SUCCESS) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "airspy_set_lna_gain() failed: %d", ret);
            return;
        }
        // Success
        lnaGain_ = lnaGain;
    }
    else if (name == "MIX")
    {
        uint8_t mixerGain = uint8_t(value);
        ret = airspy_set_mixer_gain(dev_, mixerGain);
        if(ret != AIRSPY_SUCCESS) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "airspy_set_mixer_gain() failed: %d", ret);
            return;
        }
        // Success
        mixerGain_ = mixerGain;
    }
    else if (name == "VGA")
    {
        uint8_t vgaGain = uint8_t(value);
        ret = airspy_set_vga_gain(dev_, vgaGain_);
        if(ret != AIRSPY_SUCCESS) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "airspy_set_vga_gain() failed: %d", ret);
            return;
        }
        // Success
        vgaGain_ = vgaGain;
    }
    else
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "setGain(%s) failed: unknown gain", name.c_str());
        return;
    }
}

double SoapyAirspy::getGain(const int direction, const size_t channel, const std::string &name) const
{
    if (name == "LNA")
    {
        return lnaGain_;
    }
    else if (name == "MIX")
    {
        return mixerGain_;
    }
    else if(name == "VGA")
    {
        return vgaGain_;
    }
    else
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "getGain() unknown gain name: %s", name.c_str());
        return 0;
    }
}

SoapySDR::Range SoapyAirspy::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    if (name == "LNA" || name == "MIX" || name == "VGA") {
        return SoapySDR::Range(0, 15);
    }

    return SoapySDR::Range(0, 15);
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyAirspy::setFrequency(const int direction,
                               const size_t channel,
                               const std::string &name,
                               const double frequency,
                               const SoapySDR::Kwargs &args)
{
    if(direction != SOAPY_SDR_RX) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "setFrequency() direction must be RX");
        return;
    }

    if(name == "RF") {
        uint32_t centerFrequency = static_cast<uint32_t>(frequency);
        // Only set if changed
        if(centerFrequency_ != centerFrequency) {
            int ret = airspy_set_freq(dev_, centerFrequency);
            if(ret != AIRSPY_SUCCESS) {
                SoapySDR_logf(SOAPY_SDR_ERROR, "airspy_set_freq() failed: %d", ret);
                return;
            }
            // Success
            centerFrequency_ = centerFrequency;
        }
    }
}

double SoapyAirspy::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
    if(direction != SOAPY_SDR_RX) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "getFrequency() direction must be RX");
        return 0;
    }

    if (name == "RF") {
        return (double) centerFrequency_;
    }
    else {
        SoapySDR_logf(SOAPY_SDR_ERROR, "getFrequency() unknown name: %s", name.c_str());
        return 0;
    }
}

std::vector<std::string> SoapyAirspy::listFrequencies(const int direction, const size_t channel) const
{
    std::vector<std::string> names;
    names.push_back("RF");
    return names;
}

SoapySDR::RangeList SoapyAirspy::getFrequencyRange(
        const int direction,
        const size_t channel,
        const std::string &name) const
{
    SoapySDR::RangeList results;
    if (name == "RF")
    {
        results.push_back(SoapySDR::Range(24000000, 1800000000));
    }
    return results;
}

SoapySDR::ArgInfoList SoapyAirspy::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
    SoapySDR::ArgInfoList freqArgs;

    // TODO: frequency arguments

    return freqArgs;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapyAirspy::setSampleRate(const int direction, const size_t channel, const double rate)
{
    int ret;

    if(direction != SOAPY_SDR_RX) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "setSampleRate() direction must be RX");
        return;
    }

    uint32_t sampleRate = static_cast<uint32_t>(rate);

    if(sampleRate != sampleRate_) {
        ret = airspy_set_samplerate(dev_, sampleRate);
        if(ret != AIRSPY_SUCCESS) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "airspy_set_samplerate() failed: %d", ret);
            return;
        }
        // Success
        sampleRate_ = sampleRate;
    }
}

double SoapyAirspy::getSampleRate(const int direction, const size_t channel) const
{
    if(direction != SOAPY_SDR_RX) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "getSampleRate() direction must be RX");
        return 0;
    }

    return sampleRate_;
}

std::vector<double> SoapyAirspy::listSampleRates(const int direction, const size_t channel) const
{
    std::vector<double> results;

    int ret;
    uint32_t numRates;
    std::vector<uint32_t> samplerates;

    ret = airspy_get_samplerates(dev_, &numRates, 0);
    if(ret != AIRSPY_SUCCESS) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "airspy_get_samplerates() failed: %d", ret);
        return results;
    }
    // Success
    samplerates.resize(numRates);

    ret = airspy_get_samplerates(dev_, samplerates.data(), numRates);
    if(ret != AIRSPY_SUCCESS) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "airspy_get_samplerates() failed: %d", ret);
        return results;
    }

    // Make sure it's sorted
    std::sort(samplerates.begin(), samplerates.end());
    for (const auto& samplerate : samplerates) {
        results.push_back(samplerate);
    }

    return results;
}

void SoapyAirspy::setBandwidth(const int direction, const size_t channel, const double bw)
{
    if(direction != SOAPY_SDR_RX or channel != 0) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "setBandwidth() invalid direction or channel");
        return;
    }

    // TODO: not sure about default implementation.
    SoapySDR::Device::setBandwidth(direction, channel, bw);
}

double SoapyAirspy::getBandwidth(const int direction, const size_t channel) const
{
    if(direction != SOAPY_SDR_RX or channel != 0) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "getBandwidth() invalid direction or channel");
        return 0;
    }

    // TODO: again not sure about default implementation.
    return SoapySDR::Device::getBandwidth(direction, channel);
}

std::vector<double> SoapyAirspy::listBandwidths(const int direction, const size_t channel) const
{
    std::vector<double> results;
    return results;
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyAirspy::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList setArgs;

    // // Bias-T
    // SoapySDR::ArgInfo biasOffsetArg;
    // biasOffsetArg.key = "biastee";
    // biasOffsetArg.value = "false";
    // biasOffsetArg.name = "Bias tee";
    // biasOffsetArg.description = "Enable the 4.5v DC Bias tee to power SpyVerter / LNA / etc. via antenna connection.";
    // biasOffsetArg.type = SoapySDR::ArgInfo::BOOL;

    // setArgs.push_back(biasOffsetArg);

    // // bitpack
    // SoapySDR::ArgInfo bitPackingArg;
    // bitPackingArg.key = "bitpack";
    // bitPackingArg.value = "false";
    // bitPackingArg.name = "Bit Pack";
    // bitPackingArg.description = "Enable packing 4 12-bit samples into 3 16-bit words for 25% less USB trafic.";
    // bitPackingArg.type = SoapySDR::ArgInfo::BOOL;

    // setArgs.push_back(bitPackingArg);

    return setArgs;
}

void SoapyAirspy::writeSetting(const std::string &key, const std::string &value)
{
    // TODO

    // if (key == "biastee") {
    //     bool enable = (value == "true");
    //     rfBias = enable;

    //     airspy_set_rf_bias(dev, enable);
    // }

    //  if (key == "bitpack") {
    //     bool enable = (value == "true");
    //     bitPack = enable;

    //     airspy_set_packing(dev, enable);
    // }
}

std::string SoapyAirspy::readSetting(const std::string &key) const
{
    // TODO.

    // if (key == "biastee") {
    //     return rfBias?"true":"false";
    // }
    // if (key == "bitpack") {
    //     return bitPack?"true":"false";
    // }

    // // SoapySDR_logf(SOAPY_SDR_WARNING, "Unknown setting '%s'", key.c_str());
    return "";
}
