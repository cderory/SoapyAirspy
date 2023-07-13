# Soapy SDR plugin for Airspy R2 and Mini

## Building

Make sure you have SoapySDR installed and do something like this:

    mkdir build
    cd build
    cmake ..
    make
    sudo install

## Using

Use this device string in GQRX:

    soapy=0,driver=airspy

More settings are available:

    soapy=0,driver=airspy,biastee=true|false,gains=linearity|sensitivity|manual

## This branch (v2)

**This branch is work in progress**.

The code is significantly simpler than the previous version and most
likely it has better performance. Quite a few bugs have been fixed,
and some new ones have probably been introduced. Please help me find
them! :-)

I have tested this with GQRX with good results.

## Support me

Please consider sponsoring me on GitHub if you enjoy this
work. Everything I earn through donations will go to families and
children of Ukraine who have lost their homes because of the war.

## Dependencies

* SoapySDR - https://github.com/pothosware/SoapySDR/wiki
* libairspy - https://github.com/airspy/host/wiki

## Documentation

* https://github.com/pothosware/SoapyAirspy/wiki
