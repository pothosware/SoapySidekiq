# Soapy SDR module for Epiq Solutions Sidekiq

## Dependencies

* SoapySDR — https://github.com/pothosware/SoapySDR/wiki
* Sidekiq SDK — https://epiqsolutions.com/rf-transceiver/sidekiq/
* `libtirpc` (Sun RPC; required since glibc 2.32). On Debian/Ubuntu:
  `sudo apt install libtirpc-dev`
* `pkg-config` (recommended, not required)

## Building

Point CMake at the unpacked Sidekiq SDK:

```sh
cmake -S . -B build -DSIDEKIQ_SDK_ROOT=/path/to/sidekiq_sdk_current
cmake --build build
```

The build auto-detects the host architecture (x86_64, aarch64) and selects
the matching `libsidekiq__<variant>.a`. For other targets, set
`-DSIDEKIQ_VARIANT=<variant>` (e.g. `z3u`, `msiq-x40`) to match a file in
`<SDK>/lib/libsidekiq__<variant>.a`.

Transitive deps (`libtirpc`, `libusb-1.0`, `glib-2.0`) are resolved in this
order: `pkg-config`, then `find_library` (including `/usr/lib/epiq/`),
then the SDK's bundled archives under `lib/support/<variant>/`.

## Install and verify

```sh
sudo cmake --install build
SoapySDRUtil --info                     # confirms the module loads
SoapySDRUtil --find                     # lists connected Sidekiq devices
SoapySDRUtil --probe="driver=sidekiq"   # full capability dump
```

## Documentation

* https://github.com/pothosware/SoapySidekiq/wiki

## Licensing information

* https://github.com/pothosware/SoapySidekiq/blob/master/LICENSE
