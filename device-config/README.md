This directory contains the device configuration, composed of an

- 8-byte DEV_EUI
- 8-byte APP_EUI (aka JOIN_EUI)
- 16-byte APP_KEY

This will be the device's "identity" in the LoraWAN network. Keep it constant for a single device, but also unique.

Run the following commands in this directory to generate the files:

```bash
head -c8 </dev/urandom|xxd -p -u > DEV_EUI
head -c8 </dev/urandom|xxd -p -u > APP_EUI
head -c16 </dev/urandom|xxd -p -u > APP_KEY

```