examples/nordic_softdevice_ble
==============================

This application demonstrates how to use the Nordic Semi nRF52
SoftDevice to implement a GATT peripheral.  The example was written
against Nordic SDK 15.0.0 and may or may not work with later SDK
versions.  Based on the way Nordic has been developing their SDK in
the past, it is quite likely that this example will stop working at
some point, probably with SDK 17 or 18.

The example has been tested with Nordic nRF52DK, but it should work
also with nRF52840DK.

To test, you need a BLE central.  For example, you can run
[Nordic nRF Connect](https://www.nordicsemi.com/eng/Products/Nordic-mobile-Apps/nRF-Connect-for-Mobile)
on your iPhone or Android.  Note that the iOS BLE stack is more
picky than then Android one.  Therefore it is quite easy to break the
iOS functionality and render nRF connect on an iPhone nonfunctional
while testing with Anrdoid will continue to work.  At this writing, we
are unaware of any good documentation about the differences, though.

Usage
=====

1. Build, flash and start the application:

```
   export BOARD=nrf52dk
   make
   make flash
   make term
```

The `term` make target starts a terminal emulator for your board. It
connects to a default port so you can see the debug output.  Usually
the port is `/dev/ttyUSB0` in Linux or `/dev/tty.usbmodem*` in Mac OS X.
If your port is named differently, the `PORT=/dev/yourport` variable
can be used to override this.

2. Start nRF connect or similar

Let nRF connect to scan for the nearby Bluetooth devices.  Select one
named according to your compiled example; search for `DEVICE_NAME` in
`main.c`.  The default name is `nRFSDex`.

When connected to the device, you should see a custom service identifier
`B131ABCD-7195-142B-E012-0808817F198D`.  This is basically a random
128-bit UUID.  If you are building a service of your own, you should
generate your own UUID and discard the default one, see
[ITU-T UUID generation page](https://www.itu.int/en/ITU-T/asn1/Pages/UUID/uuids.aspx).

Inside the service, you will see two custom Characteristics:
`B131`*`BBD0`*`-7195-142B-E012-0808817F198D` and
`B131`*`BBCF`*`-7195-142B-E012-0808817F198D`.  The former is used to
read the state of the push buttons on the nRF52DK, the latter to set
two of the LEDs there.  (The first two LEDs are used by default to
indicate that the demo has booted and whether it is service a
connected central or not.)
