# Zephyr OS driver for TI's cc1101 Sub-Ghz transceiver

This project implements a Zephyr OS driver for use of Texas Instrument's cc1101 sub-Ghz transceiver chip.

## Cloning and preparing the workspace

The project has been structured as an out-of-tree driver, and for this reason includes both the driver itself and a sample application.

It has been structured following the example application from zephir-RTOS project that can be found [here](https://github.com/zephyrproject-rtos/example-application).

To initialize the workspace, simply clone the repo using west, then update the zephyr tree:

```bash
$ west init -m https://github.com/studiofuga/cc1101 --mr main cc1101
$ cd cc1101
$ west update
```

## Building

Once everything has been setup, just use west to build the sample app:

```bash
$ cd cc1101
$ west build -b esp32 app
```

