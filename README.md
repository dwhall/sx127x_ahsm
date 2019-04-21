# sx127x_ahsm

A driver for the Semtech SX127X radio data modem
written in Python3 using the [farc](https://github.com/dwhall/farc)
hierarchical state machine framework
and meant to run on Linux on a Raspberry Pi 3
with a modified [Dragino LoRa GPS Hat](https://wiki.dragino.com/index.php?title=Lora/GPS_HAT).

The driver software is implemented as two state machines.
One state machine manages the SPI bus which operates the SX127X modem.
The other state machine manages the Raspberry Pi GPIO pins
which react to the SX127X DIOx pins.

This repository is designed to be a git submodule
so that it may be re-used by multiple projects.
