# sx127x_ashm

A driver for the Semtech SX127X radio data modem
written in Python3 using the [farc](https://github.com/dwhall/farc)
hierarchical state machine framework
and meant to run on Linux on a Raspberry Pi 3
with a modified [Dragino LoRa GPS Hat](https://wiki.dragino.com/index.php?title=Lora/GPS_HAT).

The driver software is implemented as a state machine to manage
the SPI bus which operates the SX127X modem and a state machine to manage
the Raspberry Pi GPIO pins which read the SX127X DIOx pins.

This repository is designed to be a git submodule
so that it may be re-used by multiple projects.
