## This project is deprecated

TL;DR: switching to [phy_sx127x](https://www.github.com/dwhall/phy_sx127x)

This project was an exploration of creating a network stack using hierarchical statecharts.
While I was able to get things to work, this project suffered from two things: inexperience and overengineering.
I was new to software design with statecharts and I mistakenly started using Events in place of methods.
This led to too many Events and complication due to trying to parse the Events' values into arguments.
The statemachine design I came up with was also overblown.  I was trying to create a TDMA system at the time
and that led to all sorts of special situations that I tried to solve with extra states.

After switching gears to a CSMA system and attempting an implementation in C on a microcontroller,
I designed a new, much simpler state machine.  From here on out, I will be workin on [phy_sx127x](https://www.github.com/dwhall/phy_sx127x)
as the PHY layer for the HeyMac project on RasPi.

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
