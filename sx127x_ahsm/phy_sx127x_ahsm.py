#!/usr/bin/env python3
"""
Copyright 2017 Dean Hall.  See LICENSE for details.

Physical Layer State Machine for SX127x device
- models desired SX127x device behavior
- SX127x device control via SPI bus
- establishes Transmit and Receive sequences
- responds to a handful of events (expected from Layer 2 (MAC))
- responds to GPIO events from the SX127x chip's DIOx pins
"""

import logging
import time

import farc

from . import phy_sx127x_spi


class SX127xSpiAhsm(farc.Ahsm):
    # Maximum amount of time to perform blocking sleep (seconds).
    # If a sleep time longer than this is requested,
    # the sleep time becomes this value.
    MAX_BLOCKING_TIME = 0.050 # secs

    # Margin time added to transmit time
    # to allow other nodes to enable their receiver
    TX_MARGIN = 0.005 # secs


    def __init__(self, spi_stngs, dflt_modem_stngs):
        super().__init__()
        self.spi_stngs = spi_stngs
        self.dflt_modem_stngs = dflt_modem_stngs


    @farc.Hsm.state
    def _initial(self, event):
        """Pseudostate: SX127xSpiAhsm:_initial
        """
        # self-signaling
        farc.Signal.register("_ALWAYS")

        # Outgoing
        farc.Signal.register("PHY_RXD_DATA")
        farc.Signal.register("PHY_TX_DONE")

        # Incoming
        farc.Signal.register("PHY_STDBY")
        farc.Signal.register("PHY_SET_LORA")

        # Incoming from higher layer
        farc.Framework.subscribe("PHY_SLEEP", self)
        farc.Framework.subscribe("PHY_CAD", self)
        farc.Framework.subscribe("PHY_RECEIVE", self)
        farc.Framework.subscribe("PHY_TRANSMIT", self)

        # Incoming from GPIO (SX127x's DIO pins)
        farc.Framework.subscribe("PHY_DIO0", self)
        farc.Framework.subscribe("PHY_DIO1", self)
        farc.Framework.subscribe("PHY_DIO2", self)
        farc.Framework.subscribe("PHY_DIO3", self)
        farc.Framework.subscribe("PHY_DIO4", self)
        farc.Framework.subscribe("PHY_DIO5", self)

        # A time event used for setting timeouts
        self.tm_evt = farc.TimeEvent("_PHY_SPI_TMOUT")

        return self.tran(SX127xSpiAhsm._initializing)


    @farc.Hsm.state
    def _initializing(self, event):
        """State: SX127xSpiAhsm:_initializing
        Opens, verifies and inits the SPI driver.
        Transitions to the _idling state if all is good;
        otherwise transitions to the _exiting state
        so the SPI driver is closed.
        """
        sig = event.signal
        if sig == farc.Signal.ENTRY:
            self.sx127x = phy_sx127x_spi.SX127xSpi(self.spi_stngs)
            self.tm_evt.post_in(self, 0.0)
            return self.handled(event)

        elif sig == farc.Signal._PHY_SPI_TMOUT:
            if self.sx127x.check_chip_ver():
                self.sx127x.init(self.dflt_modem_stngs)
                self.sx127x.set_pwr_cfg(boost=True)
                return self.tran(SX127xSpiAhsm._idling)

            logging.info("_initializing: no SX127x or SPI")
            self.tm_evt.post_in(self, 1.0)
            return self.handled(event)

        elif sig == farc.Signal.EXIT:
            self.tm_evt.disarm()
            return self.handled(event)

        return self.super(self.top)


    @farc.Hsm.state
    def _idling(self, event):
        """State: SX127xSpiAhsm:_idling
        """
        sig = event.signal
        if sig == farc.Signal.ENTRY:
            return self.handled(event)

        elif sig == farc.Signal.PHY_SLEEP:
            return self.tran(self.sleeping)

        elif sig == farc.Signal.PHY_SET_LORA:
            # TODO
            return self.handled(event)

        elif sig == farc.Signal.PHY_RECEIVE:
            self.rx_time = event.value[0]
            self.rx_freq = event.value[1]
            return self.tran(self._rx_prepping)

        elif sig == farc.Signal.PHY_TRANSMIT:
            self.tx_time = event.value[0]
            self.tx_freq = event.value[1]
            self.tx_data = event.value[2]
            return self.tran(self._tx_prepping)

        elif sig == farc.Signal.PHY_CAD:
            return self.tran(self.cad_ing)

        return self.super(self.top)


    @farc.Hsm.state
    def _working(self, event):
        """State SX127xSpiAhsm:_working
        This state provides a PHY_STDBY handler that returns the radio to stdby.
        """
        sig = event.signal
        if sig == farc.Signal.ENTRY:
            return self.handled(event)

        elif sig == farc.Signal.PHY_STDBY:
            self.sx127x.set_op_mode("stdby")
            return self.tran(self._idling)

        return self.super(self.top)


#### Receive chain
    @farc.Hsm.state
    def _rx_prepping(self, event):
        """State: SX127xSpiAhsm:_idling:_rx_prepping
        While still in radio's standby mode, get regs and FIFO ready for RX.
        If a positive rx_time is given, sleep (blocking) for a tiny amount.
        If rx_time is zero or less, receive immediately.
        Always transfer to the Receiving state.
        """
        sig = event.signal
        if sig == farc.Signal.ENTRY:

            # Enable only the RX interrupts (disable all others)
            self.sx127x.disable_lora_irqs()
            self.sx127x.enable_lora_irqs(phy_sx127x_spi.IRQFLAGS_RXTIMEOUT_MASK
                | phy_sx127x_spi.IRQFLAGS_RXDONE_MASK
                | phy_sx127x_spi.IRQFLAGS_PAYLOADCRCERROR_MASK
                | phy_sx127x_spi.IRQFLAGS_VALIDHEADER_MASK)

            # Prepare DIO0,1 to cause RxDone, RxTimeout, ValidHeader interrupts
            self.sx127x.set_dio_mapping(dio0=0, dio1=0, dio3=1)
            self.sx127x.set_lora_rx_fifo(self.dflt_modem_stngs["modulation_stngs"]["rx_base_ptr"])
            self.sx127x.set_lora_rx_freq(self.rx_freq)

            # Reminder pattern
            self.post_fifo(farc.Event(farc.Signal._ALWAYS, None))
            return self.handled(event)

        elif sig == farc.Signal._ALWAYS:
            if self.rx_time > 0:
                tiny_sleep = self.rx_time - farc.Framework._event_loop.time()
                if 0.0 < tiny_sleep < SX127xSpiAhsm.MAX_BLOCKING_TIME:
                    time.sleep(tiny_sleep)
            return self.tran(SX127xSpiAhsm._listening)

        return self.super(self._idling)


    @farc.Hsm.state
    def _listening(self, event):
        """State SX127xSpiAhsm:_working:_listening
        If the rx_time is less than zero, listen continuously;
        the caller must establish a way to end the continuous mode.
        """
        sig = event.signal
        if sig == farc.Signal.ENTRY:
            self.hdr_time = 0
            if self.rx_time < 0:
                self.sx127x.set_op_mode("rxcont")
            else:
                self.sx127x.set_op_mode("rxonce")
            return self.handled(event)

        elif sig == farc.Signal.PHY_DIO0: # RX_DONE
            if self.sx127x.check_lora_rx_flags():
                payld, rssi, snr = self.sx127x.get_lora_rxd()
                pkt_data = (self.hdr_time, payld, rssi, snr)
                farc.Framework.publish(farc.Event(farc.Signal.PHY_RXD_DATA, pkt_data))
            else:
                # TODO: crc error stats
                logging.info("rx CRC error")

            return self.tran(SX127xSpiAhsm._idling)

        elif sig == farc.Signal.PHY_DIO1: # RX_TIMEOUT
            self.sx127x.clear_lora_irqs(phy_sx127x_spi.IRQFLAGS_RXTIMEOUT_MASK)
            return self.tran(SX127xSpiAhsm._idling)

        elif sig == farc.Signal.PHY_DIO3: # ValidHeader
            self.hdr_time = event.value
            self.sx127x.clear_lora_irqs(phy_sx127x_spi.IRQFLAGS_VALIDHEADER_MASK)
            return self.tran(SX127xSpiAhsm._receiving)

        # We haven't received anything yet
        # and a request to Transmit arrives,
        # cancel the listening and go do the Transmit
        elif sig == farc.Signal.PHY_TRANSMIT:
            self.sx127x.set_op_mode("stdby")
            self.tx_time = event.value[0]
            self.tx_freq = event.value[1]
            self.tx_data = event.value[2]
            return self.tran(self._tx_prepping)

        return self.super(self._working)


    @farc.Hsm.state
    def _receiving(self, event):
        """State SX127xSpiAhsm:_working:_listening:_receiving
        NOTE: This state should not be confused with
        the MAC layer state of the same name
        """
        sig = event.signal
        if sig == farc.Signal.ENTRY:
            pass

        elif sig == farc.Signal.PHY_TRANSMIT:
            # TODO: put the pkt in the tx queue
            pass

        return self.super(self._listening)


#### Transmit chain
    @farc.Hsm.state
    def _tx_prepping(self, event):
        """State: SX127xSpiAhsm:_idling:_tx_prepping
        """
        sig = event.signal
        if sig == farc.Signal.ENTRY:

            # Enable only the TX interrupts (disable all others)
            self.sx127x.disable_lora_irqs()
            self.sx127x.enable_lora_irqs(phy_sx127x_spi.IRQFLAGS_TXDONE_MASK)
            self.sx127x.clear_lora_irqs(phy_sx127x_spi.IRQFLAGS_TXDONE_MASK)

            # Set DIO, TX/FIFO_PTR, FIFO and freq in prep for transmit
            self.sx127x.set_dio_mapping(dio0=1)
            self.sx127x.set_lora_fifo_ptr()
            self.sx127x.set_tx_data(self.tx_data)
            self.sx127x.set_tx_freq(self.tx_freq)

            # Reminder pattern
            self.post_fifo(farc.Event(farc.Signal._ALWAYS, None))
            return self.handled(event)

        elif sig == farc.Signal._ALWAYS:
            if self.tx_time > 0:
                # Calculate precise sleep time and apply a TX margin
                # to allow receivers time to get ready
                tiny_sleep = self.tx_time - farc.Framework._event_loop.time()
                tiny_sleep += SX127xSpiAhsm.TX_MARGIN

                # If TX time has passed, don't sleep
                # Else use sleep to get ~1ms precision
                # Cap sleep at 50ms so we don't block for too long
                if 0.0 < tiny_sleep: # because MAC layer uses 40ms PREP time
                    if tiny_sleep > SX127xSpiAhsm.MAX_BLOCKING_TIME:
                        tiny_sleep = SX127xSpiAhsm.MAX_BLOCKING_TIME
                    time.sleep(tiny_sleep)
            return self.tran(SX127xSpiAhsm._transmitting)

        return self.super(self._idling)


    @farc.Hsm.state
    def _transmitting(self, event):
        """State: SX127xSpiAhsm:_working:_transmitting
        """
        sig = event.signal
        if sig == farc.Signal.ENTRY:
            logging.info("tx             %f", farc.Framework._event_loop.time())
            self.sx127x.set_op_mode("tx")
            self.tm_evt.post_in(self, 1.0) # TODO: make time scale with datarate
            return self.handled(event)

        elif sig == farc.Signal.PHY_DIO0: # TX_DONE
            return self.tran(SX127xSpiAhsm._idling)

        elif sig == farc.Signal._PHY_SPI_TMOUT: # software timeout
            self.sx127x.set_op_mode("stdby")
            return self.tran(SX127xSpiAhsm._idling)

        elif sig == farc.Signal.EXIT:
            self.tm_evt.disarm()
            farc.Framework.publish(farc.Event(farc.Signal.PHY_TX_DONE, None))
            return self.handled(event)

        return self.super(self._working)


    @farc.Hsm.state
    def _exiting(self, event):
        """State: SX127xSpiAhsm:_exiting
        """
        sig = event.signal
        if sig == farc.Signal.ENTRY:
            logging.info("_exiting")
            self.sx127x.close()
            return self.handled(event)

        return self.super(self.top)
