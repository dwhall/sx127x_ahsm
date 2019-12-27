#!/usr/bin/env python3
"""
Copyright 2016 Dean Hall.  See LICENSE for details.
"""


import collections
import logging

try:
    import spidev
except:
    from . import mock_spidev as spidev

from . import phy_sx127x_stngs


SPI_CLK_MIN =   100000 # arbitrary (slowish)
SPI_CLK_MAX = 20000000
OSC_FREQ = 32e6
INV_OSC_FREQ = 1.0 / OSC_FREQ

# The SX127x's Version register value
CHIP_VERSION = 18

# Set the MSb of the register address to write to it
WRITE_BIT = 0x80

# SX127x register addresses
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_CARRIER_FREQ = 0x06
REG_PA_CFG = 0x09

# LoRa modem register addresses
REG_FIFO_PTR = 0x0D
REG_FIFO_TX_BASE_PTR = 0x0E
REG_FIFO_RX_BASE_PTR = 0x0F
REG_RX_CURRENT_ADDR = 0x10
REG_IRQ_MASK = 0x11
REG_IRQ_FLAGS = 0x12
REG_RX_NUM_BYTES = 0x13
REG_RX_HDR_CNT = 0x14
REG_RX_PKT_CNT = 0x16
REG_LORA_MODEM_STATUS = 0x18
REG_PKT_SNR = 0x19
REG_LATEST_RXD_PKT_RSSI = 0x1A
REG_CURRENT_RSSI = 0x1B
REG_MODEM_CFG_1 = 0x1D
REG_MODEM_CFG_2 = 0x1E # (0x1F is LSB)
REG_PREAMBLE_LEN = 0x20 # MSB (0x21 is LSB)
REG_PAYLD_LEN = 0x22
REG_PAYLD_MAX = 0x23
REG_RX_BYTE_ADDR=0x25
REG_MODEM_CFG_3 = 0x26
REG_TEMP = 0x3C
REG_SYNC_WORD = 0x39
REG_DIO_MAPPING1 = 0x40
REG_DIO_MAPPING2 = 0x41
REG_VERSION = 0x42

# REG_IRQ_FLAGS bit definitions
IRQFLAGS_RXTIMEOUT_MASK          = 0x80
IRQFLAGS_RXDONE_MASK             = 0x40
IRQFLAGS_PAYLOADCRCERROR_MASK    = 0x20
IRQFLAGS_VALIDHEADER_MASK        = 0x10
IRQFLAGS_TXDONE_MASK             = 0x08
IRQFLAGS_CADDONE_MASK            = 0x04
IRQFLAGS_FHSSCHANGEDCHANNEL_MASK = 0x02
IRQFLAGS_CADDETECTED_MASK        = 0x01


class SX127xSpi(object):
    """Offers methods that drive the SPI bus to control the Semtech SX127x.
    """

    def __init__(self, spi_stngs):
        """Validates the given SPI settings and saves them.
        Initializes the SPI bus with the given port,CS and clock.
        """
        # Validate arguments, open and configure SPI peripheral
        assert spi_stngs[0] in (0,1), "Not a valid SPI port index"
        assert spi_stngs[1] in (0, 1), "Not a valid SPI chip select"
        assert type(spi_stngs[2]) is int and SPI_CLK_MIN <= spi_stngs[2] <= SPI_CLK_MAX, "Not a valid SPI clock rate"
        self.spi = spidev.SpiDev()
        self.spi.open(spi_stngs[0], spi_stngs[1])
        self.spi.max_speed_hz = spi_stngs[2]
        self.spi.mode = 0 # phase=0 and polarity=0


## SPI helper methods

    def _read(self, reg_addr, nbytes=1):
        """Reads a byte (or more) from the register.
        Returns list of bytes (even if there is only one).
        """
        assert type(nbytes) is int
        assert nbytes > 0
        b = [reg_addr,]
        b.extend([0,] * nbytes)
        return self.spi.xfer2(b)[1:]


    def _write(self, reg_addr, data):
        """Writes one or more bytes to the register.
        Returns list of bytes (even if there is only one).
        """
        assert type(data) == int or isinstance(data, collections.Sequence)

        # Set the write bit (MSb)
        reg_addr |= WRITE_BIT

        # Build the list of bytes to write
        if type(data) == int:
            data &= 0xff
            b = [reg_addr, data]
        else:
            b = [reg_addr,]
            b.extend(data)

        return self.spi.xfer2(b)[1:]


## SX127x general methods

    def check_chip_ver(self,):
        """Returns True if the Semtech SX127x returns the proper value
        from the Version register.  This proves the chip and the SPI bus
        are operating.
        """
        ver = self._read(REG_VERSION)[0]
        if ver == CHIP_VERSION:
            logging.info("SPI to SX127x: PASS")
            return True
        else:
            logging.info("SPI to SX127x: FAIL (version : %d)" % ver)
            return False


    def init(self, dflt_modem_stngs):
        """Gets a few SX127x registers in order
        to initialize some required state variables.
        Leaves the SX127x in Standby mode.
        """
        assert isinstance(dflt_modem_stngs, phy_sx127x_stngs.SX127xModemSettings)

        self.set_op_mode("sleep")
        self.get_dio()
        self.get_rf_freq()
        self.set_modem(dflt_modem_stngs)
        # TODO: if dflt is lora:
        self.set_lora_settings(dflt_modem_stngs["modulation_stngs"])
        self.set_op_mode('stdby')


    def close(self,):
        """Closes the SPI port.
        """
        self.spi.close()


    def get_dio(self,):
        """Reads the current DIO mapping from the device and
        stores it so we can modify individual DIOs later.
        Returns nothing.
        """
        map1, map2 = self._read(REG_DIO_MAPPING1, 2)
        dio = bytearray()
        dio.append((map1 >> 6) & 0b11) # DIO0
        dio.append((map1 >> 4) & 0b11) # DIO1
        dio.append((map1 >> 2) & 0b11) # DIO2
        dio.append((map1 >> 0) & 0b11) # DIO3
        dio.append((map2 >> 6) & 0b11) # DIO4
        dio.append((map2 >> 4) & 0b11) # DIO5
        self.dio_mapping = dio


    def get_op_mode(self,):
        """Gets the device mode field of the Op Mode register
        and returns a string representation of the mode.
        """
        mode_lut = ("sleep", "stdby", "fstx", "tx", "fsrx", "rxcont", "rx", "cad")
        d = self._read(REG_OP_MODE)
        self.mode = mode_lut[d[0] & 0b111]
        return self.mode


    def get_regs(self,):
        """Reads in all registers from the SX127x.
        This function is meant to be used at startup
        to gather the state of the SX127x.
        """
        pass


    def set_dio_mapping(self, **dio_args):
        """Writes the DIO mapping registers.
        dio_args is a kwarg of the form {dio<x>=<int>, ...}
        where x is a value 0..5
        and <int> is an integer in the range 0..3
        """
        # If DIO mapping has not been read from device
        # create a mapping of all zeros
        if not hasattr(self, "dio_mapping"):
            self.dio_mapping = [0,] * 6

        dio_seq = self.dio_mapping

        # put any kwargs into the sequence
        for k,v in dio_args.items():
            assert k.startswith("dio"), "dio_args has a bad key"
            dio_int = int(k[-1])
            assert dio_int in (0,1,2,3,4,5), "dio_args key out of range"
            assert v in (0,1,2,3), "dio_args has a bad value"
            dio_seq[dio_int] = v

        # build the register values from the sequence
        map_reg1 = (dio_seq[0] & 0x03) << 6 \
                 | (dio_seq[1] & 0x03) << 4 \
                 | (dio_seq[2] & 0x03) << 2 \
                 | (dio_seq[3] & 0x03)
        map_reg2 = (dio_seq[4] & 0x03) << 6 \
                 | (dio_seq[5] & 0x03) << 4
        self._write(REG_DIO_MAPPING1, [map_reg1, map_reg2])


    def set_modem(self, modem_stngs):
        """Enters sleep mode and applies the modem settings.
        TODO: Sets the modulation and LF or HF mode.
        """
        # must be in sleep mode to change the modulation
        d = self._read(REG_OP_MODE)[0]
        d &= 0b00011111
        if modem_stngs["modulation"] == "lora":
            d |= 0b10000000
        # elif modem_stngs["modulation"] == "fsk" is three 0s (nothing to do)
        elif modem_stngs["modulation"] == "ook":
            d |= 0b00100000

        if modem_stngs["lf_mode"]:
            d |= 0b00001000
        self._write(REG_OP_MODE, d)


## SX127x RF block methods

    def get_rf_freq(self,):
        """Reads the frequency registers
        and returns the calculated frequency.
        WARNING: The frequency registers will contain an offset
        if the radio's last operation was receive
        (but some bandwidths have 0.0 offset).
        """
        hi,med,low = self._read(REG_CARRIER_FREQ, 3)
        val = hi << 16 | med << 8 | low
        return int(round(val * OSC_FREQ / 2**19))


    def set_rf(self, **rf_stngs):
        """Sets the RF parameters of the SX127x.
        rf_stngs is one or more of the following parameters:
            "freq", "pa_sel", "max_pwr", "out_pwr", "ocp_on", "ocp_trim",
            "lna_gain", "lna_boost_lf", "lna_boost_hf"
        These parameters correspond to fieds in regs 0x01 .. 0x0C.
        """
        keys = rf_stngs.keys()
        for key in keys:
            assert key in ("freq", "pa_sel", "max_pwr",
                           "out_pwr", "ocp_on", "ocp_trim",
                           "lna_gain", "lna_boost_lf", "lna_boost_hf")

        regs = self._read(REG_OP_MODE, 9)
        # TODO: iterate through settings, rmw regs


    def set_pwr_cfg(self, pwr=0xf, max=0x4, boost=True):
        """Sets the power, max power and use-pa-boost fields of the
        PA_CONFIG register (0x09).
        The use-pa-boost field selects the chip pin to output the RF signal.
        boost=True selects PA_BOOST as the output; whereas, False selects RFO.
        Most modules connect PA_BOOST to the antenna output, but a few use RFO.
        https://github.com/PaulStoffregen/RadioHead/blob/master/RH_RF95.h#L649
        """
        r = pwr & 0xF | (max & 0x7) << 4
        if boost:
            r |= 0b10000000
        else:
            r &= 0b01111111
        self._write(REG_PA_CFG, r)



## LoRa modem methods

    def clear_lora_counts(self,):
        """Clears the valid header count and valid packet count regs.
        """
        self._write(REG_RX_HDR_CNT, [0,0,0,0])


    def clear_lora_irqs(self, irq_bits=None):
        """Clears interrupt flags.
        If an argument is given, it is a byte with a bit set
        for each IRQ flag to clear.
        If no argument is given, all IRQ flags are cleared.
        """
        if irq_bits:
            d = irq_bits
        else:
            d = 0xFF
        self._write(REG_IRQ_FLAGS, d)


    def enable_lora_irqs(self, irq_bits=None):
        """Enables one or more IRQs.
        If an argument is given, it is a byte
        with a bit set for each IRQ to enable.
        IRQs are enabled by writing a zero
        to the bit in the mask register.
        """
        if irq_bits:
            d = ~irq_bits
        else:
            d = 0x00
        self._write(REG_IRQ_MASK, d)


    def disable_lora_irqs(self, irq_bits=None):
        """Disables one or more IRQs.
        If an argument is given, it is a byte
        with a bit set for each IRQ to disable.
        If no argument is given, all IRQs are disabled.
        IRQs are disabled by writing a one
        to the bit in the mask register.
        """
        if irq_bits:
            d = irq_bits
        else:
            d = 0xFF
        self._write(REG_IRQ_MASK, d)


    def check_lora_rx_flags(self,):
        """Checks post-receive status, clears rx-related IRQs.
        Returns True if a valid packet was received, else False.
        """
        # Get the IRQ flags
        flags = self._read(REG_IRQ_FLAGS)[0]

        # Clear rx-related IRQ flags in the reg
        flags &= ( IRQFLAGS_RXTIMEOUT_MASK
                 | IRQFLAGS_RXDONE_MASK
                 | IRQFLAGS_PAYLOADCRCERROR_MASK
                 | IRQFLAGS_VALIDHEADER_MASK )
        self._write(REG_IRQ_FLAGS, flags)

        result = bool(flags & IRQFLAGS_RXDONE_MASK)
        if flags & ( IRQFLAGS_RXTIMEOUT_MASK
                   | IRQFLAGS_PAYLOADCRCERROR_MASK):
            result = False
        return result


    def get_lora_rxd(self,):
        """Returns the most recently received payload and rf channel data.
        Assumes caller has already determined check_lora_rx_flags() is True.
        Returns a tuple of: (payld, rssi, snr)
        payld is a list of integers.
        rssi is an integer [dBm].
        snr is a float [dB].
        """
        # Get the index into the FIFO of where the pkt starts
        # and the length of the data received
        pkt_start, _, _, nbytes = self._read(REG_RX_CURRENT_ADDR, 4)

        # Error checking (that pkt started at 0)
#        if pkt_start != 0: "pkt_start was %d" % pkt_start # TODO: logging

        # Read the payload
        self._write(REG_FIFO_PTR, pkt_start)
        payld = self._read(REG_FIFO, nbytes)

        # Get the packet SNR and RSSI (2 consecutive regs)
        # and calculate RSSI [dBm] and SNR [dB]
        snr, rssi = self._read(REG_PKT_SNR, 2)
        rssi = -157 + rssi
        snr = snr / 4.0

        return (payld, rssi, snr)


    def get_lora_status(self,):
        """Gets status fields.
        Returns a dict of status fields.
        """
        d = self._read(REG_RX_HDR_CNT, 5)
        s = {}
        s["rx_hdr_cnt"] = d[0] << 8 | d[1]
        s["rx_pkt_cnt"] = d[2] << 8 | d[3]
        s["rx_code_rate"] = d[4] >> 5
        s["modem_clr"] = (d[4] & 0x10) != 0
        s["hdr_info_valid"] = (d[4] & 0x08) != 0
        s["rx_busy"] = (d[4] & 0x04) != 0
        s["sig_sync"] = (d[4] & 0x01) != 0
        s["sig_detected"] = (d[4] & 0x01) != 0
        return s


    def set_lora_settings(self, lora_stngs):
        """Applies settings values to the appropriate registers.
        Caller should ensure the device is not busy.
        This method puts the device into sleep mode and returns it
        to its current mode.
        """
        assert isinstance(lora_stngs, phy_sx127x_stngs.SX127xLoraSettings)

        # Save lora_stngs (used by other set_* methods)
        self.lora_stngs = lora_stngs

        # Transition to sleep mode to write configuration
        mode_bkup = self.get_op_mode()
        if mode_bkup != "sleep":
            self.set_op_mode("sleep")

        # Concat bandwidth | code_rate | implicit header mode
        reg_cfg1 = lora_stngs["_bandwidth_idx"] << 4 \
            | lora_stngs["_code_rate_idx"] << 1 \
            | int(lora_stngs["implct_hdr_mode"])
        # Concat spread_factor | tx_cont | upper 2 bits of symbol count
        reg_cfg2 = lora_stngs["_spread_factor_idx"] << 4 \
            | int(lora_stngs["tx_cont"]) << 3 \
            | int(lora_stngs["en_crc"]) << 2 \
            | ((lora_stngs["symbol_count"] >> 8) & 0b011)
        # Lower 8 bits of symbol count go in reg(0x1F)
        reg_sym_to = lora_stngs["symbol_count"] & 0xff
        # Write 3 contiguous regs at once
        self._write(REG_MODEM_CFG_1, [reg_cfg1, reg_cfg2, reg_sym_to])

        # Write preamble register
        reg_preamble_len = [lora_stngs["preamble_len"] >> 8, lora_stngs["preamble_len"] & 0xff]
        self._write(REG_PREAMBLE_LEN, reg_preamble_len)

        # Write Cfg3 reg
        reg_cfg3 = int(lora_stngs["en_ldr"]) << 3 | int(lora_stngs["agc_auto"]) << 2
        self._write(REG_MODEM_CFG_3, reg_cfg3)

        # Write Sync word
        self._write(REG_SYNC_WORD, lora_stngs["sync_word"])

        # Restore previous operating mode
        if mode_bkup != "sleep":
            self.set_op_mode(mode_bkup)


    def set_lora_fifo_ptr(self, offset=None):
        """Sets the FIFO_PTR and TX_BASE_PTR regs
        """
        if offset is None:
            offset = self.lora_stngs["tx_base_ptr"]
        assert type(offset) == int
        assert 0 <= offset <= 255
        self._write(REG_FIFO_PTR, [offset, offset])


    def set_tx_data(self, data):
        """Sets the PAYLOD_LEN reg
        and writes the data to the FIFO
        in preparation for transmit.
        """
        self._write(REG_PAYLD_LEN, len(data))
        self._write(REG_FIFO, data)


    def set_tx_freq(self, freq):
        """Sets the radio carrier frequency for transmit operation.
        This is isolated from the receive operation to allow
        a defined offset to improve packet rejection (Errata 2.3).
        """
        assert 137e6 < freq < 1020e6
        self._write_freq(freq)


    def _write_freq(self, f, offset=0.0):
        """Writes the given frequency (with any offset) to the registers.
        The offset is to improve Rx packet rejection (Errata 2.3).
        """
        freq = f + offset
        freq = int(round(freq * 2**19 * INV_OSC_FREQ))
        d = [(freq>>16) & 0xff, (freq>>8) & 0xff, freq&0xff]
        self._write(REG_CARRIER_FREQ, d)


    def set_op_mode(self, mode="stdby"):
        """Sets the device mode in the operating mode register to one of
        these strings: sleep, stdby, fstx, tx, fsrx, rxcont, rxonce, cad
        """
        mode_lut = {"sleep": 0b000,
                    "stdby": 0b001,
                    "standby": 0b001, # repeat for convenience
                    "fstx": 0b010,
                    "tx": 0b011,
                    "fsrx": 0b100,
                    "rxcont": 0b101,
                    "rx": 0b110, # same as rxonce
                    "rxonce": 0b110, # repeat for convenience
                    "cad": 0b111}
        d = self._read(REG_OP_MODE)[0]
        d &= 0b11111000
        d |= mode_lut[mode]
        self._write(REG_OP_MODE, d)


    def set_lora(self, **lora_stngs):
        """Applies the given LoRa register settings.
        Settings is one or more of the following strings: TBD.
        Low-frequency mode is determined directly from the given frequency.
        """
        keys = lora_stngs.keys()
        for key in keys:
            assert key in () # TODO


    def set_lora_rx_fifo(self, offset=0):
        """Sets the RX base pointer and FIFO pointer
        to the given offset (defaults to zero).
        """
        self._write(REG_FIFO_RX_BASE_PTR, offset)
        self._write(REG_FIFO_PTR, offset)


    def set_lora_rx_freq(self, freq):
        """Sets the frequency register to achieve the desired freq.
        Implements Semtech ERRATA 2.3 for improved RX packet rejection.
        NOTE: The bandwidth (BW) must be set prior to calling this method;
        this may be done by calling set_lora_settings().
        """
        # Save parameters for improved Rx packet rejection (Errata 2.3)
        rx_rejection_offset_lut = (7810.0, 10420.0, 15620.0, 20830.0, 31250.0, 41670.0, 0.0, 0.0, 0.0, 0.0)
        rx_offset = rx_rejection_offset_lut[self.lora_stngs["_bandwidth_idx"]]

        # ERRATA 2.3: offset rx freq
        self._write_freq(freq, rx_offset)

        # ERRATA 2.3: set bit 7 at 0x31 to the correct value
        r = self._read(0x31)[0]
        if self.lora_stngs["_bandwidth_idx"] == 0b1001:
            r |= 0b10000000
            self._write(0x31, r)
        else:
            r &= ~0b10000000
            self._write(0x31, r)

            # ERRATA 2.3 set values at 0x2F and 0x30
            val_2f_lut = (0x48, 0x44, 0x44, 0x44, 0x44, 0x44, 0x40, 0x40, 0x40,)
            self._write(0x2F, [val_2f_lut[self.lora_stngs["_bandwidth_idx"]], 0])


    def set_lora_symbol_count(self, symbol_count):
        """Sets the RX symbol count register.
        NOTE: the RX timeout is only used by the 'rxonce' LoRa op_mode.
        The continuous RX op_mode does NOT use the RX timeout.
        """
        assert 4 <= symbol_count <= 1023
        r1, r2 = self._read(REG_MODEM_CFG_2, 2)
        r1 &= 0xFC
        r1 |= (symbol_count >> 8)
        r2 = (symbol_count & 0xFF)
        self._write(REG_MODEM_CFG_2, [r1,r2])


    def set_lora_rx_timeout(self, secs):
        """Calculates and sets the symbol_count
        to achieve the timeout given in seconds (float).
        The symbol_count calculation also depends on
        the bandwidth (BW) and spread factor (SF) settings.
        NOTE: the RX timeout is only used by the 'rxonce' LoRa op_mode.
        The continuous RX op_mode does NOT use the RX timeout.
        """
        assert type(secs) in (int, float)
        assert secs > 0
        symbol_rate = self.lora_stngs["bandwidth"] / 2**self.lora_stngs["_spread_factor_idx"]
        symbol_count = round(secs * symbol_rate)
        self.set_lora_symbol_count(symbol_count)



## FSK/OOK modem methods

    def get_fsk_temperature(self,):
        """Returns the temperature.
        """
        # TODO: See PDF p89 for procedure & calibration
        # TODO: find way to safely [re]store FSK access to be able to read temp
        t = self._read(REG_TEMP)
        # TODO: convert t to degrees C
        return t
