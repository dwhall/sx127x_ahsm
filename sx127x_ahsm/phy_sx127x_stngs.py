#!/usr/bin/env python3
"""
Copyright 2017 Dean Hall.  See LICENSE for details.
"""


import collections
import time


class SX127xSettings(dict):
    """Base class for SX127x device settings.
    Modem, RF, LoRa and FSK settings are derived from this.
    """
    def __init__(self, stngs_dict={}):
        """Validates any given settings dict
        or inits with an empty settings dict.
        """
        assert type(stngs_dict) in (dict, self.__class__)
        super().__init__(stngs_dict)
        for k, v in stngs_dict.items():
            self[k] = v


class SX127xModemSettings(SX127xSettings):
    """Validates and stores SX127x modem settings.
    """
    def __setitem__(self, k, v):
        """Throws an AssertionError if either the key (setting)
        or value is invalid.
        """
        setting_names = list(self.__class__.validate_and_set.keys())
        setting_names.sort()
        assert k in setting_names, "setting must be one of: " + str(setting_names)
        self.validate_and_set[k](self, v)


    def _validate_modulation(self, val):
        """Validates and sets the modulation which affects
        RegOpMode fields: LongRangeMode and Modulation Type.
        """
        modulations = ("lora", "fsk", "ook")
        assert val in modulations, "setting must be one of: " + str(modulations)
        super().__setitem__("modulation", val)


    def _validate_lf_mode(self, val):
        """Validates and sets lf_mode.
        """
        assert type(val) is bool
        super().__setitem__("lf_mode", val)


    validate_and_set = {
        "modulation": _validate_modulation,
        "lf_mode": _validate_lf_mode,
    }


class SX127xLoraSettings(SX127xSettings):
    """Validates and stores SX127x LoRa settings.
    """
    def __setitem__(self, k, v):
        """Throws an AssertionError if either the key (setting)
        or value is invalid.
        """
        setting_names = list(self.__class__.validate_and_set.keys())
        setting_names.sort()
        assert k in setting_names, "setting must be one of: " + str(setting_names)
        self.validate_and_set[k](self, v)


    def _validate_op_mode(self, val):
        """Validates and sets transceiver mode.
        """
        # NOTE: dict isn't needed here.  Could reduce to tuple
        # TODO: See if there's a way to reduce dual maintenance with set_lora_op_mode()
        op_mode_lut = {
            "sleep": 0b000,
            "stdby": 0b001,
            "standby": 0b001, # repeat for convenience
            "fstx": 0b010,
            "tx": 0b011,
            "fsrx": 0b100,
            "rxcont": 0b101,
            "rx": 0b110, # same as rxonce
            "rxonce": 0b110, # repeat for convenience
            "cad": 0b111,
            }
        op_mode_options = list(op_mode_lut.keys())
        op_mode_options.sort()
        assert val in op_mode_options, "op_mode must be one of: " + str(op_mode_options)
        super().__setitem__("op_mode", val)


    def _validate_bandwidth(self, val):
        """Validates and sets bandwidth.
        """
        bandwidth_lut = {
            7810: 0b0000,
            10420: 0b0001,
            15620: 0b0010,
            20830: 0b0011,
            31250: 0b0100,
            41670: 0b0101,
            62500: 0b0110,
            125000: 0b0111,
            250000: 0b1000,
            500000: 0b1001}
        bandwidth_options = list(bandwidth_lut.keys())
        bandwidth_options.sort()
        assert val in bandwidth_options, "bandwidth must be one of: " + str(bandwidth_options)
        super().__setitem__("bandwidth", val)
        super().__setitem__("_bandwidth_idx", bandwidth_lut[val])


    def _validate_code_rate(self, val):
        """Validates and sets code_rate.
        """
        code_rate_lut = {
            "4/5": 0b001,
            "4/6": 0b010,
            "4/7": 0b011,
            "4/8": 0b100}
        code_rate_options = list(code_rate_lut.keys())
        code_rate_options.sort()
        assert val in code_rate_options, "code_rate must be one of: " + str(code_rate_options)
        super().__setitem__("code_rate", val)
        super().__setitem__("_code_rate_idx", code_rate_lut[val])


    def _validate_implct_hdr_mode(self, val):
        """Validates and sets implct_hdr_mode.
        """
        assert type(val) is bool
        super().__setitem__("implct_hdr_mode", val)


    def _validate_spread_factor(self, val):
        """Validates and sets spread_factor.
        """
        spread_factor_lut = {
            64: 6,
            128: 7,
            256: 8,
            512: 9,
            1024: 10,
            2048: 11,
            4096: 12}
        spread_factor_options = list(spread_factor_lut.keys())
        spread_factor_options.sort()
        assert val in spread_factor_options, "spread_factor must be one of: " + str(spread_factor_options)
        super().__setitem__("spread_factor", val)
        super().__setitem__("_spread_factor_idx", spread_factor_lut[val])


    def _validate_tx_cont(self, val):
        """Validates and sets tx_cont.
        """
        assert type(val) is bool
        super().__setitem__("tx_cont", val)


    def _validate_en_crc(self, val):
        """Validates and sets en_crc.
        """
        assert type(val) is bool
        super().__setitem__("en_crc", val)


    def _validate_symbol_count(self, val):
        """Validates and sets symbol_count.
        """
        assert type(val) is int, "symbol_count must be a whole number"
        assert 0 <= val <= 2**10 - 1, "symbol_count must be within the range 0 .. 1023, inclusive"
        super().__setitem__("symbol_count", val)


    def _validate_preamble_len(self, val):
        """Validates and sets preamble_len.
        """
        assert 7 <= val <= 65535, "preamble_len must be within the range 7 .. 65535, inclusive"
        super().__setitem__("preamble_len", val)


    def _validate_en_ldr(self, val):
        """Validates and sets en_ldr.
        """
        assert type(val) is bool
        super().__setitem__("en_ldr", val)


    def _validate_agc_auto(self, val):
        """Validates and sets agc_auto.
        """
        assert type(val) is bool
        super().__setitem__("agc_auto", val)


    def _validate_sync_word(self, val):
        """Validates and sets sync_word.
        """
        assert 0 <= val <= 255, "sync_word must be within the range 0 .. 255, inclusive"
        super().__setitem__("sync_word", val)


    def _validate_tx_base_ptr(self, val):
        """Validates and sets tx_base_ptr.
        """
        assert 0 <= val <= 255, "tx_base_ptr must be within the range 0 .. 255, inclusive"
        super().__setitem__("tx_base_ptr", val)


    def _validate_rx_base_ptr(self, val):
        """Validates and sets rx_base_ptr.
        """
        assert 0 <= val <= 255, "rx_base_ptr must be within the range 0 .. 255, inclusive"
        super().__setitem__("rx_base_ptr", val)


    # The names of LoRa modem settings handled by this container class
    validate_and_set = {
        "op_mode": _validate_op_mode,
        "bandwidth": _validate_bandwidth,
        "code_rate": _validate_code_rate,
        "implct_hdr_mode": _validate_implct_hdr_mode,
        "spread_factor": _validate_spread_factor,
        "tx_cont": _validate_tx_cont,
        "en_crc": _validate_en_crc,
        "symbol_count": _validate_symbol_count,
        "preamble_len": _validate_preamble_len,
        "en_ldr": _validate_en_ldr,
        "agc_auto": _validate_agc_auto,
        "sync_word": _validate_sync_word,
        "tx_base_ptr": _validate_tx_base_ptr,
        "rx_base_ptr": _validate_rx_base_ptr,
    }


if __name__ == "__main__":
    d = {"sync_word": 5}
    #d = {"sync_word": 500}
    l = SX127xLoraSettings(d)
    #l['timmy'] = 0
    l = SX127xLoraSettings()
    #l["sync_word"] = 500
    l["bandwidth"] = 125000
    l["op_mode"] = "stdby"

    print(l)
