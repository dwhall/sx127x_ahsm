from . import phy_sx127x_stngs

# Default LoRa Settings (approx. 9115 bps)
default_sx127x_lora_stngs = phy_sx127x_stngs.SX127xLoraSettings(
    {
    "trx_mode": "stdby",
    "bandwidth": 250000,
    "code_rate": "4/6",
    "implct_hdr_mode": False,
    "spread_factor": 128,
    "tx_cont": False,
    "en_crc": True,
    "symbol_count": 255, # rx timeout about 1 full frame size
    "preamble_len": 8, # chip adds 4 more symbol lengths to this
    "en_ldr": False,
    "agc_auto": True,
    "sync_word": 0x12,
    })
