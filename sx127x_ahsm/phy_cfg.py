from . import phy_sx127x_stngs

# Modem configuration
sx127x_stngs = phy_sx127x_stngs.SX127xSettings(
    bandwidth=250000,
    code_rate="4/6",
    implct_hdr_mode=False,
    spread_factor=128,
    tx_cont=False,
    en_crc=True,
    symbol_count=255, # rx timeout about 1 full frame size
    preamble_len=8, # chip adds 4 more symbol lengths to this
    en_ldr=False,
    agc_auto=True,
    sync_word=0x12)
