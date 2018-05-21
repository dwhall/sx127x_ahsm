"""
Copyright 2017 Dean Hall.  See LICENSE file for details.

HeyMac Commands for MAC frame type:
- HeyMac Small Beacon
"""

import struct
import dpkt


# HeyMac Command IDs
HEYMAC_CMD_SM_BCN = 1
HEYMAC_CMD_EXT_BCN = 2
HEYMAC_CMD_TXT = 3


class HmCmdPktSmallBcn(dpkt.Packet):
    """HeyMac Small Beacon command packet
    """

    FRAME_SPEC_BCN_EN = 0b10000000
    FRAME_SPEC_BCN_SHIFT = 7
    FRAME_SPEC_FR_ORDER_MASK = 0b01110000
    FRAME_SPEC_FR_ORDER_SHIFT = 4
    FRAME_SPEC_EB_ORDER_MASK = 0b00001111
    FRAME_SPEC_EB_ORDER_SHIFT = 0

    __byte_order__ = '!' # Network order
    __hdr__ = (
        ('cmd', 'B', HEYMAC_CMD_SM_BCN),
        ('frame_spec', 'B', 0),
        ('dscpln', 'B', 0), # 0x0X:None, 0x1X:RF, 0x2X:GPS (lower nibble is nhops to GPS)
        ('caps', 'B', 0),
        ('status', 'B', 0),
        ('asn', '4B', 0),
        # variable-length fields:
        ('tx_slots', '0s', b''),
        ('ngbr_tx_slots', '0s', b''),
    )


    def pack_hdr(self):
        """Packs header attributes into a bytes object.
        This function is called when bytes() or len() is called
        on an instance of of this class.
        """
        b = bytearray()

        # Pack the fixed-length fields
        b.append(self.cmd)
        b.append(self.frame_spec)
        b.append(self.dscpln)
        b.append(self.caps)
        b.append(self.status)
        b.extend(struct.pack(HmCmdPktSmallBcn.__byte_order__ + "i", self.asn))
        # Pack the variable-length fields
        b.extend(self.tx_slots)
        b.extend(self.ngbr_tx_slots)

        return bytes(b)


    def unpack(self, buf):
        """Unpacks a bytes object into component attributes.
        This function is called when an instance of this class is created
        by passing a bytes object to the constructor
        """
        # Unpack the fixed-length fields
        super(HmCmdPktSmallBcn, self).unpack(buf)

        # The Frame Spec defines the size of tx_slots and ngbr_tx_slots
        frOrder = (HmCmdPktSmallBcn.FRAME_SPEC_FR_ORDER_MASK & self.frame_spec) \
                >> HmCmdPktSmallBcn.FRAME_SPEC_FR_ORDER_SHIFT
        sz = (2 ** frOrder) // 8
        if sz < 1: sz = 1

        # Unpack the variable-length fields
        hl = self.__hdr_len__
        self.tx_slots = buf[hl : hl + sz]
        self.ngbr_tx_slots = buf[hl + sz:]
        assert len(self.ngbr_tx_slots) == sz

        self.data = bytes()


class HmCmdPktExtBcn(dpkt.Packet):
    """HeyMac Extended Beacon command packet
    """
    FRAME_SPEC_BCN_EN = 0b10000000
    FRAME_SPEC_BCN_SHIFT = 7
    FRAME_SPEC_FR_ORDER_MASK = 0b01110000
    FRAME_SPEC_FR_ORDER_SHIFT = 4
    FRAME_SPEC_EB_ORDER_MASK = 0b00001111
    FRAME_SPEC_EB_ORDER_SHIFT = 0

    __byte_order__ = '!' # Network order
    __hdr__ = (
        ('cmd', 'B', HEYMAC_CMD_EXT_BCN),
        # Small Beacon fields:
        ('frame_spec', 'B', 0),
        ('dscpln', 'B', 0), # 0x0X:None, 0x1X:RF, 0x2X:GPS (lower nibble is nhops to GPS)
        ('caps', 'B', 0),
        ('status', 'B', 0),
        ('asn', 'I', 0),
        # variable-length fields:
        ('tx_slots', '0s', b''),
        ('ngbr_tx_slots', '0s', b''),
        # Extended Beacon fields:
        ('station_id', '0s', b''),
        ('_nghbrs', '0s', b''),
        ('_ntwks', '0s', b''),
        ('geoloc', '0s', b''),
    )


class HmCmdPktTxt(dpkt.Packet):
    __byte_order__ = '!' # Network order
    __hdr__ = (
        ('cmd', 'B', HEYMAC_CMD_TXT),
        ('msg', '0s', b''),
    )

    def __len__(self):
        return self.__hdr_len__ + len(self.msg)


    def __bytes__(self):
        return self.pack_hdr() + bytes(self.msg)


    def unpack(self, buf):
        # Unpack the fixed-length fields
        dpkt.Packet.unpack(self, buf)

        # Unpack the variable-length field
        self.msg = buf[self.__hdr_len__:]
        self.data = bytes()


def test():
    bcn = HmCmdPktSmallBcn(
        frame_spec = HmCmdPktSmallBcn.FRAME_SPEC_BCN_EN \
                | 5 << HmCmdPktSmallBcn.FRAME_SPEC_FR_ORDER_SHIFT \
                | 10,
        dscpln=2,
        caps=3,
        status=4,
        asn=42,
    )
    bcn.tx_slots = list(range(4))
    bcn.ngbr_tx_slots = list(range(0x80, 0x80 + 4))

    print(repr(bcn))
    b = bytes(bcn)
    print(repr(HmCmdPktSmallBcn(b)))

    txt = HmCmdPktTxt(msg=b"Hell, oh! whirled")
    print(repr(txt))
    print(repr(HmCmdPktTxt(bytes(txt))))


if __name__ == '__main__':
    test()
