import logging, struct

import dpkt # pip install dpkt

import mac_cmds


# HeyMac protocol version
HEYMAC_VERSION = 1

# Frame Control Field (Fctl) subfield values
FCTL_TYPE_MIN = 0
FCTL_TYPE_SHIFT = 6
FCTL_TYPE_MAC = 0b01 << FCTL_TYPE_SHIFT
FCTL_TYPE_NLH = 0b10 << FCTL_TYPE_SHIFT
FCTL_TYPE_EXT = 0b11 << FCTL_TYPE_SHIFT
FCTL_TYPE_MASK = 0b11 << FCTL_TYPE_SHIFT

FCTL_IE_PRESENT = 1 << 5
FCTL_NETID_PRESENT = 1 << 4
FCTL_EXT_ADDR_EN = 1 << 3
FCTL_DST_ADDR_PRESENT = 1 << 2
FCTL_SRC_ADDR_PRESENT = 1 << 1
FCTL_PEND_BIT = 1 << 0


class HeyMacFrame(dpkt.Packet):
    __byte_order__ = '!' # Network order
    __hdr__ = (
        ('fctl', 'B', 0),
        # Fctl is the only field guaranteed to be present
        # Below this are optional fields
        ('_ver_seq', '0s', b''), # accessed via .ver and .seq properties
        ('exttype', '0s', b''),
        ('daddr', '0s', b''),
        ('saddr', '0s', b''),
        ('netid', '0s', b''),
    )

    # Functions to help determine which fields are present
    def _has_verseq_field(self,): # VerSeq exists in all but Min frame types
        return (self.fctl & FCTL_TYPE_MASK) != FCTL_TYPE_MIN
    def _has_exttype_field(self,): # ExtType exists when Fctl type is Extended
        return (self.fctl & FCTL_TYPE_MASK) == FCTL_TYPE_EXT
    def _has_daddr_field(self,):
        return (self.fctl & FCTL_DST_ADDR_PRESENT) != 0
    def _has_saddr_field(self,):
        return (self.fctl & FCTL_SRC_ADDR_PRESENT) != 0
    def _has_netid_field(self,):
        return (self.fctl & FCTL_NETID_PRESENT) != 0

    # Functions to determine size of variable-size fields
    def _sizeof_saddr_field(self,):
        if self._has_saddr_field():
            if(self.fctl & FCTL_EXT_ADDR_EN) != 0:
                sz = 8
            else:
                sz = 2
        else:
            sz = 0
        return sz
    def _sizeof_daddr_field(self,):
        if self._has_daddr_field():
            if(self.fctl & FCTL_EXT_ADDR_EN) != 0:
                sz = 8
            else:
                sz = 2
        else:
            sz = 0
        return sz


    # Getters/setters for _<private> fields
    @property
    def ver(self,):
        if self._ver_seq:
            return (self._ver_seq[0] & 0xF0) >> 4
        else:
            return None

    @property
    def seq(self,):
        if self._ver_seq:
            return self._ver_seq[0] & 0x0F
        else:
            return None

    @seq.setter
    def seq(self, s):
        vs = (HEYMAC_VERSION << 4) | (s & 0x0F)
        self._ver_seq = vs.to_bytes(1, "big")


    def unpack(self, buf):
        """Unpacks a bytes object into component attributes.
        This function is called when an instance of this class is created
        by passing a bytes object to the constructor
        """
        buflen = len(buf)
        super(HeyMacFrame, self).unpack(buf)

        if self._has_verseq_field():
            if len(self.data) < 1:
                raise dpkt.NeedData("for verseq")
            self._ver_seq = self.data[0:1]
            self.data = self.data[1:]

        if self._has_exttype_field():
            if len(self.data) < 1:
                raise dpkt.NeedData("for exttype")
            self.exttype = self.data[0]
            self.data = self.data[1:]

        if self._has_daddr_field():
            sz = self._sizeof_daddr_field()
            if len(self.data) < sz:
                raise dpkt.NeedData("for daddr")
            sz = self._sizeof_daddr_field()
            self.daddr = self.data[0:sz]
            self.data = self.data[sz:]

        if self._has_saddr_field():
            if len(self.data) < 1:
                raise dpkt.NeedData("for saddr")
            sz = self._sizeof_saddr_field()
            self.saddr = self.data[0:sz]
            self.data = self.data[sz:]

        if self._has_netid_field():
            if len(self.data) < 2:
                raise dpkt.NeedData("for netid")
            self.netid = self.data[0:2]
            self.data = self.data[2:]

        # TODO: IEs

        # Unpack the payload for known frame types
        if self.fctl & FCTL_TYPE_MASK == FCTL_TYPE_MAC:
            if self.data and self.data[0] == mac_cmds.HEYMAC_CMD_SM_BCN:
                self.data = mac_cmds.HmCmdPktSmallBcn(self.data)
            elif self.data and self.data[0] == mac_cmds.HEYMAC_CMD_EXT_BCN:
                self.data = mac_cmds.HmCmdPktExtBcn(self.data)
            elif self.data and self.data[0] == mac_cmds.HEYMAC_CMD_TXT:
                self.data = mac_cmds.HmCmdPktTxt(self.data)


    def pack_hdr(self):
        """Packs header attributes into a bytes object.
        This function is called when bytes() or len() is called
        on an instance of HeyMacFrame.
        """
        nbytes = 0
        l = []

        # If Fctl indicates VerSeq should be present
        # and VerSeq was not set by the caller,
        # set the version to fixed value and zero the sequence
        if self._has_verseq_field() and not self._ver_seq:
            self.seq = 0

        # If VerSeq was set by caller
        if self._ver_seq:
            # If Fctl type is MIN, error
            if not self._has_verseq_field():
                raise dpkt.PackError("VerSeq set but Fctl type is MIN")
            nbytes += 1
            l.append(self._ver_seq)

        if self.exttype:
            nbytes += 1
            self.fctl |= FCTL_TYPE_EXT
            l.append(self.exttype)

        if self.daddr:
            len_daddr = len(self.daddr)
            nbytes += len_daddr
            if len_daddr == 8:
                self.fctl |= FCTL_EXT_ADDR_EN
                self.fctl |= FCTL_DST_ADDR_PRESENT
            elif len_daddr == 2:
                self.fctl |= FCTL_DST_ADDR_PRESENT
            l.append(self.daddr)

        if self.saddr:
            len_saddr = len(self.saddr)
            nbytes += len_saddr
            if len_saddr == 8:
                self.fctl |= FCTL_EXT_ADDR_EN
                self.fctl |= FCTL_SRC_ADDR_PRESENT
            elif len_saddr == 2:
                self.fctl |= FCTL_SRC_ADDR_PRESENT
            l.append(self.saddr)

        if self.netid:
            nbytes += len(self.netid)
            self.fctl |= FCTL_NETID_PRESENT
            if len(self.netid) != 2:
                raise dpkt.PackError("invalid netid length")
            l.append(self.netid)

        # Pack Fctl last because we modify above
        l.insert(0, super(HeyMacFrame, self).pack_hdr())
        return b''.join(l)
