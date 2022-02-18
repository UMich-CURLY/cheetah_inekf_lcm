"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class reinitialization_lcmt(object):
    __slots__ = ["reinitialize_cmd"]

    __typenames__ = ["boolean"]

    __dimensions__ = [None]

    def __init__(self):
        self.reinitialize_cmd = False

    def encode(self):
        buf = BytesIO()
        buf.write(reinitialization_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">b", self.reinitialize_cmd))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != reinitialization_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return reinitialization_lcmt._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = reinitialization_lcmt()
        self.reinitialize_cmd = bool(struct.unpack('b', buf.read(1))[0])
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if reinitialization_lcmt in parents: return 0
        tmphash = (0xcb0a59f8a7a26336) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if reinitialization_lcmt._packed_fingerprint is None:
            reinitialization_lcmt._packed_fingerprint = struct.pack(">Q", reinitialization_lcmt._get_hash_recursive([]))
        return reinitialization_lcmt._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

