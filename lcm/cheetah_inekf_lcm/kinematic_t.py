"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class kinematic_t(object):
    __slots__ = ["seq", "timestamp", "num_legs", "contact"]

    __typenames__ = ["int64_t", "double", "int8_t", "int8_t"]

    __dimensions__ = [None, None, None, ["num_legs"]]

    def __init__(self):
        self.seq = 0
        self.timestamp = 0.0
        self.num_legs = 0
        self.contact = []

    def encode(self):
        buf = BytesIO()
        buf.write(kinematic_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qdb", self.seq, self.timestamp, self.num_legs))
        buf.write(struct.pack('>%db' % self.num_legs, *self.contact[:self.num_legs]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != kinematic_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return kinematic_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = kinematic_t()
        self.seq, self.timestamp, self.num_legs = struct.unpack(">qdb", buf.read(17))
        self.contact = struct.unpack('>%db' % self.num_legs, buf.read(self.num_legs))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if kinematic_t in parents: return 0
        tmphash = (0x1ccc10cf22af89e8) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if kinematic_t._packed_fingerprint is None:
            kinematic_t._packed_fingerprint = struct.pack(">Q", kinematic_t._get_hash_recursive([]))
        return kinematic_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

