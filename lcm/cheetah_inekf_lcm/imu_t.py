"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class imu_t(object):
    __slots__ = ["quat", "rpy", "omega", "acc", "good_packets", "bad_packets"]

    __typenames__ = ["float", "float", "float", "float", "int64_t", "int64_t"]

    __dimensions__ = [[4], [3], [3], [3], None, None]

    def __init__(self):
        self.quat = [ 0.0 for dim0 in range(4) ]
        self.rpy = [ 0.0 for dim0 in range(3) ]
        self.omega = [ 0.0 for dim0 in range(3) ]
        self.acc = [ 0.0 for dim0 in range(3) ]
        self.good_packets = 0
        self.bad_packets = 0

    def encode(self):
        buf = BytesIO()
        buf.write(imu_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>4f', *self.quat[:4]))
        buf.write(struct.pack('>3f', *self.rpy[:3]))
        buf.write(struct.pack('>3f', *self.omega[:3]))
        buf.write(struct.pack('>3f', *self.acc[:3]))
        buf.write(struct.pack(">qq", self.good_packets, self.bad_packets))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != imu_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return imu_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = imu_t()
        self.quat = struct.unpack('>4f', buf.read(16))
        self.rpy = struct.unpack('>3f', buf.read(12))
        self.omega = struct.unpack('>3f', buf.read(12))
        self.acc = struct.unpack('>3f', buf.read(12))
        self.good_packets, self.bad_packets = struct.unpack(">qq", buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if imu_t in parents: return 0
        tmphash = (0x710a98f509c97d55) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if imu_t._packed_fingerprint is None:
            imu_t._packed_fingerprint = struct.pack(">Q", imu_t._get_hash_recursive([]))
        return imu_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

