from miniros.util.datatypes import Datatype, Movement, Int, Vector
import numpy as np
import struct

class SLAMMap(Datatype):
    def __init__(self, mapdata: bytearray):
        super().__init__()
        self.data = mapdata

    @staticmethod
    def encode(data: "SLAMMap"):
        return data.data
    
    @staticmethod
    def decode(data: bytearray) -> "SLAMMap":
        return SLAMMap(data)
    
    def to_numpy(self, mapsize: int) -> np.ndarray:
        return np.frombuffer(self.data, dtype=np.uint8).reshape((mapsize, mapsize))

class SLAMPosition(Datatype):
    def __init__(self, pos: Vector, ang: Vector):
        self.pos = pos
        self.ang = ang
    
    @staticmethod
    def decode(data: bytes):
        la, lb = struct.unpack(">HH", data[:4])
        a, b = data[4:4+la], data[4+la:4+la+lb]
        
        return SLAMPosition(Vector.decode(a), Vector.decode(b))
        
    @staticmethod
    def encode(data: "SLAMPosition"):
        a, b = Vector.encode(data.pos), Vector.encode(data.ang)
        la, lb = len(a), len(b)
        
        return struct.pack(">HH", la, lb) + a + b

    def pos_to_numpy(self) -> np.ndarray:
        return np.array([self.pos.x, self.pos.y, self.pos.z])

    def rot_to_numpy(self) -> np.ndarray:
        return np.array([self.ang.x, self.ang.y, self.ang.z])

class SLAMAnonSave(Int): ...
class SLAMAnonLoad(Int): ...