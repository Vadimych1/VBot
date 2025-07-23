from miniros.util.datatypes import Datatype, Movement, Int, Vector
import numpy as np

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

class SLAMPosition(Movement):
    @staticmethod
    def decode(data):
        q = Movement.decode(data)
        return SLAMPosition(q.pos, q.ang)
    
    @staticmethod
    def encode(data):
        return Movement.encode(
            Movement(
                data.pos,
                data.ang
            )
        )

    def pos_to_numpy(self) -> np.ndarray:
        return np.array([self.pos.x, self.pos.y, self.pos.z])

    def rot_to_numpy(self) -> np.ndarray:
        return np.array([self.ang.x, self.ang.y, self.ang.z])

class SLAMAnonSave(Int): ...
class SLAMAnonLoad(Int): ...