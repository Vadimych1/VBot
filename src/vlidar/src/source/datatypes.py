from miniros.util.datatypes import NumpyArray
import numpy as np

class LidarData(NumpyArray):
    def __init__(self, distances: list[int] | np.ndarray[np.uint], angles: list[int] | np.ndarray[np.uint]): 
        self.distances = np.asarray(distances)
        self.angles = np.asarray(angles)

    @staticmethod
    def encode(data: "LidarData"):
        data = np.asarray(
            [
                data.distances,
                data.angles
            ]
        )
        return super().encode(data)
    
    @staticmethod
    def decode(data: "LidarData"):
        data = super().decode(data)

        return LidarData(
            data[0],
            data[1],
        )
