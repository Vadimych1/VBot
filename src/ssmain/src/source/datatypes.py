from miniros import datatypes
import struct

class RobotTask(datatypes.Datatype):
    __slots__ = ("id", "dx", "dy", "items", "target")
    _format = ">HffH"
    _size = struct.calcsize(_format)

    def __init__(self, id: int, dx: float, dy: float, items: list["TaskItem"], target: str):
        self.id = id
        self.dx = dx
        self.dy = dy
        self.items = items
        self.target = target
        
    @staticmethod
    def encode(data: "RobotTask"):
        return struct.pack(RobotTask._format, data.id, data.dx, data.dy, len(data.items)) + sum(bytes(len(item)) + item.encode() for item in data.items) + data.target.encode()

    @staticmethod
    def decode(data: bytes):
        id, dx, dy, nitems = struct.unpack(RobotTask._format, data[:RobotTask._size])
        
        offset = RobotTask._size
        
        items = []
        
        for _ in range(nitems):
            l = data[offset]
            item = data[offset + l]
            offset += l + 1
            
            items.append(TaskItem.decode(item))
            
        target = data[offset:].decode()
        
        return RobotTask(id, dx, dy, items, target)
    
    def to_json(self):
        return {
            "id": self.id,
            "dx": self.dx,
            "dy": self.dy,
            "items": [item.to_json() for item in self.items]
        }

        
class TaskItem(datatypes.Datatype):
    __slots__ = ("id", "sx", "sy", "ex", "ey", "name")
    _format = ">Hffff"
    _size = struct.calcsize(_format)
    
    def __init__(self, id: int, name: str, sx: float, sy: float, ex: float, ey: str):
        self.id = id
        self.sx = sx
        self.sy = sy
        self.ex = ex
        self.ey = ey
        self.name = name
        
    @staticmethod
    def encode(data: "TaskItem"):
        return struct.pack(TaskItem._format, data.id, data.sx, data.sy, data.ex, data.ey) + data.name.encode()
    
    @staticmethod
    def decode(data: bytes):
        size = TaskItem._size
        id, sx, sy, ex, ey = struct.unpack(TaskItem._format, data[:size])
        name = data[size:].decode()
        
        return TaskItem(id, name, sx, sy, ex, ey)

    def to_json(self):
        return {
            "name": self.name,
            "sx": self.sx,
            "sy": self.sy,
            "ex": self.ex,
            "ey": self.ey,
        }