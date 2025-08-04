import argparse
import json
import sqlite3 as sql
from miniros import AsyncROSClient, datatypes, decorators
from miniros.util.util import Ticker
from miniros_vslam.source.datatypes import SLAMMap, SLAMPosition
from miniros_ssmain.source.datatypes import RobotTask, TaskItem
import http.server as http
import asyncio
from PIL import Image
from io import BytesIO
import base64 as b64
import math

parser = argparse.ArgumentParser()
parser.add_argument("cfg_file")
parsed = parser.parse_args()


with open(parsed.cfg_file, "r") as f:
    config = json.load(f)


db_path = config["database_path"]


conn = sql.connect(db_path)
cur = conn.cursor()


cur.executescript("""
PRAGMA foreign_keys = ON;

CREATE TABLE IF NOT EXISTS product_types (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL UNIQUE,
    description TEXT
);

CREATE TABLE IF NOT EXISTS shelves (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    start_x REAL NOT NULL,
    start_y REAL NOT NULL,
    end_x REAL NOT NULL,
    end_y REAL NOT NULL
);

CREATE TABLE IF NOT EXISTS delivery_points (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL UNIQUE,
    x REAL NOT NULL,
    y REAL NOT NULL
);

CREATE TABLE IF NOT EXISTS products (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    type_id INTEGER NOT NULL,
    shelf_id INTEGER NOT NULL,
    name TEXT NOT NULL,
    
    FOREIGN KEY (type_id) REFERENCES product_types(id),
    FOREIGN KEY (shelf_id) REFERENCES shelves(id)
);

CREATE TABLE IF NOT EXISTS orders (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    delivery_point_id INTEGER NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    status TEXT NOT NULL DEFAULT 'pending',
    
    FOREIGN KEY (delivery_point_id) REFERENCES delivery_points(id)
);

CREATE TABLE IF NOT EXISTS order_items (
    order_id INTEGER NOT NULL,
    product_id INTEGER NOT NULL,
    collected BOOLEAN DEFAULT 0,
    
    PRIMARY KEY (order_id, product_id),
    FOREIGN KEY (order_id) REFERENCES orders(id),
    FOREIGN KEY (product_id) REFERENCES products(id)
);

CREATE INDEX IF NOT EXISTS idx_product_type ON products(type_id);
CREATE INDEX IF NOT EXISTS idx_product_type ON products(shelf_id);
CREATE INDEX IF NOT EXISTS idx_orders_status ON orders(status);
CREATE INDEX IF NOT EXISTS idx_items_order ON order_items(order_id);
CREATE INDEX IF NOT EXISTS idx_items_order ON order_items(product_id);
""")

class Robot:
    __slots__ = ("pos", "map", "task", "task_status")

    def __init__(self, pos: SLAMPosition | None, map: SLAMMap | None, task: RobotTask | None):
        self.pos = pos
        self.map = map
        self.task = task
        self.task_status = 0
        
    def to_json(self):
        """
        Format:
        {
            "pos": [float, float],
            "ang": float,
            "task_status": int,
            "task": {
                "id": int,
                "dx": float,
                "dy": float,
                "items": [
                    {
                        "name": str,
                        "sx": float,
                        "sy": float,
                        "ex": float,
                        "ey": float
                    }
                ]
            }
        }
        """
        pos = self.pos.pos
        ang = self.pos.ang
        
        return {
            "pos": [pos.x, pos.z],
            "ang": ang.y,
            "task_status": self.task_status,
            "task": self.task.to_json(),
        }


class SSMainClient(AsyncROSClient):
    def __init__(self, ip = "192.168.0.104", port = 3000):
        super().__init__("ssmain", ip, port)

        self.robots: dict[str, Robot] = {}
        self.generated_map: bytes = b''

    @decorators.aparsedata(SLAMMap, 1)
    async def on_map(self, data: SLAMMap, from_node: str):
        if from_node not in self.robots:
            self.robots[from_node] = Robot(
                None,
                data,
                None
            )

        else:
            self.robots[from_node].map = data

    @decorators.aparsedata(SLAMPosition, 1)
    async def on_pos(self, data: SLAMPosition, from_node: str):        
        if from_node not in self.robots:
            self.robots[from_node] = Robot(
                data,
                None,
                None
            )

        else:
            self.robots[from_node].pos = data

    async def on_taskdone(self, data: bytes, from_node: str):
        if from_node not in self.robots:
            self.robots[from_node] = Robot(
                None,
                None,
                None
            )

        else:
            val = data[0]
            
            self.robots[from_node].task_status = val
                
            if val == 255:
                # update task status
                cur.execute("UPDATE orders SET status = 'completed' WHERE id = ?", self.robots[from_node].task.id)
                conn.commit()
                
                self.robots[from_node].task = None


class HTTPHandler(http.SimpleHTTPRequestHandler):
    def on_home(self):
        return 200, "text/html", open("web/index.html", "r", -1, "utf-8").read()
    
    def on_robots_count(self):
        return 200, "text/json", json.dumps(len(client.robots.keys()))
    
    def on_getmap(self):
        if len(client.generated_map) <= 0:
            client.generated_map = b'\x00' * 100
            
        print(client.generated_map, len(client.generated_map))
        
        size = int(math.sqrt(len(client.generated_map)))
        image = Image.frombytes("L", (size, size), client.generated_map).convert("RGB")
        
        image.save("image.png")
        
        buffer = BytesIO()
        image.save(buffer, format="PNG")
        data = b'data:image/png;base64,' + b64.b64encode(buffer.getbuffer())
        
        return 200, "text/plain", data
    
    
    def on_getrobots(self):
        return 200, "text/json", json.dumps({k: v.encode() for k, v in client.robots.items()})
    
    def on_getorders(self):        
        return 200, "text/json", json.dumps(orders if orders is not None else [])
    
    def do_GET(self):
        path = self.path

        try:
            status, dtype, data = {
                "/": self.on_home,
                "/get_count": self.on_robots_count,
                "/map": self.on_getmap,
                "/robots": self.on_getrobots,
                "/orders": self.on_getorders,
            }[path]()

            self.send_response(status)
            self.send_header("Content-Type", dtype)
            self.end_headers()
            self.wfile.write(data.encode() if type(data) is not bytes else data)
        except Exception as e:
            print(e)
            self.send_error(404)


httpserver = http.HTTPServer(("localhost", 5000), HTTPHandler)
httpthread = decorators.threaded()(httpserver.serve_forever)()


ticker = Ticker(2)
ticker_05 = Ticker(0.25)


client = SSMainClient()

orders = None

async def main():
    await asyncio.gather(
        client.run(),
        run(),
    )
    

async def run():
    global orders
    
    await client.wait()

    map_topic = await client.topic("map", SLAMMap)
    task_topic = await client.topic("task", RobotTask)
    otherpos_topic = await client.topic("otherpos", datatypes.Dict)

    while True:
        await ticker.tick_async()

        if ticker_05.check():
            cur.execute("SELECT * FROM orders WHERE status = 'pending' ORDER BY created_at")
            orders = cur.fetchall()
            
            for name in client.robots:
                robot = client.robots[name]
                
                if robot.task is None:
                    id, delivery_point_id, created_at, status = orders.pop(0)
                    
                    # update task status
                    cur.execute("UPDATE orders SET status = 'processing' WHERE id = ?", id)
                    conn.commit()
                    
                    # get delivery point coordinates
                    cur.execute("SELECT x, y FROM delivery_points WHERE id = ?", delivery_point_id)
                    dx, dy = cur.fetchone()
                    
                    # get delivery items
                    cur.execute("SELECT p.id, p.name, s.start_x, s.start_y, s.end_x, s.end_y FROM order_items oi \
                        JOIN products p ON p.id = oi.product_id \
                        JOIN shelves s ON s.id = p.shelf_id WHERE oi.order_id = ?", id)
                    
                    items = list(map(lambda x: TaskItem(*x), cur.fetchall()))
                    
                    robot.task = RobotTask(delivery_point_id, dx, dy, items, name)
                    task_topic.post(robot.task)
                    

        await otherpos_topic.post(
            {
                k: datatypes.Vector(v.pos.pos.x, v.pos.ang.y, v.pos.pos.z) for k, v in client.robots.items() if v.pos is not None and v.pos.pos is not None and v.pos.ang is not None
            }
        )

        # TODO: merge maps and post


asyncio.run(main())
