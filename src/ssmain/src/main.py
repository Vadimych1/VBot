import argparse
import json
import sqlite3 as sql
from miniros import AsyncROSClient, datatypes, decorators
from miniros.util.util import Ticker
from miniros_vslam.source.datatypes import SLAMMap, SLAMPosition
from miniros_ssmain.source.datatypes import Task as TaskDatatype
import http.server as http
import asyncio

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

CREATE TABLE IF NOT EXISTS Products (
    id INTEGER UNIQUE PRIMARY KEY,
    name TEXT
);

CREATE TABLE IF NOT EXISTS Shelfs (
    id INTEGER UNIQUE PRIMARY KEY,
    name_qr TEXT
);

CREATE TABLE IF NOT EXISTS Vault (
    id INTEGER UNIQUE PRIMARY KEY,
    product_id INTEGER,
    shelf_id INTEGER,
            
    FOREIGN KEY (product_id)
        REFERENCES Products(id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
            
    
    FOREIGN KEY (shelf_id)
        REFERENCES Shelfs(id)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);
                  
CREATE TABLE IF NOT EXISTS Orders (
    id INTEGER UNIQUE PRIMARY KEY,
    output_qr TEXT,
    state INT
);
                  
CREATE TABLE IF NOT EXISTS OrderItems (
    id INTEGER UNIQUE PRIMARY KEY,
    order_id INTEGER,
    product_id INTEGER,

    FOREIGN KEY (order_id)
        REFERENCES Orders(id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,         

    FOREIGN KEY (product_id)
        REFERENCES Products(id)
        ON DELETE CASCADE
        ON UPDATE CASCADE                  
);
""")

class RobotTask:
    __slots__ = ("id", "output_qr", "state")

    def __init__(self, id: int, output_qr: str, state: int):
        self.id = id
        self.output_qr = output_qr
        self.state = state

class Robot:
    __slots__ = ("pos", "map", "task")

    def __init__(self, pos: SLAMPosition | None, map: SLAMMap | None, task: RobotTask | None):
        self.pos = pos
        self.map = map
        self.task = task

class SSMainClient(AsyncROSClient):
    def __init__(self, ip = "localhost", port = 3000):
        super().__init__("ssmain", ip, port)

        self.robots: dict[str, Robot] = {}

    @decorators.parsedata(SLAMMap, 1)
    async def on_map(self, data: SLAMMap, from_node: str):
        if from_node not in self.robots:
            self.robots[from_node] = Robot(
                None,
                data,
                None
            )

        else:
            self.robots[from_node].map = data

    @decorators.parsedata(SLAMPosition, 1)
    async def on_pos(self, data: SLAMPosition, from_node: str):
        if from_node not in self.robots:
            self.robots[from_node] = Robot(
                data,
                None,
                None
            )

        else:
            self.robots[from_node].pos = data

    async def on_taskdone(self, data: datatypes.Int, from_node: str):
        if from_node not in self.robots:
            self.robots[from_node] = Robot(
                None,
                None,
                None
            )

        else:
            self.robots[from_node].task = None

class HTTPHandler(http.SimpleHTTPRequestHandler):
    def on_home(self):
        return 200, "text/html", open("web/index.html", "r", -1, "utf-8").read()
    
    def do_GET(self):
        path = self.path

        try:
            status, dtype, data = {
                "/": self.on_home,
            }[path]()

            self.send_response(status),
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
if __name__ == "__main__":
    client = SSMainClient()
    
    async def run():
        while not client.client._is_running:
            await asyncio.sleep(0.1)

        map_topic = await client.topic("map", SLAMMap)
        # task_topic = await client.topic("task", TaskDatatype)
        otherpos_topic = await client.topic("otherpos", datatypes.Dict)

        while True:
            await ticker.tick_async()

            if ticker_05.check():
                cur.execute("SELECT * FROM Orders WHERE state = 0")
                pending_orders = list(map(lambda v: RobotTask(*v), cur.fetchall()))

                for order in pending_orders:
                    for name, robot in client.robots.items():
                        if robot.task is None:
                            robot.task = order
                            await client.anon(name, "task", TaskDatatype.encode(
                                {
                                    "id": order.id,
                                    "output_qr": order.output_qr,
                                }
                            ))

                            break

            await otherpos_topic.post(
                {
                    k: datatypes.Vector(v.pos.pos.x, v.pos.ang.y, v.pos.pos.z) for k, v in client.robots.items() 
                }
            )

            # TODO: merge maps and post

    async def main():
        await asyncio.gather(
            client.run(),
            run(),
        )