import sqlite3
import sys
import json

cpath = sys.argv[1]
cfg = json.load(open(cpath))

conn = sqlite3.connect(cfg["database_path"])
cur = conn.cursor()

while True:
    cur.execute(input("> "))
    print("<", cur.fetchall())