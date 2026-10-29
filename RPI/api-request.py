import os, requests
from typing import List

from dotenv import load_dotenv
load_dotenv()

PC_HOST = os.getenv("API_IP")
PC_PORT = int(os.getenv("API_PORT"))
BASE_URL = f"http://{PC_HOST}:{PC_PORT}/"
PATHFINDING_ROUTE = "pathfinding/"

def pathfinding(start, obstacles, map_spec=None):
    if map_spec is None:
        map_spec = {"width": 20, "height": 20, "cell_size_cm": 10}

    payload = {
        "map": map_spec,
        "start": list(start),
        "obstacles": obstacles,
    }
    URL = BASE_URL + PATHFINDING_ROUTE
    r = requests.post(URL, json=payload, timeout=(2, 10))
    r.raise_for_status()
    return r.json()

if __name__ == "__main__":
    obstacles = [
        {"x": 5, "y": 7, "orientation": "D"}
    ]
    res = pathfinding((0,0), obstacles)
    print(res)
