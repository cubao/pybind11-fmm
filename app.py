"""FastAPI backend for FMM visualization tool."""

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
from pybind11_fmm import Network, FastMapMatch
import numpy as np
import json

app = FastAPI()

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global cache for road networks
road_networks = {}


def load_network(road_network_id: str) -> Network:
    """Load road network from GeoJSON file."""
    if road_network_id in road_networks:
        return road_networks[road_network_id]

    # Load from file
    file_path = f"./{road_network_id}.geojson"
    try:
        with open(file_path, "r") as f:
            data = json.load(f)
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="Road network not found")

    # Create network
    network = Network()
    for feature in data["features"]:
        if feature["geometry"]["type"] == "LineString":
            edge_id = feature["properties"].get("id", hash(json.dumps(feature)))
            coords = np.array(feature["geometry"]["coordinates"], dtype=np.float64)
            network.add_edge(edge_id, coords, is_wgs84=True)

    road_networks[road_network_id] = network
    return network


@app.get("/")
async def read_index():
    """Serve the main HTML file."""
    return FileResponse('index.html')


@app.get("/fmm")
async def fmm_endpoint(
    road_network_id: str,
    trajectory: str
):
    """Perform map matching on a trajectory."""
    # Load network
    network = load_network(road_network_id)

    # Parse trajectory
    try:
        traj_points = [list(map(float, p.split(','))) for p in trajectory.split(';')]
        traj_array = np.array(traj_points, dtype=np.float64)
    except (ValueError, IndexError):
        raise HTTPException(status_code=400, detail="Invalid trajectory format")

    # Match trajectory
    fmm = FastMapMatch(network)
    result = fmm.match_traj(traj_array)

    if not result.success:
        return {"error": "Matching failed"}

    # Get geometry of matched path
    matched_geometries = []
    for edge_id in result.optimal_path:
        geom = network.geometry(edge_id)
        matched_geometries.append(geom.coords_.tolist())

    return {
        "matched_path": result.optimal_path,
        "matched_geometries": matched_geometries,
        "score": result.score
    }


if __name__ == "__main__":
    # Create a dummy road network for testing
    road_network = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {"id": 1},
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[0, 0], [1, 1], [2, 1]]
                }
            },
            {
                "type": "Feature",
                "properties": {"id": 2},
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[2, 1], [3, 2], [4, 2]]
                }
            }
        ]
    }
    with open("test_network.geojson", "w") as f:
        json.dump(road_network, f)

    uvicorn.run(app, host="0.0.0.0", port=8000)
