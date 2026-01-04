# FMM Visualization Tool

This is an interactive web-based visualization tool for the Fast Map Matching (FMM) algorithm.

## Features

- **Interactive Map**: Click to create GPS trajectories
- **Real-time Matching**: See matched road segments highlighted immediately
- **Multiple Basemaps**: Switch between different map providers (高德, OpenStreetMap, Esri Satellite)
- **Custom Road Networks**: Load your own GeoJSON road networks

## Quick Start

### 1. Install Dependencies

```bash
pip install fastapi uvicorn
pip install -e .  # Install pybind11_fmm
```

### 2. Start the Backend Server

```bash
python app.py
```

The server will start on http://localhost:8000

### 3. Open the Visualization Tool

Open your browser and navigate to:
```
http://localhost:8000/
```

## Usage Guide

### Loading a Road Network

1. Click the settings button (⚙️) in the top-left corner
2. Enter the road network ID (e.g., `test_network`)
3. Click "加载路网" (Load Network)

The tool will load `<network_id>.geojson` from the current directory.

### Creating a Trajectory

1. Enable "绘制模式" (Drawing Mode) in the settings panel
2. Click on the map to add GPS points
3. Each click adds a point to the trajectory (shown in red)

### Matching the Trajectory

1. After adding at least 2 points, click "执行匹配" (Execute Match)
2. The matched road segments will be highlighted in green
3. Match results (path and score) will be displayed in the panel

### Controls

- **绘制模式** (Drawing Mode): Toggle trajectory drawing on/off
- **执行匹配** (Execute Match): Run FMM on the current trajectory
- **清除轨迹** (Clear Trajectory): Remove the current trajectory and match result
- **清除全部** (Clear All): Remove everything including the road network
- **底图** (Basemap): Switch between different map providers
- **透明度** (Opacity): Adjust basemap opacity

## Road Network Format

Road networks should be in GeoJSON format with LineString features:

```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "properties": {
        "id": 1
      },
      "geometry": {
        "type": "LineString",
        "coordinates": [[lon1, lat1], [lon2, lat2], ...]
      }
    }
  ]
}
```

### Example: Creating a Test Network

```python
import json

road_network = {
    "type": "FeatureCollection",
    "features": [
        {
            "type": "Feature",
            "properties": {"id": 1},
            "geometry": {
                "type": "LineString",
                "coordinates": [
                    [116.40, 39.90],  # Beijing area
                    [116.41, 39.91],
                    [116.42, 39.91]
                ]
            }
        },
        {
            "type": "Feature",
            "properties": {"id": 2},
            "geometry": {
                "type": "LineString",
                "coordinates": [
                    [116.42, 39.91],
                    [116.43, 39.92],
                    [116.44, 39.93]
                ]
            }
        }
    ]
}

with open("my_network.geojson", "w") as f:
    json.dump(road_network, f)
```

## API Reference

### GET /fmm

Perform map matching on a trajectory.

**Query Parameters:**
- `road_network_id`: ID of the road network (string)
- `trajectory`: Semicolon-separated list of lon/lat pairs (string)
  - Format: `lon1,lat1;lon2,lat2;lon3,lat3`

**Example:**
```
/fmm?road_network_id=test_network&trajectory=116.40,39.90;116.41,39.91;116.42,39.91
```

**Response:**
```json
{
  "matched_path": [1, 2],
  "matched_geometries": [
    [[116.40, 39.90], [116.41, 39.91], [116.42, 39.91]],
    [[116.42, 39.91], [116.43, 39.92], [116.44, 39.93]]
  ],
  "score": -15.234
}
```

## Troubleshooting

### Road Network Not Loading

- Check that the GeoJSON file exists in the current directory
- Verify the file has the correct format
- Check the browser console for error messages

### Matching Fails

- Ensure trajectory has at least 2 points
- Check that points are within the search radius of road segments
- Try increasing the search radius in the FMM configuration

### CORS Errors

- Make sure the FastAPI server is running
- Check that CORS middleware is properly configured in `app.py`

## Advanced Usage

### Custom FMM Configuration

Edit `app.py` to customize FMM parameters:

```python
from pybind11_fmm import FastMapMatchConfig

config = FastMapMatchConfig(
    k=50,              # Max candidates per point
    radius=200.0,      # Search radius (meters)
    gps_error=50.0,    # GPS error std dev (meters)
)

fmm = FastMapMatch(network, config)
```

### Custom Basemaps

Add new basemaps by editing the `initMapSources()` function in `index.html`:

```javascript
const initMapSources = () => {
    return {
        // ... existing basemaps ...
        my_custom_map: {
            type: 'raster',
            tiles: ['https://your-tile-server/{z}/{x}/{y}.png'],
            tileSize: 256,
            maxzoom: 18,
        }
    };
};
```

## Technology Stack

- **Frontend**: Vue 3, Vuetify 3, MapLibre GL
- **Backend**: FastAPI, pybind11_fmm
- **Map Rendering**: MapLibre GL JS
- **Styling**: Vuetify Material Design

## License

Same as pybind11-fmm project license.
