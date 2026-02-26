# Adaptive Scanning

## Setup

### 0. Download Python3.12 (if you haven't already)
Python 13 does not support Open3D, the plotting software used in this repo.

### 1. Create a virtual environment

```powershell
py -3.12 -m venv myvenv  ## The -3.12 is important here. Python 13 does not support Open3D.
```

### 2. Activate the virtual environment

**Windows (PowerShell):**
```powershell
.\myvenv\Scripts\Activate.ps1
```

**Windows (Command Prompt):**
```cmd
myvenv\Scripts\activate.bat
```

**macOS/Linux:**
```bash
source myvenv/bin/activate
```

### 3. Install dependencies

With the virtual environment activated, run:

```powershell
pip install -r requirements.txt
```

This installs all packages listed in `requirements.txt` (numpy, open3d, noise).


## Using ScanSim

### 1. Create a ScanSimulator object.
```python
scansim = ScanSim.ScanSimulator()
```

### 2. Adjust terrain parameters

Synthetic terrain is generated using gradient-noise–based fractal brownian motion.

```python
scansim.terrain.roughness_amplitude = 0.5
scansim.terrain.slope_x_deg = 30.0  # Average uphill/downhill angle of the terrain in the x-direction.
scansim.terrain.slope_y_deg = 10.0  # Average uphill/downhill angle of the terrain in the y-direction.
scansim.terrain.lacunarity = 2.0  # Factor by which feature size shrinks between successive detail layers.
scansim.terrain.persistence = 0.3  # How strongly smaller-scale features contribute relative to large-scale ones.
scansim.terrain.octaves = 5  # Number of progressively smaller-scale terrain features added on top of large hills.
scansim.terrain.base_wavelength = 15  # Characteristic size (meters) of the largest rolling hills or ground undulations.
```
The values you see here are in fact the defaults, which will be used if not set by the user.

### 3. Adjust Projection Path Parameters
```python
scansim.projectionPath.centre = [0, 0]
scansim.projectionPath.width = 20
scansim.projectionPath.height = 20
scansim.projectionPath.orientation = 45
scansim.projectionPath.increment = 0.1
```
Projection paths are designed to represent building floor plans, etc, and are thus 2D. 
Only rectangles are currently supported.

### 4. Generate your terrain

```python
scansim.terrain.generate(seed)
```
Generate a terrain with your chosen parameters. `seed` is an integer - different seed values will generate different terrain variant with the same chosen parameters.


### 5. Generate your projection path
```python
scansim.projectionPath.generate()
```
Generate the projection path with your chosen parameters. 

### 6. Choose a pose for the simulated LiDAR device
```python
scansim.rangefinder.pose = [x, y, z, roll, pitch, yaw]
```

### 7. Take a LiDAR reading
```python
scansim.rangefinder.takeMeasurement(theta, phi, plot=False)
```
The function returns a cartesian point which is the point on the terrain which the device is pointed at.
If the device is pointing away from the generated terrain, the function returns `None`.


### Save and load terrains
Use 
```python
scansim.saveTerrain(filename)
```
to save the currently generated terrain as a .bin file. `filename` should also contain the relative path.

Use
```python.scansim.loadTerrain(filename, plot=False)
```
to load a saved terrain. Set `plot=True` to visualise upon loading.

### Plot anytime to visualise the simulation
Use
```python
scansim.plotProjectionPath()
```
and
```python
scansim.plotTerrain()
```
as well as setting `plot=True` in `takeMeasurement` to visualise things.


### A note on speed
`takeMeasurement` currently runs at an average of 450us (assuming `plot=False`).


### The system makes the following assumptions:
- X = North, Y = East, Z = Up
- `roll` is anticlockwise about X+
- `pitch` is clockwise about Y+
- `yaw` is clockwise about Z+
- The local forward "neutral" vector is [1,0,0]
- Rotations are performed in the order of yaw (1st) -> pitch (2nd) -> roll (3rd).

### In this project
- `theta` will refer to the yaw of the rangefinder beam
- `phi` will refer to the pitch of the rangefinder beam
- roll, pitch and yaw will refer to the device's orientation in the world frame (as laid out above)
- Note that for this arrangement, roll of the rangefinder beam has no effect on the beam's (world) vector
