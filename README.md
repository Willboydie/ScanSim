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

### 4. Run the project

Example:

```powershell
python terrain_generation.py
```

## Summary
 ________________________________________________________________
|           Step             |             Command               |
|----------------------------|-----------------------------------|
| Activate venv (PowerShell) | `.\myvenv\Scripts\Activate.ps1`   |
| Install all dependencies   | `pip install -r requirements.txt` |
