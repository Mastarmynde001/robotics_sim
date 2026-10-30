# Robotics Simulation: Digital Twin Quadrotor with C++20 Core, Pybind11 & PyBullet

[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg?style=flat&logo=c%2B%2B)](https://en.cppreference.com/w/cpp/20)
[![Python 3.12](https://img.shields.io/badge/Python-3.12-3776AB.svg?style=flat&logo=python)](https://www.python.org/)
[![Physics Engine](https://img.shields.io/badge/Physics-PyBullet-orange.svg)](https://pybullet.org/)
[![Binding](https://img.shields.io/badge/Bindings-Pybind11-red.svg)](https://pybind11.readthedocs.io/)
[![Package Manager](https://img.shields.io/badge/Package_Manager-uv-blueviolet.svg)](https://github.com/astral-sh/uv)
[![CI Pipeline](https://img.shields.io/badge/CI%2FCD-GitHub_Actions-green.svg?logo=githubactions)](.github/workflows/ci.yml)

An enterprise-grade, high-performance **Digital Twin Quadrotor Simulation Platform** combining a high-throughput **C++20 robotics engine** for state estimation and feedback control with a real-time **PyBullet rigid-body physics simulator** and an automated scientific telemetry analysis pipeline.

The platform demonstrates full hardware-in-the-loop (HIL) mathematical modeling, real-time noisy sensor filtering via a custom discrete **3D Kalman Filter**, bounded **3D PID control with integral anti-windup and actuator saturation**, dynamic parameter tuning through an interactive GUI, and automated headless continuous integration (CI/CD).

---

## Architecture Overview

The system utilizes a split-core design pattern: computationally intensive state estimation and control mathematics execute natively in optimized C++20 binaries, bridged via low-overhead Pybind11 bindings into Python 3.12 for physics orchestration and scientific visualization.

```mermaid
graph TD
    subgraph C++20 Engine [C++20 Shared Module: robotics_sim.so]
        PS[PositionSensor<br/>Gaussian Noise Generator] -->|Noisy Reading| KF[KalmanFilter3D<br/>Predict & Update State]
        KF -->|Filtered State Estimate| PID[PIDController<br/>Anti-Windup & Saturation]
        PID -->|Control Force Vector| Force[Actuator Forces Fx, Fy, Fz]
    end

    subgraph Simulation Environment [Python Orchestration: sim_runner.py]
        PB[PyBullet Physics Engine<br/>240 Hz Rigid Body Dynamics]
        Quad[Quadrotor Drone<br/>quadrotor.urdf]
        GUI[Debug Parameter Sliders<br/>Target & PID Gains]
        CSV[(telemetry_data.csv<br/>Time-Series Log)]
    end

    subgraph Analytics & Verification [Telemetry Pipeline: plot_telemetry.py]
        Plot[Scientific Figure Renderer<br/>telemetry_performance.png]
        Metrics[Performance Evaluation<br/>RMSE, Variance, Settling Time]
    end

    PB -->|Ground Truth State| PS
    GUI -->|Dynamic Setpoints| PID
    Force -->|applyExternalForce| Quad
    Quad -->|State Dynamics| PB
    PB -->|Ground Truth & Estimates| CSV
    CSV --> Metrics
    CSV --> Plot
```

---

## Key Features

1. **High-Performance C++20 Robotics Core (`robotics_sim.cpp`)**
   - **`Vector3`**: High-efficiency 3D vector arithmetic, squared distance, and Euclidean distance computations.
   - **`PositionSensor`**: Realistic sensor degradation model introducing additive white Gaussian noise $\mathcal{N}(0, \sigma^2)$ using hardware-seeded Mersenne Twister distributions.
   - **`KalmanFilter3D`**: Multi-axis linear Kalman filter performing decoupled 3D state estimation (position and velocity) with dynamic error covariance tracking.
   - **`PIDController`**: Independent 3-axis proportional-integral-derivative controller featuring trapezoidal/discrete integration, configurable anti-windup clamping, and physical force saturation limits.

2. **Zero-Copy Pybind11 Integration**
   - Direct, idiomatic Python bindings exposing C++ classes (`Vector3`, `Robot`, `PositionSensor`, `KalmanFilter3D`, `PIDController`) without unnecessary heap allocations or marshaling overhead.

3. **Physics Simulation & URDF Quadrotor Modeling (`quadrotor.urdf`, `sim_runner.py`)**
   - Real-world inertia tensor dynamics ($m = 0.5\text{ kg}$, $I_{xx} = 0.0023, I_{yy} = 0.0023, I_{zz} = 0.0040$).
   - High-fidelity visual links with avionics hub, structural arms, and colored directional rotor pairs.
   - Real-time $240\text{ Hz}$ collision resolution and world-frame actuation in PyBullet.

4. **Dynamic Runtime Reconfiguration**
   - Interactive GUI sliders enable live tuning of $K_p$, $K_i$, $K_d$, integral clamp limits, and actuator saturation while the drone is in flight, alongside interactive 3D target setpoint adjustments.

5. **Automated Telemetry & Analytics Pipeline (`main.py`, `plot_telemetry.py`)**
   - High-throughput CSV logging capturing simulation time, ground truth position, noisy sensor measurements, Kalman estimates, setpoints, and actuator forces per tick.
   - Automatic post-flight computation of Root Mean Square Error (RMSE), sensor noise reduction percentages, and $2\%$ setpoint settling time metrics.
   - Publication-quality engineering multi-panel plots saved to `telemetry_performance.png`.

6. **Automated GitHub Actions CI/CD Pipeline (`.github/workflows/ci.yml`)**
   - Clean headless build and test cycle using Astral `uv`, CMake 3.18+, GCC/Clang, and automated unit verification across C++ bindings.

---

## Directory Structure

```plaintext
robotics_sim/
├── .github/
│   └── workflows/
│       └── ci.yml               # Automated GitHub Actions CI/CD Pipeline
├── .gitignore                   # Ignores build artifacts, venv, caches, and logs
├── CMakeLists.txt               # CMake C++20 configuration with Pybind11 module build
├── DETAILS.md                   # In-depth mathematical derivations and technical analysis
├── main.py                      # Master pipeline entrypoint (Sim -> CSV -> Analytics)
├── plot_telemetry.py            # Scientific telemetry parser, metric analyzer & plotter
├── quadrotor.urdf               # Custom physical & visual description of the quadrotor
├── README.md                    # Project documentation and developer guide
├── robotics_sim.cpp             # Core C++20 robotics engine (Sensors, Kalman, PID, Pybind11)
├── sim_runner.py                # PyBullet real-time simulation runtime & GUI interface
└── telemetry_performance.png    # High-resolution benchmark analysis graphic
```

---

## Prerequisites & Installation

### System Requirements
- **Operating System**: Linux (Ubuntu 20.04+ recommended) or macOS
- **Compiler**: GCC 10+ or Clang 11+ with full C++20 support
- **Build System**: CMake 3.18+
- **Python**: Python 3.12+
- **Package Manager**: [Astral uv](https://github.com/astral-sh/uv) (recommended) or standard `pip`

### Step-by-Step Setup

1. **Clone the Repository**:
   ```bash
   git clone git@github.com:Mastarmynde001/robotics_sim.git
   cd robotics_sim
   ```

2. **Initialize Python Environment with `uv`**:
   ```bash
   uv venv
   source .venv/bin/activate
   uv pip install pybind11 pybullet matplotlib pandas numpy
   ```

3. **Configure and Compile the C++ Engine via CMake**:
   ```bash
   uv run cmake -B build -DCMAKE_PREFIX_PATH="$(uv run python -c 'import pybind11; print(pybind11.get_cmake_dir())')"
   cmake --build build --config Release
   ```
   *Note: Compilation generates `robotics_sim.cpython-312-*.so` directly into the repository root for immediate import in Python.*

4. **Verify the C++ Compilation**:
   ```bash
   uv run python -c 'import robotics_sim; print("C++ Shared Module Imported Successfully!")'
   ```

---

## Usage Guide

### 1. Run the End-to-End Automated Pipeline
Execute the master pipeline which launches the interactive digital twin simulation, waits for completion, validates telemetry data, and automatically triggers statistical analysis and figure generation:
```bash
uv run python main.py
```

### 2. Run the Interactive Simulator Directly
Launch the real-time PyBullet simulation directly:
```bash
uv run python sim_runner.py
```
**Interactive Controls in PyBullet GUI**:
- **Start / Reset Simulation**: Toggle/increment button to spawn or re-center the quadrotor at `(0, 0, 0)` and reset filter/controller states.
- **Target X / Y / Z**: Sliders to manipulate the red 3D setpoint sphere in real time.
- **Kp, Ki, Kd**: Tune proportional, integral, and derivative gains on the fly.
- **Max Integral Clamp / Max Force Saturation**: Adjust anti-windup bounds and physical actuator thrust limits.

### 3. Generate Telemetry Analysis & Plots
Process an existing `telemetry_data.csv` run to recalculate metrics and generate plots:
```bash
uv run python plot_telemetry.py
```

Sample console output:
```plaintext
==================================================
      DIGITAL TWIN CONTROL & FILTER METRICS       
==================================================
 Raw Sensor Noise RMSE     : 0.0812 m
 Kalman Filtered RMSE      : 0.0214 m
 Noise Variance Reduction  : 73.65%
 2% Settling Time          : 1.48 seconds
==================================================
[Plotter] Performance figure saved to 'telemetry_performance.png'
```

---

## Telemetry & Performance Benchmark

The telemetry engine evaluates state estimation accuracy and closed-loop control dynamics across three synchronized panels:

![Telemetry Performance Benchmark](telemetry_performance.png)

1. **State Estimation Tracking (Top)**: Evaluates ground truth quadrotor altitude ($Z$) against noisy sensor measurements ($\sigma = 0.08\text{ m}$) and the smoothed, optimal estimate from the C++ Kalman Filter.
2. **Euclidean Error Vector Norm (Middle)**: Computes the 3D Euclidean distance between the drone and the target setpoint, highlighting the $2\%$ settling time threshold.
3. **Actuator Control Effort (Bottom)**: Plots the instantaneous orthogonal force commands ($F_x, F_y, F_z$) showing anti-windup compliance and saturation clamping at $\pm 150\text{ N}$.

---

## Continuous Integration (CI/CD)

The project includes a GitHub Actions workflow (`.github/workflows/ci.yml`) executed on every `push` and `pull_request` targeting `main` or `master`. The workflow verifies:
- Clean CPython 3.12 environment creation.
- Dependency installation via Astral `uv`.
- Release-mode CMake compilation of the C++20 shared library.
- Headless execution and verification of `KalmanFilter3D` and `PIDController` bindings.

---

## Documentation

For a comprehensive mathematical breakdown, algorithm state-space formulations, covariance update matrices, and memory management analysis, refer to:
- [**DETAILS.md**](DETAILS.md): In-depth Technical & Mathematical Architecture Document.

---

## License

This project is licensed under the MIT License - see the repository root for details.
