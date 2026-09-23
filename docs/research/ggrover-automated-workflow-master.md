# ggGridRunner Automated Engineering Workflow & Architecture Master Guide

This document specifies the end-to-end automated engineering workflow and physical architecture for **ggGridRunner**. It details the journey from cloud-native parametric CAD in **Onshape**, through symbolic kinematics (**Rumoca** + **CasADi**), 3D physics simulation (**Webots**), to headless slicing (**PrusaSlicer CLI**) and wireless print deployment (**OctoPi + LulzBot Mini**).

This guide also serves as a reusable template and workflow blueprint for future automated robotics and hardware projects.

---

## 1. Executive Summary & Tool Selection Rationale Matrix

When designing an automated hardware development pipeline, selecting tools that support headless operation, API scriptability, and declarative modeling is essential.

| Layer | Selected Tool | Alternative Considered | Selection Rationale |
| :--- | :--- | :--- | :--- |
| **Cloud CAD** | **Onshape** | Fusion 360 / SolidWorks | Cloud-native REST API allows headless querying of feature variables, mass properties, interference checks, and step/urdf exports without local GUI dependencies. |
| **Local CAD Scripting** | **FreeCAD MCP** | OpenSCAD | FreeCAD Python API wrapped in MCP enables local parametric geometric sweeps, Ackermann angle checks, and headless step processing. |
| **Physics Compiler** | **Rumoca** | OpenModelica C++ | Written in Rust; parses declarative Modelica (`.mo`) multi-domain physical equations and flattens them into DAEs without heavy legacy C++ toolchain overhead. |
| **Symbolic Mathematics** | **CasADi** | SymPy / MATLAB | C++/Python algorithmic differentiation engine built specifically for computing exact Jacobians/Hessians for fast non-linear optimization (IPOPT) and MPC. |
| **3D Simulator** | **Webots** | Gazebo / Chrono | Lightweight out-of-the-box ODE physics engine with high-performance Python Supervisor API; ideal for fast desktop SIL without full ROS 2 stack requirements. |
| **Firmware Testing** | **PlatformIO Native** | QEMU / Hardware HIL | Compiles C++ firmware logic directly on desktop host (`native` env) against Rumoca-generated physical equations for rapid unit testing. |
| **Headless Slicer** | **PrusaSlicer CLI** | Cura CLI | Fast background STL slicing (0.5s execution) providing precise plastic mass metadata to update simulation URDF before physical printing. |
| **Print Deployment** | **OctoPi (OctoPrint API)** | Klipper / Manual SD | REST API enables automated G-code upload, print start triggers, and webcam monitoring. |
| **XBee Radio SIL** | **pyserial & digi-xbee** | Physical USB Dongles | Simulates serial packet streams (`ControlPacket` / `TelemetryPacket`) over virtual null-modem serial ports (`COM3 ↔ COM4`). |

---

## 2. End-to-End System Architecture

```mermaid
flowchart TD
    subgraph Phase 1: Parametric CAD & Mechanical Assembly (Onshape + FreeCAD)
        Onshape["Onshape Cloud CAD"]
        FreeCAD["FreeCAD MCP Server"]
        Agent["AI Agent (Antigravity)"]
        
        Agent <-->|Onshape REST API / onshape-to-robot| Onshape
        Agent <-->|MCP Tools| FreeCAD
        
        note1["- Verify subframe fits 152mm bed<br/>- Mutate tie-rod & horn variables<br/>- Perform steering sweeps"]
    end

    subgraph Phase 2: Equation Compilation & Symbolic Optimization (Rumoca + CasADi)
        MO["Modelica Vehicle Model (.mo)<br/>(4WD Hub Motors / 4WIS / Skid)"]
        Rumoca["Rumoca (Rust Modelica Compiler)"]
        CasADi["CasADi (Symbolic Math Engine)"]

        Agent --> MO --> Rumoca --> CasADi
        CasADi -->|Ackermann Ratios & Trajectory Limits| Agent
    end

    subgraph Phase 3: Unified 3D Simulation & SIL (Webots)
        URDF["Exported URDF + Meshes"]
        Sim["Webots Unified Physics Engine"]
        RoverFW["Rover STM32 Firmware (PlatformIO env:native)"]
        CtrlFW["Controller Pro Micro Firmware (PlatformIO env:native)"]
        RadioSim["XBee Virtual Emitter/Receiver (57600 Baud)"]

        Agent -->|Export URDF| URDF --> Sim
        Rumoca -->|C++ Header Export| RoverFW
        CtrlFW <-->|20Hz ControlPacket| RadioSim <-->|10Hz TelemetryPacket| RoverFW
        RoverFW <-->|Sensors & Motors| Sim
    end

    subgraph Phase 4: Headless Slicing & OctoPi Deployment
        Slicer["PrusaSlicer CLI (0.5s Background Slice)"]
        OctoPi["OctoPi (LulzBot Mini REST API)"]

        Agent -->|Export STL| Slicer
        Slicer -->|Read Exact Printed Mass| Agent
        Agent -->|Update Link Mass| URDF
        Slicer -->|Upload G-code & Auto-Print| OctoPi
    end
```

---

## 3. Physical Rover Architecture & Mechanical Strategy

### Drive & Steering System
- **Drive System:** 4-wheel drive using direct hub motors (or JGA25-370 motors with 21.3:1 integrated gearboxes). Eliminates central differential gears and drive shafts.
- **Steering System:** 4-Wheel Independent Steering (4WIS) utilizing 4x MG996R high-torque servos. Supports multiple motion modes:
  - **Ackermann Steering:** Variable inner/outer wheel turning angles for smooth cornering.
  - **Crab Walk:** All 4 wheels turn in parallel for diagonal strafing.
  - **Spin-in-Place:** Diagonally opposed wheel angles for zero-radius turning.
- **Suspension:** Independent double wishbone suspension on all 4 wheels (42mm upper arm, 60mm lower arm, 100mm shocks).

### Modular Subframe Strategy (LulzBot Mini Fit)
The vehicle's overall dimensions ($220\text{ mm}$ wheelbase $\times 240\text{ mm}$ track width) exceed the LulzBot Mini build volume (**$152\text{ mm} \times 152\text{ mm} \times 158\text{ mm}$**).

To overcome this, the chassis is split into three modular subframes:
1. **Front Module ($\le 145\text{ mm}$):** Houses front steering knuckles, MG996R servos, and suspension mounts.
2. **Center Module ($\le 145\text{ mm}$):** Encloses the STM32 Blue Pill, XBee module, BME280 sensor, and 4S4P 18650 battery pack.
3. **Rear Module ($\le 145\text{ mm}$):** Holds rear drive motors/hub mounts and rear suspension arms.
4. **Structural Interconnects:** All 3 modules are securely joined using $6\text{ mm}$ carbon fiber tubes and aluminum standoff rails.

---

## 4. Tool Prerequisites & Installation Guide

### Installed Environment Packages

| Tool / Package | Purpose | Installation Command |
| :--- | :--- | :--- |
| **PlatformIO Core** | MCU build system & `env:native` SIL compilation | Built-in (`pio --version`) |
| **CasADi** | Algorithmic differentiation & symbolic math | `pip install casadi` |
| **onshape-to-robot & onshape-client** | Pull Onshape CAD assemblies $\rightarrow$ URDF + STL meshes | `pip install onshape-to-robot onshape-client` |
| **pyserial & digi-xbee** | Radio packet inspection & virtual serial testing | `pip install pyserial digi-xbee` |
| **FreeCAD 1.1** | Headless local parametric CAD script sweeps | Installed (`C:\Program Files\FreeCAD 1.1\bin`) |
| **PrusaSlicer CLI** | Headless background slicing & mass extraction | `winget install Prusa3D.PrusaSlicer` |
| **Webots** | Unified 3D physics, firmware, & radio simulator | `winget install Cyberbotics.Webots` |
| **Rust & Rumoca** | Modelica `.mo` parser & DAE C++ header exporter | `winget install Rustlang.Rustup` then `cargo install rumoca` |
| **Digi XCTU** | Hardware XBee radio configuration utility | Download GUI from [digi.com/xctu](https://www.digi.com/xctu) |

### API Key Configuration
To connect to Onshape programmatically, set environment variables:
```powershell
$env:ONSHAPE_ACCESS_KEY="YOUR_ACCESS_KEY"
$env:ONSHAPE_SECRET_KEY="YOUR_SECRET_KEY"
```

---

## 5. Parametric CAD & Kinematic Sweeps (Onshape & FreeCAD MCP)

### Onshape Naming Conventions
To enable automated querying and updating via the Onshape REST API, standard variable and mate names are enforced:
- **FeatureStudio Variables:**
  - `#servo_horn_length` (e.g. `20 mm`)
  - `#tie_rod_length` (e.g. `55 mm`)
  - `#steering_arm_length` (e.g. `22 mm`)
  - `#chassis_track_width` (e.g. `240 mm`)
  - `#chassis_wheelbase` (e.g. `220 mm`)
- **Assembly Mates:**
  - `Revolute_Servo_FL`, `Revolute_Servo_FR`, `Revolute_Servo_RL`, `Revolute_Servo_RR`
  - `Revolute_Kingpin_FL`, `Revolute_Kingpin_FR`, `Revolute_Kingpin_RL`, `Revolute_Kingpin_RR`

### FreeCAD MCP Integration
Using the local FreeCAD MCP server, the AI Agent can:
- Step steering servo angles from $-45^\circ$ to $+45^\circ$ in software.
- Detect mechanical binding or suspension frame collision during steering sweeps.
- Measure inner vs. outer wheel turning angles to confirm Ackermann compliance.

---

## 6. Symbolic Kinematics & Physics Engine (Rumoca + CasADi)

### Technical Distinction: Rumoca vs. CasADi

| Aspect | **Rumoca** | **CasADi** |
| :--- | :--- | :--- |
| **Role** | **Modelica Frontend / Compiler** (Rust) | **Symbolic Math & Algorithmic Differentiation Engine** |
| **Primary Function** | Parses declarative multi-physics `.mo` files into Differential-Algebraic Equations (DAEs). | Calculates exact Jacobians/Hessians and interfaces with non-linear solvers (IPOPT). |
| **Code Level** | High-level component equations (masses, dampers, motor curves). | Matrix graph expressions and symbolic vector calculus. |
| **Output** | Flattened state equations $\dot{x} = f(x, u)$ & zero-dependency C++ code. | Optimal control trajectories and feedback gains. |

### Modelica & C++ SIL Header Generation
1. **Physical Equations (`ggGridRunner_physics.mo`):** Defines 4-wheel rotational speeds $(w_{FL}, w_{FR}, w_{RL}, w_{RR})$, friction scrub coefficients $\mu(\omega)$, motor back-EMF, and steering angles $\delta_i$.
2. **Rumoca C++ Export:** Compiles `.mo` into lightweight C++ structs representing system dynamics.
3. **Desktop Unit Testing:** The generated headers are included directly in PlatformIO `native` desktop unit tests (`test/test_odometry.cpp`) to verify `Odometry.cpp` dead-reckoning logic against simulated wheel slip without requiring physical hardware.

---

## 7. Unified 3D Simulation & Software-in-the-Loop (SIL)

Webots serves as the single unified simulator combining physics, code execution, and XBee radio communication:

1. **3D Physics Domain (Webots ODE Engine):** Loads the URDF exported from Onshape via `onshape-to-robot`. Simulates 4WD hub motor torque, double-wishbone suspension travel, and tire scrub against ground terrain.
2. **Firmware Code Domain (PlatformIO `env:native`):** `rover-firmware` (STM32 Blue Pill) and `controller-firmware` (Arduino Pro Micro) are compiled for desktop and run directly as Webots controller plugins.
3. **Radio Communication Domain (Virtual Emitter / Receiver Node):** Webots built-in Emitter/Receiver nodes (or virtual null-modem serial ports `COM3 ↔ COM4`) emulate the 57600 baud binary `ControlPacket` and `TelemetryPacket` protocols (`shared/Protocol.h`), with configurable latency and packet drop rates.

---

## 8. Headless Slicing & Wireless 3D Print Deployment

### Step 1: Background Slicing via PrusaSlicer CLI
The AI Agent triggers a fast 0.5-second headless slice using PrusaSlicer CLI with 20% Gyroid infill:

```bash
prusa-slicer --export-gcode \
  --load ~/.config/PrusaSlicer/printer/LulzBot_Mini_SE_0.5.ini \
  --fill-density 20% \
  --fill-pattern gyroid \
  --output Subframe_Front_Steering.gcode \
  Subframe_Front_Steering.stl
```

### Step 2: Digital Twin Mass Feedback Loop
The AI Agent reads the sliced G-code header metadata (e.g., `Plastic mass: 18.4 g`), and automatically updates the corresponding mass property (`<mass value="0.0184"/>`) in the simulation URDF link definition. This ensures **100% real-world mass match in 3D physics simulation**.

### Step 3: Wireless Auto-Print via OctoPi API
The sliced G-code is dispatched directly to the 3D printer via OctoPi's HTTP REST API:

```bash
curl -H "X-Api-Key: YOUR_OCTOPRINT_API_KEY" \
  -F "file=@Subframe_Front_Steering.gcode" \
  -F "print=true" \
  "http://octopi.local/api/files/local"
```

---

## 9. Phased Implementation Roadmap & Reusability

### Phased Roadmap
- **Phase 1 (Current):** Low-level STM32 microcontroller bringup (`rover-firmware`) and hardware validation.
- **Phase 2 (SIL & Physics Integration):** Export Rumoca C++ physics models to test `Odometry.cpp` and `HeadingSystem.cpp` dead reckoning under simulated tire slip.
- **Phase 3 (3D Simulation):** Integrate full Webots URDF model for gamepad control and obstacle avoidance validation.
- **Phase 4 (Automated Build Pipeline):** Fully automate the Onshape $\rightarrow$ PrusaSlicer $\rightarrow$ OctoPi workflow for fast physical iterations.

### Reusability Blueprint
This architecture is modular and directly adaptable to future robotics projects:
- Replace `ggGridRunner_physics.mo` with robotic arm or quadrupeds physics equations.
- Maintain the same Onshape REST API, Rumoca $\rightarrow$ CasADi, Webots, and PrusaSlicer CLI pipeline for any physical hardware build.
