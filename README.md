# Smart Vehicle

## 1. Project Overview
This project implements a small embedded vehicle platform that integrates power distribution, PWM-based motor and steering control, camera-based ArUco marker detection, and a web-based control interface.

The main objective was to design and test a hardware-software integrated system.  
The project focused on power stability, actuator control, web-to-vehicle communication.

## 2. Key Features
- Raspberry Pi-based vehicle control server
- PWM control for ESC and steering servo through PCA9685
- Battery-powered hardware system with voltage conversion and fuse protection
- Web-based grid map interface for path setting
- WASD manual control interface
- Camera-based ArUco marker detection

## 3. System Flowchart
Web Interface  
|  
| HTTP / WebSocket  
v  
Raspberry Pi 3 Model B  
|  
| I2C  
v  
PCA9685 PWM Driver  
|  
| PWM Signal  
|  
|------------------> ESC --> DC Motor  
|  
|------------------> Steering Servo 

<br/>

Camera Module  
|  
v  
OpenCV / ArUco Marker Detection 

<br/>

Battery Pack  
|  
|----> Step-down Moduule / UBEC ---> Raspberry Pi, Servo, PWM Driver  
|  
|----> ESC ---> DC Motor

## 4. Hardware Components
### Computing and Sensing Unit
| Component | Specification | Role |
|------------------------|------------|-----------------|
| Raspberry Pi 3 Model B | 5 V, 2.5 A | Main controller |
| Raspberry Pi Camera Module 3 | CSI-2 camera | ArUco marker recognition |
| Step-down Module | 5 V output | Stable power conversion |

### Actuation Unit
| Component | Specification | Role |
|-----------------|-------------------------|------------------|
| PCA9685 | 16-channel PWM driver | Servo and ESC PWM signal generation |
| DC Motor | 7.2 V, 2.3 A | Rear-wheel drive |
| ESC | 2 - 3 S | DC motor speed control |
| Servo Motor | 7.4 V | Front steering control |
| UBEC | 2 - 8 S, 5 - 7.4 V output	| Stable power conversion |

### Power Supplying and Charging Unit
| Component | Specification | Role |
|-----------------|-------------------------|------------------|
| Battery Pack | 2S2P battery configuration	| Main power source |
| BMS | 2S, 8.4 V, 20 A | Battery protection |
| Charging Module | 1 - 4 S, 5 - 26 V input | Battery charging |
| USB-C PD Trigger Module | 20 V PD output| Power supplying |

### Auxiliary Components
| Component | Specification | Role |
|--------------------------|-------|-------------------|
| Power Distribution Board | 200 A | Power distridution|
| Capacitor | 1000 µF | Voltage drop preventing |
| Blade Fuse | Rated according to current path | Overcurrent protection |
| LED(Red / Green, Blue)	| 2.2 V / 3.2 V, 20 mA | Status indicator |
| Hex Screw | M3, 8 mm / 10 mm / 12 mm | Unit assembling |
| Nut | M3 | Unit assembling |

## 5. Power Design
The power system was designed by separating the load requirements of each unit.

- The Raspberry Pi is powered by a regulated 5 V supply.
- The steering servo is powered through a UBEC to reduce voltage drop and supply stable current.
- The DC motor is powered through the ESC.
- LEDs are connected with a current-limiting resistor.
- Fuses are placed according to the estimated current of each power branch.

### Load-Side Power Budget
| Load | Voltage | Current | Estimated Power | Calculation |
|------------------------|-----|-------|--------|---------|
| Raspberry Pi 3 Model B | 5 V | 2.5 A | 12.50 W | 5 × 2.5 |
| Servo Motor| 7.4 V | 3.4 A | 25.16 W | 7.4 × 3.4 |
| DC Motor | 7.2 V | 2.3 A | 16.56 W | 7.2 × 2.3 |
| LED(Red / Green, Blue) | 2.2 V / 3.2 V | 20 mA | 0.044 W / 0.064 W | 2.2 × 0.02 / 3.2 × 0.02 |

### Main Load Subtotal
| Branch | Load-Side Power | Assumed Efficiency | Battery-Side Power | Calculation |
|------------------------|---------|-----|---------|--------------|
| Raspberry Pi 3 Model B | 12.50 W | 85% | 14.71 W | 12.50 / 0.85 |
| Servo Motor| 25.16 W | 85% | 29.60 W | 25.16 / 0.85 |
| DC Motor | 16.56 W | Direct | 16.56 W | 16.56 |
| LEDs | 0.172 W | - | 0 W(Too small) | - |
| Total(Excluding LEDs) | 54.22 W | - | 60.87 W | 14.71 + 29.60 + 16.56 |

### Battery-Side Power Budget
| Battery Condition | Battery Voltage	| Estimated Battery-Side Power |	Estimated Current | Calculation |
|---------------|-------|---------|--------|--------------------|
| Fully charged |	8.4 V	| 60.87 W |	7.25 A | 60.87 / 8.4 = 7.25 |
| Nominal	| 7.4 V	| 60.87 W	| 8.23 A | 60.87 / 7.4 = 8.23 |
| Drain |	6.0 V	| 60.87 W	| 10.15 A | 60.87 / 6.0 = 10.15 |

### Fuse Selection
| Fuse Position	| Current Basis |	Recommended Fuse |
|-------------------|-----------------------------|---------------------|
| Main battery line	| Normal current about 8–10 A |	15 A slow-blow fuse |
| Charging line	| 1–4 A charging current | 5 A fuse for 4 A charge / 2 A fuse for 1 A charge |
| Raspberry Pi 5 V branch	|	5 V, 2.5 A | 3 A fuse |
| Servo / PCA9685 power branch | 7.4 V, 3.4 A | 5 A fuse |

### BMS Selection
| Item | Basis | Recommended Specification |
|--------------|--------------------|---------------|
| Battery type | 21700 Li-ion cells | 2S Li-ion BMS |
| Battery configuration | 2S2P pack | 8.4 V full-charge compatible |
| Normal current demand | about 8–10 A | 20 A continuous discharge minimum |
| Peak current demand | DC motor startup + servo load | 30 A continuous discharge preferred |
| Peak discharge rating | short motor/servo surge | 40–60 A peak discharge preferred |
| Charging current | 2–3 A charging branch | 3–5 A charge rating |
| Cell balancing | 2 cells in series | Balancing function recommended |

The power design was checked using Ohm's law, Kirchhoff's laws, and basic circuit theory:

- V = I × R (Ohm's law)
- P = V × I (Power equation)
- R = (V_supply - V_f) / I (Current-limiting resistor)
- I_1 + I_2 + I_3 + ⋯ + I_n = 0 (Kirchhoff's current law, KCL)
- V_1 + V_2 + V_3 + ⋯ + V_n = 0 (Kirchhoff's voltage law, KVL)
- I_total = I_1 + I_2 + I_3 + ⋯ + I_n (Current addition in a parallel circuit)
- V_supply = V_1 + V_2 + V_3 + ⋯ + V_n (Voltage addition in a series circuit)
- C_pack = C_1 + C_2 + ⋯ + C_n (Capacity addition in parallel)
- I_cell ≈ I_pack / n (Current division in parallel)
- η = P_out / P_in (Power conversion efficiency)

## 6. Motor and Steering Control
The ESC and steering servo are controlled using PWM signals generated by the PCA9685.

- ESC channel: 0
- PWM frequency: 50 Hz
- Neutral pulse width: 1500 µs
- Steering control: left/right angle adjustment through PWM
- Stop logic: ESC signal returns to neutral pulse width

The steering servo is designed to return to the neutral position when no active steering command is given.

## 7. Main Software Functions
- Web server for vehicle command transmission
- Manual driving control using WASD keys
- PWM signal generation for ESC and servo
- Camera streaming
- ArUco marker detection
- Emergency stop and neutral control logic
- Path and marker data loading from a grid mapping file

## 8. Limitations
- Camera recognition is sensitive to lighting conditions.
- Battery voltage drop can affect actuator stability.
- Mechanical steering precision is limited by servo calibration.
- The current system is designed for a small test environment.
- The project uses a Simple Pathfinding Algorithm(BFS).

## 9. Future Improvements
- Improve lighting control and camera exposure settings.
- Add battery voltage monitoring
- Calibrate the servo center and steering pulse range more precisely.
- Test the system on a larger and more complex track.
- Apply A* or other advanced path-planning algorithms.

## 10. Team Member Roles
**Kim Minseong [[Seasoning3](https://github.com/Seasoning3)]**
- Circuit Design
- Circuit configuration
- Vehicle Design
- Vehicle Making
- Web Programming
- Vehicle Programming
- Test Environment Setup
- Archiving

**Park Gibeom [[gibeom308](https://github.com/gibeom308)]**
- Test Environment Setup

**Paek Youngjae [[Paek Yeongjae](https://github.com/paekyeongjae)]**
- Test Environment Setup

**Jeon Seungmin [[tmdals914](https://github.com/tmdals914)]**
- Vehicle Programming
- Test Environment Setup
