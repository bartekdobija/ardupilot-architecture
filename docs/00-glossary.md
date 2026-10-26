# 00 · Glossary

Acronyms and terms used across this analysis, in the ArduPilot source tree and in its documentation. Entries marked *(name)* are brand or project names rather than acronyms, but are listed because they appear everywhere.

**[← Back to index](../README.md)**

---

## The project and its vehicles

| Term | Stands for | What it means in ArduPilot |
|---|---|---|
| UAV | Unmanned Aerial Vehicle | Any drone/robot vehicle; ArduPilot also drives boats, rovers and submarines |
| UGV | Unmanned Ground Vehicle | Wheeled/tracked rovers (`AP_MotorsUGV`) |
| ROV | Remotely Operated (underwater) Vehicle | The vehicle ArduSub controls |
| VTOL | Vertical Take-Off and Landing | Quadplane tailsitters, tiltrotors and similar |
| FPV | First-Person View | Camera-piloted drones; also the `DJI_FPV` serial protocol |
| GCS | Ground Control Station | Mission Planner, QGroundControl, MAVProxy… — talks MAVLink to the vehicle |

## Platform, boards and operating systems

| Term | Stands for | What it means in ArduPilot |
|---|---|---|
| HAL | Hardware Abstraction Layer | The [`AP_HAL`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_HAL) interface set with one backend per platform, reached through the global `hal` object |
| RTOS | Real-Time Operating System | ChibiOS on STM32 targets; plain POSIX everywhere else |
| ChibiOS | *(name)* | The RTOS behind all ~460 STM32 board targets ([`AP_HAL_ChibiOS`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_HAL_ChibiOS)) |
| SITL | Software-In-The-Loop | Firmware compiled as a desktop executable against [`libraries/SITL`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/SITL) physics models |
| HITL / HIL | Hardware-In-The-Loop | A real flight board flying against simulated sensors |
| POSIX | Portable Operating System Interface | The UNIX API family used by the SITL and Linux backends |
| QURT | Qualcomm "QuRT" | The RTOS on Qualcomm Hexagon DSPs (Snapdragon boards) — [`AP_HAL_QURT`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_HAL_QURT) |
| ESP32 | *(Espressif SoC family)* | IoT-class MCU backend ([`AP_HAL_ESP32`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_HAL_ESP32)) built on ESP-IDF |
| WASM | WebAssembly | Experimental in-browser backend |
| STM32 | *(STMicroelectronics MCU family)* | The ARM Cortex-M chips of most supported flight boards |
| MCU | Microcontroller Unit | The single chip a flight controller is built around |
| FMU | Flight Management Unit | Pixhawk board-generation naming (FMUv2…FMUv6) |
| IOMCU | I/O co-processor MCU | Pixhawk-class companion chip doing RC-in/PWM-out ([`AP_IOMCU`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_IOMCU)) |
| hwdef | hardware definition | Per-board data file that generates the HAL configuration |
| ADC | Analog-to-Digital Converter | Analog inputs: battery dividers, airspeed sensors, RSSI pins |
| DMA | Direct Memory Access | Peripheral transfers without CPU help (UART receive, DShot output) |
| DSP | Digital Signal Processing | FFT helpers exposed by the HAL (used by CAN nodes, audio) |
| WDT | Watchdog Timer | Resets the MCU if the main loop stalls |

## Onboard buses and low-level interfaces

| Term | Stands for | What it means in ArduPilot |
|---|---|---|
| GPIO | General-Purpose Input/Output | Digital pins via `AP_HAL::GPIO`; on ChibiOS, `GPIOx_*` parameters |
| UART | Universal Asynchronous Receiver-Transmitter | Serial ports (`AP_HAL::UARTDriver`), mapped with `SERIALx_*` parameters |
| USB | Universal Serial Bus | Serial ports, SLCAN, logging on many boards |
| SPI | Serial Peripheral Interface | Fast sensor bus (`AP_HAL::SPIDevice`) |
| I²C | Inter-Integrated Circuit | Low-speed sensor bus — compasses, barometers, rangefinders |
| CAN | Controller Area Network | Robust differential bus carrying DroneCAN traffic |
| DroneCAN | *(protocol suite)* | CAN protocol for sensors, ESCs, batteries, GNSS ([`AP_DroneCAN`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_DroneCAN), libcanard) |
| UAVCAN | *(former name)* | DroneCAN's pre-2020 name — older docs and code still use it |
| SLCAN | Serial-Line CAN | CAN-over-a-USB-serial-adapter protocol |
| DSDL | Data Structure Description Language | The `.ds` files that define DroneCAN messages |
| PWM | Pulse-Width Modulation | Classic 1000–2000 µs servo pulses, driven by timer peripherals |
| DShot | *(protocol name)* | Digital ESC protocol over one wire; the bidirectional variant adds telemetry |
| PPM | Pulse-Position Modulation | Legacy "PPM-sum" encoding of all RC channels on a single pin |

## RC and pilot-link protocols

| Term | Stands for | What it means in ArduPilot |
|---|---|---|
| RC | Radio Control | The pilot's transmitter/receiver pair, decoded in the `rcin` thread ([`AP_RCProtocol`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_RCProtocol)) |
| RSSI | Received Signal Strength Indicator | RC-link quality, reported to the GCS |
| SBUS | Serial Bus *(Futaba)* | Inverted-UART RC protocol carrying up to 16 channels |
| CRSF | Crossfire *(TBS)* | Serial RC + telemetry protocol; also spoken by ExpressLRS |
| ELRS | ExpressLRS | Open-source RC link firmware that speaks CRSF |
| DSM | Digital Spectrum Modulation *(Spektrum)* | Satellite-receiver serial protocol (DSM2/DSMX) |
| SRXL / SRXL2 | *(Spektrum-led serial receiver protocol)* | Multi-vendor serial receiver protocol |
| ST24, SUMD, IBus… | *(legacy receiver protocols)* | Further serial receiver formats decoded by `AP_RCProtocol` |
| MSP | MultiWii Serial Protocol | Talks to Betaflight-style FCs and DisplayPort OSDs |
| LTM | Light Telemetry | Minimal telemetry-to-OSD protocol |

## Telemetry and external communication

| Term | Stands for | What it means in ArduPilot |
|---|---|---|
| MAVLink | Micro Air Vehicle Link | The primary GCS/vehicle protocol (v2); generated code, streams, routing ([`GCS_MAVLink`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/GCS_MAVLink)) |
| MAVProxy | *(GCS tool)* | Console GCS; the foundation of the `autotest` framework |
| QGC | QGroundControl | Cross-platform ground station |
| UDP / TCP | User / Transmission Control Protocol | IP transports for SITL, GCS and companion links |
| FDM | Flight Dynamics Model | SITL's coupling to external simulators (JSBSim, crrcsim) over UDP |
| LTE | Long-Term Evolution | Cellular telemetry modems (drivers and Lua examples) |
| NMEA | National Marine Electronics Association | GPS/depth sentence protocol; also an output format |
| SBP / SBF | Swift Binary Protocol / Septentrio Binary Format | Binary GNSS protocols supported by `AP_GPS` |
| RTK | Real-Time Kinematic | Centimetre-grade GNSS using carrier-phase corrections |
| RTCM | Radio Technical Commission for Maritime Services | Correction message format (RTCM3) injected for RTK |
| ADS-B | Automatic Dependent Surveillance–Broadcast | Aircraft transponder reception ([`AP_ADSB`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_ADSB)) |
| AIS | Automatic Identification System | Marine transponder reception for surface vehicles |
| EFI | Electronic Fuel Injection | Engine/generator monitoring ([`AP_EFI`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_EFI)) |
| OSD | On-Screen Display | Text overlay on the video feed ([`AP_OSD`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_OSD)) |
| VTX | Video Transmitter | Controlled via [`AP_VideoTX`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_VideoTX) (Tramp, SmartAudio) |
| SPort | SmartPort *(FrSky)* | FrSky sensor-telemetry bus (with passthrough mode) |
| DDS | Data Distribution Service | Pub/sub middleware; [`AP_DDS`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_DDS) bridges it to ROS 2 |
| XRCE | (DDS-)XRCE | DDS profile for tiny devices — Micro-CDR wire format used by `AP_DDS` |
| ROS 2 | Robot Operating System 2 | Robotics framework bridged over DDS/XRCE |

## Sensors and state estimation

| Term | Stands for | What it means in ArduPilot |
|---|---|---|
| IMU | Inertial Measurement Unit | Gyro + accelerometer package ([`AP_InertialSensor`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_InertialSensor)); IMU samples pace the scheduler |
| INS | Inertial Navigation System | The namespace of the IMU frontends and chip drivers |
| GPS | Global Positioning System | Loosely: any GNSS receiver ([`AP_GPS`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_GPS)) |
| Baro | barometer | Pressure-based altitude ([`AP_Baro`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_Baro)) |
| Mag | magnetometer | Compass heading ([`Compass`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_Compass)) |
| LiDAR | Light Detection and Ranging | Rangefinder family ([`Rangefinder`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_RangeFinder)) |
| AHRS | Attitude and Heading Reference System | [`AP_AHRS`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_AHRS): the fused attitude/position/velocity API over the EKF |
| EKF | Extended Kalman Filter | EKF3 is the current 24-state estimator (EKF2 legacy); one lane per enabled IMU |
| GSF | Gaussian-Sum Filter | EKF3's yaw estimator when magnetic/GPS heading is unavailable |
| DCM | Direction Cosine Matrix | The legacy attitude estimator (and the matrix math itself), fallback behind EKF3 |
| DAL | Data Abstraction Layer | [`AP_DAL`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_DAL) replay-safe wrappers enabling deterministic EKF log replay |
| NED | North-East-Down | ArduPilot's frame convention for vectors and fields |
| TAS | True Airspeed | Airspeed used by TECS and the EKF |
| DOF | Degrees of Freedom | e.g. `AP_Motors6DOF` for submarine thrusters |

## Control and navigation

| Term | Stands for | What it means in ArduPilot |
|---|---|---|
| PID | Proportional–Integral–Derivative | The controller primitive ([`AC_PID`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AC_PID)) |
| TECS | Total Energy Control System | Fixed-wing altitude/airspeed control via total energy ([`AP_TECS`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_TECS)) |
| L1 | *(Park–Deyst–How guidance law)* | Nonlinear lateral path-following guidance ([`AP_L1_Control`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_L1_Control)) |
| WP | WayPoint | Mission items interpreted by [`AP_Mission`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_Mission) |
| RTL | Return-To-Launch | Automatic return-home mode (SmartRTL ≈ path-simplified variant) |
| ESC | Electronic Speed Controller | Motor drivers commanded through `hal.rcout` (PWM/DShot); some send telemetry |

## Storage, parameters and logging

| Term | Stands for | What it means in ArduPilot |
|---|---|---|
| EEPROM | Electrically Erasable Programmable ROM | Parameter storage area — flash-emulated on most modern boards |
| FRAM | Ferroelectric RAM | Non-volatile storage backend of [`StorageManager`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/StorageManager) on some boards |
| ROMFS | Read-Only Memory File System | Build-time embedded files (Lua scripts, OSD fonts, default params) via `AP_ROMFS` |
| SD | Secure Digital | Removable card holding logs, scripts and parameters |
| Dataflash | *(historical name)* | The flight log ([`AP_Logger`](https://github.com/ArduPilot/ardupilot/tree/e96bd9053e/libraries/AP_Logger)) — named for the flash chips of early boards |

## Build, testing and tooling

| Term | Stands for | What it means in ArduPilot |
|---|---|---|
| waf | *(build tool)* | The Python build system with ArduPilot-specific extensions |
| CI | Continuous Integration | GitHub Actions builds and test runs per PR |
| gtest | googletest | Unit-test framework in `libraries/*/tests` |
| autotest | *(test framework)* | The Python-driven SITL integration suite (~160 test classes) |
| Doxygen | *(docs tool)* | API-reference generator, configured in `ardupilot/docs/` |
| API | Application Programming Interface | The set of functions/classes a library exposes |
| RAII | Resource Acquisition Is Initialization | C++ pattern used by HAL helpers like `WITH_SEMAPHORE()` |
| GPLv3 | GNU General Public License v3 | The ArduPilot firmware license |

## Naming conventions you will see everywhere

| Pattern | Meaning | Example |
|---|---|---|
| `AP_*` | "ArduPilot" — shared library usable by (almost) any vehicle | `AP_GPS`, `AP_Mission` |
| `AC_*` | "ArduCopter" lineage — the multicopter control stack | `AC_AttitudeControl`, `AC_WPNav` |
| `AR_*` | "ArduRover" lineage — the ground-vehicle control stack | `AR_WPNav`, `AR_Motors` |
| `SITL::` | simulator classes feeding the POSIX backend | `SITL::SIM`, `SITL::SIM_Airspeed` |
| `*_Backend` | the device driver behind a frontend | `AP_GPS_Backend`, `AP_InertialSensor_Backend` |
| `hal.*` | the one global `AP_HAL::HAL` instance | `hal.console`, `hal.rcout` |
| `ATT`, `POS`, `XKF`… | three/four-letter log message tags written by `AP_Logger` | `XKF1` = EKF3 lane 0 |

---

**[← Back to index](../README.md)** · Next: [01 · Architecture Overview](01-architecture-overview.md)