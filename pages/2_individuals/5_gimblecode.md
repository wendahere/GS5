---
title: Gimbal Control System (Software)
parent: Individual Contributions
nav_order: 4
permalink: /gimble/
---


<div style="display:flex">
  <img src="{{site.baseurl}}/assets/images/profiles/jg.jpg" alt="Hang Jin Guang" width="200" style="border-radius:50%">
  <div style="margin-left: 20px">
    <h2>Gimbal Control System (Software) </h2>
  </div>
</div>


[**4\. Gimbal Control System (Software)	3**](#4.-gimbal-control-system-\(software\))

[4.1 Scope of Project	3](#4.1-scope-of-project)

[4.1.1 Problem Statement	3](#4.1.1-problem-statement)

[4.1.2 Objective	3](#4.1.2-objective)

[4.1.3 Current Progress	4](#4.1.3-current-progress)

[4.2 Design Requirements	4](#4.2-design-requirements)

[4.3 System Architecture	5](#4.3-system-architecture)

[4.4 Detailed Software Design	6](#4.4-detailed-software-design)

[4.4.1 gclib.py – Galil Motor Controller Interface	6](#4.4.1-gclib.py-–-galil-motor-controller-interface)

[4.4.2 gimbal\_lib.py – High-Level Motion Control	6](#4.4.2-gimbal_lib.py-–-high-level-motion-control)

[4.4.3 TLE\_Set\_Builder.py – Orbital Data Retrieval	6](#4.4.3-tle_set_builder.py-–-orbital-data-retrieval)

[4.4.4 TLE\_Parser\_Test.py – Orbital Data Processing	6](#4.4.4-tle_parser_test.py-–-orbital-data-processing)

[4.4.5 pass\_scheduler.py – Pass Scheduling	6](#4.4.5-pass_scheduler.py-–-pass-scheduling)

[4.4.6 gps\_reader.py – GPS Data Acquisition	6](#4.4.6-gps_reader.py-–-gps-data-acquisition)

[4.4.7 live\_with\_gps.py – Real-Time Tracking	6](#4.4.7-live_with_gps.py-–-real-time-tracking)

[4.x Interpolation in Gimbal Motion Control	7](#4.5-interpolation-in-gimbal-motion-control)

[4.x.1 Purpose of Interpolation	7](#4.5.1-purpose-of-interpolation)

[4.x.2 Linear Interpolation	7](#4.5.2-linear-interpolation)

[4.x.3 Quadratic Interpolation	7](#4.5.3-quadratic-interpolation)

[4.x.4 PCHIP (Piecewise Cubic Hermite Interpolating Polynomial)	7](#4.5.4-pchip-\(piecewise-cubic-hermite-interpolating-polynomial\))

[4.x.5 Justification for Choosing PCHIP	7](#4.5.5-justification-for-choosing-pchip)

[4.x.6 Implementation in Code	8](#4.5.6-implementation-in-code)

[4.5 Current Challenges	8](#4.6-current-challenges)

[4.6 Future Work	8](#4.7-future-work)

[4.6.1 Software Improvements	8](#4.6.1-software-improvements)

[4.6.2 Hardware Integration	8](#4.7.2-hardware-integration)

[4.7 Key Implementation Lines (for Cross-Reference)	9](#4.8-key-implementation-lines-\(for-cross-reference\))

[A. gclib.py \- Galil Motor Controller Interface	9](#a.-gclib.py---galil-motor-controller-interface)

[B. gimbal\_lib.py \- High-Level Motion Control (imports gclib.py)	9](#b.-gimbal_lib.py---high-level-motion-control-\(imports-gclib.py\))

[C. live\_with\_gps.py \- Real-Time Tracking Loop	10](#c.-live_with_gps.py---real-time-tracking-loop)

[D. pass\_scheduler.py \- Next-Pass Selection	10](#d.-pass_scheduler.py---next-pass-selection)

[E. TLE\_Parser\_Test.py \- Pose Preview & .pass Generation	10](#e.-tle_parser_test.py---pose-preview-&-.pass-generation)

[F. TLE\_Set\_Builder.py \- Daily TLE Fetch	10](#f.-tle_set_builder.py---daily-tle-fetch)

[4.8 Annex Reference	11](#4.8-annex-reference)

# **4\. Gimbal Control System (Software)** {#4.-gimbal-control-system-(software)}

### **4.1 Scope of Project** {#4.1-scope-of-project}

The gimbal control system forms the **software backbone of the satellite ground station**, enabling automated and precise tracking of satellites in real time.  
 It integrates **orbital data**, **GPS positional feedback**, and **motor-controller communication** to continuously align the antenna with the satellite’s position throughout its visible pass.

The software framework was designed to be **modular and scalable**, ensuring ease of maintenance and adaptability for future improvements. Each module handles a specific process, such as orbital prediction, GPS integration, or real-time motion control.  
 This chapter details the system’s current implementation, its operational workflow, and ongoing challenges in achieving sub-degree accuracy and motion smoothness.

---

### **4.1.1 Problem Statement** {#4.1.1-problem-statement}

Commercial ground-station gimbal controllers are often expensive, proprietary, and limited in flexibility for research use.  
 Furthermore, **no open-source system** was available that could integrate TLE-based orbital prediction, GPS feedback, and motion control into one unified real-time framework.

Another significant challenge was that the **Galil motor controller lacked any available reference code or public documentation.**  
 This required the development of a custom interface library in Python through **trial and error** to establish stable communication and command execution.

The project therefore aimed to build a **fully custom gimbal control software**, capable of real-time satellite tracking with live GPS and TLE data integration, while remaining cost-effective and extensible.

---

### **4.1.2 Objective** {#4.1.2-objective}

The objective of the gimbal control system software is to achieve **real-time, autonomous satellite tracking** with smooth and stable motion, guided by accurate positional and heading feedback.

The system integrates:

* **Satellite orbital data** from TLE files

* **GPS-based** ground-station coordinates and heading

* **Motor control commands** issued through the Galil interface

By combining these data sources, the software maintains continuous satellite alignment during passes.  
 Although the current system achieves high reliability, **further improvement is required** to reach the targeted GPS heading accuracy of \< 0.2°.

---

### **4.1.3 Current Progress** {#4.1.3-current-progress}

At the current stage, the software has achieved full integration across all functional modules:

* **TLE parsing and prediction** using Skyfield

* **GPS positional feedback** through the dual-antenna receiver

* **Smooth gimbal motion** via PCHIP Hermite interpolation

* **Real-time tracking** with continuous motor updates

The gimbal successfully follows satellite trajectories and responds dynamically to position updates.  
 However, measured GPS heading accuracy **remains above 0.2°**, affecting overall pointing precision.  
 In addition, motion smoothness can be improved by optimizing interpolation step resolution and timing intervals.

---

### **4.2 Design Requirements** {#4.2-design-requirements}

| Parameter | Requirement | Description |
| ----- | ----- | ----- |
| **Real-Time Tracking** | Yes | System must continuously compute and update azimuth/elevation during passes. |
| **Heading Accuracy** | \< 0.2° (Target) | Current performance does not yet achieve this level; improvement ongoing. |
| **Position Accuracy** | ± 2.5 m | Based on GPS module capability. |
| **Update Rate** | ≥ 10 Hz | Ensures smooth and responsive control. |
| **Interpolation** | PCHIP Hermite | Monotonic cubic interpolation without overshoot. |
| **Communication Interface** | Serial / TCP (Galil gclib) | For command exchange with controller. |
| **Software Modularity** | High | Independent modules for GPS, TLE, scheduling, and motion. |

---

### **4.3 System Architecture** {#4.3-system-architecture}

The gimbal software is composed of **seven Python modules**, each handling a specific subsystem.

| No. | Module | Function / Description |
| ----- | ----- | ----- |
| **1** | `gclib.py` | Low-level hardware driver that communicates with the Galil controller through official API functions. |
| **2** | `gimbal_lib.py` | Imports `gclib.py` and implements high-level motion functions, angle limits, safety controls, and PT mode for continuous updates. |
| **3** | `TLE_Set_Builder.py` | Downloads and compiles Two-Line Element (TLE) data from CelesTrak. |
| **4** | `TLE_Parser_Test.py` | Parses TLE data and computes azimuth/elevation relative to the ground station. Generates `.pass` files for scheduling. |
| **5** | `pass_scheduler.py` | Identifies the next visible satellite pass based on current time. |
| **6** | `gps_reader.py` | Reads GPS coordinates and heading data, providing live positional feedback. |
| **7** | `live_with_gps.py` | Main execution script that integrates all modules for real-time satellite tracking. |

---

### **4.4 Detailed Software Design** {#4.4-detailed-software-design}

#### **4.4.1 gclib.py – Galil Motor Controller Interface** {#4.4.1-gclib.py-–-galil-motor-controller-interface}

Acts as the **foundation for motion control**, defining communication between Python and the Galil hardware library.  
 Implements command execution, connection handling, and motion-completion feedback.

#### **4.4.2 gimbal\_lib.py – High-Level Motion Control** {#4.4.2-gimbal_lib.py-–-high-level-motion-control}

Imports `gclib.py` and adds advanced control features such as:

* Limit enforcement

* Smooth acceleration/deceleration

* `degSteer()` for continuous steering under Position Tracking mode  
   This ensures stable and precise motion during live satellite passes.

#### **4.4.3 TLE\_Set\_Builder.py – Orbital Data Retrieval** {#4.4.3-tle_set_builder.py-–-orbital-data-retrieval}

Fetches up-to-date TLE data daily from CelesTrak and stores them locally for the parser and scheduler.

#### **4.4.4 TLE\_Parser\_Test.py – Orbital Data Processing** {#4.4.4-tle_parser_test.py-–-orbital-data-processing}

Processes TLE data using Skyfield to calculate azimuth and elevation, implements flip logic near ± 90°, and generates `.pass` files.

#### **4.4.5 pass\_scheduler.py – Pass Scheduling** {#4.4.5-pass_scheduler.py-–-pass-scheduling}

Reads `.pass` files, determines the next visible satellite pass, and provides both UTC and local times for tracking operations.

#### **4.4.6 gps\_reader.py – GPS Data Acquisition** {#4.4.6-gps_reader.py-–-gps-data-acquisition}

Reads live data from the GPS module, extracting latitude, longitude, altitude, and heading.  
 Currently, heading accuracy exceeds 0.2°, requiring further refinement.

#### **4.4.7 live\_with\_gps.py – Real-Time Tracking** {#4.4.7-live_with_gps.py-–-real-time-tracking}

Integrates TLE and GPS inputs, computes real-time AZ/EL, and uses PCHIP interpolation for smooth motion.  
 Streams continuous position updates to the gimbal controller.

---

### **4.5 Interpolation in Gimbal Motion Control** {#4.5-interpolation-in-gimbal-motion-control}

#### **4.5.1 Purpose of Interpolation** {#4.5.1-purpose-of-interpolation}

In the gimbal tracking system, the controller receives discrete position updates at fixed time intervals from orbital data.  
 Interpolation is required to estimate intermediate pointing angles between consecutive data points.  
 Without interpolation, the gimbal would move in abrupt steps, producing jerky motion, higher mechanical wear, and degraded stability.  
 Interpolation ensures that motor commands are smooth, continuous, and dynamically feasible.

#### **4.5.2 Linear Interpolation** {#4.5.2-linear-interpolation}

Connects consecutive data points with straight lines-simple and efficient but assumes constant angular velocity, causing sudden acceleration changes and vibration.  
 Adequate only for coarse control, not for precision tracking.

#### **4.5.3 Quadratic Interpolation** {#4.5.3-quadratic-interpolation}

Uses second-order polynomials, introducing curvature for smoother velocity transitions but lacking acceleration continuity and prone to overshoot or oscillation, leading to transient pointing errors.

#### **4.5.4 PCHIP (Piecewise Cubic Hermite Interpolating Polynomial)** {#4.5.4-pchip-(piecewise-cubic-hermite-interpolating-polynomial)}

PCHIP fits a monotonic cubic polynomial between data points, preserving both value and slope continuity.  
 Unlike natural cubic splines, it enforces **monotonicity** through the Fritsch–Carlson slope limiter to prevent overshoot.  
 Key advantages:

* Smooth (C¹ continuous) position and velocity transitions

* No overshoot even with irregular spacing

* Real-time numerical stability for control use

#### **4.5.5 Justification for Choosing PCHIP** {#4.5.5-justification-for-choosing-pchip}

The gimbal requires smooth, non-oscillatory motion with predictable velocity control.  
 PCHIP provides the best balance of computational simplicity, smoothness, and stability compared with linear or quadratic interpolation, making it ideal for real-time gimbal motion.

#### **4.5.6 Implementation in Code** {#4.5.6-implementation-in-code}

Implemented in `live_with_gps.py` via `pchip_slopes()` and `hermite_eval()` (**Annex B.A.7, L94–127**).  
 These compute cubic Hermite slopes and evaluate interpolated angles between consecutive TLE-derived positions.  
 Interpolated azimuth/elevation values are then streamed to the controller through `degSteer()` in `gimbal_lib.py`.

---

### **4.6 Current Challenges** {#4.6-current-challenges}

| Challenge | Description | Key Impact |
| ----- | ----- | ----- |
| **GPS Heading Accuracy** | Heading error has not yet achieved the \< 0.2° target. | Affects stability & orientation precision. |
| **PCHIP Interpolation** | Requires optimization of step timing & resolution. | Impacts motion smoothness. |
| **Spoilt Ethernet Port** | Physical Ethernet port on controller is damaged. | Causes unstable connection to the gimbal. |
| **Cable Management** | Motor-cable routing needs improvement. | Risk of entanglement during multi-axis motion. |

---

### **4.7 Future Work** {#4.7-future-work}

#### **4.6.1 Software Improvements** {#4.6.1-software-improvements}

* Refine interpolation step size and timing.

* Implement predictive motion algorithms for fast passes.

* Integrate feedback-based GPS-heading error correction.

#### **4.7.2 Hardware Integration** {#4.7.2-hardware-integration}

1. **Hardware Integration** – Achieve end-to-end connectivity across ground-station components (dish, feedhorn, LNB, GPS).

2. **Receiver Hardware Development** – Identify and validate the optimal receive chain for satellite downlink.

3. **Space Segment Radio Payload Integration** – Perform thermal and mechanical compatibility testing of the radio payload with the satellite bus.

---

### **4.8 Key Implementation Lines (for Cross-Reference)** {#4.8-key-implementation-lines-(for-cross-reference)}

**Purpose.** This subsection identifies the specific lines of code that implement each critical function used by the gimbal control software. Full source is provided in Annex B with line numbers; references use (Annex B.A.i, Lstart–Lend).

#### **A. gclib.py \- Galil Motor Controller Interface** {#a.-gclib.py---galil-motor-controller-interface}

* Open/Close controller: `GOpen`, `GClose` → (Annex B.A.1, L151–158; L161–168)

* Send commands: `GCommand` → (L171–180)

* Query info: `GInfo` → (L235–240)

* Assignments: `GAssign` → (L263–271)

* Transfers: `GProgramDownload`, `GArrayDownload` → (L293–302; L334–346)

* Wait for motion: `GMotionComplete` → (L435–442)

* Error handling: `_rc` \+ `GclibError` → (L116–121)

#### **B. gimbal\_lib.py \- High-Level Motion Control (imports gclib.py)** {#b.-gimbal_lib.py---high-level-motion-control-(imports-gclib.py)}

* PT mode: `_enter_pt`, `_exit_pt` → (L163–172; L174–181)

* Wait/settle: `_wait_inpos`, `_wait_settle_counts` → (L263–293)

* Motions: `move_absolute`, `move_relative` → (L355–406)

* Real-time steer: `degSteer` → (L408–456)

* Limits/scaling: constants → (L29–43)

#### **C. live\_with\_gps.py \- Real-Time Tracking Loop** {#c.-live_with_gps.py---real-time-tracking-loop}

* PCHIP math: `pchip_slopes`, `hermite_eval` → (L94–127)

* Flip thresholds: `FLIP_ON_DEG`, `FLIP_OFF_DEG` → (L56–57)

* Flip logic: `decide_flip_with_hysteresis`, `map_to_gimbal_frame_with_state` → (L129–156)

* Main execution: `main` → (L161–255)

* Command stream: `gimbal.degSteer(...)` → (L245)

#### **D. pass\_scheduler.py \- Next-Pass Selection** {#d.-pass_scheduler.py---next-pass-selection}

* Datetime conversion: `_ensure_utc_datetime`, `convert_datetime` → (L49–109)

* Future pass selection: `select_pass` → (L132–167)

* Machine output: `NEXT_PASS,...` → (L205; L254)

#### **E. TLE\_Parser\_Test.py \- Pose Preview & .pass Generation** {#e.-tle_parser_test.py---pose-preview-&-.pass-generation}

* Pass list: `generate_pass_list` → (L52–89)

* Main function: `main` → (L117–154)

#### **F. TLE\_Set\_Builder.py \- Daily TLE Fetch**  {#f.-tle_set_builder.py---daily-tle-fetch}

* **CelesTrak source & writeout:** `TLE_URL`, `requests.get(...)`, save file → **(Annex B.A.3, L12; L23; L27–35)**

**Why these matter:** Ensures you run with up-to-date orbital elements.

---

G. `gps_reader.py` \- GPS Fix & Heading (UBX-first)

* **Unit scaling (heading):** `_auto_heading_deg` → **(Annex B.A.6, L26–35)**

* **NMEA degrees-minutes → decimal:** `_dm_to_deg` → **(Annex B.A.6, L37–44)**

* **One-shot fix (UBX NAV-PVT; optional RELPOSNED):** `read_once` → **(Annex B.A.6, L47–142)**

  * UBX position match: **(L66)**

  * UBX dual-antenna heading: **(L80)**

  * NMEA fallbacks: **GGA (L92–106)**, **GLL (L108–120)**

* **Live monitor:** `stream_status` → **(Annex B.A.6, L144–177)**

**Why these matter:** These lines verify whether your **heading accuracy target (\<0.2°)** is met and document the data source (UBX vs NMEA).

### **4.8 Annex Reference** {#4.8-annex-reference}

The complete Python source code listings for the gimbal control system are provided in **Annex B – Gimbal Control Software Listings**, including inline documentation and comments for:

* `gclib.py`

* `gimbal_lib.py`

* `TLE_Set_Builder.py`

* `TLE_Parser_Test.py`

* `pass_scheduler.py`

* `gps_reader.py`

* `live_with_gps.py`

Each listing includes explanations of function usage and software flow to support reproducibility and future system development.
