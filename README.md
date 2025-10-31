# ⚙️ Robust Control Synthesis: $H_{\infty}$ and $\mu$-Synthesis for Active Suspension System

## 🌟 Overview

This project focuses on designing and validating **robust controllers** for a quarter-car **Active Suspension System**. We implement and compare the performance of two industry-standard robust control methodologies—**$H_{\infty}$ Synthesis** and **$\mu$-Synthesis**—with a critical focus on balancing **Passenger Comfort** (Body Acceleration) and **Road Handling** (Suspension Deflection).

The work includes detailed state-space modeling, frequency domain analysis (Bode plots), actuator uncertainty modeling, and time-domain simulation under various road disturbances.

---

## ✨ Key Technical Achievements

| Category | Methodology / Tool | Contribution Highlight |
| :--- | :--- | :--- |
| **Control Theory** | **$H_{\infty}$ Synthesis** | Designed and optimized controller gains ($\gamma$) for performance criteria (Comfort, Balanced, Handling). |
| **Robust Control** | **$\mu$-Synthesis (Structure Singular Value)** | Implemented the $\mu$-Synthesis controller to guarantee **robust performance** against **actuator uncertainties**. |
| **System Modeling** | **State-Space Representation** | Derived the full linear time-invariant (LTI) state-space model of the quarter-car suspension system. |
| **Uncertainty Modeling** | **Multiplicative Actuator Uncertainty** | Modeled the actuator dynamics with a **multiplicative uncertainty block ($\Delta_{inc}$ and $W_{inc}$)** to simulate real-world variations (e.g., gain variation up to 40% at low frequencies and 2000% at high frequencies). |
| **Tools** | **MATLAB & Robust Control Toolbox** | Implemented the entire control pipeline using `ss`, `hinfsyn`, and `musyn` commands. |

---

## 🧠 Methodology and Design

### 1. Model and State Variables

The quarter-car model features two masses ($m_s$: sprung mass, $m_{us}$: unsprung mass).
* **State Vector ($\mathbf{x}$):**
    * $x_s$: Body travel 
    * $\dot{x}_s$: Body velocity
    * $x_{us}$: Wheel travel
    * $\dot{x}_{us}$: Wheel velocity
* **Inputs:** Road disturbance ($r$) and Actuator force ($f_s$).
* **Performance Outputs:** Body travel ($x_s$), Suspension deflection ($s_d = x_s - x_{us}$), and Body Acceleration ($\ddot{x}_s$).

### 2. $H_{\infty}$ Weighting Functions

Control objectives are met using high-pass and low-pass **weighting functions** ($W_{sd}$, $W_{ab}$) to penalize system gains in critical frequency bands.

* **Comfort:** Attenuate the gain $G_{11}$ ($\frac{x_s}{r}$) and $G_{31}$ ($\frac{\ddot{x}_s}{r}$) near the primary resonance frequency ($\approx 7.1~rad/s$).
* **Road Handling:** Attenuate $G_{21}$ ($\frac{s_d}{r}$) and $G_{22}$ ($\frac{s_d}{f_s}$).
* The $\beta$ parameter is used to balance the trade-off between Comfort ($\beta=0.01$) and Handling ($\beta=0.99$).

---

## 📊 Results and Robustness Comparison

### Time-Domain Validation (Nominal Performance)

Simulation results show that the $H_{\infty}$ controller successfully suppresses the primary resonance in body acceleration ($\ddot{x}_s$) compared to the open-loop system when subjected to various disturbances (sine bump, step, multiple steps).

### Robustness under Uncertainty

We compare the standard **$H_{\infty}$ controller** with the robust **$\mu$-Synthesis controller** (Balanced objective, $\beta=0.5$), simulating **20 random actuator models** within the defined uncertainty range.

<div align="center">
   
  <p><em>Fig 1: Time-domain response under multiple step disturbances (simulating poor road conditions) with 20 uncertain actuator models.</em></p>
</div>

| Controller | Observation on Uncertain Models (Yellow Envelope) | Robustness demonstrated |
| :--- | :--- | :--- |
| **$H_{\infty}$** | Shows noticeable **spread and oscillation** in the output envelope (especially body acceleration) when models deviate from nominal. | Good, but sensitive to high-frequency actuator changes. |
| **$\mu$-Synthesis** | The output envelope is **significantly tighter and more bounded**, demonstrating superior control across the entire family of uncertain actuators. | **Superior Robustness.** The controller guarantees performance bounds, meeting the advanced robust design objective. |

## 🚀 Reproduction

### Prerequisites
* **MATLAB** (Required for the code base)
* **MATLAB Control System Toolbox**
* **MATLAB Robust Control Toolbox** (Required for `hinfsyn` and `musyn` functions)

### Files
**`Code_toolbox.m`:** The main script containing the system definitions, weighting function setup, $H_{\infty}$ synthesis, $\mu$-Synthesis, and time-domain simulations.

### Execution
1.  Open **MATLAB**.
2.  Run the **`Code_toolbox.m`** script. The script automatically executes the entire synthesis and generates the Bode and time-domain plots.

---
