# MOTION: Phase-Resonance Control

**MOTION** is a nonlinear control primitive designed for bounded actuation, extreme energy efficiency, and deterministic execution on resource-constrained embedded systems.
https://zenodo.org/records/19394737

## Key Features
* **Geometric Bounding:** Uses the intrinsic geometry of the $S^1$ manifold to bound control effort ($u = -K \sin(\Delta\varphi)$), avoiding artificial saturation artifacts.
* **Port-Hamiltonian Foundation:** Formally proven stability (AGAS), passivity, and incremental stability via contraction analysis.
* **FPU-Free Architecture:** Pure 16-bit integer arithmetic with 10-bit Sine LUT, optimized for MCU ISR execution (STM32, ESP32, AVR).
* **High Efficiency:** Demonstrated **30.8% reduction** in control effort compared to classical PID-clamping in large-angle regimes.

## Repository Structure
- `MOTION.pdf`: The technical white paper (Journal-ready version).
- `motion_logic.h/cpp`: Production-ready C++ core.
- `killer_benchmark.py`: Python script for dynamic performance comparison.
- `motion_angle_sweep_ieee.png`: Energy scaling benchmark results.

## Theoretical Background
The system is modeled as a damped second-order gradient-like dissipative system on a compact Riemannian manifold. For multi-DOF systems, it acts as an energy-based analog computing substrate similar to Kuramoto networks and Ising machines.

## License
This project is released as **Open Prior Art** under the [CC-BY-4.0](https://creativecommons.org/licenses/by/4.0/) license.
