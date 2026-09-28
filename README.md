# multithread_sims

A fast C++ 6-DOF rocket Monte Carlo flight simulator for WPI HPRC. Targeting 10ms latency per flight to drogue deployment.

It is to be a software-in-the-loop (SITL) sim: the real flight software, compiled for desktop, flies thousands of randomized flights in parallel (one process per flight). Controllers run either inside the flight software or directly from the sim, so new controllers can be tested before they are integrated. The same plant is built to later drive a hardware-in-the-loop (HITL) rig, where the real flight computer runs in real time.

Design: [proposal](proposal.md) and the [diagrams](docs/diagrams/).
