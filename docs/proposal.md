# Multithreaded Rocket Simulation Proposal - Architecture & Plan

**Contents**

- [Multithreaded Rocket Simulation Proposal - Architecture \& Plan](#multithreaded-rocket-simulation-proposal---architecture--plan)
  - [Problem](#problem)
  - [Objective](#objective)
  - [Constraints](#constraints)
  - [Proposed Solution](#proposed-solution)
    - [Interfaces](#interfaces)
        - [Rocket Interface](#rocket-interface)
        - [Controller interface](#controller-interface)
          - [Simulink Exporting](#simulink-exporting)
        - [Flight software interface](#flight-software-interface)
        - [Sensor model](#sensor-model)
        - [State estimate](#state-estimate)
        - [Actuator model (within the plant)](#actuator-model-within-the-plant)
    - [Performance Aware Design Decisions](#performance-aware-design-decisions)
    - [Monte Carlo methodology](#monte-carlo-methodology)
        - [Selection of QMC Sampling](#selection-of-qmc-sampling)
        - [Error Modeling](#error-modeling)
        - [Epistemic Error Modeling](#epistemic-error-modeling)
        - [Aleatory Error Modeling](#aleatory-error-modeling)
        - [Approach](#approach)
          - [σ table:](#σ-table)
          - [Exploration Strategy (approach to sampling epistemic points)](#exploration-strategy-approach-to-sampling-epistemic-points)
          - [Example MonteCarlo Driver Config](#example-montecarlo-driver-config)
    - [Results and the Deliverable](#results-and-the-deliverable)
  - [Verification and validation](#verification-and-validation)
  - [Out of Scope](#out-of-scope)
  - [Other Notes](#other-notes)

## Problem
The current Simulink/Matlab flight sim takes ~100s per flight on Rohan's machine and ~600s on Christopher's. There has been no way to quantify the rocket's stability margins in a reliable and fast way, i.e.,
> **How far can our aerodynamic coefficients and physical parameters be off before
> the rocket becomes unstable or the controller fails?'

## Objective
We seek to implement a full 6-DOF rocket flight simulator in C++ fast enough to run large ensembles of Monte Carlo. The same system will eventually serve as a test harness for HPRC controller development, facilitate continuous integration, and drive a HITL rig. 

## Constraints
- Simulations run from launch through drogue deployment. Real-time flights are ~300s total with roughly 40s to apogee. Results should be reproducible and bit-exact for a given binary (requires pining the compiler + C library in a container). Any run must be replayable and visualizable. Target run latency is 10ms with baseline of 100s. 
- Controllers are implemented in Simulink (exported as C) or written by hand (ex. the canard LQR). We assume all controllers run at the same control rate. 
- x86-64 Linux is priority. 
- The sim is to run the flight software itself, not a copy. The flight software is being rewritten in Rust this year. Flights run as separate processes because the current flight software uses globals; if the flight software keeps all its state in one struct, threads work instead.

## Proposed Solution



### Interfaces
The purpose of the following design choices is to allow the simulation to minimize change necessary to the implementation of the simulation when the physical rocket or definitions of potential source of error are modified. 

##### Rocket Interface
The vehicle is defined in a config that lists the set of sensors, nominal state estimate error, N controllers, and actuator settings. These are loaded in at runtime (in addition to the intervals and distributions being swept). 

Below is a rough example of the json config (the RunConfig):
```json
{
  "vehicle": {    // paths, not values — the yellow box
    "parameters":  "Data/30k_v2/parameters.json",
    "aero_tables": "Data/30k_v2/drag_curve.csv",
    "thrust":      "Data/30k_v2/thrust_source.csv"
  },

  "sim": { "dt_s": 0.001, "ctl_period_s": 0.01, "t_max_s": 60, "integrator": "rk4" },

  "sensors": { "imu": { "...": "..." }, "baro": {}, "mag": {}, "gps": {} },

  "estimate_error": {   // truth + sampled error, no filter
    "att_bias_rad": 0.01, "att_drift_radps": 0.001,
    "vel_bias_mps": 0.5, "corr_time_s": 10.0, "latency_s": 0.02,
    "update_hz": { "att": 100, "alt": 25 }  // held between updates
  },

  "flight_software": true,   // step the pinned flight software (estimator flag is a build option)

  "controllers": [   // source: called by the flight software, or by the sim directly
    { "name": "canard_lqr", "type": "GainSchedulerLQR", "source": "fsw", "rate_hz": 100,
      "drives": ["canard_yp", "canard_ym", "canard_zp", "canard_zm"],
      "params": { "gain_table": "Data/30k_v2/K.csv" } },
    { "name": "apogee_ctl", "type": "AirbrakeController", "source": "sim", "rate_hz": 100,
      "drives": ["airbrake"],
      "params": { "target_apogee_m": 9144 } }
  ],

  "actuators": [ { "name": "canard_yp", "...": "..." } ],

  "events": {
    "apogee": { "method": "brent" },
    "deploy": { "rule": "vel_ned_window", "threshold_mps": 0.2,   // only when flight software is off; otherwise its pyro output
                "source": "estimate", "t_end_buffer_s": 2.0 }
  },

  "logging": { "coarse_hz":0, "apogee_window_hz": 100 }
}
```
Two constraints keep later additions (ex. the bending moment calculator) from needing a plant refactor:
- The plant's derivative function fills a plant outputs struct (forces, moments, accelerations, mass properties, q, Mach) rather than only returning $\dot{s}$. Anything that only observes the flight (loads, logging of $\alpha$ and q$\alpha$) reads from it instead of recomputing aero.
- Vehicle data stays in files the RunConfig points to, so a new model's data (ex. a joint table: station, diameters, mass ahead, share of normal force ahead vs Mach) is one more file, not a RunConfig change.
##### Controller interface
Controllers are to be declared with the actuator(s) they drive, the number of outputs, and a single update function called by both the sim and flight computer. 
```cpp
enum class Actuator { CANARD_YP, CANARD_YM, CANARD_ZP, CANARD_ZM, AIRBRAKE };

class IController {
public:
    virtual void reset() = 0;
    virtual const Actuator* drives() const = 0;     // which surfaces, declared once. Setup catches multiple controllers on same actuator
    virtual int  n_outputs() const = 0;
    virtual void update(const StateEstimate& x, float t, float dt, float* u) = 0;
    virtual ~IController() {}
};
```

Everything the controller needs should arrive through the constructor or update() arguments (ex. gain tables, target apogee). We model the controller's gain (if the controller is stronger or weaker than designed) and delay (which we can then compute phase margin from). 

Note on implementation: each sim owns one array of actuator commands (where each controller has it's own offset determined before the run). When every controller has written, the sim passes the whole array to the plant, which holds it constant until the next control tick.

Each controller runs in one of two places, set per controller by `source` in the RunConfig, with the same controller code in both:
- `fsw`: the sim steps the flight software, which calls the controller at its own rate and flight phase.
- `sim`: the sim calls it directly at the control rate. This is for controllers not yet in the flight software. When the flight software is also running, a sim-side controller uses the flight software's phase and estimate, so it sees what it would once integrated.

Mixing is allowed (ex. canard LQR in the flight software, a new airbrake controller from the sim). Each actuator is driven by exactly one source, checked at setup. Running the nominal flight with a controller in each place and comparing checks its integration.

Delay is modeled at the edges of the flight computer, so it applies the same way wherever a controller runs: estimate latency and update rate on the way in (see State estimate), controller gain and delay on the commands on the way out. With the flight software running, its own scheduling adds real delay on top, so the swept delay there stands for what the sim does not model (ex. servo communication, execution time).

So that the C++ flight software or the Rust rewrite can pick up any controller, a controller:
- exposes a C interface (init, step) with all its state in one struct passed in. No globals or singletons, which also keeps threads possible later.
- makes no hardware or clock calls (`millis()`, logging, file reads). Time and gains arrive as arguments.
- does not allocate in step.
- states its inputs, outputs, units, and rate.

The flight software team then only writes glue: fill the inputs from the estimate, call step when control is due, and send the outputs to the servos. In Rust, that is `bindgen` on the controller's header plus a safe wrapper.

###### Simulink Exporting
Simulink controllers must be exported as C with reusable function packaging, which meets the rules above. For sims, we simply write a wrapper.

```cpp
template <class Gen>
class SimulinkController : public IController {
    Gen gen_;   // Simulink output, untouched
public:
    void reset() override { gen_.initialize(); }
    void update(const StateEstimate& x, float t, float dt, float* u) override {
        gen_.rtU.roll_rad = x.rpy[0];             // standard bus, same every model
        gen_.rtU.altitude_m = x.altitude;
        // ...
        gen_.step();
        for (int i = 0; i < n_; ++i) u[i] = gen_.rtY.u[i];
    }
};
```
Template + inheritance allows abstraction of which generated model and allows all controllers to sit in one array 🤩. If all models use the same Simulink bus object, this wrapper is written once. 

- Simulink Export Requirements (which later can be enforced on PR)
	- C, reusable function code interface packaging (all state in one struct)
	- disabled memory allocation
	- non-finite support enabled
	- MAT-file logging
	- fixed-step discrete solver
	- data type that matches the data type of the state estimate
	- hardware implementation matching the build target (x86-64)
##### Flight software interface
The flight software lives in its own repo, owned by the flight software team, and is pulled in as a git submodule pinned to one commit (recorded in provenance). It is compiled from source as part of the sim's build, for x86-64 instead of the STM32, and linked into the sim binary, so each flight's copy runs inside that flight's process. Updating the flight software means moving the pin. The flight software's CI builds against the sim, so a change that breaks the interface is caught there.

The sim talks to it through a small, versioned C interface that both the current C++ flight software and the Rust rewrite can expose:
```c
FlightSoftware* fsw_create(const FswConfig* config);    // flight start
void fsw_step(FlightSoftware* fsw, const FswInputs* inputs, FswOutputs* outputs);   // every plant step (1 kHz)
void fsw_destroy(FlightSoftware* fsw);                   // flight end
```
- `FlightSoftware`: an opaque handle to one running copy of the flight software. The sim never looks inside it.
- Config: startup switches only, the estimator mode and which actuators the sim drives in mixed mode (see Controller interface). Sensors and controllers are whatever the flight software build contains.
- In: sim time (`uint64_t` µs, never the host clock), the latest reading from each sensor with its sample time, and the state estimate in truth mode.
- Out: actuator commands, deploy/pyro events, flight phase, and the flight software's own estimate (used by sim-side controllers).
- Everything hardware specific (clock, sensors, servos, SD card, radio) sits behind the interface. In the sim, logging and radio do nothing.
- Estimator flag: MEKF or truth + sampled error. A field in `FswConfig`, so one binary covers both. Sweeps run truth mode; MEKF mode is for HITL and CI.
- Deterministic: the same math library in both builds (ex. the `libm` crate in Rust) and no unseeded randomness, so replays stay bit-exact.

Every flight starts with a few simulated seconds on the pad, since the flight software calibrates and detects launch from sensor data. At 1 kHz, ~40 s to apogee is ~40,000 steps, so each step gets ~250 ns of the 10 ms target. Truth mode may fit; MEKF mode likely won't. To be measured early. Desktop timing is not the STM32's, so overruns need a timing model (modeled execution time added to the sim clock), which is a spike.
##### Sensor model
Sensor models turn truth into what each sensor would read, errors included, and feed the flight software every run: its state machine detects launch and burnout from the accelerometer and barometer even in truth mode. The MEKF also reads them, but only in HITL and CI. They also produce synthetic logs for Abhay's navigator.

We need to model:
- individual rate of the sensor
- sensor latencies, biases, noise
- mounting error
- dropout
- saturation/clipping

Each added sensor is simply a new struct + config entry in the RunConfig, along with a defined reading function from a truth vector. The "corruption" function of a perfect reading is simply defined once. 

```json
"sensors": {
  "imu":  { "rate_hz": 100, "latency_s": 0.0,
            "accel_noise_mps2": 0.05, "accel_bias_mps2": 0.1, "accel_bias_rw": 0.001,
            "gyro_noise_radps": 0.002, "gyro_bias_radps": 0.01, "gyro_bias_rw": 1e-5,
            "accel_range_g": 32, "gyro_range_dps": 2000,
            "misalign_rad": 0.002, "dropout_prob": 0.0 },
  "baro": { "rate_hz": 25, "latency_s": 0.02, "noise_pa": 5.0, "bias_pa": 20.0 },
  "mag":  { "rate_hz": 50, "...": "..." },
  "gps":  { "rate_hz": 1,  "latency_s": 0.1, "noise_m": 2.5 }
}
```


##### State estimate
We define a state estimate struct owned by the simulation driver, filled from truth plus a sampled error, and handed to the flight software in place of its MEKF output (truth mode, used for every sweep). Abhay's MEKF is tuned against real flight logs, so any sensor noise we invent here would only tune a filter against our own fiction. What we take from him instead is the error characterization from real flight residuals.
```cpp
void estimate(const State& truth, const EstError& err, double t, StateEstimate& out);
```
Error is bias, drift rate, correlation time, latency, and update rate. Each channel is held between updates at its `update_hz` (ex. altitude at the barometer's 25 Hz), which adds staleness on top of latency. Zero error is a sample rather than a separate mode, so nominal and perturbed runs go through identical code. White noise is not the interesting part: zero-mean noise on the estimate averages out in the controller, while bias and latency eat phase margin. For HITL the real filter runs onboard and we send raw measurements instead.
##### Actuator model (within the plant)
To consider both the dynamics of the actuator and its effectiveness, we model the dynamics with first order lag, rate/movement limit, and position limit. Dynamics go straight into the integrator. The effectiveness is determined by an aerodynamics and environment submodule in the plant that takes in the rocket's physical configuration. 
```json
"actuators": [
  { "name": "canard_yp", "type": "canard",
    "phi_deg": 0, "x_m": 0.65, "area_m2": 0.0034, "cl_delta": 0.25,
    "tau_s": 0.02, "rate_max_radps": 5.2, "pos_lim_rad": [-0.26, 0.26] },
  { "name": "canard_ym", "phi_deg": 180, "...": "..." },
  { "name": "canard_zp", "phi_deg":  90, "...": "..." },
  { "name": "canard_zm", "phi_deg": 270, "...": "..." },
  { "name": "airbrake", "type": "airbrake", "x_m": 2.9,
    "cd_table": "airbrake_cd.csv",
    "tau_s": 0.15, "rate_max": 0.8, "pos_lim": [0.0, 1.0] }
]
```


### Performance Aware Design Decisions
- Analysis of the step size and convergence should be done early as possible. It is the biggest performance lever. 
- Interleave coefficient table. For a given mach and $\alpha$, we get each coefficient. For locality, we fetch them all in the same lookup. We trip the mach range too, which allows use of doubles. (Tables can still stay in L1). Cache the lookup cursor since the mach changes slowly between steps. 
- Divergent samples stop sooner than drogue deploy.
- Per-phase (aero, EOM, state estimate, etc.) timing counters are placed behind a compile flag. Not really a profiler, just accumulated counters. Allows targeted optimization. 
- Persistent pool of worker processes that fetch from an atomic job queue, one process per flight. The results array lives in shared memory created before fork, and each index is written by exactly one process, so no locks. At ~8 KB per record (summary + coarse trajectory + apogee window) a 20,000-sample sweep holds ~160 MB in RAM and writes it once, after every worker exits.
- Pre-touch the results array before the sweep, or first-write page faults land inside the hot loop.
- Have each worker grab 64 indices at a time (conventional chunk size, can be adjusted) that they then work on rather than grabbing one index at a time from the job queue in the MonteCarlo Driver. 

### Monte Carlo methodology

##### Selection of QMC Sampling
Plain MC has error of $\frac{\sigma}{\sqrt{N}}$. So standard error shrinks with $N^{-0.5}$. To halve the error you need 4x more samples. 

Quasi-Monte Carlo replaces random points with a low discrepancy sequence (a set of deterministic numbers that fills a space more evenly and with fewer gaps than random or pseudorandom numbers). Simply put, we get a standard error that shrinks with $N^{-1}$ instead. 
##### Error Modeling
We consider two types of uncertainty, Aleatory (inherent randomness modeled by a probability distribution) and Epistemic (uncertainty due to lack of knowledge, modeled with an interval). 
- Aleatory example: wind on launch day. Epistemic example: Coeff. of Normal Force at $8° \alpha$ 

Adding a new source of error requires a simple entry into the $\sigma$ table (described further below) and the parameter manifest. Based on the blast radius of the change, the impact can be as simple as tweaking a model (a couple lines of change) to refactoring code (adding new physics/dynamics). 

All QMC sampled values, along with a seed, are stored in a sample struct per simulation. 

The `Sample` struct is generated at build time via a python script on a YAML config file. Names, units, model consuming it are dclared in the YAML and the python script emits the enum, the struct, and the name table:

```cpp
enum Param : int { k_CN, k_CD, k_Cm, rail_angle, /* ... */ N_PARAMS };
inline constexpr const char* kParamNames[] = { "k_CN", "k_CD", ... };

struct Sample {
    double   v[N_PARAMS];
    uint64_t seed;
    constexpr double operator[](Param p) const { return v[p]; }
    void dump(FILE*) const;     // named output for logs and the debugger
};
```

Three things follow. The $\sigma$ table's `parameter` column is validated against `kParamNames` at load, so a typo becomes a startup error naming the row rather than a parameter silently dropped from the sweep. The sampler becomes a loop over `v[]` instead of one assignment per parameter. And the manifest and the $\sigma$ table are separate files on purpose: the manifest changes when new physics is added, the $\sigma$ table changes every study, and only the first triggers a rebuild. Every swept parameter is a continuous `double`. 
##### Epistemic Error Modeling
Interval selected throughs sources such as research papers, manufacturing tolerance/spec sheet, spread from estimation methods, or documented engineering judgement. 

| Source                           | Current value                                      | Lands in                                           |
| -------------------------------- | -------------------------------------------------- | -------------------------------------------------- |
| normal force multiplier          | 1.0                                                | Aerodynamics                                       |
| drag multiplier                  | 1.0                                                | Aerodynamics                                       |
| pitch moment multiplier          | 1.0                                                | Aerodynamics                                       |
| canard effectiveness `CL_delta`  | 0.25, labelled a guess                             | Aerodynamics                                       |
| roll damping `Cd_x`              | 0.3, labelled a guess                              | Aerodynamics                                       |
| pitch/yaw damping `Cd_y`, `Cd_z` | 0.5, labelled guesses                              | Aerodynamics                                       |
| CP location, transonic shift     | held constant through transonic                    | Aerodynamics                                       |
| controller gain error            | 1.0                                                | Controllers                                        |
| controller delay                 | 0                                                  | Controllers                                        |
| state estimate error             | bias, drift, corr time, latency — from Abhay; update rate | State Estimate                                     |
| sensor dropout rate              | 0                                                  | Sensor Models                                      |
| IMU bias vs temperature coeff    | absent                                             | Sensor Models                                      |
| actuator `tau`                   | 0.02 s                                             | Actuator Dynamics                                  |
| actuator `rate_max`              | **0.2 vs 5.2 rad/s unresolved** (MATLAB vs Config) | Actuator Dynamics                                  |
| bearing coulomb                  | absent                                             | Spin can submodel in Mass/Inertia and Aerodynamics |
| bearing viscous                  | absent                                             | Spin can submodel in Mass/Inertia and Aerodynamics |

Sensor Model rows (here and in the aleatory table) apply to every run, since the flight software's state machine reads the sensors.

##### Aleatory Error Modeling
Access distributions from sources such as launch site climatology, manufacture specs, or build tolerance. 

| Source                    | Lands in                   |
| ------------------------- | -------------------------- |
| wind speed                | Aerodynamics + Environment |
| wind direction            | Aerodynamics + Environment |
| turbulence realisation    | Aerodynamics + Environment |
| launch day temperature    | Aerodynamics + Environment |
| launch day pressure       | Aerodynamics + Environment |
| rail angle                | Initial Conditions         |
| rail azimuth              | Initial Conditions         |
| motor lot to lot impulse  | Mass, Inertia + Propulsion |
| thrust misalignment angle | Mass, Inertia + Propulsion |
| mass                      | Mass, Inertia + Propulsion |
| lateral CG offset         | Mass, Inertia + Propulsion |
| inertia                   | Mass, Inertia + Propulsion |
| per fin cant error        | Aerodynamics               |
| sensor bias realisation   | Sensor Models              |
| sensor noise sequence     | Sensor Models              |
| which samples drop        | Sensor Models              |

##### Approach
We run a sample over a nested loop as the different sources of error that we do not combine because they represent fundamentally different types of uncertainty. Cost multiplies as `(epistemic points) × (aleatory samples)`, so keep the outer loop coarse: corners and a few interior points of the interval box, not a dense grid.

Raw output is a p-box: a bounded family of CDFs rather than one curve. So the deliverable is not *"3% of flights go unstable"* but *"between 1% and 9%, and the width of that band is the cost of not knowing our aerodynamics."* 

###### σ table:

| Column         | What it holds                                                               |
| -------------- | --------------------------------------------------------------------------- |
| `parameter`    | the binding key, must match the `Sample` field name exactly                 |
| `nominal`      | value when nothing is perturbed, `1.0` for every multiplier by construction |
| `variation`    | how far from nominal, read according to `distribution`                      |
| `distribution` | how to interpret `variation`, and which loop the row lives in               |
| `correlation`  | other parameters this one moves with, and how strongly                      |
| `bounds`       | hard physical limits, separate from `variation`                             |
| `applies_to`   | which model consumes it                                                     |
| `source`       | provenance, including "guess"                                               |
| `role`         | `swept` or `fixed`, carrying the evidence if fixed                          |

- Correlation column is required to prevent independent sampling from producing physically impossible vehicles that inflates the reported tolerance. 
- Given that we are using correlation, we use Shapley effects over Sobol indices (which assumes independence). 
###### Exploration Strategy (approach to sampling epistemic points)

| `strategy`  | Outer points                          | Gives                                                                      | Misses               |
| ----------- | ------------------------------------- | -------------------------------------------------------------------------- | -------------------- |
| `corners`   | `2^k` vertices of the box             | a bound, if monotone (P(fail) increasing/decreasing as single param moves) | nothing, if monotone |
| `levels`    | `k x levels`, one parameter at a time | a tolerance per parameter                                                  | interactions         |
| `hypercube` | `n` points scattered through the box  | interactions, an observed max                                              | no guarantee         |


###### Example MonteCarlo Driver Config
```json
{
  "name":        "k_CN_tolerance_v3",
  "run_config":  "config.json",
  "sigma_table": "sigma_table.csv",
  "output_dir":  "results/2026-09-19_kCN_v3",

  "explore": {
    "strategy": "levels",     // "levels" | "corners" | "hypercube"
    "levels":   25,           // used by "levels"
    "points":   500           // used by "hypercube"
  },

  "samples_per_point": 1000,  // aleatory draws at each outer point
  "seed":              20260919,

  "destruction": [            // hard clamps on truth, end the run
    { "channel": "tilt_deg", "max": 30 },
    { "channel": "roll_deg", "min": -30, "max": 30 }
  ],

  "workers": 0, // 0 means all cores

  "output": {
    "animate_worst":  5,
    "animate_random": 3
  }
}
```




### Results and the Deliverable
Result format is still an open question for the team leads. 

Failure is not defined by the sim. One pass/fail isn't general enough across teams, so the sim reports per-run metrics and each team applies its own failure definition to summary.parquet afterwards. The p-box is computed per team query at report time, so changing a failure definition never needs a rerun. The exception is destruction, which the sim always reports (count, percentage, cause, time):
- Hard clamps on truth (ex. tilt/roll ±30°), extendable to any per-step channel. Clamp tilt from vertical rather than Euler pitch/yaw, which are singular for a vertical rocket.
- Controller output going non-finite.
- Flight computer abort. Currently a stub in the flight software, so this waits on the flight software team.

Structural failure (bending) is judged retrospectively, not clamped: the sim reports peak structural load (peak q$\alpha$ for now, peak bending moment and axial load per joint once the calculator is integrated), and structures applies allowables afterwards. The plant is a rigid body, so bending moment never feeds back into the trajectory and a threshold applied afterwards gives the same answer. The allowable moments are unknown right now, so they are not sampled. Anything after a joint's first exceedance (apogee, RMSE) is not physical, so queries should treat it as the end of the run.

Error is reported against the nominal flight: every σ table row at its nominal value. Nominal is sample 0, run first through the identical code path, and its 100 Hz trajectory is frozen read-only. Each worker accumulates RMSE against it at control ticks. Nominal doubles as a determinism check at the start of every sweep.

Results of each simulation are written into memory. Once all simulations are finished, the Monte Carlo driver writes a directory of results. Worst runs can be found by sorting summary.parquet by any metric (ex. peak $\alpha$). 

Note that provenance refers to git SHA, binary hash (of the executable), compiler + flags, CPU, libc, wall time. Allows results to be defended and reproduced. 

```
results/<timestamp>_<config_hash>/
  manifest.json      provenance
  config.json        the exact RunConfig
  sigma_table.csv    the exact σ table
  summary.parquet    one row per sample, the thing you query
  traj.bin           coarse trajectories, fixed stride, memory-mappable by index
  report.md          the deliverable, one per sweep
  figs/*.png         referenced by report.md
```

summary.parquet columns: destroyed/aborted + cause + time, apogee and Δ vs nominal, peak tilt, peak $\alpha$, peak roll rate, min static margin, RMSE vs nominal per channel, controller saturation time, peak q$\alpha$, peak bending moment per joint (+ time and axial load at peak, once integrated), out-of-table counters. Full time series (ex. bending moment) only for replayed runs, since every run is reproducible from its seed.

The report contains summary of run config, along with sensitivity to different coefficients/parameters, destruction counts by cause, RMSE vs nominal charts, per-team failure queries (and their p-boxes), and command to replay any sample.

## Verification and validation
Post verification (does the code solve the equations correctly), we can validate the simulations against: 
- RocketPy: Python 6-DOF, validated against real flights. Computes normal force from dimensions itself, but drag must be supplied. 
- jsbsim: Independent EOM and integrated. Not independent aero.
- Or upcoming flight data, wind tunnel data, ANSYS CFD. 


## Out of Scope
- The simulation focuses on testing the rocket's performance rather than elements such as MEKF implementation validation, Code quality CI. 
- Descent/parachute simulation is considered a reach end-of-year goal by Rohan. 
- MacOS deployment of the simulations. Priority is x86-64 Linux. 
- Optimization such as SIMD (single instruction, multiple data), custom allocators (like object pools), lock free structures, GPU offload, distributed execution beyond a job pool are all out of scope. 
- An originally proposed "estimator model" where an MEKF/direct truth state could be interchangeable. Abhay, the student working on the MEKF, can provide empirical error bounds from real flight residuals. Integrating an MEKF into the sim yields nothing about estimator accuracy, since the sensor noise would be our own invention. So sweeps run truth mode: the flight software reads a state estimate filled from truth plus sampled error (bias, drift rate, correlation time, latency). The MEKF stays in the flight software behind the estimator flag, for HITL and CI only. 
- NASA has a library called Trick recommended by Rohan. It is more of a scheduling framework where you write models and its interface code generator parses your headers and schedules jobs. It was built for multiple subsystem models from many teams over decades, a pressure that does not exist with WPI's HPRC team. May revisit when designing HITL system. 
- Any built in screening (ex. Morris pass) to cut down on the number of epistemic parameters to those that actually matter. Any inputted epistemic parameters and intervals are assumed to be important to test. 
- We do not model chirp. We instead use gain and pure delay. The chirp would need to be at a frozen operating point that a rocket never holds. 
- We do not consider frequency-dependent uncertainty. We assume that coefficient error is frequency-flat across the loop bandwidth. Thus, any unsteady aerodynamics (flow lags a changing $\alpha$) and structural bending modes are absent. 
- A SysId optimizer wrapped around the plant that fits parameters to recorded flight telemetry. Out of scope for now. 
- for the HITL: the HITL would be single thread, real-time driver instead of MonteCarlo driver that logs overruns such as issues with OS scheduler preemption, page faults, cache misses, interrupt handling, transport latency. The driver would sleep until the start of the next time frame. The controllers would run on an onboard MEKF while the servos within the Gimbal are driven by the simulation plant. 
- Note the eventual goal of also having any changes to the open rocket file across subteams runs a set of C++ sims and automatically creates a PR to update the simulation config.
- Bending moment calculator integration. On preliminary screen, the previous year's [bending moment calculator](https://github.com/Helios-Rocket/rocket-bending-calculator) lacks canards, axial load, and mass change over the burn (part masses are frozen from the .ork), and its Barrowman aero would disagree with our aero tables. Until it is integrated, peak q$\alpha$ (dynamic pressure × $\alpha$) is the structural load proxy. The plant should still expose what it needs so integration stays easy: per-section normal force, lateral and angular acceleration, axial force, and mass distribution vs burn time. No allowable is sampled or clamped either way (see Results).

## Other Notes
- Scope of parallelism: one process per complete flight, due to the sequential nature of time stepping. Parallelism is across the Monte Carlo samples.
- Flight computer: STM32H753ZIT6: Cortex-M7 at 240 MHz with a hardware double-precision FPU, 512 KB RAM, 2 MB flash.
- Development: I have a Mac machine and a mini-pc with x86-64 Linux Ubuntu. 
- No global mutable state in the plant. 
- No heap allocation inside integration loop.
- Seed per sample index, not per worker.
- Reproducibility by pinning the build. Record the build env in every results file. Determinism tests. 
