# Project 7: Optimization of Handover Mechanisms in 5G Vehicular Networks (V2X)

## Project Description

This project develops and evaluates an optimized handover strategy for **5G Vehicle-to-Everything (V2X)** communication to reduce connection drop rates and latency in high-speed vehicular scenarios.

As vehicles travel at highway speeds, they frequently switch connections between base stations (handovers). In legacy LTE networks, this process becomes unreliable above 100 km/h. This project addresses that problem through three complementary approaches:

- **MDP + Q-Learning** — An intelligent handover decision agent that learns the optimal policy (when to stay vs. hand over) based on signal quality, vehicle speed, and ping-pong history.
- **High-Speed Mobility Simulation** — Models a highway scenario with 11 base stations to measure handover failure rates and latency across speeds from 60 to 200 km/h.
- **LTE vs. 5G NR Comparison** — Benchmarks both technologies on failure rate, latency, throughput, and ping-pong count under identical conditions.

---

## File Structure

```
Project7_V2X_Handover/
│
├── handover_mdp_qlearning.m          # Step 1: MDP Q-Learning agent
├── handover_highspeed_simulation.m   # Step 2: High-speed handover simulation
├── handover_LTE_vs_5G_comparison.m   # Step 3: LTE vs 5G NR comparison
└── README.md                         # This file
```

---

## How to Run the MATLAB Code

### Prerequisites
- MATLAB **R2020b or later** (see Dependencies section below)
- No additional toolboxes are strictly required for the core simulation

### Steps

**1. Open MATLAB** and set the working directory to the project folder:
```matlab
cd('path/to/Project7_V2X_Handover')
```

**2. Run Step 1 — MDP Q-Learning:**
```matlab
run('handover_mdp_qlearning.m')
```
- Trains a Q-Learning agent over 5,000 episodes
- Prints the learned optimal policy table to the Command Window
- Generates a 4-panel figure: reward convergence, HO count, failure count, Q-value preference

**3. Run Step 2 — High-Speed Simulation:**
```matlab
run('handover_highspeed_simulation.m')
```
- Simulates a vehicle traversing 11 base stations at 5 different speeds (60–200 km/h)
- Prints a summary table of failures and latency per speed
- Generates a 6-panel figure including RSRP trace and serving BS log

**4. Run Step 3 — LTE vs 5G Comparison:**
```matlab
run('handover_LTE_vs_5G_comparison.m')
```
- Runs both LTE and 5G NR simulations under identical conditions
- Prints a side-by-side performance improvement summary
- Generates a 6-panel comparison figure (failure rate, latency, throughput, ping-pong, HO count, latency trend)

> Each script is fully self-contained and can be run independently in any order.

---

## Dependencies

| Requirement | Version | Notes |
|---|---|---|
| MATLAB | R2020b or later | Core language requirement |
| Statistics and Machine Learning Toolbox | Optional | Only needed if extending with statistical tests |
| Communications Toolbox | Optional | Not used in current scripts; useful for future extensions with realistic channel models |
| 5G Toolbox | Optional | Not used currently; recommended for future work with 3GPP NR channel models |

> **No paid toolboxes are required to run the three main scripts.** All simulations use built-in MATLAB functions only (`rand`, `randi`, `movmean`, `bar`, `plot`, etc.).

---

## Expected Output

### Step 1
```
Episode  500 | Avg Reward: 183.26 | Epsilon: 0.082
Episode 1000 | Avg Reward: 241.37 | Epsilon: 0.010
...
Episode 5000 | Avg Reward: 245.34 | Epsilon: 0.010

Avg Reward (last 500 eps):   245.34
Avg HO Count (last 500 eps): 26.21
Avg Failures (last 500 eps): 2.88
```

### Step 2
```
Speed:  60 km/h | HOs:  4 | Failures: 1 | Fail Rate: 25.0% | Avg Latency: 48.0 ms
Speed: 200 km/h | HOs: 15 | Failures: 5 | Fail Rate: 33.3% | Avg Latency: 60.0 ms
```

### Step 3
```
Avg Throughput — LTE: 618.2 Mbps | 5G NR: 2906.4 Mbps
Avg Latency    — LTE: 130.6 ms   | 5G NR:   44.2 ms
Overall Latency Improvement: 66.2%
```

---

## Key Results Summary

| Metric | LTE | 5G NR | Improvement |
|---|---|---|---|
| Avg Handover Latency | 130.6 ms | 44.2 ms | **66.2% reduction** |
| Avg Throughput | 618 Mbps | 2906 Mbps | **4.7x increase** |
| Failure Rate @ 200 km/h | 23.1% | 16.7% | Reduced |
| Time-To-Trigger (TTT) | 160 ms | 40 ms | 4x faster |
