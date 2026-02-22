# Offcenter Visualization

Live demo: https://offcenter-visualization.streamlit.app/

**Overview**

- **Purpose:** A small interactive Streamlit tool and analytics script that visualizes and solves the "shooter face-target" yaw problem: compute the robot yaw so an off-center shooter on the robot points at a world target.
- **Components:** `main.py` — Streamlit visualization and interactive solver; `sim.py` — numerical solver and Monte Carlo simulator for convergence analysis.

**How It Works**

- **Pose math:** Uses 2D poses (x, y, yaw) and 2×2 rotation matrices to compose robot and shooter transforms.
- **Goal:** Find robot yaw θ_r so that shooter heading θ_r + θ_s points from the shooter position to the target: θ_r + θ_s = atan2(y_t - y_s(θ_r), x_t - x_s(θ_r)).
- **Approach:** `main.py` uses a simple iterative solve (fixed-point style) to find a yaw that satisfies the aim equation and visualizes old/new robot and shooter boxes, headings, and the shooter→target vector. `sim.py` implements a Newton-style root solver (`solve_yaw`) and a Monte Carlo `run_sim` to measure convergence, failures, and iteration counts.

**Files**

- **`main.py`**: Streamlit app. Key features:
  - Interactive inputs for robot/shooter bounding boxes and offsets, robot pose, and target position.
  - Iterative solver (up to 30 iterations) to find a robot yaw that points the shooter at the target.
  - Matplotlib visualization embedded in Streamlit showing robot and shooter before/after solution, headings, and vectors.
  - Displays final pointing error (deg) and solved robot yaw (deg).
- **`sim.py`**: Simulation/analysis script. Key features:
  - `solve_yaw(rx, ry, rtheta, sx, sy, tx, ty, max_iter, tol_deg)` — Newton-style solver that returns (iterations, converged).
  - `run_sim(N, ...)` — runs many random trials, reporting failures and iteration stats and printing the worst-case input.
- **`requirements.txt`**: Python deps for Streamlit and plotting (ensure installed before running the app).

**Usage**

- **Run the Streamlit app locally:**

```bash
pip install -r requirements.txt
streamlit run main.py
```

- **Open the hosted demo:**

Use the live demo at: https://offcenter-visualization.streamlit.app/

- **Run the simulation locally:**

```bash
python sim.py
```

**Streamlit inputs (summary)**

- **Bounding Box Sizes:** `Robot Width`, `Robot Height`, `Shooter Width`, `Shooter Height`.
- **Robot Pose:** `Robot X`, `Robot Y`, `Robot yaw (deg)` (initial guess).
- **Shooter Offset:** `Shooter X offset`, `Shooter Y offset`, `Shooter yaw (deg)` (shooter pose relative to robot frame).
- **Target:** `Target X`, `Target Y`.

**Notes & Tips**

- The numerical solver in `sim.py` is more robust and uses Newton-style updates with analytic derivatives; `main.py` uses a simple iterative fixed-point loop for clarity and quick visualization.
- If the target is extremely close to the shooter (division by near-zero), the solver returns failure in `sim.py` to avoid numerical issues.

If you'd like, I can also:

- add example screenshots to this README,
- add a small badge with the live demo link, or
- update `requirements.txt` to pin versions used for the demo.
