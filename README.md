# ODE/PDE Solvers in Scala (Maven)

This repository provides two numerical examples implemented in **Scala**:

1. **PDE**: 2D heat transfer by conduction (explicit FTCS finite differences)
2. **ODE**: Cart–pole inverted pendulum with a **Sliding Mode Controller (SMC)**, including:
   - Exactly two disturbances at **t = 0.5 s** and **t = 5.0 s**
   - Each disturbance lasts **0.5 s**
   - Disturbance visualized as a constant-length arrow for **0.5 s** after each start
   - Frames saved to PNG, and an MP4 encoded if **ffmpeg** is available on `PATH`

All outputs are written under the `output/` folder (relative to where you run the commands).

Important note (Windows locale): the code forces **English/Latin digits** for plot tick labels even if your Windows display language is not English.

---

## Prerequisites (Windows 10)

- **JDK 11+** installed and available on `PATH`
- **Maven** installed and available on `PATH`
- Optional (for MP4): **ffmpeg** installed and on `PATH`

Quick checks (CMD or Git Bash):

```bash
java -version
mvn -version
ffmpeg -version
```

If `ffmpeg -version` fails, you will still get frames (PNG), but MP4 creation will be skipped.

---

## Build + run (three commands)

From the project root:

```bash
mvn -q -DskipTests package
mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.heat2d.Heat2DApp exec:java
mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.pendulum_sliding_mode.CartPoleSmcApp exec:java
```

Optional preview window during cart–pole generation:

```bash
mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.pendulum_sliding_mode.CartPoleSmcApp -Dexec.args="--preview" exec:java
```

Optional frame-count override (default is 6000 frames):

```bash
mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.pendulum_sliding_mode.CartPoleSmcApp -Dexec.args="--frames 6000" exec:java
```

---

## Outputs

### Heat 2D (`output/heat2d/`)

- `heat_tXXXX.png` : heatmap snapshots
- `centerline_final.png` : final centerline temperature
- `center_point_vs_time.png` : center temperature vs time
- `heat2d_log.csv` : CSV log of center temperature

### Cart–Pole SMC (`output/pendulum_sliding_mode/`)

- `frames/frame_000000.png ...` : rendered frames
- `pendulum_smc_10s_6000f.mp4` : encoded MP4 (only if ffmpeg is found)
- `cart_position.png`, `pole_angle.png`, `control_force.png`, `disturbance_torque.png`, `equivalent_bob_force.png`, `sliding_surface.png` : plots
- `cartpole_log.csv` : CSV log

---

## Plot quality settings

All saved charts are configured for:

- High-resolution PNG output sized for **~300 DPI** printing (default 2400×1800)
- Large fonts (titles, axis labels, tick labels)
- Thick plot lines
- At most ~10 tick labels per axis (by explicit tick spacing)
- At least 20 px padding on all sides
- English/Latin digits for numeric tick labels (locale-safe)

---

## File-by-file explanation (all files in this repository)

### Project root

- `pom.xml`
  - Maven build file: configures Scala compilation (scala-maven-plugin), dependencies (JFreeChart), and the `exec-maven-plugin` used to run the two applications.

### `scripts/` (optional convenience wrappers)

- `scripts/run_heat2d.cmd`
  - Windows CMD helper to build and run the Heat 2D example.
- `scripts/run_pendulum_smc.cmd`
  - Windows CMD helper to build and run the Cart–Pole SMC example.
- `scripts/run_heat2d.sh`
  - Git Bash / POSIX shell helper to build and run the Heat 2D example.
- `scripts/run_pendulum_smc.sh`
  - Git Bash / POSIX shell helper to build and run the Cart–Pole SMC example.

### Scala source code

- `src/main/scala/com/mohammadijoo/odepde/common/CsvUtils.scala`
  - Minimal CSV writer utilities:
    - strict column-length checks
    - 2-column and N-column CSV writing

- `src/main/scala/com/mohammadijoo/odepde/common/PlotUtils.scala`
  - High-quality plotting utilities (JFreeChart):
    - line plots (thick lines, large fonts, limited tick count)
    - heatmaps (XY block renderer + color legend)
    - forces English/Latin digits in tick labels (Locale.US numeric formatter)

- `src/main/scala/com/mohammadijoo/odepde/heat2d/Heat2DApp.scala`
  - PDE example entrypoint:
    - explicit FTCS time integration for the 2D heat equation
    - Dirichlet boundaries (left hot, other edges cold)
    - periodic snapshot heatmaps + final plots + CSV log

- `src/main/scala/com/mohammadijoo/odepde/pendulum_sliding_mode/CartPoleSmcApp.scala`
  - ODE example entrypoint:
    - nonlinear cart–pole dynamics integrated by RK4
    - sliding mode controller with boundary-layer saturation
    - two timed disturbance torques and arrow visualization
    - frame rendering (PNG), optional preview window, optional MP4 encoding via ffmpeg
    - plots + CSV log
