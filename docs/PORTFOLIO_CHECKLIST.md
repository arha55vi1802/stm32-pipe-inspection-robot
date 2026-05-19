# Portfolio checklist — do these in order

Use this after your code is on GitHub. Check off each item when done.

---

## Step A — Screenshots (15 minutes)

1. Run the robot and open the dashboard: `http://169.254.52.10/`
2. Take a **full-screen browser screenshot** → save as `docs/images/dashboard.png`
3. Take a **photo of the hardware** (Nucleo + sensors + motors) → `docs/images/hardware.png`
4. Optional: short test in pipe → `docs/images/demo.png`
5. In **GitHub Desktop**: commit message `Add portfolio images` → **Push origin**

The README gallery will show images automatically once files exist.

---

## Step B — GitHub profile (5 minutes)

1. Open [github.com/arha55vi1802](https://github.com/arha55vi1802) → **Edit profile**
2. Bio example: *Embedded systems · STM32 · sensors · robotics firmware*
3. **Customize your pins** → pin **stm32-pipe-inspection-robot**
4. On the repo page → **⚙ About** → **Topics**: `stm32`, `embedded-systems`, `robotics`, `iot`, `c`, `sensor-fusion`

---

## Step C — README personal touches (2 minutes)

In `README.md` (Author section), add when ready:

- Professional email
- Demo video link (30–60 s: dashboard + motors or pipe test)

Commit: `Update README contact and demo link` → Push.

---

## Step D — Fiverr gig (20 minutes)

1. Open `docs/FIVERR_GIG_COPY_PASTE.txt`
2. Create gig → paste **title**, **description**, **tags**
3. Add **3 images** (same as `docs/images/`)
4. Set price (e.g. Basic $25–40 for sensor integration)
5. Paste GitHub link in gig description

---

## Step E — LinkedIn post (10 minutes)

Template:

> Built firmware for a pipe-inspection robot on STM32F401: multi-sensor monitoring, W5500 web dashboard, and encoder–IMU odometry (~2.6% distance error in testing).  
> Code: https://github.com/arha55vi1802/stm32-pipe-inspection-robot  
> Open to embedded / STM32 freelance work.

Attach `hardware.png` or `dashboard.png`.

---

## Step F — Viva / defense (reference)

- **Distance:** encoders + Kalman filter; IMU for ZUPT and display velocity
- **Not claimed:** full 3D SLAM, GPS, or map-based localization
- **MCU choice:** F401RE — sufficient perf, Nucleo ecosystem, HAL support
- **Validation:** ~1.56 m physical → ~1.60 m reported

---

## Done?

When A–E are checked, your public portfolio loop is complete: **GitHub → Fiverr → LinkedIn → viva slides**.
