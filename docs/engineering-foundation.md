# DROVER Digital Twin: engineering foundation

## Repository baseline

The starting repository provided only `configuration/drover_config.json`. It describes an approximately 300 mm X-frame, a 2.5 kg target mass, 0.5 kg payload capacity, a skirted air cushion, two rear motors, and target simulation frequencies. It does not include a CAD model, hardware calibration data, model code, or validation measurements. The motor, fan, skirt, inertia, servo, and battery data remain unknown.

## Coordinate and unit conventions

- SI units are used internally: metres, kilograms, seconds, newtons, pascals, radians, volts, amperes, and joules.
- Body axes are +X forward, +Y left, +Z up; the initial origin is the main-frame geometric centre.
- Center of mass is an independently configured body-frame offset. It is currently unknown.
- Orientation is represented internally by a normalized quaternion mapping body vectors into world coordinates. Euler angles are display outputs only.
- World +Z is up. Gravity acts along world -Z. Propulsion and lift forces are accumulated at their configured body-frame application points, then transformed into world coordinates for translation.
- The integrated translational state is the world position of the current combined center of mass. The rendered frame origin is recovered by subtracting the rotated center-of-mass offset.
- Three.js coordinates must be adapted at the rendering boundary; rendering conventions must not leak into physics calculations.

## Mass and inertia data structure

The physics configuration reserves dry mass, dry center of mass, and full symmetric dry inertia tensor about that center in body axes. All are unset pending a reconciled mass budget and CAD or measurement-derived inertia. The configured payload has a mass, location, and intrinsic inertia tensor; payload changes update total mass, center of mass, and inertia with the parallel-axis theorem.

The 2.5 kg target is not yet resolved as to whether it includes the stated 0.5 kg battery. It is treated as an unverified assembled-vehicle target, not a measured dry mass. Do not derive an inertia tensor from the approximate external dimensions.

## Model policy

- Every parameter is tagged by provenance: measured, manufacturer-specified, estimated, assumed, derived, or unknown.
- `null` represents an unavailable value. A calculation that depends on an unavailable value must report a configuration error; it must not silently substitute a plausible value.
- Simulation outputs are modeled results, not measurements. Uncalibrated models must identify that status in saved run metadata and the interface.
- The lift model tracks cushion air mass and pressure. Fan flow is bilinearly interpolated from a configured RPM/pressure/flow matrix; operation outside its measured or estimated domain raises an error. Leakage is based on configured geometry and pressure regime. The lift force comes from pressure differential times effective area; it is not rotor downwash thrust.
- Motor thrust, fan flow and pressure, actuator response, and electrical power require configured curves or clearly labeled estimated models.

## Numerical method

The initial documented method is fixed-step semi-implicit Euler for translational and angular velocity, with quaternion orientation advanced from angular velocity and renormalized each step. The target is 100 Hz. Force and moment evaluation, actuator dynamics, and cushion mass balance use the same simulation clock. This method is a starting implementation choice, not a claim of validated accuracy; timestep sensitivity and deterministic replay must be assessed before scientific use.

## Phase plan

1. **Foundation (current):** configuration schemas, coordinate conventions, model provenance, repository structure, and engineering documentation.
2. **Physics:** validated configuration loading, rigid-body dynamics, lift fan/cushion/leakage, propulsion, servos, and electrical model.
3. **Control and runtime:** constrained actuator commands, manual/differential/vectoring control, PID assistance, and deterministic simulation clock.
4. **Data and experiments:** persistence, raw telemetry, repeatable batch runs, eight experiment definitions, exports, and statistical analysis.
5. **Frontend:** dashboard and physics-driven 3D view using explicitly identified placeholder geometry until a DROVER CAD asset is supplied.
6. **Validation:** analytical/numerical checks and an import/compare workflow for prototype measurements. No experimental-validation claim without those measurements.

Each phase should have its own review gate. A generated file or UI view alone does not complete a phase; the underlying model, data lineage, and limitations must be documented.

## Inputs needed before calibrated simulation

- Vehicle mass budget, component locations, center of mass, and inertia tensor.
- Lift fan pressure-flow curve and fan motor response/efficiency data.
- Cushion effective area and volume; skirt perimeter, clearance, leakage geometry, and discharge coefficient.
- Separate left/right motor-propeller thrust and torque curves, RPM limits, and response data.
- Motor locations, tilt axis geometry, servo range/rate/response/torque data.
- Battery voltage, capacity, internal resistance, and measured channel power/efficiency data.
- Test ambient pressure, temperature, air density, surface condition, and prototype reference measurements.

