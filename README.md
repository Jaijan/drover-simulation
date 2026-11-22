# DROVER Digital Twin

Research-oriented simulation scaffold for the DROVER X-frame skirted air-cushion vehicle. Physics is implemented in SI units and is separated from the React/Three.js dashboard. The dashboard loads the supplied `drover.glb` from `frontend/public/models/`; simulation outputs are model-generated and the repository contains no prototype validation data.

## Quick start

Requires Python 3.11+ and Node.js 20+.

From the repository root:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e .
uvicorn backend.api.main:app --reload
```

In another terminal:

```powershell
cd frontend
npm install
npm run dev
```

The dashboard is at `http://localhost:5173`; interactive API documentation is at `http://localhost:8000/docs`. SQLite is created as `drover.sqlite3` in the repository root. Simulation start will report each required missing parameter until configured.

## Hosting

The frontend can be deployed to Vercel with the Vercel project root set to `frontend`, framework preset `Vite`, build command `npm run build`, and output directory `dist`. Set `VITE_API_URL` and `VITE_WS_URL` in the Vercel project before building; use the backend's HTTPS URL and its `wss://.../ws/simulation` URL. The API currently supports single-use interactive and experiment sessions; its WebSocket state lives in the process, so run one backend instance for now.

For the backend, build the repository's `Dockerfile` on a container host with a persistent disk mounted at `/data`, then set:

- `APP_ENV=production`
- `DROVER_ADMIN_TOKEN` to a long random secret
- `DROVER_CORS_ORIGINS` to the exact deployed frontend origin, such as `https://your-app.vercel.app`
- `PORT` if the host supplies a port variable

The container stores SQLite and editable configuration files under `/data`; the entrypoint copies the supplied configuration templates there on first startup. Keep the service to a single replica while using SQLite, because replicas must not share a SQLite file. Deploying the backend as Vercel Functions instead requires replacing SQLite and file-based configuration with durable storage, plus implementing WebSocket reconnect/session recovery for function restarts and duration limits. `PUT /api/config/{kind}` requires `Authorization: Bearer <DROVER_ADMIN_TOKEN>` when `DROVER_ADMIN_TOKEN` is set.

## Configuration before simulation

Edit through the JSON configuration files or `PUT /api/config/{vehicle|physics|environment}`. Use `GET /api/config` to inspect current values and the list of required mechanical/environment values. A null parameter means unknown. Do not replace it with a plausible value unless explicitly labeled as an estimate/assumption and sourced in metadata.

The 2.5 kg value is a user-supplied target, not a measured dry mass. Enter dry mass properties separately in `configurations/physics_config.json`. Ambient density, pressure, and temperature are set in `configurations/environment_config.json`. The payload model uses its own mass, location, and intrinsic inertia; nonzero payload requires its location and inertia. The maximum payload is the supplied 0.5 kg limit. Fan and motor curves are required input data. The pressure/flow matrix uses `rpm_points`, `pressure_pa`, and a `flow_m3_s` matrix with rows per RPM. Matrix axes must be strictly increasing and operation outside the configured domain is rejected. Shaft power curves are arrays of `[rpm, shaft_power_w]` pairs and must span the motor's operating range.

The battery module becomes active only when its battery parameters, channel efficiencies, shaft-power curves, and servo power are supplied. Until then telemetry marks electrical output unavailable; other configured mechanical simulation may still run. For rough or uneven surface profiles, configure `surface.height_map` with increasing `x_m` and `y_m` axes and a matching `height_m` matrix; heights are offsets from `ground_height_m` and are interpolated bilinearly.

## Physics and limits

- Fixed-step semi-implicit Euler at configured physics frequency; quaternion orientation update, normalized after each step.
- Rigid-body equations include gravity, aerodynamic quadratic drag, force/moment arms, gyroscopic propeller reaction, cushion lift, disturbance, and compliant ground contact/friction.
- Cushion state uses air mass conservation and isothermal ideal-gas pressure; leakage uses an orifice relation. The flow characteristic is configured fan data; the isothermal chamber, fixed volume, and orifice leakage are engineering approximations pending prototype calibration.
- Propeller thrust uses `T=kT·omega²`; configured separate left/right coefficients and acceleration/response constraints are used. Vertical thrust tilt is in the body XZ plane, positive toward +Z.
- Assisted control provides heading PID through differential thrust and height PID through lift-fan RPM. This geometry does not provide independent roll authority; control authority is reported rather than implied.
- A seed is recorded and drives configured Gaussian disturbance noise. With random standard deviations unset, the disturbance is deterministic and consists only of the configured fixed force/moment.

## Experiments and exports

`POST /api/experiments/run` executes a batch and stores every 100 Hz sample. It accepts an experiment type, duration, repeat count, seed, initial commands, and a command sweep map (Cartesian product; up to 500 condition/repeat runs). EXP-02 defaults to the stated payload values; other sweeps require explicit user-selected actuator values because hardware limits are not supplied. EXP-07 requires assisted PID gains. Unsupported/missing parameters cause an explicit error.

Stored run outputs are available as full JSON, flattened CSV, summary PDF, and analysis JSON. CSV emits one row per sample. Plotly supports SVG image export from the chart mode bar. The measurement comparison endpoint derives the simulation series from a stored run and accepts user-supplied measured values and optional timestamps. It reports error metrics but does not declare calibration or validation automatically.

## Project map

- `backend/physics`, `dynamics`, `lift`, `propulsion`, `servos`, `controllers`, `battery`, `environment`: simulation models.
- `backend/api`: FastAPI and WebSocket runtime.
- `backend/database`: SQLite persistence boundary.
- `backend/experiments`, `analysis`: experiment runner and metrics.
- `frontend/src`: React dashboard, supplied GLB model, controls, telemetry, Plotly graph and run exports.
- `configuration`, `configurations`: vehicle, physics, and environment settings.
- `docs/engineering-foundation.md`: coordinate convention, provenance policy, numerical method, and data requirements.

## Validation

Install development extras with `python -m pip install -e ".[dev]"` and run `python -m pytest backend/tests`. Build the frontend with `cd frontend; npm install; npm run build`. These checks need Python and Node.js respectively. No test claim is made until those commands run successfully in an environment with the dependencies installed.

The supplied `drover.glb` is copied into `frontend/public/models/drover.glb` and loaded by the viewport; the UI no longer builds a substitute vehicle mesh. Confirm its origin, units, and axis orientation against the documented body frame when comparing geometry and simulation poses. Physical validation remains pending measured mass/inertia, fan, motor, servo, battery, and prototype response data.
