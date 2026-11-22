"""Repeatable experiment definitions and execution against the deterministic simulator."""
from __future__ import annotations
import itertools
import uuid
from dataclasses import dataclass
from backend.database.store import Repository, Store
from backend.simulation import Simulator, read_configs

CATALOG = {
    "EXP-01": {"name": "Lift performance", "method": "Sweep configured lift RPM; record cushion pressure, lift force, and airflow."},
    "EXP-02": {"name": "Payload variation", "method": "Sweep 0, 0.1, 0.25, 0.4, and 0.5 kg payload conditions; requires payload mass properties."},
    "EXP-03": {"name": "Differential thrust", "method": "Sweep left/right RPM combinations and record yaw response and power."},
    "EXP-04": {"name": "Servo thrust vectoring", "method": "Sweep independent tilt commands and record force and pitch response."},
    "EXP-05": {"name": "Combined propulsion", "method": "Sweep RPM and tilt combinations; inspect modeled force/moment authority."},
    "EXP-06": {"name": "Battery performance", "method": "Record current, power, energy, and remaining operating energy using configured electrical models."},
    "EXP-07": {"name": "PID control", "method": "Track configured heading/height targets using configured assisted controller gains."},
    "EXP-08": {"name": "Disturbance response", "method": "Apply configured fixed and seeded random disturbances; record pressure and attitude response."},
}

@dataclass
class ExperimentRequest:
    experiment_type: str
    duration_s: float
    name: str | None = None
    initial_commands: dict | None = None
    initial_conditions: dict | None = None
    seed: int = 0
    sweep: dict | None = None
    repeat_count: int = 1

def run_experiment(req: ExperimentRequest, store: Repository | None = None):
    if req.experiment_type not in CATALOG: raise ValueError("Unknown experiment type")
    if req.duration_s <= 0 or req.duration_s > 3600: raise ValueError("duration_s must be within (0, 3600]")
    if req.repeat_count < 1 or req.repeat_count > 100: raise ValueError("repeat_count must be between 1 and 100")
    allowed={"lift_rpm","left_rpm","right_rpm","left_servo_rad","right_servo_rad","payload_kg"}
    sweep=req.sweep or {}
    if not set(sweep).issubset(allowed): raise ValueError(f"Sweep keys must be from {sorted(allowed)}")
    if any(not vals for vals in sweep.values()): raise ValueError("Sweep value lists cannot be empty")
    if req.experiment_type=="EXP-02" and not sweep: sweep={"payload_kg":[0.0,0.1,0.25,0.4,0.5]}
    base, physics, env = read_configs()
    conditions=[dict(zip(sweep.keys(),values)) for values in itertools.product(*sweep.values())] or [{}]
    run_count=len(conditions)*req.repeat_count
    if run_count>500: raise ValueError("Batch limited to 500 condition/repeat runs")
    samples_per_run=int(req.duration_s*float(physics["integration"]["physics_frequency_hz"]))+1
    if samples_per_run*run_count>2_000_000: raise ValueError("Batch limited to two million total telemetry samples")
    store=store or Store()
    batch_id=str(uuid.uuid4()); runs=[]; failed=False
    for condition in conditions:
        for repeat in range(req.repeat_count):
            run_id=str(uuid.uuid4()); seed=req.seed+repeat
            meta={"experiment_id":run_id,"batch_id":batch_id,"experiment_type":req.experiment_type,
                  "methodology":CATALOG[req.experiment_type]["method"],"configuration":physics,
                  "vehicle_configuration":base,"environment":env,"simulation_version":"0.1.0",
                  "physics_model_version":physics["model_version"],"random_seed":seed,"repeat_index":repeat,
                  "condition":condition,"initial_conditions":req.initial_conditions or {},
                  "initial_commands":req.initial_commands or {},"duration_requested_s":req.duration_s,
                  "measurement_class":"simulation","validation_status":"uncalibrated"}
            store.create_run(run_id,req.name or CATALOG[req.experiment_type]["name"],physics,meta)
            try:
                sim=Simulator(base,physics,env,seed=seed)
                if req.initial_conditions: sim.set_initial_conditions(req.initial_conditions)
                commands={**(req.initial_commands or {}),**condition}
                if req.experiment_type=="EXP-07": commands.setdefault("mode","assisted")
                samples=sim.run(req.duration_s,commands,lambda sample: store.record(run_id,sample))
                store.finish(run_id,"complete")
                runs.append({"id":run_id,"sample_count":len(samples),"status":"complete","metadata":meta})
            except Exception as ex:
                store.fail(run_id,str(ex))
                failed=True
                runs.append({"id":run_id,"sample_count":len(store.rows(run_id)),"status":"failed","error":str(ex),"metadata":meta})
    return {"id":batch_id,"runs":runs,"status":"partial" if failed else "complete"}

def export_bundle(store: Repository, run_id: str):
    return {"metadata":store.run(run_id),"samples":store.rows(run_id)}
