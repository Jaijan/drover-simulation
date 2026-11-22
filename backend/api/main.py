from __future__ import annotations
import asyncio, csv, io, json, uuid
import numpy as np
import os, secrets
from pathlib import Path
from typing import Any
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel, Field
from backend.simulation import Simulator, read_configs, CONFIG_ROOT
from backend.database.store import Store
from backend.experiments.runner import CATALOG, ExperimentRequest, run_experiment, export_bundle
from backend.analysis.statistics import summarize, compare_simulated_to_measured, correlation

app=FastAPI(title="DROVER Digital Twin API",version="0.1.0",description="Configurable 6-DOF skirted air-cushion vehicle simulator. Uncalibrated outputs are simulations, not measurements.")
if os.getenv("APP_ENV","development").lower()=="production" and not os.getenv("DROVER_ADMIN_TOKEN"):
    raise RuntimeError("DROVER_ADMIN_TOKEN must be set in production to protect configuration writes")
_origins=[x.strip() for x in os.getenv("DROVER_CORS_ORIGINS","http://localhost:5173,http://127.0.0.1:5173").split(",") if x.strip()]
app.add_middleware(CORSMiddleware,allow_origins=_origins,allow_credentials=True,allow_methods=["GET","POST","PUT","OPTIONS"],allow_headers=["Authorization","Content-Type"])
store=Store()

class ExperimentBody(BaseModel):
    experiment_type: str
    duration_s: float=Field(gt=0,le=3600)
    name: str|None=None
    initial_commands: dict[str,float]|None=None
    initial_conditions: dict[str,Any]|None=None
    seed: int=0
    sweep: dict[str,list[float]]|None=None
    repeat_count: int=Field(default=1,ge=1,le=100)

@app.get("/api/health")
def health(): return {"status":"ok","service":"drover-digital-twin","version":"0.1.0"}

def missing_params():
    _,p,e=read_configs()
    required_paths=[("physics.rigid_body.dry_mass_kg","rigid_body.dry_mass_kg"),("physics.rigid_body.dry_center_of_mass_body_m","rigid_body.dry_center_of_mass_body_m"),
      ("physics.rigid_body.dry_inertia_tensor_body_kg_m2","rigid_body.dry_inertia_tensor_body_kg_m2"),("physics.aerodynamics.reference_area_m2","aerodynamics.reference_area_m2"),
      ("physics.aerodynamics.drag_coefficient","aerodynamics.drag_coefficient")]
    for k in ("effective_area_m2","cushion_volume_m3","fan_curve","initial_air_mass_kg","fan_max_speed_rpm","fan_response_time_s","fan_max_acceleration_rpm_s","fan_efficiency","leakage_area_m2","discharge_coefficient","lift_force_position_body_m"):
        required_paths.append((f"physics.lift.{k}",f"lift.{k}"))
    for side in ("left","right"):
        for k in ("thrust_coefficient_n_per_rad_s2","max_speed_rad_s","max_acceleration_rad_s2","response_time_s","reaction_torque_coefficient_nm_per_n","propeller_rotation_sign","propeller_rotor_inertia_kg_m2"):
            required_paths.append((f"physics.propulsion.motors.{side}.{k}",f"propulsion.motors.{side}.{k}"))
    for path in ("mount_positions_body_m.left","mount_positions_body_m.right","servo_angle_limits_rad","servo_rate_limit_rad_s","servo_response_time_s"):
        required_paths.append((f"physics.propulsion.{path}",f"propulsion.{path}"))
    required_paths.extend((f"environment.{k}",k) for k in ("air_density_kg_m3","ambient_pressure_pa","temperature_k"))
    for path in ("surface.cushion_clearance_m","surface.friction_coefficient","surface.contact_stiffness_n_m","surface.contact_damping_n_s_m"):
        required_paths.append((f"environment.{path}",path))
    missing=[]
    for label,path in required_paths:
        try:
            val=p if label.startswith("physics.") else e
            for part in path.split("."): val=val[part]
            if val is None: missing.append(label)
        except (KeyError,TypeError): missing.append(label)
    if float(p.get("payload",{}).get("mass_kg",0))>0:
        for k in ("position_body_m","inertia_tensor_body_kg_m2"):
            if p["payload"].get(k) is None: missing.append(f"physics.payload.{k}")
    if e.get("surface",{}).get("profile") in ("rough","uneven") and e.get("surface",{}).get("height_map") is None:
        missing.append("environment.surface.height_map")
    return missing

@app.get("/api/config")
def config():
    v,p,e=read_configs()
    missing=missing_params()
    return {"vehicle":v,"physics":p,"environment":e,"missing_required":missing,"simulation_ready":not missing}

@app.put("/api/config/{kind}")
def update_config(kind: str, body: dict[str,Any], request: Request):
    expected=os.getenv("DROVER_ADMIN_TOKEN")
    if expected:
        supplied=request.headers.get("Authorization","").removeprefix("Bearer ")
        if not secrets.compare_digest(expected,supplied): raise HTTPException(401,"A valid administrator bearer token is required")
    files={"vehicle":"configuration/drover_config.json","physics":"configurations/physics_config.json","environment":"configurations/environment_config.json"}
    if kind not in files: raise HTTPException(404,"Unknown configuration kind")
    if not isinstance(body,dict): raise HTTPException(422,"Configuration document must be a JSON object")
    required_blocks={"vehicle":("project","vehicle","lift_system","propulsion","servos","battery","simulation"),
                     "physics":("schema_version","rigid_body","payload","integration","air","lift","propulsion","battery"),
                     "environment":("schema_version","surface","disturbance")}
    absent=[key for key in required_blocks[kind] if key not in body]
    if absent: raise HTTPException(422,f"Configuration is missing required sections: {absent}")
    try: encoded=json.dumps(body,indent=2,allow_nan=False)+"\n"
    except (TypeError,ValueError) as ex: raise HTTPException(422,f"Configuration contains values that are not valid JSON: {ex}")
    target=CONFIG_ROOT/files[kind]
    # Preserve no implicit defaults: store exactly the explicit user supplied document.
    target.write_text(encoded,encoding="utf-8")
    missing=missing_params()
    return {"saved":kind,"missing_required":missing,"simulation_ready":not missing}

@app.get("/api/experiments")
def experiments(): return {"catalog":CATALOG,"runs":store.list_runs()}

@app.post("/api/experiments/run")
def start_experiment(req: ExperimentBody):
    try: return run_experiment(ExperimentRequest(**req.model_dump()),store)
    except (ValueError,KeyError) as ex: raise HTTPException(422,str(ex))
    except Exception as ex: raise HTTPException(500,f"Simulation failed: {ex}")

@app.get("/api/runs/{run_id}")
def get_run(run_id: str):
    try: return export_bundle(store,run_id)
    except KeyError: raise HTTPException(404,"Run not found")

@app.get("/api/runs/{run_id}/analysis")
def analyze_run(run_id: str):
    try: rows=store.rows(run_id)
    except KeyError: raise HTTPException(404,"Run not found")
    if not rows: return {"measurement_class":"simulation","n":0,"status":"no_samples"}
    result={"measurement_class":"simulation","n":len(rows)}
    for key in ("position_world_m","velocity_world_m_s","acceleration_world_m_s2"):
        for i,axis in enumerate("xyz"):
            result[f"{key}_{axis}"]=summarize([r[key][i] for r in rows])
    series={"cushion_pressure_pa":"lift.pressure_differential_pa","cushion_absolute_pressure_pa":"lift.cushion_pressure_pa","lift_force_n":"lift.lift_force_n","hover_height_m":"hover_height_m","left_motor_rpm":"propulsion.left.rpm","right_motor_rpm":"propulsion.right.rpm","left_thrust_n":"propulsion.left.thrust_n","right_thrust_n":"propulsion.right.thrust_n","left_servo_rad":"propulsion.left.servo_rad","right_servo_rad":"propulsion.right.servo_rad","yaw_rate_rad_s":"omega_body_rad_s.2","power_w":"electrical.power_w","energy_j":"electrical.energy_j"}
    for metric,path in series.items():
        values=[]
        for row in rows:
            value=row
            for key in path.split("."): value=value[int(key)] if isinstance(value,list) else value[key]
            if value is not None: values.append(float(value))
        result[metric]=summarize(values) if values else {"n":0,"status":"model_unconfigured"}
    store.save_analysis(run_id,result)
    return result

@app.get("/api/compare-runs")
def compare_runs(a: str, b: str, field: str="lift.lift_force_n"):
    try: rows_a=store.rows(a); rows_b=store.rows(b)
    except KeyError: raise HTTPException(404,"Run not found")
    def values(rows,path):
        out=[]
        for row in rows:
            val=row
            for key in path.split("."): val=val[key]
            out.append(float(val))
        return out
    try: x,y=values(rows_a,field),values(rows_b,field)
    except (KeyError,ValueError) as ex: raise HTTPException(422,f"Invalid telemetry field: {ex}")
    n=min(len(x),len(y)); x=x[:n]; y=y[:n]
    return {"field":field,"run_a":a,"run_b":b,"n_compared":n,"run_a_stats":summarize(x),"run_b_stats":summarize(y),"correlation_by_sample_index":correlation(x,y) if n>1 else None,"measurement_class":"simulation_vs_simulation"}

class MeasurementBody(BaseModel):
    run_id: str
    field: str
    units: str
    measured: list[float]
    timestamps_s: list[float]|None=None

@app.post("/api/measurements/compare")
def compare_measurements(body: MeasurementBody):
    try: store.run(body.run_id); rows=store.rows(body.run_id)
    except KeyError: raise HTTPException(404,"Run not found")
    try:
        simulated=[]
        times=[]
        for row in rows:
            value=row
            for key in body.field.split("."): value=value[key]
            simulated.append(float(value)); times.append(float(row["simulation_time_s"]))
        measured_simulated=simulated
        if body.timestamps_s is not None:
            if len(body.timestamps_s)!=len(body.measured): raise ValueError("Measured timestamps must match the measured sample count")
            if any(t<times[0] or t>times[-1] for t in body.timestamps_s): raise ValueError("Measurement timestamps must lie within the simulation interval")
            measured_simulated=np.interp(body.timestamps_s,times,simulated).tolist()
        result=compare_simulated_to_measured(measured_simulated,body.measured)
    except (ValueError,KeyError,TypeError,IndexError) as ex: raise HTTPException(422,f"Measurement alignment failed: {ex}")
    import uuid
    result.update({"run_id":body.run_id,"field":body.field,"units":body.units,"physical_measurements_supplied":True,"calibration_claim":"not_assessed_by_automated_comparison"})
    records={"timestamps_s":body.timestamps_s,"values":body.measured}
    store.save_measurement_comparison(str(uuid.uuid4()),body.run_id,body.field,body.units,records,result)
    return result

@app.get("/api/runs/{run_id}/measurements")
def list_measurements(run_id: str):
    try: return {"run_id":run_id,"measurements":store.measurements(run_id)}
    except KeyError: raise HTTPException(404,"Run not found")

@app.get("/api/runs/{run_id}/export.json")
def export_json(run_id: str):
    try: data=export_bundle(store,run_id)
    except KeyError: raise HTTPException(404,"Run not found")
    return data

@app.get("/api/runs/{run_id}/report.pdf")
def report_pdf(run_id: str):
    try: run=store.run(run_id); rows=store.rows(run_id)
    except KeyError: raise HTTPException(404,"Run not found")
    from reportlab.lib.pagesizes import letter
    from reportlab.pdfgen import canvas
    stream=io.BytesIO(); pdf=canvas.Canvas(stream,pagesize=letter); width,height=letter; y=height-48
    pdf.setTitle(f"DROVER research report {run_id}"); pdf.setFont("Helvetica-Bold",16); pdf.drawString(42,y,"DROVER Digital Twin — Experiment Summary"); y-=30
    pdf.setFont("Helvetica",9)
    lines=[f"Run ID: {run_id}",f"Name: {run['name']}",f"Status: {run['status']}",f"Started (UTC): {run['started_utc']}",f"Samples: {len(rows)}", "Result class: Simulation-generated; not a physical measurement.", "Calibration status: Uncalibrated unless model metadata explicitly states otherwise."]
    for line in lines: pdf.drawString(42,y,line); y-=15
    y-=7; pdf.setFont("Helvetica-Bold",11); pdf.drawString(42,y,"Descriptive statistics"); y-=18; pdf.setFont("Helvetica",9)
    if rows:
        fields=[("Cushion pressure (Pa)",[r["lift"]["pressure_differential_pa"] for r in rows]),("Lift force (N)",[r["lift"]["lift_force_n"] for r in rows]),("Hover height (m)",[r["position_world_m"][2] for r in rows]),("Forward velocity (m/s)",[r["velocity_world_m_s"][0] for r in rows]),("Electrical power (W)",[r["electrical"]["power_w"] for r in rows if r["electrical"]["power_w"] is not None])]
        for label,values in fields:
            if not values: continue
            s=summarize(values); pdf.drawString(42,y,f"{label}: mean {s['mean']:.5g}, SD {s['std']:.5g}, min {s['min']:.5g}, max {s['max']:.5g}, n={s['n']}"); y-=15
    y-=8; pdf.setFont("Helvetica-Bold",11); pdf.drawString(42,y,"Method and configuration provenance"); y-=17; pdf.setFont("Helvetica",8)
    meta=run["metadata"]
    report_meta=json.dumps({"experiment_type":meta.get("experiment_type"),"methodology":meta.get("methodology"),"seed":meta.get("random_seed"),"condition":meta.get("condition"),"physics_model_version":meta.get("physics_model_version"),"validation_status":meta.get("validation_status")},sort_keys=True)
    for i in range(0,len(report_meta),110): pdf.drawString(42,y,report_meta[i:i+110]); y-=12
    pdf.save(); stream.seek(0)
    return StreamingResponse(stream,media_type="application/pdf",headers={"Content-Disposition":f"attachment; filename={run_id}-report.pdf"})

@app.get("/api/runs/{run_id}/export.csv")
def export_csv(run_id: str):
    try: rows=store.rows(run_id)
    except KeyError: raise HTTPException(404,"Run not found")
    if not rows: raise HTTPException(404,"Run has no telemetry samples")
    def flatten(value,prefix=""):
        result={}
        if isinstance(value,dict):
            for k,v in value.items(): result.update(flatten(v,f"{prefix}{k}__"))
        elif isinstance(value,list):
            axes=("x","y","z") if len(value)==3 else tuple(map(str,range(len(value))))
            for axis,v in zip(axes,value): result.update(flatten(v,f"{prefix}{axis}__"))
        else: result[prefix[:-2]]=value
        return result
    flat=[flatten(row) for row in rows]; fields=list(dict.fromkeys(k for row in flat for k in row))
    stream=io.StringIO(); writer=csv.DictWriter(stream,fieldnames=fields); writer.writeheader(); writer.writerows(flat)
    return StreamingResponse(iter([stream.getvalue()]),media_type="text/csv",headers={"Content-Disposition":f"attachment; filename={run_id}.csv"})

@app.websocket("/ws/simulation")
async def simulation_socket(ws: WebSocket):
    await ws.accept(); sim=None; running=False; scale=1.0; accumulator=0.0; run_id=None; estop=False
    try:
        while True:
            try: message=await asyncio.wait_for(ws.receive_json(),timeout=0.01)
            except asyncio.TimeoutError: message=None
            if message:
                action=message.get("action")
                if action=="start":
                    if estop:
                        await ws.send_json({"type":"error","message":"Emergency stop is latched; reset the simulation before starting again"})
                    else:
                        if sim is None: sim=Simulator()
                        if run_id is None:
                            accumulator=0.0; run_id=str(uuid.uuid4())
                            vehicle,p,e=read_configs(); meta={"experiment_id":run_id,"experiment_type":"interactive","physics_model_version":p["model_version"],"configuration":p,"vehicle_configuration":vehicle,"environment":e,"measurement_class":"simulation","validation_status":"uncalibrated"}
                            store.create_run(run_id,"Interactive simulation",p,meta)
                        running=True
                elif action=="pause": running=False
                elif action=="resume" and not estop: running=True
                elif action=="reset":
                    if run_id: store.finish(run_id,"reset")
                    sim=Simulator(); running=False; estop=False; run_id=None
                elif action=="stop":
                    running=False
                    if run_id: store.finish(run_id,"stopped")
                elif action=="emergency_stop" and sim:
                    sim.commands.update({"lift_rpm":0.0,"left_rpm":0.0,"right_rpm":0.0})
                    sim.commands["mode"]="manual"; sim.commands["forward_command"]=0.0; estop=True; running=True
                    if run_id: store.finish(run_id,"emergency_stopped")
                elif action=="command" and sim and not estop: sim.commands.update(message.get("values",{}))
                elif action=="time_scale": scale=float(message.get("value",1)); scale=max(.1,min(10,scale))
            if running and sim:
                accumulator+=scale*10
                steps=int(accumulator); accumulator-=steps
                for _ in range(steps):
                    sample=sim.step()
                    if run_id: store.record(run_id,sample)
                if steps: await ws.send_json({"type":"telemetry","data":sample})
                await asyncio.sleep(.1)
    except WebSocketDisconnect:
        if run_id and running: store.finish(run_id,"disconnected")
    except Exception as ex:
        try: await ws.send_json({"type":"error","message":str(ex)})
        except Exception: pass
