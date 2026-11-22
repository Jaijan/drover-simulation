"""Deterministic fixed-step simulation. Unconfigured hardware fails explicitly."""
from __future__ import annotations
import json
import os
from pathlib import Path
import numpy as np
from backend.physics.models import AirCushion, Motor, Servo, RigidBody, G, quat_rotate, point_force_moment
from backend.battery.model import Battery, curve_value
from backend.controllers.pid import PID, allocate_differential_thrust

ROOT = Path(__file__).resolve().parents[1]
CONFIG_ROOT = Path(os.environ.get("DROVER_CONFIG_DIR", ROOT)).resolve()

def read_configs():
    return tuple(json.loads((CONFIG_ROOT/path).read_text(encoding="utf-8")) for path in (
        "configuration/drover_config.json", "configurations/physics_config.json", "configurations/environment_config.json"))

def required(obj, path):
    val = obj
    for key in path.split("."):
        val = val.get(key) if isinstance(val, dict) else None
    if val is None:
        raise ValueError(f"Required engineering parameter is unset: {path}")
    return val

def surface_height(surface, x, y):
    """Bilinearly interpolate an optional configured surface height grid."""
    base=float(surface["ground_height_m"]); grid=surface.get("height_map")
    if grid is None: return base
    xs=np.asarray(grid["x_m"],float); ys=np.asarray(grid["y_m"],float); z=np.asarray(grid["height_m"],float)
    if z.shape!=(len(ys),len(xs)) or len(xs)<2 or len(ys)<2 or np.any(np.diff(xs)<=0) or np.any(np.diff(ys)<=0):
        raise ValueError("Surface height map must be an increasing x/y grid with a matching height matrix")
    ix=int(np.clip(np.searchsorted(xs,x)-1,0,len(xs)-2)); iy=int(np.clip(np.searchsorted(ys,y)-1,0,len(ys)-2))
    tx=float(np.clip((x-xs[ix])/(xs[ix+1]-xs[ix]),0,1)); ty=float(np.clip((y-ys[iy])/(ys[iy+1]-ys[iy]),0,1))
    h=(1-ty)*((1-tx)*z[iy,ix]+tx*z[iy,ix+1])+ty*((1-tx)*z[iy+1,ix]+tx*z[iy+1,ix+1])
    return base+float(h)

class Simulator:
    def __init__(self, vehicle=None, physics=None, environment=None, seed=None):
        base, phys, env = read_configs()
        self.vehicle, self.config, self.environment = vehicle or base, physics or phys, environment or env
        c, v, e = self.config, self.vehicle, self.environment
        rb = c["rigid_body"]
        self.dry_mass = float(required(rb,"dry_mass_kg"))
        self.dry_com = np.asarray(required(rb,"dry_center_of_mass_body_m"),float)
        self.dry_inertia = np.asarray(required(rb,"dry_inertia_tensor_body_kg_m2"),float)
        payload=c["payload"]
        self.payload_position=np.asarray(required(payload,"position_body_m"),float) if float(payload.get("mass_kg",0))>0 else np.zeros(3)
        self.payload_inertia=np.asarray(payload["inertia_tensor_body_kg_m2"],float) if payload.get("inertia_tensor_body_kg_m2") is not None else np.zeros((3,3))
        self.payload_mass=float(payload.get("mass_kg",0))
        self.body = RigidBody(self.dry_mass, self.dry_inertia.copy(), np.array([0.,0.,0.]), np.zeros(3), np.array([1.,0.,0.,0.]), np.zeros(3))
        self.com = self.dry_com.copy()
        self.dt = 1/float(c["integration"]["physics_frequency_hz"])
        self.time_s = 0.0
        lift_cfg=c["lift"]
        self.fan_rpm=0.0
        self.commands = {"lift_rpm": 0.0, "left_rpm": 0.0, "right_rpm": 0.0, "left_servo_rad": 0.0, "right_servo_rad": 0.0}
        self.seed=int(e["disturbance"].get("random_seed",0) if seed is None else seed)
        self.rng=np.random.default_rng(self.seed)
        self.heading_pid=None; self.height_pid=None
        ctl=c.get("controllers",{})
        if ctl.get("assisted_enabled"):
            h=ctl["heading_pid"]; z=ctl["hover_height_pid"]
            self.heading_pid=PID(required(h,"kp_rpm_per_rad"),required(h,"ki_rpm_per_rad_s"),required(h,"kd_rpm_s_per_rad"),
                                 (-required(h,"max_differential_rpm"),required(h,"max_differential_rpm")),(-1e9,1e9))
            maxfan=required(c,"lift.fan_max_speed_rpm")
            self.height_pid=PID(required(z,"kp_rpm_per_m"),required(z,"ki_rpm_per_m_s"),required(z,"kd_rpm_s_per_m"),(0,maxfan),(-1e9,1e9))
        air = c["air"]
        lift = c["lift"]
        self.cushion = AirCushion(float(required(lift,"cushion_volume_m3")), float(required(lift,"effective_area_m2")),
            required(lift,"fan_curve"), float(required(lift,"leakage_area_m2")), float(required(lift,"discharge_coefficient")),
            float(required(air,"specific_gas_constant_j_kg_k")), float(required(e,"temperature_k")),
            float(required(e,"ambient_pressure_pa")), float(required(e,"air_density_kg_m3")),
            float(required(lift,"initial_air_mass_kg")))
        p = c["propulsion"]
        self.motors = []
        self.servos = []
        for side in ("left","right"):
            m = p["motors"][side]
            self.motors.append(Motor(float(required(m,"thrust_coefficient_n_per_rad_s2")), float(required(m,"max_speed_rad_s")),
                float(required(m,"response_time_s")), float(required(m,"max_acceleration_rad_s2")),
                float(required(m,"reaction_torque_coefficient_nm_per_n")), float(required(m,"propeller_rotation_sign")),
                float(required(m,"propeller_rotor_inertia_kg_m2"))))
            self.servos.append(Servo(float(required(p,"servo_angle_limits_rad")[0]), float(required(p,"servo_angle_limits_rad")[1]),
                float(required(p,"servo_rate_limit_rad_s")), float(required(p,"servo_response_time_s"))))
        self.mount_positions = [np.asarray(required(p,"mount_positions_body_m.left"),float),
                       np.asarray(required(p,"mount_positions_body_m.right"),float)]
        self.lift_position_origin = np.asarray(required(c,"lift.lift_force_position_body_m"),float)
        self._update_mass_properties(self.payload_mass)
        initial_ground=surface_height(e["surface"],0,0)
        self.body.position_world_m[2]=initial_ground+float(required(e,"surface.cushion_clearance_m"))+float(quat_rotate(self.body.orientation_body_to_world,self.com)[2])
        self.battery=None
        b=c["battery"]
        if all(b.get(k) is not None for k in ("nominal_voltage_v","capacity_ah","internal_resistance_ohm")):
            self.battery=Battery(b["nominal_voltage_v"],b["capacity_ah"],b["internal_resistance_ohm"])
        self.last = {}
        self.running = False

    def set_initial_conditions(self, conditions):
        allowed={"position_world_m","velocity_world_m_s","orientation_quaternion_body_to_world","omega_body_rad_s"}
        if not set(conditions).issubset(allowed): raise ValueError(f"Initial condition keys must be from {sorted(allowed)}")
        for key,attr,length in (("position_world_m","position_world_m",3),("velocity_world_m_s","velocity_world_m_s",3),("orientation_quaternion_body_to_world","orientation_body_to_world",4),("omega_body_rad_s","omega_body_rad_s",3)):
            if key in conditions:
                value=np.asarray(conditions[key],float)
                if value.shape!=(length,) or not np.all(np.isfinite(value)): raise ValueError(f"{key} must be a finite vector of length {length}")
                setattr(self.body,attr,value)
        q=self.body.orientation_body_to_world
        if np.linalg.norm(q)==0: raise ValueError("Initial quaternion cannot have zero norm")
        self.body.orientation_body_to_world=q/np.linalg.norm(q)

    def _update_mass_properties(self, payload_mass):
        p=self.config["payload"]
        payload_mass=float(payload_mass)
        if payload_mass<0 or payload_mass>float(required(p,"max_mass_kg")): raise ValueError("Payload mass outside configured limit")
        if payload_mass>0 and (p.get("position_body_m") is None or p.get("inertia_tensor_body_kg_m2") is None):
            raise ValueError("Payload location and inertia are required when payload mass is nonzero")
        c1=self.dry_com; m1=self.dry_mass; c2=self.payload_position; m2=payload_mass
        total=m1+m2
        com=(m1*c1+m2*c2)/total
        I=np.eye(3)
        d1=c1-com; d2=c2-com
        inertia=self.dry_inertia+m1*((d1@d1)*I-np.outer(d1,d1))
        if m2>0: inertia+=self.payload_inertia+m2*((d2@d2)*I-np.outer(d2,d2))
        self.body.mass_kg=total; self.body.inertia_kg_m2=inertia; self.com=com
        self.mounts=[r-com for r in self.mount_positions]
        self.lift_position=self.lift_position_origin-com
        self.payload_mass=payload_mass

    def step(self, commands=None):
        if commands: self.commands.update(commands)
        u, e, c = self.commands, self.environment, self.config
        if "payload_kg" in u and float(u["payload_kg"])!=self.payload_mass: self._update_mass_properties(float(u["payload_kg"]))
        frame_origin_world=self.body.position_world_m-quat_rotate(self.body.orientation_body_to_world,self.com)
        ground=surface_height(e["surface"],float(frame_origin_world[0]),float(frame_origin_world[1]))
        if u.get("mode")=="assisted":
            if self.heading_pid is None or self.height_pid is None: raise ValueError("Assisted mode requires configured heading and hover-height PID controllers")
            leftmax=float(required(c,"propulsion.motors.left.max_speed_rad_s"))*60/(2*np.pi)
            rightmax=float(required(c,"propulsion.motors.right.max_speed_rad_s"))*60/(2*np.pi)
            base=float(np.clip(u.get("forward_command",0.),0,1))*min(leftmax,rightmax)
            measured_yaw=self._euler_for_display()[2]
            yaw_error=(float(u.get("heading_target_rad",0.))-measured_yaw+np.pi)%(2*np.pi)-np.pi
            yaw=self.heading_pid.step(yaw_error,0.,self.dt)
            u["left_rpm"],u["right_rpm"]=allocate_differential_thrust(base,yaw,(leftmax,rightmax))
            u["lift_rpm"]=self.height_pid.step(float(u.get("height_target_m",0.)),max(0.,float(frame_origin_world[2]-ground)),self.dt)
        lift_target=float(np.clip(u["lift_rpm"],0.,required(c["lift"],"fan_max_speed_rpm")))
        fan_rate=np.clip((lift_target-self.fan_rpm)/max(required(c["lift"],"fan_response_time_s"),self.dt),
                         -required(c["lift"],"fan_max_acceleration_rpm_s"),required(c["lift"],"fan_max_acceleration_rpm_s"))
        self.fan_rpm=float(np.clip(self.fan_rpm+fan_rate*self.dt,0.,required(c["lift"],"fan_max_speed_rpm")))
        lift = self.cushion.step(self.fan_rpm, self.dt, max(0.,float(frame_origin_world[2]-ground)))
        lift_force_b=np.array([0.,0.,lift["lift_force_n"]])
        force_b = lift_force_b.copy()
        moment_b = point_force_moment(self.lift_position,lift_force_b)
        prop = []
        for i, side in enumerate(("left","right")):
            thrust, reaction = self.motors[i].step(float(u[f"{side}_rpm"])*2*np.pi/60,self.dt)
            angle = self.servos[i].step(u[f"{side}_servo_rad"],self.dt)
            direction = np.array([np.cos(angle),0.,np.sin(angle)]) # positive servo command tilts thrust toward body +Z
            f = direction*thrust
            force_b += f
            moment_b += point_force_moment(self.mounts[i],f) + reaction*direction
            moment_b += -np.cross(self.body.omega_body_rad_s,direction*self.motors[i].rotation_sign*self.motors[i].rotor_inertia_kg_m2*self.motors[i].omega)
            prop.append({"rpm":self.motors[i].omega*60/(2*np.pi),"thrust_n":thrust,"servo_rad":angle,"force_body_n":f.tolist()})
        wind = np.asarray(e["wind_velocity_world_m_s"],float)
        relative_v = self.body.velocity_world_m_s-wind
        rho = required(e,"air_density_kg_m3")
        drag_area = float(required(c,"aerodynamics.reference_area_m2"))
        cd = float(required(c,"aerodynamics.drag_coefficient"))
        drag_world = -0.5*float(rho)*cd*drag_area*np.linalg.norm(relative_v)*relative_v
        random_force=np.zeros(3) if e["disturbance"].get("random_force_std_world_n") is None else self.rng.normal(0,np.asarray(e["disturbance"]["random_force_std_world_n"],float))
        random_moment=np.zeros(3) if e["disturbance"].get("random_moment_std_body_nm") is None else self.rng.normal(0,np.asarray(e["disturbance"]["random_moment_std_body_nm"],float))
        force_world = quat_rotate(self.body.orientation_body_to_world,force_b)+drag_world+np.array([0.,0.,-self.body.mass_kg*G]) + np.asarray(e["disturbance"]["force_world_n"])+random_force
        moment_b += np.asarray(e["disturbance"]["moment_body_nm"])+random_moment
        clearance=float(required(e,"surface.cushion_clearance_m"))
        contact_z=ground+clearance
        penetration=max(0.0,contact_z-frame_origin_world[2])
        if penetration>0:
            normal=max(0.0,float(required(e,"surface.contact_stiffness_n_m"))*penetration-float(required(e,"surface.contact_damping_n_s_m"))*min(self.body.velocity_world_m_s[2],0.0))
            force_world[2]+=normal
            friction_world=np.zeros(3)
            vh=self.body.velocity_world_m_s[:2]
            speed=np.linalg.norm(vh)
            if speed>0:
                friction=min(float(required(e,"surface.friction_coefficient"))*normal/self.body.mass_kg, speed/self.dt)
                friction_world[:2]=-vh/speed*friction*self.body.mass_kg
                force_world[:2]+=friction_world[:2]
            contact_force_body=quat_rotate(np.array([self.body.orientation_body_to_world[0],-self.body.orientation_body_to_world[1],-self.body.orientation_body_to_world[2],-self.body.orientation_body_to_world[3]]),friction_world+np.array([0.,0.,normal]))
            moment_b+=point_force_moment(-self.com+np.array([0.,0.,-clearance]),contact_force_body)
        acc = self.body.step(force_world,moment_b,self.dt)
        self.time_s += self.dt
        elec_missing=[]
        required_electrical=["lift.fan_efficiency","battery.esc_efficiency","battery.lift_motor_efficiency","battery.propulsion_motor_efficiency.left","battery.propulsion_motor_efficiency.right","propulsion.servo_electrical_power_w","battery.nominal_voltage_v","battery.capacity_ah","battery.internal_resistance_ohm"]
        for side in ("left","right"): required_electrical.append(f"propulsion.motors.{side}.static_shaft_power_curve")
        for path in required_electrical:
            try: required(c,path)
            except ValueError: elec_missing.append(path)
        channel_power={}
        if not elec_missing:
            esc_eff=float(c["battery"]["esc_efficiency"]); lift_motor_eff=float(c["battery"]["lift_motor_efficiency"])
            channel_power={"lift":lift["pressure_differential_pa"]*lift["airflow_m3_s"]/float(c["lift"]["fan_efficiency"])/lift_motor_eff/esc_eff}
            for i,side in enumerate(("left","right")):
                mech=curve_value(c["propulsion"]["motors"][side]["static_shaft_power_curve"],self.motors[i].omega*60/(2*np.pi))
                channel_power[side]=mech/float(c["battery"]["propulsion_motor_efficiency"][side])/esc_eff
            channel_power["servos"]=float(c["propulsion"]["servo_electrical_power_w"])*sum(abs(s.rate_rad_s)>1e-6 for s in self.servos)
            electrical=self.battery.step(channel_power,self.dt)
            electrical["channel_power_w"]=channel_power; electrical["model_status"]="configured_model"
        else:
            electrical={"voltage_v":None,"current_a":None,"power_w":None,"energy_j":None,"state_of_charge":None,"remaining_energy_j":None,"model_status":"unconfigured","missing_parameters":elec_missing,"channel_power_w":None}
        if electrical.get("model_status")=="configured_model":
            electrical["estimated_remaining_duration_s"]=None if electrical["power_w"]<=0 else electrical["remaining_energy_j"]/electrical["power_w"]
        self.last = {"simulation_time_s":self.time_s,"position_world_m":self.body.position_world_m.tolist(),"center_of_mass_body_m":self.com.tolist(),
            "velocity_world_m_s":self.body.velocity_world_m_s.tolist(),"acceleration_world_m_s2":acc["acceleration_world_m_s2"].tolist(),
            "orientation_quaternion_body_to_world":self.body.orientation_body_to_world.tolist(),"omega_body_rad_s":self.body.omega_body_rad_s.tolist(),"angular_acceleration_body_rad_s2":acc["angular_acceleration_body_rad_s2"].tolist(),
            "lift":{"fan_rpm":self.fan_rpm,"ambient_pressure_pa":self.cushion.ambient_pressure_pa,**lift},"propulsion":{"left":prop[0],"right":prop[1],"total_force_body_n":(np.asarray(prop[0]["force_body_n"])+np.asarray(prop[1]["force_body_n"])).tolist()},
            "total_force_world_n":force_world.tolist(),"total_moment_body_nm":moment_b.tolist(),"payload_kg":self.payload_mass,"actuator_commands":dict(u),"disturbance_force_world_n":(np.asarray(e["disturbance"]["force_world_n"])+random_force).tolist(),"disturbance_moment_body_nm":(np.asarray(e["disturbance"]["moment_body_nm"])+random_moment).tolist(),"random_seed":self.seed,
            "mass_kg":self.body.mass_kg,"inertia_body_kg_m2":self.body.inertia_kg_m2.tolist(),
            "electrical":electrical,
            "attitude_euler_rad":self._euler_for_display(),"hover_height_m":float(frame_origin_world[2]-ground),"surface_height_m":ground,
            "measurement_class":"simulation","model_calibration_status":"uncalibrated","control_mode":u.get("mode","manual"),
            "physics_model_version":c["model_version"],"vehicle_configuration_id":self.vehicle.get("model_version","unversioned"),
            "control_authority":{"yaw":"differential_thrust","vertical":"air_cushion_pressure","roll":"no_independent_control_authority","pitch":"servo_thrust_vectoring_only"}}
        return self.last

    def _euler_for_display(self):
        w,x,y,z=self.body.orientation_body_to_world
        roll=np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y))
        pitch=np.arcsin(np.clip(2*(w*y-z*x),-1,1))
        yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
        return [float(roll),float(pitch),float(yaw)]

    def run(self, duration_s, commands=None, callback=None):
        if duration_s <= 0: raise ValueError("duration_s must be positive")
        samples=[]
        for _ in range(int(np.ceil(duration_s/self.dt))):
            sample=self.step(commands)
            samples.append(sample)
            if callback: callback(sample)
        return samples
