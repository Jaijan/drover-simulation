"""Small SI-unit physical models; each parameter is supplied by configuration."""
from __future__ import annotations
from dataclasses import dataclass
from math import sqrt
from typing import Sequence
import numpy as np

G = 9.80665

@dataclass
class AirCushion:
    volume_m3: float
    area_m2: float
    fan_curve: dict  # flow-versus-pressure matrix at configured fan RPM points
    leakage_area_m2: float
    discharge_coefficient: float
    gas_constant: float
    temperature_k: float
    ambient_pressure_pa: float
    density_kg_m3: float
    air_mass_kg: float

    def step(self, rpm: float, dt: float, height_m: float = 0.0) -> dict:
        if min(self.volume_m3, self.area_m2, self.temperature_k, self.density_kg_m3) <= 0 or dt <= 0:
            raise ValueError("Cushion volume, area, temperature, density, and dt must be positive")
        dp = max(0.0, self.air_mass_kg * self.gas_constant * self.temperature_k / self.volume_m3 - self.ambient_pressure_pa)
        # Bilinear interpolation of configured fan flow-pressure data; no free-flow derating surrogate.
        if not isinstance(self.fan_curve,dict): raise ValueError("Fan curve must be a pressure-flow matrix")
        rpms=np.asarray(self.fan_curve["rpm_points"],float); pressures=np.asarray(self.fan_curve["pressure_pa"],float); flows=np.asarray(self.fan_curve["flow_m3_s"],float)
        if flows.shape!=(len(rpms),len(pressures)) or not len(rpms) or not len(pressures): raise ValueError("Fan flow matrix dimensions do not match its RPM and pressure axes")
        if np.any(np.diff(rpms)<=0) or np.any(np.diff(pressures)<=0) or np.any(flows<0): raise ValueError("Fan curve axes must be increasing and flow nonnegative")
        if dp<pressures[0] or dp>pressures[-1] or rpm<rpms[0] or rpm>rpms[-1]: raise ValueError("Operating point is outside the configured fan curve")
        q_by_rpm=np.array([np.interp(dp,pressures,row) for row in flows])
        q_in=float(np.interp(max(0.0,rpm),rpms,q_by_rpm))
        # Clearance-dependent opening is clamped nonnegative; a zero clearance can still have a configured leak area.
        effective_leak = self.leakage_area_m2 * max(0.0, 1.0 + height_m / max(sqrt(self.area_m2), 1e-6))
        mdot_out = min(self.discharge_coefficient * effective_leak * sqrt(2.0 * self.density_kg_m3 * dp),self.air_mass_kg/dt)
        mdot_in = self.density_kg_m3 * q_in
        self.air_mass_kg = max(0.0, self.air_mass_kg + (mdot_in - mdot_out) * dt)
        pressure = self.air_mass_kg * self.gas_constant * self.temperature_k / self.volume_m3
        dp = max(0.0, pressure - self.ambient_pressure_pa)
        return {"cushion_pressure_pa": pressure, "pressure_differential_pa": dp,
                "lift_force_n": dp * self.area_m2, "airflow_m3_s": q_in,
                "mass_flow_in_kg_s": mdot_in, "leakage_kg_s": mdot_out,
                "air_mass_kg": self.air_mass_kg}

@dataclass
class Motor:
    thrust_coefficient: float
    max_speed_rad_s: float
    response_time_s: float
    max_accel_rad_s2: float
    reaction_torque_coefficient: float
    rotation_sign: float
    rotor_inertia_kg_m2: float
    omega: float = 0.0

    def __post_init__(self):
        if self.thrust_coefficient<0 or self.max_speed_rad_s<=0 or self.response_time_s<=0 or self.max_accel_rad_s2<=0 or self.reaction_torque_coefficient<0 or self.rotor_inertia_kg_m2<0 or abs(self.rotation_sign)!=1:
            raise ValueError("Motor constants must be physical, positive where required, and rotation_sign must be -1 or +1")

    def step(self, command_rad_s: float, dt: float) -> tuple[float, float]:
        target = float(np.clip(command_rad_s, 0.0, self.max_speed_rad_s))
        response = max(self.response_time_s, dt)
        rate = np.clip((target-self.omega)/response, -self.max_accel_rad_s2, self.max_accel_rad_s2)
        self.omega = float(np.clip(self.omega + rate*dt, 0.0, self.max_speed_rad_s))
        thrust = self.thrust_coefficient * self.omega**2
        return thrust, self.rotation_sign*self.reaction_torque_coefficient*thrust

@dataclass
class Servo:
    minimum_rad: float
    maximum_rad: float
    max_rate_rad_s: float
    response_time_s: float
    angle_rad: float = 0.0
    rate_rad_s: float = 0.0

    def __post_init__(self):
        if self.minimum_rad>=self.maximum_rad or self.max_rate_rad_s<=0 or self.response_time_s<=0:
            raise ValueError("Servo angle limits, rate, and response time are invalid")

    def step(self, command_rad: float, dt: float) -> float:
        target = float(np.clip(command_rad, self.minimum_rad, self.maximum_rad))
        rate = np.clip((target-self.angle_rad)/max(self.response_time_s, dt), -self.max_rate_rad_s, self.max_rate_rad_s)
        previous=self.angle_rad
        self.angle_rad = float(np.clip(self.angle_rad+rate*dt, self.minimum_rad, self.maximum_rad))
        self.rate_rad_s=(self.angle_rad-previous)/dt
        return self.angle_rad

def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w,x,y,z=a; W,X,Y,Z=b
    return np.array([w*W-x*X-y*Y-z*Z, w*X+x*W+y*Z-z*Y,
                     w*Y-x*Z+y*W+z*X, w*Z+x*Y-y*X+z*W])

def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    return quat_multiply(quat_multiply(q, np.r_[0.0, v]), np.array([q[0],-q[1],-q[2],-q[3]]))[1:]

@dataclass
class RigidBody:
    mass_kg: float
    inertia_kg_m2: np.ndarray
    position_world_m: np.ndarray
    velocity_world_m_s: np.ndarray
    orientation_body_to_world: np.ndarray
    omega_body_rad_s: np.ndarray

    def step(self, force_world_n: np.ndarray, moment_body_nm: np.ndarray, dt: float) -> dict:
        I = self.inertia_kg_m2
        if self.mass_kg <= 0 or dt <= 0 or I.shape!=(3,3) or not np.all(np.isfinite(I)) or not np.allclose(I,I.T,atol=1e-10) or np.min(np.linalg.eigvalsh(I)) <= 0:
            raise ValueError("Mass and timestep must be positive and inertia positive definite")
        acceleration = force_world_n/self.mass_kg
        alpha = np.linalg.solve(I, moment_body_nm-np.cross(self.omega_body_rad_s, I@self.omega_body_rad_s))
        # Semi-implicit Euler: update velocities before positions/orientation.
        self.velocity_world_m_s = self.velocity_world_m_s + acceleration*dt
        self.position_world_m = self.position_world_m + self.velocity_world_m_s*dt
        self.omega_body_rad_s = self.omega_body_rad_s + alpha*dt
        qdot = 0.5*quat_multiply(self.orientation_body_to_world, np.r_[0.0, self.omega_body_rad_s])
        self.orientation_body_to_world += qdot*dt
        self.orientation_body_to_world /= np.linalg.norm(self.orientation_body_to_world)
        return {"acceleration_world_m_s2": acceleration, "angular_acceleration_body_rad_s2": alpha}

def point_force_moment(position_body_m: np.ndarray, force_body_n: np.ndarray) -> np.ndarray:
    return np.cross(position_body_m, force_body_n)

def orifice_mass_flow(cd: float, area_m2: float, density_kg_m3: float, pressure_delta_pa: float) -> float:
    return cd*area_m2*sqrt(2*density_kg_m3*max(0.0, pressure_delta_pa))
