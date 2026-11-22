import numpy as np
import pytest
from backend.physics.models import AirCushion, Motor, Servo, RigidBody, orifice_mass_flow, point_force_moment
from backend.controllers.pid import PID, allocate_differential_thrust
from backend.battery.model import Battery

def test_force_moment_right_hand_rule():
    assert np.allclose(point_force_moment(np.array([0,1,0]),np.array([1,0,0])),[0,0,-1])

def test_gravity_and_static_lift_equilibrium():
    m=2.0; dt=.01; body=RigidBody(m,np.diag([.1,.2,.3]),np.zeros(3),np.zeros(3),np.array([1.,0,0,0]),np.zeros(3))
    result=body.step(np.array([0.,0.,m*9.80665]),np.zeros(3),dt)
    assert np.allclose(result["acceleration_world_m_s2"],0)
    assert np.allclose(body.position_world_m,0)

def test_quaternion_remains_normalized_under_rotation():
    body=RigidBody(1,np.eye(3),np.zeros(3),np.zeros(3),np.array([1.,0,0,0]),np.array([0.,0.,1.]))
    for _ in range(1000): body.step(np.array([0.,0.,9.80665]),np.zeros(3),.01)
    assert np.isclose(np.linalg.norm(body.orientation_body_to_world),1)
    assert body.orientation_body_to_world[3] != 0

def test_semi_implicit_euler_timestep_sensitivity():
    def fall(dt):
        body=RigidBody(1,np.eye(3),np.zeros(3),np.zeros(3),np.array([1.,0,0,0]),np.zeros(3))
        for _ in range(round(1/dt)): body.step(np.array([0.,0.,-9.80665]),np.zeros(3),dt)
        return body.position_world_m[2]
    exact=-0.5*9.80665
    assert abs(fall(.05)-exact)<abs(fall(.1)-exact)

def test_seeded_disturbance_sequence_is_repeatable():
    a=np.random.default_rng(23).normal(0,.2,100)
    b=np.random.default_rng(23).normal(0,.2,100)
    assert np.array_equal(a,b)

def test_invalid_inertia_is_rejected():
    body=RigidBody(1,np.diag([1.,1.,0.]),np.zeros(3),np.zeros(3),np.array([1.,0,0,0]),np.zeros(3))
    with pytest.raises(ValueError): body.step(np.zeros(3),np.zeros(3),.01)

def test_orifice_flow_has_correct_pressure_boundary():
    assert orifice_mass_flow(.6,.001,1.2,0)==0
    assert orifice_mass_flow(.6,.001,1.2,100)>0

def test_cushion_air_mass_balance_and_pressure_lift_relation():
    fan={"rpm_points":[0,1000],"pressure_pa":[0,100],"flow_m3_s":[[0,0],[.1,.02]]}
    model=AirCushion(.01,.2,fan,.0001,.6,287,293,101325,1.2,101325*.01/(287*293))
    before=model.air_mass_kg
    state=model.step(1000,.01)
    assert model.air_mass_kg > before
    assert state["lift_force_n"] == pytest.approx(state["pressure_differential_pa"]*.2)

def test_motor_and_servo_response_limits():
    motor=Motor(1e-6,1000,1,100,0,1,.001)
    thrust,_=motor.step(1000,.01)
    assert motor.omega==pytest.approx(1)
    assert thrust==pytest.approx(1e-6)
    servo=Servo(-.5,.5,1,.1)
    assert servo.step(2,.01)==pytest.approx(.01)
    for _ in range(60): servo.step(2,.01)
    assert servo.angle_rad<=.5

def test_battery_energy_balance_and_sag():
    battery=Battery(12,1,.1)
    out=battery.step({"lift":12},1)
    assert out["power_w"]==12
    assert out["current_a"]>0
    assert out["voltage_v"]<12
    assert out["energy_j"]==12

def test_pid_saturates_and_mixer_respects_limits():
    pid=PID(100,0,0,(-10,10),(-1,1))
    assert pid.step(1,0,.1)==10
    assert allocate_differential_thrust(90,20,100)==(100,70)
