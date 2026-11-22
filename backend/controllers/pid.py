from __future__ import annotations

class PID:
    """Discrete PID with derivative-on-measurement and bounded integral anti-windup."""
    def __init__(self,kp,ki,kd,output_limits,integral_limits):
        self.kp=float(kp); self.ki=float(ki); self.kd=float(kd)
        self.low,self.high=map(float,output_limits); self.i_low,self.i_high=map(float,integral_limits)
        if self.low>=self.high or self.i_low>self.i_high: raise ValueError("Invalid PID limits")
        self.integral=0.; self.previous_measurement=None
    def reset(self): self.integral=0.; self.previous_measurement=None
    def step(self,setpoint,measurement,dt):
        if dt<=0: raise ValueError("PID timestep must be positive")
        error=float(setpoint)-float(measurement)
        derivative=0. if self.previous_measurement is None else -(float(measurement)-self.previous_measurement)/dt
        candidate=max(self.i_low,min(self.i_high,self.integral+error*dt))
        raw=self.kp*error+self.ki*candidate+self.kd*derivative
        out=max(self.low,min(self.high,raw))
        if out==raw or (out>=self.high and error<0) or (out<=self.low and error>0): self.integral=candidate
        self.previous_measurement=float(measurement)
        return out

def allocate_differential_thrust(base_rpm,yaw_delta_rpm,max_rpm):
    """Rear pair mixer; yaw authority comes from the measured/configured arm geometry."""
    left_limit,right_limit=max_rpm if isinstance(max_rpm,tuple) else (max_rpm,max_rpm)
    return (max(0.,min(left_limit,base_rpm+yaw_delta_rpm)),max(0.,min(right_limit,base_rpm-yaw_delta_rpm)))
