from __future__ import annotations
import numpy as np

class Battery:
    """Battery with configurable open-circuit voltage, resistance, capacity, and channel powers."""
    def __init__(self, nominal_voltage_v, capacity_ah, resistance_ohm):
        if min(nominal_voltage_v,capacity_ah)<=0 or resistance_ohm<0: raise ValueError("Invalid battery parameters")
        self.nominal=float(nominal_voltage_v); self.capacity_j=float(capacity_ah)*3600*self.nominal
        self.resistance=float(resistance_ohm); self.energy_used_j=0.0
    def step(self, channel_power_w, dt):
        if self.energy_used_j>=self.capacity_j: raise ValueError("Battery depleted: configured capacity exhausted")
        p=max(0.0,float(sum(channel_power_w.values())))
        if self.resistance==0: current=p/self.nominal
        else:
            disc=self.nominal**2-4*self.resistance*p
            if disc<0: raise ValueError("Requested power exceeds configured battery maximum-power boundary")
            current=(self.nominal-np.sqrt(disc))/(2*self.resistance)
        voltage=self.nominal-current*self.resistance
        self.energy_used_j+=p*dt
        return {"voltage_v":float(voltage),"current_a":float(current),"power_w":p,"energy_j":self.energy_used_j,
                "state_of_charge":float(max(0.0,1-self.energy_used_j/self.capacity_j)),"remaining_energy_j":max(0.0,self.capacity_j-self.energy_used_j)}

def curve_value(curve, x):
    if not curve: raise ValueError("A measured or estimated power curve is required")
    pts=sorted(curve)
    if x<pts[0][0] or x>pts[-1][0]: raise ValueError("Operating speed is outside the configured power curve")
    return float(np.interp(x,[p[0] for p in pts],[p[1] for p in pts]))
