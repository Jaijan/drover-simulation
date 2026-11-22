"""Scientific descriptive statistics with explicit sample provenance."""
from __future__ import annotations
import numpy as np

def summarize(values):
    x=np.asarray(values,dtype=float)
    x=x[np.isfinite(x)]
    if len(x)==0: raise ValueError("No finite samples to summarize")
    sem=float(np.std(x,ddof=1)/np.sqrt(len(x))) if len(x)>1 else 0.0
    return {"n":int(len(x)),"mean":float(np.mean(x)),"median":float(np.median(x)),"std":float(np.std(x,ddof=1)) if len(x)>1 else 0.0,
            "min":float(np.min(x)),"max":float(np.max(x)),"rms":float(np.sqrt(np.mean(x*x))),"ci95_mean":[float(np.mean(x)-1.96*sem),float(np.mean(x)+1.96*sem)]}

def compare_simulated_to_measured(simulated, measured):
    a=np.asarray(simulated,float); b=np.asarray(measured,float)
    if a.shape!=b.shape or not len(a): raise ValueError("Measured and simulated arrays must have equal nonzero shape")
    err=a-b
    return {"rmse":float(np.sqrt(np.mean(err**2))),"mean_absolute_error":float(np.mean(np.abs(err))),"percent_error_mean":None if np.mean(np.abs(b))==0 else float(100*np.mean(np.abs(err))/np.mean(np.abs(b))),"comparison_class":"simulation_vs_physical_measurement"}

def correlation(x,y):
    a=np.asarray(x,float); b=np.asarray(y,float)
    if a.shape!=b.shape or a.size<2: raise ValueError("Correlation requires equal-length series with at least two points")
    if np.std(a)==0 or np.std(b)==0: return None
    return float(np.corrcoef(a,b)[0,1])
