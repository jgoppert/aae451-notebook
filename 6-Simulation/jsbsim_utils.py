from pathlib import Path
import time
import pandas as pd
import numpy as np
import scipy.optimize
import pandas as pd

import jsbsim


class Logger:
    
    def __init__(self, topics=None):
        self.time = []
        self.data = {}
        self.topics = topics

    def new_frame(self, index, data):
        self.time.append(index)
        if self.topics is not None:
            save_keys = self.topics
        else:
            save_keys = data.keys()
        for key in save_keys:
            if key not in self.data.keys():
                self.data[key] = []
            self.data[key].append(data[key])
    
    def to_pandas(self):
        return pd.DataFrame(self.data, pd.Index(self.time, name='t, sec'))

    
def set_location_purdue_airport(fdm):
    # location, Purdue airport
    fdm['ic/terrain-elevation-ft'] = 584.5
    fdm['ic/lat-geod-deg'] = 40.4110213580397
    fdm['ic/long-gc-deg'] = -86.92756398413263
    fdm['ic/psi-true-deg'] = 280


def trim(aircraft, ic, design_vector, x0, verbose, cost=None, **kwargs):
    root = Path('.').resolve()
    fdm = jsbsim.FGFDMExec(str(root)) # The path supplied to FGFDMExec is the location of the folders "aircraft", "engines" and "systems"
    fdm.set_debug_level(0)    
    fdm.load_model(aircraft)

    # set location
    set_location_purdue_airport(fdm)
    
    fdm_props = fdm.get_property_catalog('')
    for key in design_vector + list(ic.keys()):
        if key not in fdm_props.keys():
            raise KeyError(key)
    
    if cost is None:
        def cost(fdm):
            # compute cost, force moment balance
            udot = fdm['accelerations/udot-ft_sec2']
            vdot = fdm['accelerations/vdot-ft_sec2']
            wdot = fdm['accelerations/wdot-ft_sec2']
            pdot = fdm['accelerations/pdot-rad_sec2']
            qdot = fdm['accelerations/qdot-rad_sec2']
            rdot = fdm['accelerations/rdot-rad_sec2']
            return udot**2 + vdot**2 + wdot**2 + pdot**2 + qdot**2 + rdot**2

    # cost function for trimming
    def trim_cost(fdm, xd):
        # set initial condition
        for var in ic.keys():
            fdm[var] = ic[var]

        # set design vector
        for i, var in enumerate(design_vector):
            fdm[var] = xd[i]
        
        # trim propulsion
        prop = fdm.get_propulsion()
        prop.init_running(-1)

        # set initial conditions
        fdm.run_ic()
        
        return cost(fdm)
    
    res = scipy.optimize.minimize(fun=lambda xd: trim_cost(fdm, xd), x0=x0, **kwargs)
    if verbose:
        print(res)
    for i, var in enumerate(design_vector):
        ic[var] = res['x'][i]
    
    return ic, fdm


def simulate(aircraft, op_0, op_list=None, op_times=None, tf=50, realtime=False):
    if op_list is None:
        op_list = []
    
    if op_times is None:
        op_times = []
        
    root = Path('.').resolve()
    fdm = jsbsim.FGFDMExec(str(root)) # The path supplied to FGFDMExec is the location of the folders "aircraft", "engines" and "systems"

    # load model
    fdm.load_model(aircraft)
    fdm.set_output_directive(str(root/ 'data_output' / 'flightgear.xml'))
    fdm_props = fdm.get_property_catalog('')
    
    # add a method to check that properties being set exist
    def set_opereating_point(props):
        for key in props.keys():
            if key not in fdm_props.keys():
                raise KeyError(key)
            else:
                fdm[key] = props[key]
    
    # set location
    set_location_purdue_airport(fdm)
    
    # set operating points
    set_opereating_point(op_0)
    
    # start engines
    prop = fdm.get_propulsion()
    prop.init_running(-1)
    fdm.run_ic()

    # start loop
    log = Logger()
    nice = True
    sleep_nseconds = 500
    initial_seconds = time.time()
    frame_duration = fdm.get_delta_t()
    op_count = 0
    result = fdm.run()

    while result and fdm.get_sim_time() < tf:
        t = fdm.get_sim_time()
        if op_count < len(op_times) and t > op_times[op_count]:
            set_opereating_point(op_list[op_count])
            op_count += 1

        if realtime:
            current_seconds = time.time()
            actual_elapsed_time = current_seconds - initial_seconds
            sim_lag_time = actual_elapsed_time - fdm.get_sim_time()

            for _ in range(int(sim_lag_time / frame_duration)):
                result = fdm.run()
                current_seconds = time.time()
        else:
            result = fdm.run()
            if nice:
                time.sleep(sleep_nseconds / 1000000.0)
        log.new_frame(
            t, fdm.get_property_catalog(''))

    log = log.to_pandas()
    return log