from pathlib import Path
import time
from copy import deepcopy

import matplotlib.pyplot as plt
import jsbsim
import scipy.optimize
import pandas as pd
import numpy as np
import pandas as pd
import control

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
    fdm['ic/terrain-elevation-ft'] = 586
    fdm['ic/lat-geod-deg'] = 40.4110213580397
    fdm['ic/long-gc-deg'] = -86.92756398413263
    fdm['ic/psi-true-deg'] = 280


def trim(aircraft, ic, design_vector, x0, verbose,
         cost=None, eq_constraints=None, tol=1e-6, ftol=None, show=False, **kwargs):
    root = Path('.').resolve()
    fdm = jsbsim.FGFDMExec(str(root)) # The path supplied to FGFDMExec is the location of the folders "aircraft", "engines" and "systems"
    if show:
        fdm.set_output_directive(str(root/ 'data_output' / 'flightgear.xml'))
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
            theta = fdm['attitude/theta-rad']
            udot = fdm['accelerations/udot-ft_sec2']
            vdot = fdm['accelerations/vdot-ft_sec2']
            wdot = fdm['accelerations/wdot-ft_sec2']
            pdot = fdm['accelerations/pdot-rad_sec2']
            qdot = fdm['accelerations/qdot-rad_sec2']
            rdot = fdm['accelerations/rdot-rad_sec2']
            # (theta < 0) term prevents nose down trim
            return udot**2 + vdot**2 + wdot**2 + \
                pdot**2 + qdot**2 + rdot**2 + + 1e-3*(theta < 0)

    # cost function for trimming
    def eval_fdm_func(xd, fdm, ic, fdm_func):
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
        
        return fdm_func(fdm)

    # setup constraints
    constraints = []
    if eq_constraints is None:
        eq_constraints = []
    for con in eq_constraints:
        constraints.append({
            'type': 'eq',
            'fun': eval_fdm_func,
            'args': (fdm, ic, con)
        })
    
    # solve
    res = scipy.optimize.minimize(
        fun=eval_fdm_func,
        args=(fdm, ic, cost), x0=x0, 
        constraints=constraints, **kwargs)
    
    if verbose:
        print(res)
    
    # update ic
    for i, var in enumerate(design_vector):
        ic[var] = res['x'][i]
    
    if verbose:
        for con in constraints:
            print('constraint', con['type'], con['fun'](res['x'], *con['args']))
    
    if not res['success'] or ftol is not None and abs(res['fun']) > ftol:
        raise RuntimeError('trim failed:\n' + str(res) + '\n' + \
                           str(fdm.get_property_catalog('acceleration')))
    props = fdm.get_property_catalog('')
    del fdm
    return ic, props


def simulate(aircraft, op_0, op_list=None, tf=50, realtime=False, verbose=False):
    if op_list is None:
        op_list = []
        
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
    low_fuel_warned = False
    
    while result and fdm.get_sim_time() < tf:
        t = fdm.get_sim_time()
        if op_count < len(op_list) and op_list[op_count][2](fdm):
            if verbose:
                print('operating point reached: ', op_list[op_count][0])
            set_opereating_point(op_list[op_count][1])
            op_count += 1
        
        # rough approximation of grade of runway 28 at purdue airport
        if fdm['position/distance-from-start-mag-mt'] < 2000:
            fdm['position/terrain-elevation-asl-ft'] = 586 + 0.02*fdm['position/distance-from-start-mag-mt']
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
        
        if not low_fuel_warned and fdm['propulsion/total-fuel-lbs'] < 10:
            print('warning: LOW FUEL {:0.2f} lbs, restart simulation'.format(fdm['propulsion/total-fuel-lbs']))
            low_fuel_warned = True

        log.new_frame(t, fdm.get_property_catalog(''))

    log = log.to_pandas()
    del fdm
    return log


def linearize(aircraft, states, states_deriv, inputs, outputs, ic, dx, n_round=3):
    assert len(states_deriv) == len(states)

    root = Path('.').resolve()
    fdm = jsbsim.FGFDMExec(str(root)) # The path supplied to FGFDMExec is the location of the folders "aircraft", "engines" and "systems"
    fdm.set_debug_level(0)    
    fdm.load_model(aircraft)
        
    fdm_props = fdm.get_property_catalog('')
    for key in states + inputs + outputs + list( ic.keys()):
        if key not in fdm_props.keys():
            raise KeyError(key)
    
    for key in states_deriv:
        if key not in fdm_props.keys() and key != 'approximate':
            raise KeyError(key)
    
    n = len(states)
    p = len(inputs)
    m = len(outputs)
    
    def set_ic():
        set_location_purdue_airport(fdm)
        for key in ic.keys():
            fdm[key] = ic[key]
        fdm.get_propulsion().init_running(-1)
        fdm.run_ic()
    
    A = np.zeros((n, n))
    C = np.zeros((m, n))
    for i, state in enumerate(states):
        set_ic()
        start = fdm.get_property_catalog('')
        fdm[state] = start[state] + dx
        fdm.resume_integration()
        fdm.run()
        for j, state_deriv in enumerate(states_deriv):
            if state_deriv == 'approximate':
                A[j, i] = (fdm[states[j]] - start[states[j]])/fdm.get_delta_t()
            else:
                A[j, i] = (fdm[state_deriv] - start[state_deriv]) / dx
        for j, output in enumerate(outputs):
            C[j, i] = (fdm[output] - start[output]) / dx

    set_ic()

    B = np.zeros((n, p))
    D = np.zeros((m, p))
    for i, inp in enumerate(inputs):
        set_ic()
        start = fdm.get_property_catalog('')
        fdm[inp] = start[inp] + dx
        fdm.resume_integration()
        fdm.run()
        for j, state_deriv in enumerate(states_deriv):
            if state_deriv == 'approximate':
                B[j, i] = (fdm[states[j]] - start[states[j]])/fdm.get_delta_t()
            else:
                B[j, i] = (fdm[state_deriv] - start[state_deriv]) / dx
        for j, output in enumerate(outputs):
            D[j, i] = (fdm[output] - start[output]) / dx
    
    del fdm
    A = np.round(A, n_round)
    B = np.round(B, n_round)
    C = np.round(C, n_round)
    D = np.round(D, n_round)
    return (A, B, C, D)


def rootlocus(sys, kvect=None):
    if kvect is None:
        kvect = np.logspace(-3, 0, 1000)
    rlist, klist = control.rlocus(sys, plot=False, kvect=kvect)
    for root in rlist.T:
        plt.plot(np.real(root), np.imag(root))
        plt.plot(np.real(root[-1]), np.imag(root[-1]), 'bs')
    for pole in control.pole(sys):
        plt.plot(np.real(pole), np.imag(pole), 'rx')
    for zero in control.zero(sys):
        plt.plot(np.real(zero), np.imag(zero), 'go')
    plt.grid()
    plt.xlabel('real')
    plt.ylabel('imag')


def clean_tf(G, tol=1e-5):
    num = G.num
    den = G.den
    for poly in num, den:
        for i in range(len(poly)):
            for j in range(len(poly[i])):
                poly[i][j] = np.where(np.abs(poly[i][j]) < tol, 0, poly[i][j])
    return control.tf(num, den)
