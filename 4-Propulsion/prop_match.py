#%%
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import prop_uiuc

prop_db = prop_uiuc.parse_database(False)
#%%

class SymTable(dict):
    '''
    A bookkeeping class for symbolic variables.
    '''
    
    def declare(self, name: str, desc: str, value: float, lower: float, upper: float, unit: str):
        sym = ca.MX.sym(name)
        self[sym] = {
            'name': name,
            'desc': desc,
            'value': value,
            'lower': lower,
            'upper': upper,
            'unit': unit
        }
        return sym


syms = SymTable()

# variables
r_prop_in = syms.declare(name='r_prop_in', desc='prop radius, in', value=1, lower=0.01, upper=0.3, unit='in')
rpm_prop = syms.declare(name='rpm_prop', desc='prop rpm', value=0, lower=0, upper=100, unit='rad/s')
R_motor = syms.declare(name='R motor', desc='Motor resistance', value=0.1, lower=0, upper=1e6, unit='Ohms')
Kv_motor = syms.declare(name='Kv_motor', desc='Motor constant', value=1000, lower=100, upper=20000, unit='rpm/volt')
i0_motor = syms.declare(name='i0_motor', desc='No load current', value=0, lower=0, upper=100, unit='rpm/volt')
v_batt = syms.declare(name='v_batt', desc='Battery voltage', value=11.1, lower=3, upper=12, unit='volt')
V_inf = syms.declare(name='V_inf', desc='Freestream velocity', value=1, lower=0, upper=10, unit='volt')
rho = syms.declare(name='rho', desc='air density', value=1.225, lower=1, upper=2, unit='kg/m^3')
CT = syms.declare(name='CT', desc='Motor thrust constant', value=0, lower=0, upper=10, unit='')
CP = syms.declare(name='CP', desc='Motor torque constant', value=0, lower=0, upper=0, unit='')


n = rpm_prop / 60  # angular velocity, rev/s
omega_prop = n*2*ca.pi  # angular velocity of prop, rad/s

in_to_m = 0.0254
rpm_to_rps = 2*np.pi/60  # rev/min *(2*pi rad/rev)*(1min/60 sec) -> rad/sec

Kv_motor_rps = Kv_motor * rpm_to_rps  # covert Kv to rad/s
i_motor = (v_batt - omega_prop/Kv_motor_rps)/R_motor  # current motor
i_motor = ca.if_else(i_motor < 0, 0, i_motor)
Q_motor = (i_motor - i0_motor)/Kv_motor_rps  # torque motor
P_shaft = Q_motor*omega_prop  # power of shaft
P_elec = v_batt*i_motor  # electrical power
eta_motor = P_shaft/P_elec  # motor efficiency

# propeller
r_prop = r_prop_in*in_to_m  # radius of prop, m
J = V_inf/(n*2*r_prop)
V_prop = omega_prop*r_prop  # velocity of prop tip, m/s
q_prop = rho*V_prop**2/2  # dynamics pressure of prop tip, N/m^2
A_prop = ca.pi*r_prop**2  # area of prop disc, m^2
T_prop = q_prop*A_prop*CT  # thrust of prop
Q_prop = q_prop*A_prop*r_prop*CP  # torque of prop


#%%
def solve_nlp(prop_name):
    rho0 = 1.225
    i0_motor0 = 0.04
    V_inf0 = 1
    R_motor0 = 0.05
    T_desired = 0.1

    prop_data = prop_db[prop_name]
    key0 = list(prop_data['dynamic'].keys())[0]
    my_prop = prop_data['dynamic'][key0]['data']
    CT_lut = ca.interpolant('CT','bspline',[my_prop.index],my_prop.CT)
    CP_lut = ca.interpolant('CP','bspline',[my_prop.index],my_prop.CP)
    eta_lut = ca.interpolant('CP','bspline',[my_prop.index],my_prop.eta)
    eta_prop_lut = ca.interpolant('eta','bspline',[my_prop.index],my_prop.eta)
    Q_prop_lut = ca.substitute(Q_prop, CP, CP_lut(J))
    T_prop_lut = ca.substitute(T_prop, CT, CT_lut(J))

    states = ca.vertcat(rpm_prop, v_batt, Kv_motor)
    params = ca.vertcat(rho, r_prop_in, i0_motor, V_inf, R_motor)
    p0 = [rho0, prop_data['D']/2, i0_motor0, V_inf0, R_motor0]

    f_Q_prop = ca.Function('Q_prop', [states, params], [Q_prop_lut])
    f_Q_motor = ca.Function('Q_motor', [states, params], [Q_motor])
    f_eta_motor = ca.Function('eta_motor', [states, params], [eta_motor])
    f_eta_prop = ca.Function('eta_prop', [states, params], [eta_prop_lut(J)])
    f_eta = ca.Function('eta', [states, params], [eta_prop_lut(J)*eta_motor])
    f_T_prop = ca.Function('T_prop', [states, params], [T_prop_lut])

    #%%

    nlp = {
        'f': -f_eta(states, p0),
        'x': states,
        'g': ca.substitute(ca.vertcat(
            Q_prop_lut - Q_motor,
            T_prop_lut - T_desired
            ), params, p0)
    }
    S = ca.nlpsol('eff_max', 'ipopt', nlp,
        {'print_time': 0,
        'ipopt': {
            'sb': 'yes',
            'print_level': 0,
            }
        })
    res = S(x0=[1000, 5, 1000], lbg=[0, 0], ubg=[0, 0])
    stats = S.stats()

    if stats['success']:
        print(prop_name, res['x'], res['g'], -res['f'])
        v0 = float(res['x'][1])
        kv0 = float(res['x'][2])
        if -res['f'] > 0.3:
            rpm_sweep(prop_name, v0, kv0, i0_motor0, V_inf0, R_motor0)

    return res, stats

#%%
def rpm_sweep(prop_name, v0, kv0, i0_motor0, V_inf0, R_motor0):
    prop_data = prop_db[prop_name]
    key0 = list(prop_data['dynamic'].keys())[0]
    my_prop = prop_data['dynamic'][key0]['data']
    CT_lut = ca.interpolant('CT','bspline',[my_prop.index],my_prop.CT)
    CP_lut = ca.interpolant('CP','bspline',[my_prop.index],my_prop.CP)
    eta_lut = ca.interpolant('CP','bspline',[my_prop.index],my_prop.eta)
    eta_prop_lut = ca.interpolant('eta','bspline',[my_prop.index],my_prop.eta)
    Q_prop_lut = ca.substitute(Q_prop, CP, CP_lut(J))
    T_prop_lut = ca.substitute(T_prop, CT, CT_lut(J))

    states = ca.vertcat(rpm_prop)
    params = ca.vertcat(rho, r_prop_in, v_batt, Kv_motor, i0_motor, V_inf, R_motor)

    p0 = [1.225, prop_data['D']/2, v0, kv0, i0_motor0, V_inf0, R_motor0]

    f_Q_prop = ca.Function('Q_prop', [states, params], [Q_prop_lut])
    f_Q_motor = ca.Function('Q_motor', [states, params], [Q_motor])
    f_T_prop = ca.Function('T_prop', [states, params], [T_prop_lut])
    f_eta_motor = ca.Function('eta_motor', [states, params], [eta_motor])
    f_eta_prop = ca.Function('eta_prop', [states, params], [eta_prop_lut(J)])
    f_eta = ca.Function('eta', [states, params], [eta_prop_lut(J)*eta_motor])

    rpm = np.array([np.linspace(0, 4000, 1000)])

    plt.figure()
    plt.subplot(311)
    plt.plot(rpm.T, f_Q_motor(rpm, p0).T, label='motor')
    plt.plot(rpm.T, f_Q_prop(rpm, p0).T, label='prop')
    plt.title('prop motor matching')
    plt.legend()
    plt.ylabel('torque, N-m')
    plt.grid(True)

    plt.subplot(312)
    plt.plot(rpm.T, f_T_prop(rpm, p0).T, label='prop')
    plt.legend()
    plt.ylabel('thrust, N')
    plt.grid(True)

    plt.subplot(313)
    plt.plot(rpm.T, f_eta_motor(rpm, p0).T, label='motor')
    plt.plot(rpm.T, f_eta_prop(rpm, p0).T, label='prop')
    plt.plot(rpm.T, f_eta(rpm, p0).T, label='total')
    plt.xlabel('prop angular velocity, rpm')
    plt.ylabel('efficiency')
    plt.grid(True)
    plt.legend()
    plt.gca().set_ylim(0, 1)
    plt.show()

#%%
for prop_name in prop_db.keys():
    res, stats = solve_nlp(prop_name)
