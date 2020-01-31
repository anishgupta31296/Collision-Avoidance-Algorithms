"""

Frenet optimal trajectory generator

Modified : Mithun
Original author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import fplan.cubic_spline_planner as cubic_spline_planner
import pdb
import numpy as np
import time
from joblib import Parallel, delayed
import multiprocessing
from functools import wraps
import xlwt
from xlwt import Workbook
import pandas as pd

import keras
# from torch.utils.data import Dataset
from keras.callbacks import ModelCheckpoint
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM
from keras import optimizers
import matplotlib.pyplot as plt
from keras import backend as K
from keras.models import load_model
from sklearn.metrics import mean_squared_error

SIM_LOOP = 500

# Parameter (don't change them)
# ford mustang can do it!!!
MAX_ACCEL = 4.0  # maximum acceleration [m/ss]
MAX_DCCEL = -6.0
# 3m turning radius
MAX_CURVATURE = 2 # maximum curvature [1/m]
# smaller samples are better but slow
D_ROAD_W = 0.5 # road width sampling length [m] 
DT = 0.1  # time tick [s]
MAXT = 4.0  # max prediction time [m]
MINT = 0.2  # min prediction time [m]
D_S_S = 0.1 # target distance sampling length [m]
N_S_S_SAMPLE = 30
ROBOT_RADIUS = 2.8 # robot radius [m]
NUM_CORES = multiprocessing.cpu_count()

# cost weights (dont change them)
KJ11 = 0.01 # jerk 1.0
KJ12 = 0.01 # 1.0
KT1 = 1.0 # Ti 10.0
KD1 = 4.0 # d 4.0
KS1 = 1.0 # distance along path 0.18
KV1 = 0.01 # (vd-vc)^2
KLAT1 = 0.2
KLON1 = 0.2

# cost weights (dont change them)
KJ2 = 0.1 # jerk
KT2 = 1.0 # Ti
KD2 = 0.5 # deviation form center line (need to be tuned)
KS2 = 0.2 # distance along path
KV2 = 0.0 # (vd-vc)^2
KLAT2 = 1.0
KLON2 = 1.0

def simple_time_tracker(log_fun):
    def _simple_time_tracker(fn):
        @wraps(fn)
        def wrapped_fn(*args, **kwargs):
            start_time = time.time()

            try:
                result = fn(*args, **kwargs)
            finally:
                elapsed_time = time.time() - start_time

                # log the result
                log_fun({
                    'function_name': fn.__name__,
                    'total_time': elapsed_time,
                })
                
            return result

        return wrapped_fn
    return _simple_time_tracker

def _log(message):
    print('[SimpleTimeTracker] {function_name} {total_time:.3f}'.format(**message))


class quintic_polynomial_t:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):

        # calc coefficient of quintic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[T**3, T**4, T**5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T**2,
                      vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t**2

        return xt



class quintic_polynomial_s:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, s0, se):

        # calc coefficient of quintic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        A = np.array([[1, s0, s0**2, s0**3, s0**4, s0**5],
                      [0, 1, 2*s0, 3*s0**2, 4*s0**3, 5*s0**4 ],
                      [0, 0, 2, 6*s0, 12*s0**2, 20*s0**3],
                      [1, se, se**2, se**3, se**4, se**5],
                      [0, 1, 2*se, 3*se**2, 4*se**3, 5*se**4 ],
                      [0, 0, 2, 6*se, 12*se**2, 20*se**3]])
        b = np.array([xs,
                      vxs,
                      axs,
                      xe,
                      vxe,
                      axe])
        x = np.linalg.solve(A, b)

        self.a0 = x[0]
        self.a1 = x[1]
        self.a2 = x[2]
        self.a3 = x[3]
        self.a4 = x[4]
        self.a5 = x[5]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t**2

        return xt


class quartic_polynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, T):

        # calc coefficient of quintic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * T ** 2, 4 * T ** 3],
                      [6 * T, 12 * T ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class Frenet_path:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []
        self.vx = []
        self.dth = []
        self.ax = []
        self.kr = []
        self.thr = []
        self.w = []


def calc_frenet_dtv(csp, ob, c_speed, c_d, c_d_d, c_d_dd, c_s_dd, s0, di, Ti, DTFPS, OPTION, SETS, p):
    
    MAX_SPEED = SETS['MAX_SPEED']
    TARGET_SPEED = SETS['TARGET_SPEED']
    D_T_S = SETS['D_T_S']
    N_S_SAMPLE = SETS['N_S_SAMPLE'] 

    fp = Frenet_path()
    if OPTION[0] == 'OVERTAKE':
        tv = OPTION[1]
        lon_qp = quartic_polynomial(s0, c_speed, c_s_dd, tv, 0.0, Ti)
        fp.t = [t for t in np.arange(0.0, Ti, DTFPS)]
        fp.s = [lon_qp.calc_point(t) for t in fp.t]
        fp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
        fp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
        fp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]
    elif OPTION[0] == 'STOP':
        s_e = OPTION[1]
        lon_qp = quintic_polynomial_t(s0, c_speed, c_s_dd, s_e, 0.0, 0.0, Ti)
        fp.t = [t for t in np.arange(0.0, Ti, DTFPS)]
        fp.s = [lon_qp.calc_point(t) for t in fp.t]
        fp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
        fp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
        fp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]
    lat_qp = quintic_polynomial_s(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, s0, fp.s[-1])
    fp.d = [lat_qp.calc_point(s) for s in fp.s]
    fp.d_d = [lat_qp.calc_first_derivative(s) for s in fp.s]
    fp.d_dd = [lat_qp.calc_second_derivative(s) for s in fp.s]
    fp.d_ddd = [lat_qp.calc_third_derivative(s) for s in fp.s]

    pts = len(fp.t)
    # x,y
    for i in range(pts):
        ix, iy = csp.calc_position(fp.s[i])
        if ix is None:
            break
        iyaw = csp.calc_yaw(fp.s[i])
        di = fp.d[i]
        fx = ix + di * math.cos(iyaw + math.pi / 2.0)
        fy = iy + di * math.sin(iyaw + math.pi / 2.0)
        fp.x.append(fx)
        fp.y.append(fy)

    pts = len(fp.x)
    kr = []
    thr = []
    for i in range(pts):
        kr.append(csp.calc_curvature(fp.s[i]))
        thr.append(csp.calc_yaw(fp.s[i]))

    fp.kr = kr
    fp.thr = thr
    kr_d = []
    for i in range(len(kr)-1):
        kr_d.append((kr[i+1]-kr[i])/(fp.s[i+1]-fp.s[i]))
    if len(kr_d):
        kr_d.append(kr_d[-1])
    else:
        kr_d.append(0.0)
    # yaw
    for i in range(pts):
        temp_dth = math.atan(fp.d_d[i]/(1-kr[i]*fp.d[i]))
        fp.yaw.append(thr[i] + temp_dth)
        fp.vx.append(fp.s_d[i]*(1-kr[i]*fp.d[i])/(math.cos(temp_dth)))
        temp_dd = fp.d_dd[i] + (kr_d[i]*fp.d[i] + kr[i]*fp.d_d[i])*math.tan(temp_dth)
        temp_dd = kr[i] + temp_dd*((math.cos(temp_dth)**2)/(1-kr[i]*fp.d[i]))
        fp.c.append(temp_dd*math.cos(temp_dth)/(1-kr[i]*fp.d[i]))
        fp.dth.append(temp_dth)

    for i in range(pts-1):
        temp_dd = fp.s_dd[i]*((1-kr[i]*fp.d[i])/(math.cos(fp.dth[i])))
        temp_di = ((1-kr[i]*fp.d[i])*math.tan(fp.dth[i])*((fp.dth[i+1]-fp.dth[i])/(fp.s[i+1]-fp.s[i])) - \
                    (kr_d[i]*fp.d[i] + kr[i]*fp.d_d[i]))
        temp_dd = temp_dd + (((fp.s_d[i]**2)*temp_di)/(math.cos(fp.dth[i])))
        fp.ax.append(temp_dd)
        fp.w.append((fp.yaw[i+1]-fp.yaw[i])/DTFPS)
    if len(fp.ax):
        fp.ax.append(fp.ax[-1])
        fp.w.append(fp.w[-1])
    else:
        fp.ax.append(0.0)
        fp.w.append(0.0)

    Jp = sum(np.power(fp.d_ddd, 2))  # square of jerk
    Js = sum(np.power(fp.s_ddd, 2))  # square of jerk


    if OPTION[0] == 'OVERTAKE':
        dist_cost = (TARGET_SPEED*(0+MAXT)/2)**2
        fp.cd = KJ11 * Jp + KS1 * (fp.s[-1]-s0-dist_cost)**2 + KD1 * fp.d[-1]**2
        ds = (TARGET_SPEED - fp.vx[-1])**2
        fp.cv = KJ12 * Js + KT1 * Ti + KV1 * ds
        fp.cf = KLAT1 * fp.cd + KLON1 * fp.cv
    elif OPTION[0] == 'STOP':
        fp.cd = KJ2 * Jp + KS2 * (fp.s[-1]-s0)**2 + KD2 * fp.d[-1]**2    
        TARGET_DISTANCE = OPTION[1]
        ds = (TARGET_DISTANCE - fp.s[-1])**2
        fp.cv = KJ2 * Js + KT2 * Ti + KV2 * ds
        fp.cf = KLAT2 * fp.cd + KLON2 * fp.cv

    fp.J1 = Jp
    fp.J2 = (fp.s[-1]-s0)**2
    fp.J3 = fp.d[-1]**2
    fp.J4 = Js
    fp.J5 = Ti
    fp.J6 = ds

    if any([abs(thdd) > (math.pi/2.0) for thdd in fp.dth]):
        return (None, -1)
    elif any([(1-kri*di) < 0 for kri, di in zip(fp.kr, fp.d)]):
        return (None, -2)
    elif any([v > MAX_SPEED for v in fp.vx]):  # Max speed check
        return (None,1)
    elif any([a > MAX_ACCEL for a in fp.ax]):  # Max accel check
        return (None,2)
    elif any([a < MAX_DCCEL for a in fp.ax]):  # Max accel check
        return (None,2)
    elif any([abs(c) > MAX_CURVATURE for c in fp.c]):  # Max curvature check
        return (None,3)
    elif not check_collision(fp, ob):
        return (None, 4)

    #print(" ti : "+str(Ti)+" tv : "+str(OPTION[1])+" -- finalcost : "+str(fp.cf))
    return (fp, 0)


def get_obstacle_prediction_cv(x0, y0, th0, v, w, dt, MAXT, obstacle_df, iters):
    x = [x0]
    y = [y0]
    th = [th0]
    time = np.arange(0+dt, MAXT+dt, dt)
    for t in time:
        th.append(th[-1] + w*dt)
        x.append(x[-1] + v*math.cos(th[-1])*dt)
        y.append(y[-1] + v*math.sin(th[-1])*dt)
    return x, y, th
    ##ground_truth
    #for _ in time:
    #    obs_df = obstacle_df[(obstacle_df["frame_num"] == iters)]
    #    obs_df = obs_df.iloc[0]
    #    x.append(obs_df["x"].item())
    #    y.append(obs_df["y"].item())
    #    th.append(obs_df["yaw"].item()*(np.pi/180.0))
    #    iters = iters + 1

    #return x, y, th

def deltaConv(x):
    t = np.zeros_like(x)
    t[1:,:] = np.diff(x,axis=0)
    return t

def get_obstacle_prediction_network(model, past_frames, loc_x, loc_y, ego_x, ego_y, ego_yaw, MAXT,
                                    x0, y0, th0, v, w, dt, obstacle_df, iters):
    # global to local frame conversion

    ego_yaw = np.radians(ego_yaw)
    rot_matrix = np.array([[np.cos(ego_yaw), np.sin(ego_yaw)], [-np.sin(ego_yaw), np.cos(ego_yaw)]])
    rot_matrix_inv = rot_matrix.T
    x, y, inp = [], [], []
    for i in range(len(loc_x)):
        diff = np.array([[loc_x[i] - ego_x], [loc_y[i] - ego_y]])
        local_coord = np.matmul(rot_matrix,diff)
        local_x = local_coord[0,0]
        local_y = local_coord[1,0]
        if (local_x <= -40) or (local_x >= 80) or (local_y <= -30) or (local_y >= 30):
            flag = 0
            return get_obstacle_prediction_cv(x0, y0, th0, v, w, dt, MAXT, obstacle_df, iters)
        x.append(local_x)
        y.append(local_y)

    inp.append(x)
    inp.append(y)
    inp = np.array(inp).T
    
    # If there is not any motion => Vehicle is stationary
    if np.mean((inp[0] - inp[past_frames-1])**2) < (past_frames/5.0):
        pass

    # Load Model
    future = int(MAXT*10) # Predict 20 frames into the future
    predicted = inp[:past_frames]

    # Future Prediction
    for start_index in range(future):
       cur_pred = model.predict(deltaConv(predicted[start_index:start_index+past_frames, :]).reshape(1, past_frames, 2))
       predicted = np.append(predicted, cur_pred + predicted[-1], axis=0)

    
    # Convert local to global frame
    ego_loc = np.array([[ego_x, ego_y]]).T
    nn_global = np.matmul(rot_matrix_inv, predicted.T) + ego_loc
    nn_global = nn_global[:,past_frames:]

    return list(nn_global[0,:]), list(nn_global[1,:]), []


def check_collision(fp, ob):
    
    for i in range(len(ob)):
        d = [((jx - ob[i]['x'][j])**2 + (jy - ob[i]['y'][j])**2)
             for j,(jx, jy) in enumerate(zip(fp.x, fp.y))]

        collision = any([di <= ROBOT_RADIUS**2 for di in d])

        if collision:
            return False

    return True


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, c_s_dd, ob, dl, dr, dsp, DTFPS,
                            OPTION, SETS, p, obstacle_df, iters, PRED_MODEL, ego_loc_list, hero_id, model, past_frames):
    
    MAX_SPEED = SETS['MAX_SPEED']
    TARGET_SPEED = SETS['TARGET_SPEED']
    D_T_S = SETS['D_T_S']
    N_S_SAMPLE = SETS['N_S_SAMPLE']

    # loop over obstacles
    for i in range(len(ob)):
        nn_flag = 0
        if (PRED_MODEL == "nn") and (iters > past_frames):
            # check ego_loc_list for matching id and 20 previous states
            for j in range(len(ego_loc_list)):
                if len(ego_loc_list[j])>past_frames and ego_loc_list[j][2][0] == ob[i]['id']:
                    loc_list = ego_loc_list[j]
                    nn_flag = 1
                    break
        if nn_flag == 1: 
            for j in range(len(ego_loc_list)):
                if ego_loc_list[j][2][0] == hero_id:
                    egox = ego_loc_list[j][-past_frames][1]
                    egoy = ego_loc_list[j][-past_frames][2]
                    egoyaw = ego_loc_list[j][-past_frames][4]
                    break
            # nn prediction part
            locx = [xt[1] for xt in loc_list[-past_frames:]]
            locy = [yt[2] for yt in loc_list[-past_frames:]]
            x0 = ob[i]['x0']
            y0 = ob[i]['y0']
            th0 = ob[i]['th0']
            v = ob[i]['v']
            w = ob[i]['w']
            if OPTION[0] == 'OVERTAKE':
                ob[i]['x'], ob[i]['y'], ob[i]['th'] = get_obstacle_prediction_network(model, past_frames, locx, locy, egox, egoy, egoyaw, MAXT,
                                                                                     x0, y0, th0, v, w, DTFPS, -100, -100 )
            elif OPTION[1] == 'STOP':
                ob[i]['x'], ob[i]['y'], ob[i]['th'] = get_obstacle_prediction_network(model, past_frames, locx, locy, egox, egoy, egoyaw, MAXT+5.0,
                                                                                     x0, y0, th0, v, w, DTFPS, -100, -100 )
        else:
            # cv prediction
            x0 = ob[i]['x0']
            y0 = ob[i]['y0']
            th0 = ob[i]['th0']
            v = ob[i]['v']
            w = ob[i]['w']
            if OPTION[0] == 'OVERTAKE':
                ob[i]['x'], ob[i]['y'], ob[i]['th'] = get_obstacle_prediction_cv(x0, \
                                                        y0, th0, v, w, DTFPS, MAXT, obstacle_df, iters)
            elif OPTION[0] == 'STOP':
                ob[i]['x'], ob[i]['y'], ob[i]['th'] = get_obstacle_prediction_cv(x0, \
                                                y0, th0, v, w, DTFPS, MAXT+5.0, obstacle_df, iters)   

    # finds lateral distiance limits to plan
    if OPTION[0] == 'OVERTAKE':
        ti = np.arange(MINT, MAXT, DT)
        tv = np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE-0.05, TARGET_SPEED , D_T_S)
    elif OPTION[0] == 'STOP':
        if OPTION[2] == 'VEHICLE':
            ti = np.arange(MINT, MAXT + 5.0, DT)
            temp_ssd = max((OPTION[1] - D_S_S * N_S_S_SAMPLE) + s0, s0)
            tv = np.array([OPTION[1] + s0]) # np.arange(temp_ssd, OPTION[1]+s0 , D_S_S) #
        elif OPTION[2] == 'LIGHT':

            ti = np.arange(MINT, MAXT + 5.0, DT)
            temp_ssd = max((OPTION[1] - D_S_S * (N_S_S_SAMPLE//2)) + s0, s0)
            tv = np.arange(temp_ssd, OPTION[1]+s0 , D_S_S) #np.array([OPTION[1] + s0])
    tti = []
    tdi = []
    tdv = []
    min_s = 1e20
    for m, val in enumerate(dsp):
        if (val-s0)*(val-s0) <= min_s:
            min_s = (val-s0)*(val-s0)
            start_idx = m
        else:
            break

    max_d = -1

    for i, vali in enumerate(ti):
        if OPTION[0] == 'OVERTAKE':
            for j, valj in enumerate(tv):
                temp_s = s0 + vali*(valj+c_speed)/(2.0)
                min_s = 1e20
                idx = -1
                for m, val in enumerate(dsp):
                    if (val-temp_s)*(val-temp_s) <= min_s:
                        min_s = (val-temp_s)*(val-temp_s)
                        idx = m
                    else:
                        break
                dleft = max(dl[start_idx:idx+1])
                dright = min(dr[start_idx:idx+1])
                di = np.arange(dleft, dright, D_ROAD_W)
                for k, valk in enumerate(di):
                    if abs(valk) >= max_d:
                        max_d = abs(valk)
                    tti.append(vali)
                    tdv.append(valj)
                    tdi.append(valk)
        elif OPTION[0] == 'STOP':
            for j, valj in enumerate(tv):
                temp_s = valj
                min_s = 1e20
                idx = -1
                for m, val in enumerate(dsp):
                    if (val-temp_s)*(val-temp_s) <= min_s:
                        min_s = (val-temp_s)*(val-temp_s)
                        idx = m
                    else:
                        break
                dleft = max(dl[start_idx:idx+1])
                dright = min(dr[start_idx:idx+1])
                di = np.arange(dleft, dright, D_ROAD_W)
                for k, valk in enumerate(di):
                    if abs(valk) >= max_d:
                        max_d = abs(valk)
                    tti.append(vali)
                    tdv.append(valj)
                    tdi.append(valk)
    if OPTION[0] == 'OVERTAKE':
        fplist = Parallel(n_jobs=NUM_CORES)(delayed(calc_frenet_dtv)(csp, ob, c_speed, c_d, \
            c_d_d, c_d_dd, c_s_dd, s0, tdi[i], tti[i], DTFPS, (OPTION[0], tdv[i]), SETS, p) for i in range(len(tdi)))
    elif OPTION[0] == 'STOP':
        fplist = Parallel(n_jobs=NUM_CORES)(delayed(calc_frenet_dtv)(csp, ob, c_speed, c_d, \
            c_d_d, c_d_dd, c_s_dd, s0, tdi[i], tti[i], DTFPS, (OPTION[0], tdv[i], OPTION[1]), SETS, p) for i in range(len(tdi)))


    # fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0, dl, dr, dsp)
    # fplist = calc_global_paths(fplist, csp)
    # fplist = check_paths(fplist, ob)

    counters = [0, 0, 0, 0, 0, 0]
    tcount = 0
    for fp in fplist:
        if fp[0] == None:
            if fp[1] == -1:
                counters[0] += 1
            elif fp[1] == -2:
                counters[1] += 1
            elif fp[1] == 1:
                counters[2] += 1
            elif fp[1] == 2:
                counters[3] += 1
            elif fp[1] == 3:
                counters[4] += 1
            elif fp[1] == 4:
                counters[5] += 1
        tcount = tcount + 1
        
    if tcount == counters[0]:
        print('first condition fail (pi/2)')
    elif tcount == counters[1]:
        print('second condition fail (0)')
    elif tcount == counters[2]:
        print('speed fail')
    elif tcount == counters[3]:
        print('accel fail')
    elif tcount == counters[4]:
        print('curvature fail')
    elif tcount == counters[5]:
        print('obstacle fail')
    else:
        print('path selected', counters)
    print(sum(counters), tcount)

    # find minimum cost path
    mincost = float("inf")
    bestpath = None
    selected_paths = []
    i = 1
    for fp in fplist:
        if fp[0] == None:
            continue
        elif mincost >= fp[0].cf:
            mincost = fp[0].cf
            bestpath = fp[0]
        selected_paths.append(fp[0])
        i = i+1

    allowed_cut = False
    if max_d >= 2.5:
        allowed_cut = True

    ob_cv = []
    for i in range(len(ob)):
        ob_cv.append({})
        # cv prediction
        x0 = ob[i]['x0']
        y0 = ob[i]['y0']
        th0 = ob[i]['th0']
        v = ob[i]['v']
        w = ob[i]['w']
        if OPTION[0] == 'OVERTAKE':
            ob_cv[i]['x'], ob_cv[i]['y'], ob_cv[i]['th'] = get_obstacle_prediction_cv(x0, \
                                                    y0, th0, v, w, DTFPS, MAXT, obstacle_df, iters)
        elif OPTION[0] == 'STOP':
            ob_cv[i]['x'], ob_cv[i]['y'], ob_cv[i]['th'] = get_obstacle_prediction_cv(x0, \
                                            y0, th0, v, w, DTFPS, MAXT+5.0, obstacle_df, iters)

    return bestpath, selected_paths, allowed_cut, ob, ob_cv


def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp
