from math import sqrt
from numpy import *
from numpy.random import randn
import math
import matplotlib.pyplot as plt
import plotter as bp
import random 
import ekf

def HJacobian_at(x):
    """ compute Jacobian of H matrix for state x """

    horiz_dist = x[0]
    altitude   = x[2]
    denom = sqrt(horiz_dist**2 + altitude**2)
    return array([[horiz_dist/denom, 0., altitude/denom]])

def hx(x):
    """ compute measurement for slant range that would correspond 
    to state x.
    """
    
    return (x[0]**2 + x[2]**2) ** 0.5


class RadarSim(object):
    """ Simulates the radar signal returns from an object flying 
    at a constant altityude and velocity in 1D. 
    """
    
    def __init__(self, dt, pos, vel, alt):
        self.pos = pos
        self.vel = vel
        self.alt = alt
        self.dt = dt
        self.pos_err = 0.
    def get_range(self):
        """ Returns slant range to the object. Call once for each
        new measurement at dt time from last call.
        """
        
        # add some process noise to the system
        self.vel = self.vel  + random.randint(-1,1)
        self.alt = self.alt + .1*randn()
        self.pos = self.pos + self.vel*self.dt
    
        # add measurement noise
        err = self.pos * 0.05*randn()
        self.pos_err = err
        slant_dist = math.sqrt(self.pos**2 + self.alt**2)
        
        return slant_dist + err


def plot_radar(xs, track, time):
    plt.figure()
    bp.plot_track(time, track[:, 0])
    bp.plot_filter(time, xs[:, 0])
    plt.legend(loc=4)
    plt.xlabel('time (sec)')
    plt.ylabel('position (m)')

    plt.figure()
    bp.plot_track(time, track[:, 1])
    bp.plot_filter(time, xs[:, 1])
    plt.legend(loc=4)
    plt.xlabel('time (sec)')
    plt.ylabel('velocity (m/s)')

    plt.figure()
    bp.plot_track(time, track[:, 2])
    bp.plot_filter(time, xs[:, 2])
    plt.ylabel('altitude (m)')
    plt.legend(loc=4)
    plt.xlabel('time (sec)')
    plt.ylim((900, 1600))
    plt.show()



dt = 0.005
rk = ekf.EKF(dim_x=3)
radar = RadarSim(dt, pos=0., vel=17., alt=1000.)

# make an imperfect starting guess
rk.x = array([radar.pos, radar.vel, radar.alt+1000])

F=  array([[0, 1, 0],
                       [0, 0, 0],
                       [0, 0, 0]])
rk.F = eye(3) + F*dt + F*dt**2/2 + F*dt**3/6 + F*dt**4/24 + F*dt**5/120 + F*dt**6/720

rk.R = radar.alt * 0.05 # 5% of distance
rk.Q = array([[0, 0, 0],
              [0, 1, 0],
              [0, 0, 1]]) * 0.001
rk.P *= 100
xs = []
track = []
for i in range(int(20/dt)):
    z = radar.get_range()
    track.append((radar.pos, radar.vel, radar.alt))
    #print("Ölçüm:",[radar.pos, radar.vel, radar.alt])


    err = abs(sum(array([0,0,0])-array(rk.x)))
    if abs(sum(array([radar.pos, radar.vel, radar.alt])-array(rk.x))) > err/4:
        xs.append(-rk.x)
        rk.update(array([z]), HJacobian_at, hx)
    else:
        rk.update(array([z]), HJacobian_at, hx)
        xs.append(rk.x)
    rk.predict()
    #print("Predict:",rk.x)


xs = asarray(xs)
track = asarray(track)
time = arange(0, len(xs)*dt, dt)
plot_radar(xs, track, time)