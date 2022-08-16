from dataclasses import dataclass
import numpy as np

@dataclass
class DifferentialDriveKinematics:
    """
    https://www.mathworks.com/help/robotics/ug/mobile-robot-kinematics-equations.html

    Differential Drive Kinematics

    (x,y): position [m]
    theta: heading [rad]
    phiL|R: wheel speed [m/s]
    r: wheel radius [r]
    d: track width, or distance between wheels [m]
    v: speed [m/s]
    w: vehicle heading angular velocity [rad/s]
    """
    r: float
    d: float

    def eqns(self, t, x, u):
        theta = x[2]

        phiL = u[0]
        phiR = u[1]

        v = self.r*(phiR + phiL)/2
        w = self.r/(2*self.d) * (phiR - phiL)

        ret = np.zeros(3)
        ret[0] = np.cos(theta)*v # dx
        ret[1] = np.sin(theta)*v # dy
        ret[2] = w               # dtheta

        return ret

@dataclass
class PointMass:

    def eqns(self, dt, x, u=None):
        """
        x = [pos, vel, quaternion]
        u = [accels, gyros]
        """
        if u is None:
            u = np.zeros(6)
        else:
            a = u[:3]
            w = u[3:6]

        q = Quaternion(*x[6:])

        dp = x[3:6]

        rot = np.array(q.to_rot())
        g = np.array([0,0,9.8])
        dv = rot @ a - g

        W = Quaternion(0,*w)
        dq = 0.5*q*W

        xx = np.array([0,0,0, 0,0,0, dq.w,dq.x,dq.y,dq.z])
        xx[:3] = dp
        xx[3:6] = dv

        return xx

# @dataclass
class Drone:
    def __init__(self, params=None):
        if params is None:
            params = {}
        #     raise Exception("No parameters given")
        # if not isinstance(params, dict):
        #     raise Exception("Parameters are not a dictionary")

        self.m = params.get('m', 1)
        self.gg = params['km'] / params['kf']
        self.J = params['J']
        self.kf = params['kf']
        self.L = params['l']
        # self.g = 9.81

        self.q = Quaternion()

    def eqns(self, t, x, u):
        """
        x = [p,v,w,q] => 13
        """
        gg = self.gg
        kf = self.kf
        L = self.L
        m = self.m

        p,q,r = x[6:9]

        n1,n2,n3,n4 = u

        # R converts body to inertial
        R = np.array(self.q.to_rot())
        # convert inertial gravity to body frame, hence R.T
        g = R.T @ np.array([0,0,9.81])

        Jx, Jy, Jz = self.J

        F1 = kf*n1**2
        F2 = kf*n2**2
        F3 = kf*n3**2
        F4 = kf*n4**2
        F = np.array([0,0,-F1-F2-F3-F4])

        MJ = np.array([
            L*((F2+F3)-(F1+F4))/Jx,
            L*((F1+F3)-(F2+F4))/Jy,
            0.1*(F1+F2-F3-F4)/Jz
        ])

        wJw = np.array([
            (Jy - Jz)/Jx*q*r,
            (Jz - Jx)/Jy*p*r,
            (Jx - Jy)/Jz*p*q,
        ])

        # ans = [p,v,w,q] => 13
        ans = np.zeros(13)

        v = x[3:6]
        w = x[6:9]

        ans[:3] = v  # dot pos
        ans[3:6] = F/m+g - np.cross(w,v) # dot vel
        ans[6:9] = MJ-wJw                         # dot w

        q = Quaternion(*x[9:])
        w = Quaternion(0,*w)
        ans[9:] = 0.5*q*w # dot q

        return ans