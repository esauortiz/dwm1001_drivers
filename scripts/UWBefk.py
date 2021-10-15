#
# author:       Alberto Ortiz (University of the Balearic Islands)
# project:      BUGWRIGHT2
# date:         October 2021
# description:  EKF filters for UWB data processing and position estimation
#

import numpy as np

class UWBfilter3D(): # constant velocity model, no heading in the state
    
    def __init__(self, ftype, x0, dt, std_acc, std_rng, landmarks):
        self.ftype = ftype         # filter type: EKF, IEKF, AIEKF
        self.dt = dt               # sampling period
        self.std_acc = std_acc     # standard deviation associated to acceleration noise
        self.std_rng = std_rng     # standard deviation range measurements
        self.landmarks = landmarks # landmarks positions, assumed numl x 3
        self.numl = landmarks.shape[0]  # number of available landmarks
        
        self.F = np.array([[1, 0, 0, dt, 0, 0],
                           [0, 1, 0, 0, dt, 0],
                           [0, 0, 1, 0, 0, dt],
                           [0, 0, 0, 1, 0, 0 ],
                           [0, 0, 0, 0, 1, 0 ],
                           [0, 0, 0, 0, 0, 1 ]])
        self.Fjac = np.array([[1, 0, 0, 0,  0, 0],
                              [0, 1, 0, 0,  0, 0],
                              [0, 0, 1, 0,  0, 0],
                              [0, 0, 0, dt, 0, 0],
                              [0, 0, 0, 0, dt, 0],
                              [0, 0, 0, 0, 0, dt]])
        self.Q = np.array([[(dt**4)/4, 0, 0, (dt**3)/2, 0, 0],
                           [0, (dt**4)/4, 0, 0, (dt**3)/2, 0],
                           [0, 0, (dt**4)/4, 0, 0, (dt**3)/2],
                           [(dt**3)/2, 0, 0, dt**2, 0, 0],
                           [0, (dt**3)/2, 0, 0, dt**2, 0],
                           [0, 0, (dt**3)/2, 0, 0, dt**2]]) * std_acc
        
        self.x = x0
        self.P = np.identity(6).astype(float)
        
        self.xk_1 = np.zeros((6,1)).astype(float)
        self.Pk_1 = np.zeros(6).astype(float)
        
    def predict(self):
        self.xk_1 = np.matmul(self.F, self.x)
        self.Pk_1 = np.matmul(np.matmul(self.Fjac, self.P), self.Fjac.T) + self.Q
    
    def updateEKF(self, ranges):
        hk = []
        Hk = []
        yk = []
        num_avail = 0
        for i in range(self.numl):
            if ranges[i] >= 0: # < 0 => n.a.
                xdif = self.xk_1[0:3] - self.landmarks[i,:]
                rangep = np.linalg.norm(xdif)
                hk.append( rangep )
                Hki = np.zeros((1,6)).flatten()
                Hki[0:3] = xdif / (rangep + 1e-8)
                Hk.append( Hki )
                yk.append( ranges[i] - rangep )
                num_avail = num_avail + 1
        hk = np.asarray(hk)
        Hk = np.asarray(Hk)
        yk = np.asarray(yk)
        Rk = self.std_rng * np.identity(num_avail) 
        Sk = np.matmul(np.matmul(Hk, self.Pk_1), Hk.T) + Rk
        Kk = np.matmul(np.matmul(self.Pk_1, Hk.T), np.linalg.inv(Sk))
        self.x = self.xk_1 + np.matmul(Kk, yk)
        self.P = np.matmul((np.identity(6) - np.matmul(Kk, Hk)), self.Pk_1)
        return
        
    def updateIEKF(self, ranges, niter): 
        xk = self.xk_1
        Pk = self.Pk_1
        for k in range(niter):
            hk = []
            Hk = []
            yk = []
            num_avail = 0
            for i in range(self.numl):
                if ranges[i] >= 0: # < 0 => n.a.
                    xdif = xk[0:3] - self.landmarks[i,:]
                    rangep = np.linalg.norm(xdif)
                    hk.append( rangep )
                    Hki = np.zeros((1,6)).flatten()
                    Hki[0:3] = xdif / (rangep + 1e-8)
                    Hk.append( Hki )
                    yk.append( ranges[i] - rangep )
                    num_avail = num_avail + 1
            hk = np.asarray(hk)
            Hk = np.asarray(Hk)
            yk = np.asarray(yk)
            Rk = self.std_rng * np.identity(num_avail) 
            Sk = np.matmul(np.matmul(Hk, self.Pk_1), Hk.T) + Rk
            Kk = np.matmul(np.matmul(self.Pk_1, Hk.T), np.linalg.inv(Sk))
            xk = self.xk_1 + np.matmul(Kk, (yk - hk - np.matmul(Hk, (self.xk_1 - xk))))
        Pk = (np.identity(6) - np.matmul(np.matmul(Kk, Hk)), self.Pk_1)
        self.x = xk
        self.P = Pk
        return
    
    def updateAIEKF(self, ranges, niter):
        return
    
    def update(self, ranges, niter=None):
        if self.ftype == 'EKF':
            self.updateEKF(ranges)
        elif self.ftype == 'IEKF':
            self.updateIEKF(ranges, niter)
        else : #  ftype == 'AIEKF'
            self.updateAIEKF(ranges, niter)                
        return    