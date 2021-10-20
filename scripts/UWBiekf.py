#
# author:       Alberto Ortiz (University of the Balearic Islands)
# project:      BUGWRIGHT2
# date:         October 2021
# description:  EKF, IEKF & IEKF-L filters for UWB data processing and position estimation
#

import numpy as np

class UWB3D_iekf(): # constant velocity model, no heading in the state
    
    def __init__(self, ftype, x0, dt, std_acc, std_rng, landmarks):
        self.ftype = ftype         # filter type: EKF, IEKF, IEKFL (for IEKF-L)
        self.dt = dt               # sampling period
        self.std_acc = std_acc     # standard deviation associated to acceleration noise
        self.std_rng = std_rng     # standard deviation range measurements
        self.landmarks = landmarks # landmarks positions, assumed numl x 3
        self.numl = landmarks.shape[0]  # number of available landmarks
        self.prev_ranges = None

        if dt is not None:
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
                               [0, 0, (dt**3)/2, 0, 0, dt**2]]) * (self.std_acc ** 2)
        
        self.x = x0
        self.P = np.identity(6).astype(float)
        
        self.xk_1 = np.zeros((6,1)).astype(float)
        self.Pk_1 = np.zeros(6).astype(float)
        
    def predict(self, dt=None):
        if self.dt is None:
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
                               [0, 0, (dt**3)/2, 0, 0, dt**2]]) * (self.std_acc ** 2)
    
        # self.xk_1 = self.F @ self.x
        # self.Pk_1 = (self.Fjac @ self.P) @ self.Fjac.T + self.Q
        self.xk_1 = np.matmul(self.F , self.x) 
        self.Pk_1 = np.matmul(np.matmul(self.Fjac , self.P) , self.Fjac.T) + self.Q
        
    def updateEKF(self, ranges):
        ranges = np.array(ranges)
        if self.prev_ranges is None:
            ranges_condition = np.ones(ranges.shape, dtype=bool)
        else:
            delta_ranges = np.abs(self.prev_ranges - ranges)
            ranges_condition = delta_ranges < 2 * self.std_rng

        Hk = []
        vk = []
        num_avail = 0
        for i in range(self.numl):
            if ranges[i] >= 0.0 and ranges_condition[i]: # < 0 => n.a.
                xdif = self.xk_1[0:3] - self.landmarks[i,:]
                rangep = np.linalg.norm(xdif) # h_i(X_k)
                Hki = np.zeros((1,6)).flatten()
                Hki[0:3] = xdif / (rangep + 1e-8)
                Hk.append( Hki )
                vk.append( ranges[i] - rangep ) # y_i,k - h_i(X_k)
                num_avail = num_avail + 1
        if num_avail > 0:
            Hk = np.asarray(Hk)
            vk = np.asarray(vk)
            Rk = (self.std_rng ** 2) * np.identity(num_avail) 
            # Sk = (Hk @ self.Pk_1) @ Hk.T + Rk
            # Kk = (self.Pk_1 @ Hk.T) @ np.linalg.inv(Sk)
            # self.x = self.xk_1 + Kk @ vk
            # self.P = (np.identity(6) - Kk @ Hk) @ self.Pk_1
            Sk = np.matmul(np.matmul(Hk , self.Pk_1) , Hk.T) + Rk
            Kk = np.matmul(np.matmul(self.Pk_1 , Hk.T) , np.linalg.inv(Sk))
            self.x = self.xk_1 + np.matmul(Kk , vk)
            self.P = np.matmul((np.identity(6) - np.matmul(Kk , Hk)) , self.Pk_1)
        else: 
            self.x = self.xk_1
            self.P = self.Pk_1

        self.prev_ranges = ranges
        return 0 # num_iters_done
        
    def updateIEKF(self, ranges, niter): 
        num_iters_done = 0
        xk = self.xk_1
        Pk = self.Pk_1

        num_avail = 0
        for i in range(self.numl):
            if ranges[i] >= 0.0: # < 0 => n.a.
                num_avail = num_avail + 1

        if num_avail > 0:
            Rk = (self.std_rng ** 2) * np.identity(num_avail) 
            for j in range(niter):
                num_iters_done = j
                Hk = []
                vk = []
                for i in range(self.numl):
                    if ranges[i] >= 0.0: # < 0 => n.a.
                        xdif = xk[0:3] - self.landmarks[i,:]
                        rangep = np.linalg.norm(xdif) # h_i(X_k)
                        Hki = np.zeros((1,6)).flatten()
                        Hki[0:3] = xdif / (rangep + 1e-8)
                        Hk.append( Hki )
                        vk.append( ranges[i] - rangep ) # y_i,k - h_i(X_k)
                Hk = np.asarray(Hk)
                vk = np.asarray(vk)
                Rk = (self.std_rng ** 2) * np.identity(num_avail) 
                # Sk = (Hk @ self.Pk_1) @ Hk.T + Rk
                # Kk = (self.Pk_1 @ Hk.T) @ np.linalg.inv(Sk)
                # xk = self.xk_1 + Kk @ (vk - Hk @ (self.xk_1 - xk))
                Sk = np.matmul(np.matmul(Hk , self.Pk_1) , Hk.T) + Rk
                Kk = np.matmul(np.matmul(self.Pk_1 , Hk.T) , np.linalg.inv(Sk))
                xk_ = xk.copy()
                xk = self.xk_1 + np.matmul(Kk , (vk - np.matmul(Hk , (self.xk_1 - xk))))
                if np.linalg.norm(xk - xk_) < 1e-6: # convergence
                    break
            # Pk = (np.identity(6) - Kk @ Hk) @ self.Pk_1
            Pk = np.matmul((np.identity(6) - np.matmul(Kk , Hk)) , self.Pk_1)

        self.x = xk
        self.P = Pk
        return num_iters_done
    
    def updateIEKFL(self, ranges, niter): 
        num_iters_done = 0
        xk = self.xk_1
        Pk = self.Pk_1

        num_avail = 0
        for i in range(self.numl):
            if ranges[i] >= 0.0: # < 0 => n.a.
                num_avail = num_avail + 1

        if num_avail > 0:
            Rk = (self.std_rng ** 2) * np.identity(num_avail) 
            iRk = (1.0 / (self.std_rng ** 2)) * np.identity(num_avail) # np.linalg.inv(Rk)
            iPk_1 = np.linalg.inv(self.Pk_1)
            for j in range(niter):
                num_iters_done = j
                Hk = []
                vk = []
                for i in range(self.numl):
                    if ranges[i] >= 0.0: # < 0 => n.a.
                        xdif = xk[0:3] - self.landmarks[i,:]
                        rangep = np.linalg.norm(xdif) # h_i(X_k)
                        Hki = np.zeros((1,6)).flatten()
                        Hki[0:3] = xdif / (rangep + 1e-8)
                        Hk.append( Hki )
                        vk.append( ranges[i] - rangep ) # y_i,k - h_i(X_k)
                Hk = np.asarray(Hk)
                vk = np.asarray(vk)
                # Sk = (Hk @ self.Pk_1) @ Hk.T + Rk
                # Kk = (self.Pk_1 @ Hk.T) @ np.linalg.inv(Sk)
                Sk = np.matmul(np.matmul(Hk , self.Pk_1) , Hk.T) + Rk
                Kk = np.matmul(np.matmul(self.Pk_1 , Hk.T) , np.linalg.inv(Sk))
                deltax = self.xk_1 - xk
                # deltak = deltax + Kk @ (vk - Hk @ deltax)
                deltak = deltax + np.matmul(Kk , (vk - np.matmul(Hk , deltax)))
                minVk, mina = float('inf'), 0.0
                for a in np.arange(0.1,1.1,0.1): # a = 0.5
                    xka = xk + a * deltak
                    deltaxa = self.xk_1 - xka
                    vk = []
                    for i in range(self.numl):
                        if ranges[i] >= 0.0: # < 0 => n.a.
                            xdif = xka[0:3] - self.landmarks[i,:]
                            rangep = np.linalg.norm(xdif) # h_i(X_k)
                            vk.append( ranges[i] - rangep ) # y_i,k - h_i(X_k)
                    vk = np.asarray(vk)
                    # Vk = (vk.T @ iRk) @ vk + (deltaxa.T @ iPk_1) @ deltaxa
                    Vk = np.matmul(np.matmul(vk.T , iRk) , vk) + np.matmul(np.matmul(deltaxa.T , iPk_1) , deltaxa)
                    if Vk < minVk:
                        minVk, mina = Vk, a
                xk_ = xk.copy()
                xk = xk + mina * deltak
                if np.linalg.norm(xk - xk_) < 1e-6: # convergence
                    break
            # Pk = (np.identity(6) - Kk @ Hk) @ self.Pk_1
            Pk = np.matmul((np.identity(6) - np.matmul(Kk , Hk)) , self.Pk_1)
            
        self.x = xk
        self.P = Pk
        return num_iters_done
    
    def update(self, ranges, niter=None):
        if self.ftype == 'EKF':
            return self.updateEKF(ranges)
        elif self.ftype == 'IEKF':
            return self.updateIEKF(ranges, niter)
        elif self.ftype == 'IEKFL':
            return self.updateIEKFL(ranges, niter)                
        else: # not available
            print(self.ftype + ' is not available')
        return 0    
        
    def predict_and_update(self, ranges, dt=None, niter=None):
        self.predict(dt)
        num_iters_done = self.update(ranges, niter = niter)
        return num_iters_done
        