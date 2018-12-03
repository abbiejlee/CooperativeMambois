"""
  kalman_filter.py
  Abbie Lee | 16.30

  Implementation of a Kalman filter for position estimation using the position
  and velocity sensor readings.

  Source: https://arxiv.org/pdf/1204.0375.pdf
"""
import numpy as np

class KalmanFilter:
    def __init__(self, A, B, C, D, X0, U0):
        """
        Give matrices for an estimator of the form

        xhat' = Axhat + Bu + L(y - yhat)
        yhat = Cxhat + Du

        X0: initial state matrix
        U0: control input matrix

        type of all matrices is np.array
        """
        self.A = A
        self.B = B
        self.C = C
        self.D = D

        self.Xhat = X0
        self.U = U0

        # TODO(abbielee): check initialization of below matrices

        # Initialize covariance matrices
        self.P = self.init_p()                  # Error covariance
        self.Rw = eye(self.X.shape()[0])        # Process noise covariance
        self.Rv = eye(self.X.shape()[0])        # Measurement noise covariance

        # Initialize estimation gains
        self.L = self.update_L()

        # Measurement matrices
        self.Yhat = np.array([np.dot(self.A, self.X)+(np.dot(self.D, self.U))])


    def predict(X, P, A, Q, B, U):
        X = np.dot(A, X) + np.dot(B, U)
        P = np.dot(A, np.dot(P, A.T)) + Q
        return(X,P)

    def update(X, P, Y, H, R):
        IM = np.dot(H, X)
        IS = R + np.dot(H, np.dot(P, H.T))
        K = np.dot(P, np.dot(H.T, np.linalg.inv(IS)))
        X = X + np.dot(K, (Y-IM))
        P = P - np.dot(K, np.dot(IS, K.T))
        LH = gauss_pdf(Y, IM, IS)
        return (X,P,K,IM,IS,LH)

    def gauss_pdf(X, M, S):
        if M.shape()[1] == 1:
        DX = X - np.tile(M, X.shape()[1])
        E = 0.5 * np.sum(DX * (np.dot(np.linalg.inv(S), DX)), axis=0)
        E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))
        P = exp(-E)
        elif X.shape()[1] == 1:
        DX = tile(X, M.shape()[1])- M
        E = 0.5 * np.sum(DX * (np.dot(np.linalg.inv(S), DX)), axis=0)
        E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))
        P = exp(-E)
        else:
        DX = X-M
        E = 0.5 * np.dot(DX.T, np.dot(np.linalg.inv(S), DX))
        E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))
        P = exp(-E)
        return (P[0],E[0])
