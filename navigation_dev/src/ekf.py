import numpy as np


class EKF:
    INF = 1e6

    def __init__(self):
        pass


    def predict(self, mu, sigma, u, dt, Rt):
        """ The prediction step of EKF.
            The motion is a velocity based one.

            Parameters:
            -----------
            - mu:
                The estimation of last timestep, has shape (2*N + 3)

            - sigma:
                The state noise covariance matrix of last timestep

            - u:
                The control values, linear and angular velocity (v, w)

            - dt:
                Time difference between current and previous timestep

            - Rt:
                The covariance matrix process noise

            Returns:
            --------
            - mu_bar:
                Prediction of next state

            - sigma_bar:
                Prediction of the state noise
        """

        N = len(mu)     # get the dimension of the state
        (v, w) = u      # get the control values

        # the motion model of the robot:
        x, y, theta = mu[0], mu[1], mu[2] # robot state
        motion = np.array([
            [-(v / w) * np.sin(theta) + (v / w) * np.sin(theta + w * dt)],
            [(v / w) * np.cos(theta) + (v / w) * np.cos(theta + w * dt)],
            [w * dt]
        ])
        F = np.append(np.eye(3),np.zeros((3,n-3)),axis=1) # F translate motion to state space

        # prediciton of the new state
        mu_bar = mu + (F.T).dot(motion)

        # get G from the motion model's jacobian
        J = np.array([
            [0, 0, -(v / w) * np.cos(theta) + (v / w) * np.cos(theta + w * dt)],
            [0, 0, -(v / w) * np.sin(theta) + (v / w) * np.sin(theta + w * dt)],
            [0, 0, 1]
        ])
        G = np.eye(N) + (F.T).dot(J).dot(F)

        # predict the new noise covariance
        sigma_bar = G.dot(sigma).dot(G.T) + (F.T).dot(Rt).dot(F)

        # TODO: debug print
        print(f"Robot pose prediction: ({mu_bar[0]}, {}, {})")
        return mu_bar, sigma_bar


    def update(mu, sigma, observations, Qt):
        """ Assume that `j` in observation will be given
            `j` is the position the tag is in the state vector
            (state vector size will vary)
        """

        N = len(mu)

        # get the estimated position of the robot
        x_, y_, theta_ = mu[0][0], mu[1][0], mu[2][0]

        for [r, phi, j] in observations:

            i = 3 + 2 * j # i = the index of the landmark in the state vector

            # Initialize landmark if it has not been observed before
            # TODO: should this be moved to outside?
            if sigma[i][i] >= self.INF and sigma[i+1][i+1] >= self.INF:
                # assign directly from measurement
                mu[i][0] = x_ + r * np.cos(phi + theta_)    # x of tag
                mu[i+1][0] = y_ + r * np.cos(phi + theta_)  # y of tag
            
            # expected observation
            delta = np.array([mu[i][0] - x_, mu[i+1][0] - y_])
            q = delta.T.dot(delta)
            sq = np.sqrt(q)

            # in the range-bearing format
            z_hat = np.array([
                [sq],
                [np.arctan2(delta[1],delta[0]) - theta_]
            ])

            # calculate the jacobian for z_hat
            J = 1 / q * np.array([
                [-sq * delta[0], -sq * delta[1], 0, sq * delta[0], sq * delta[1]],
                [delta[1], -delta[0], -q, -delta[1], -delta[0]]                
            ])
            
            # map the expected observation back to high dimensional space (state space)
            F = np.array((5, N))
            F[:3, :3] = np.eye(3)
            F[3, i] = 1
            F[4, i+1] = 1
            H = J.dot(F)

            # Calculate the Kalman gain
            K = sigma.dot(H.T).dot(np.linalg.inv(H.dot(sigma).dot(H.T) + Qt))

            # difference between expected observation and actual measurements
            z_diff = np.array([[r], [phi]]) - z_hat

