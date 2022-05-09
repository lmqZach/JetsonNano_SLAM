#!/usr/bin/env python

import rospy
import cv2
import apriltag
from std_msgs.msg import Float32MultiArray
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 
import numpy as np

ctrl_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=2)


def predict(mu, cov, u, Qi):
    (v, w) = u
    w += 1e-6  # to avoid division by 0
    x, y, theta = mu[0][0], mu[1][0], mu[2][0]  # robot state
    motion = np.array([
        [-(v / w) * np.sin(theta) + (v / w) * np.sin(theta + w)],
        [(v / w) * np.cos(theta) - (v / w) * np.cos(theta + w)],
        [w]
    ])
    F = np.append(np.eye(3), np.zeros((3, N - 3)), axis=1)

    # Predict new state
    mu_bar = mu + F.T.dot(motion)
    mu_bar[2][0] %= 2*np.pi  # round to 0~2pi

    # Jacobian
    J = np.array([
        [0, 0, -(v / w) * np.cos(theta) + (v / w) * np.cos(theta + w)],
        [0, 0, -(v / w) * np.sin(theta) + (v / w) * np.sin(theta + w)],
        [0, 0, 1]
    ])
    G = np.eye(N) + F.T.dot(J).dot(F)

    # Predict new covariance
    cov_bar = G.dot(cov).dot(G.T) + F.T.dot(Qi).dot(F)
    return mu_bar, cov_bar


def update(mu, cov, obs, Ri):
    # get the estimated robot state
    x_, y_, theta_ = mu[0][0], mu[1][0], mu[2][0]

    for [w_cor, r_cor, j] in obs:
        i = 3 + 2 * j  # i = the index of the landmark in the state vector

        # Initialize landmark if it has not been observed before
        if cov[i][i] >= inf and cov[i + 1][i + 1] >= inf:
            # assign directly from measurement
            mu[i: i + 2] = w_cor

        # expected observation
        delta = np.array([mu[i][0] - x_, mu[i + 1][0] - y_])
        q = delta.T.dot(delta)
        sq = np.sqrt(q)

        z_hat = np.array([
            [sq],
            [np.arctan2(delta[1], delta[0]) - theta_]
        ])

        # calculate the jacobian for z_hat
        J = 1 / q * np.array([
            [-sq * delta[0], -sq * delta[1], 0, sq * delta[0], sq * delta[1]],
            [delta[1],       -delta[0],     -q,     -delta[1],     -delta[0]]
        ])

        # map the expected observation back to high dimensional space (state space)
        F = np.zeros((5, N))
        F[:3, :3] = np.eye(3)
        F[3, i] = 1
        F[4, i + 1] = 1
        H = J.dot(F)

        # Calculate the Kalman gain
        K = cov.dot(H.T).dot(np.linalg.inv(H.dot(cov).dot(H.T) + Ri))

        # difference between expected observation and actual measurements
        z_diff = np.array([[np.sqrt(np.sum(r_cor**2))], [np.arctan2(r_cor[1][0], r_cor[0][0])]]) - z_hat
        z_diff[1] = (z_diff[1] + np.pi) % (2 * np.pi) - np.pi

        # update state vector and covariance matrix
        mu = mu + K.dot(z_diff)
        cov = (np.eye(N) - K.dot(H)).dot(cov)
    return mu, cov


def pose_callback(msg):
    global mu, cov, counter, movement
    cmd_msg = Float32MultiArray()
    pose_mat = np.array(msg.pose.matrix)

    u = [(movement[0]+movement[1])/25, (movement[1]-movement[0])*0.6]
    mu_bar, cov_bar = predict(mu, cov, u, Qi)
    # data association
    x_, y_, theta_ = mu_bar[0][0], mu_bar[1][0], mu_bar[2][0]
    measure = []
    for i in range(len(pose_mat) // 3):
        feature = pose_mat[i * 3: i * 3 + 3]
        tag_id = int(feature[0])
        tag_r_cor = feature[1:].reshape(2, 1)  # relative coordinate
        tag_tm = np.array([[np.cos(theta_), -np.sin(theta_)],
                           [np.sin(theta_), np.cos(theta_)]])
        r_trans = np.matmul(tag_tm, tag_r_cor)
        # transform to world coordinate
        tag_w_cor = r_trans + np.array([[x_, y_]]).T
        assigned = False
        assigned_pos = -1
        min_dist = 10
        for tag_pos in TagDict[tag_id]:
            tag_w_cor_est = mu_bar[(3 + tag_pos * 2):(3 + tag_pos * 2 + 2)]
            # calculate error
            dist = np.sqrt(np.sum((tag_w_cor_est - tag_w_cor) ** 2))
            if dist < min_dist:
                min_dist = dist
                # if assignment is full, we assign to the min
                if len(TagDict[tag_id]) == 6 or dist < 0.8:
                    assigned_pos = tag_pos
                    assigned = True
        if not assigned:
            # generate a new tag
            assigned_pos = len(TagDict[1]) + len(TagDict[2])
            TagDict[tag_id].append(assigned_pos)
        measure.append([tag_w_cor, tag_r_cor, assigned_pos])

    mu, cov = update(mu_bar, cov_bar, measure, Ri)

    print(mu, TagDict)
    # comment the following line for circle motion
    # counter = (counter + 1) % 240
    if 60 <= counter < 180:
        movement = [0.3, 0.2]
    else:
        movement = [0.2, 0.3]
    cmd_msg.data = [0.0] + movement
    ctrl_pub.publish(cmd_msg)


if __name__ == "__main__":
    inf = 1e6
    n_tag = 12
    mu = np.append(np.array([[0, 0, 0]]).T, np.zeros((2 * n_tag, 1)), axis=0)
    cov = inf * np.eye(2 * n_tag + 3)
    cov[:3, :3] = np.zeros((3, 3))
    Qi = np.array([[0.1, 0, 0],
                   [0, 0.1, 0],
                   [0, 0, 0.1]])
    Ri = np.array([[0.1, 0],
                   [0, 0.1]])
    TagDict = {1: [], 2: []}
    N = len(mu)
    counter = 0
    movement = [0., 0.]
    rospy.init_node('planner_node')
    rospy.Subscriber("/current_pose", Pose, pose_callback)
    rospy.spin()
