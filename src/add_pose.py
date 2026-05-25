
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate):
    # Add the odometry factor between X(4) and X(5)

    # Add the odometry factor between X(3) and X(4)
    graph.add(gtsam.BetweenFactorPose2(X(3), X(4), gtsam.Pose2(2.0, 0, np.pi/2), ODOMETRY_NOISE))

    # Derive X(4) initial estimate by composing X(3)'s estimate with the odometry
    if not initial_estimate.exists(X(4)):
        initial_estimate.insert(X(4), gtsam.Pose2(4.0 + math.sqrt(2), math.sqrt(2), np.pi / 2))



    return graph, initial_estimate