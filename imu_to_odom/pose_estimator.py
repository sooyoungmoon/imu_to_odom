import numpy as np
from scipy.spatial.transform import Rotation as R

class PoseEstimator:
    def __init__(self):
        self.orientation = R.from_quat([0, 0, 0, 1])
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.last_time = None

    def update(self, imu_data, current_time):
        if self.last_time is None:
            self.last_time = current_time
            return self.position, self.orientation.as_quat()

        dt = current_time - self.last_time
        self.last_time = current_time

        # Update orientation
        delta_orientation = R.from_euler('xyz', imu_data['angular_velocity'] * dt)
        self.orientation = self.orientation * delta_orientation

        # Update velocity
        acceleration = np.array(imu_data['linear_acceleration'])
        self.velocity += acceleration * dt

        # Update position
        self.position += self.velocity * dt

        return self.position, self.orientation.as_quat()

# Example usage
if __name__ == "__main__":
    imu_data = {
        'angular_velocity': np.array([0.01, 0.02, 0.03]),
        'linear_acceleration': np.array([0.1, 0.2, 0.3])
    }
    pose_estimator = PoseEstimator()
    current_time = 0.1  # Example timestamp
    position, orientation = pose_estimator.update(imu_data, current_time)
    print("Position:", position)
    print("Orientation:", orientation)