import numpy as np
import math

from tb3_localization.mcl.motion_model import sample_odometry_motion

class ParticleFilter:
    def __init__(self, 
                map_loader,
                sensor_model,
                N=800,
                alpha_slow=0.001,
                alpha_fast=0.1,
                w_thresh=0.5,
                seed=42):
        
        np.random.seed(seed)
        self.map = map_loader
        self.sensor = sensor_model
        self.N = N
        self.alpha_slow = alpha_slow
        self.alpha_fast = alpha_fast
        self.w_thresh = w_thresh
        
        self.particles = np.zeros((N, 4))  # x, y, theta, weight
        self.w_slow = self.w_fast = 0.0
        
        # Initialize with yaw diversity over free space
        self._initialize_particles()

    def _initialize_particles(self):
        for i in range(self.N):
            x, y, _ = self.map.sample_free_pose()
            theta = np.random.uniform(-np.pi, np.pi)
            self.particles[i] = [x, y, theta, 1.0 / self.N]
        print(f"[PF] Initialized {self.N} particles with yaw diversity")

    def motion_update(self, odom_prev, odom_curr):
        x_p, y_p, th_p = odom_prev
        x_c, y_c, th_c = odom_curr
        
        for i in range(self.N):
            x, y, th = self.particles[i, :3]
            x_new, y_new, th_new = sample_odometry_motion(x, y, th, x_c, y_c, th_c)
            
            # Boundary check + validity
            if self.map.is_valid_pose(x_new, y_new, margin_cells=3):
                self.particles[i, :3] = [x_new, y_new, th_new]
            else:
                # Rejection: respawn in free space
                x_f, y_f, th_f = self.map.sample_free_pose()
                self.particles[i, :3] = [x_f, y_f, th_f]

    def measurement_update(self, z_t):
        # Select valid beams
        valid_indices = [i for i in range(0, len(z_t), 5) 
                        if 0.1 < z_t[i] < 5.5 and not math.isnan(z_t[i])]

        if len(valid_indices) < 10:
            print("[PF] Bad scan — skipping update")
            return

        log_weights = np.zeros(self.N)
        for i in range(self.N):
            x, y, th = self.particles[i, :3]
            log_weights[i] = self.sensor.beam_range_finder_model(
                z_t, (x, y, th), valid_indices=valid_indices
            )
        
        # Importance weights
        weights = np.exp(log_weights - np.max(log_weights))
        weights += 1e-300
        weights /= weights.sum()
        self.particles[:, 3] = weights

        # Adaptive KLD-style trigger
        w_avg = np.mean(weights)
        self.w_slow += self.alpha_slow * (w_avg - self.w_slow)
        self.w_fast += self.alpha_fast * (w_avg - self.w_fast)

        Neff = 1.0 / np.sum(weights**2)
        if Neff < self.N * self.w_thresh:
            self._low_variance_resample()
            self.w_slow = self.w_fast = 0.0
            print(f"[PF] RESAMPLED — Neff = {Neff:.1f}")

    def _low_variance_resample(self):
        new_particles = []
        r = np.random.uniform(0, 1.0 / self.N)
        c = self.particles[0, 3]
        j = 0
        for m in range(self.N):
            U = r + m * (1.0 / self.N)
            while U > c:
                j += 1
                if j >= self.N:
                    j = 0
                c += self.particles[j, 3]
            new_particles.append(self.particles[j].copy())
        self.particles = np.array(new_particles)
        self.particles[:, 3] = 1.0 / self.N

    def get_best_pose(self):
        best_idx = np.argmax(self.particles[:, 3])
        x, y, th = self.particles[best_idx, :3]
        return x, y, th

    def get_pose_array(self):
        return self.particles.copy()