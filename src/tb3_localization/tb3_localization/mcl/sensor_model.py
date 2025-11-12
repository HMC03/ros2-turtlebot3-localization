import numpy as np

class LikelihoodFieldModel:
    def __init__(self, map_loader, 
                 z_hit=0.95, 
                 z_rand=0.05, 
                 sigma_hit=0.15,   # meters
                 max_beam_range=6.0):
        
        self.map = map_loader
        self.z_hit = z_hit
        self.z_rand = z_rand
        self.sigma_hit = sigma_hit
        self.max_beam_range = max_beam_range

    def beam_range_finder_model(self, z_t, x_t, map=None, valid_indices=None):
        """
        z_t: list of ranges [r1, r2, ..., rn]
        x_t: (x, y, theta)
        Returns: log-likelihood of entire scan
        """
        if map is None:
            map = self.map
            
        x, y, theta = x_t
        log_prob = 0.0
        n_beams = len(z_t)

        # Use provided valid indices or subsample
        if valid_indices is not None:
            indices = valid_indices
        else:
            indices = range(0, n_beams, 4)
        
        # Subsample
        for i0 in indices:
            z = z_t[i0]
            if z >= self.max_beam_range or z <= 0.1 or np.isnan(z):
                continue
                
            angle = theta + (i0 - n_beams//2) * (np.pi / 180.0)
            x_z = x + z * np.cos(angle)
            y_z = y + z * np.sin(angle)
            
            dist = map.get_distance(x_z, y_z)
            p_hit = np.exp(-0.5 * (dist**2) / (self.sigma_hit**2))
            p_rand = 1.0 / self.max_beam_range
            p = self.z_hit * p_hit + self.z_rand * p_rand
            log_prob += np.log(p + 1e-10)
            
        return log_prob