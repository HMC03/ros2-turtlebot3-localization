import numpy as np

def sample_odometry_motion(x_prev, y_prev, th_prev,
                          x_new, y_new, th_new,
                          alpha=[0.05, 0.005, 0.05, 0.005]):
    """
    Probabilistic Robotics Table 5.4 — Odometry Motion Model
    Returns sampled (x, y, th)
    """
    # Compute odometry deltas
    dx = x_new - x_prev
    dy = y_new - y_prev
    trans = np.hypot(dx, dy)

    rot1 = np.arctan2(dy, dx) - th_prev
    rot2 = th_new - th_prev - rot1

    # Wrap angles
    rot1 = np.arctan2(np.sin(rot1), np.cos(rot1))
    rot2 = np.arctan2(np.sin(rot2), np.cos(rot2))

    a1, a2, a3, a4 = alpha

    # Sample noisy motions
    rot1_hat = rot1 - np.random.normal(0, np.sqrt(a1 * rot1**2 + a2 * trans**2))
    trans_hat = trans - np.random.normal(0, np.sqrt(a3 * trans**2 + a4 * (rot1**2 + rot2**2)))
    rot2_hat = rot2 - np.random.normal(0, np.sqrt(a1 * rot2**2 + a2 * trans**2))

    # Apply motion
    x = x_prev + trans_hat * np.cos(th_prev + rot1_hat)
    y = y_prev + trans_hat * np.sin(th_prev + rot1_hat)
    th = th_prev + rot1_hat + rot2_hat
    th = np.arctan2(np.sin(th), np.cos(th))

    return x, y, th