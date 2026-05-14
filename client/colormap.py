import numpy as np


def rainbow_colormap(values: np.ndarray) -> np.ndarray:
    """Vectorized rainbow colormap matching _getRainbowColor from msensor.

    Args:
        values: Normalized intensity array in [0, 1].

    Returns:
        (N, 3) uint8 RGB array.
    """
    values = np.clip(values, 0.0, 1.0)

    h = values * 5.0 + 1.0
    i = np.floor(h).astype(np.int32)
    f = h - i

    even_mask = (i & 1) == 0
    f = np.where(even_mask, 1.0 - f, f)
    n = 1.0 - f

    r = np.zeros_like(values)
    g = np.zeros_like(values)
    b = np.zeros_like(values)

    m = i <= 1
    r[m] = n[m]; b[m] = 1.0

    m = i == 2
    g[m] = n[m]; b[m] = 1.0

    m = i == 3
    g[m] = 1.0; b[m] = n[m]

    m = i == 4
    r[m] = n[m]; g[m] = 1.0

    m = i >= 5
    r[m] = 1.0; g[m] = n[m]

    colors = np.empty((len(values), 3), dtype=np.uint8)
    colors[:, 0] = (r * 255.0).clip(0, 255).astype(np.uint8)
    colors[:, 1] = (g * 255.0).clip(0, 255).astype(np.uint8)
    colors[:, 2] = (b * 255.0).clip(0, 255).astype(np.uint8)
    return colors
