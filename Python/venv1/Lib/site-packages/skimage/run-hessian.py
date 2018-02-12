import numpy as np
from skimage import morphology

im = np.zeros((6, 6))
im[:, 3:] = 1

seeds = np.zeros(im.shape, dtype=int)
seeds[2, 0] = 1
seeds[2, 3] = 2

ws = morphology.watershed(im, seeds, compactness=0.01)
print(ws)
