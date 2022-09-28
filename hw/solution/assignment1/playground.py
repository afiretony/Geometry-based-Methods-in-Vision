# %% 
import numpy as np


# %%
a = np.array([3, 4, 1])
b = np.array([3,4,2])
np.cross(a, b)
# %%
H = [[3,0,0,],[0,2,0],[0,0,1]]
H = np.array(H)
H_inv = np.linalg.inv(H)
# %%
H_inv
# %%
C = [[1,0,0],[0,1,0],[0,0,0]]
C = np.array(C)
H_inv@C@H_inv


 # %%
1/9
# %%
