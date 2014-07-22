import numpy as np
import pickle
import scipy.io

data = pickle.load(open('right_arm_poses.p', 'rb'))
array = np.zeros((48,3))
for i in range(48):
  pos = data[str(i)][0].pose.position
  array[i][0] = pos.x
  array[i][1] = pos.y
  array[i][2] = pos.z


scipy.io.savemat('data.mat', mdict={'array':array})

  
