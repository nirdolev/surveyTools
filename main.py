

import MGA2Grid
import numpy as np
import math as mt

deg2rad=mt.pi/180


#read input
grid_points_file=r'XYZ 211012 L1 SCAN TARGETS RK PG.txt'
proj_cp_points=np.loadtxt(grid_points_file)
proj_cp_points = proj_cp_points[np.argsort(proj_cp_points[:, 0])]
proj_cp_points=proj_cp_points[:,1:3]

mga_coords_file=r'XYZ 211012 L1 SCAN TARGETS RK.txt'
mga_cp_points=np.loadtxt(mga_coords_file,dtype=float,comments='#',delimiter=',')
mga_cp_points = mga_cp_points[np.argsort(mga_cp_points[:, 0])]
mga_cp_points=mga_cp_points[:,1:3]

#choose subset to calc transformation
num_of_points=mga_cp_points.shape[0]
ind=np.array(range(0,num_of_points))
ind_cp=np.array([3,9,13,21,1])
ind_ck=np.setdiff1d(ind, ind_cp)

#calc transformation
[grid_points_trans,trans_mat]=MGA2Grid.MGA2Grid(mga_cp_points[ind_cp,:],proj_cp_points[ind_cp,:],mga_cp_points[ind_ck,:])
print(grid_points_trans-proj_cp_points[ind_ck,:])
rot_ang_deg=mt.acos(trans_mat[0,0])/deg2rad
print(rot_ang_deg)

#transformation params (as in file)
center_point=np.linalg.inv(trans_mat)*np.matrix([[0,0,1]]).transpose()
print(center_point[0:2,:].transpose())
