import numpy as np
import math as mt

''' 
normalize_coors
---------------
normalizes coordinates with translation and scaling for better numerical stability.

input:
1.cp_points - 2D coordinates matrix n*2 [e,n;...]

output:
1. cp_points_norm - 2D normalized coordinates matrix n*2 [e,n;...]
2. trans_mat - 3*3 matrix which normalizes the original coors to normalized coords, such as: cp_points_norm=trans_mat*cp_points
'''
def normalize_coors(cp_points):
    cp_points_norm=np.copy(cp_points)

    #reduce means
    means=np.mean(cp_points, axis=0)
    num_of_points=cp_points.shape[0]
    for counter in range(0,num_of_points):
        cp_points_norm[counter,0]-=means[0]
        cp_points_norm[counter,1]-=means[1]
    
    #scale so all elements will be in range of [0,1]
    cp_points_norm_abs=np.absolute(cp_points_norm)
    scale=1/np.amax(cp_points_norm_abs)
    cp_points_norm=cp_points_norm*scale

    trans_mat=np.matrix([[scale,0,-scale*means[0]],[0,scale,-scale*means[1]],[0,0,1]])

    return [cp_points_norm,trans_mat]


'''
MGA2Grid
--------
calculates transformation between MGA and projectgrid coordinates.

input:
1.mga_cp_points - MGA coordinates matrix n*2 [e,n;...]
2.proj_cp_points - grid coordinates matrix n*2 [e,n;...]
3.mga_points_2_trans - mga coordinates to transform matrix m*2 [e,n;...]

output:
1.grid_points_trans - transformed mga coordinates matrix m*2 [e,n;...]
2.trans_mat - 3*3 matrix which transforms mga coordinates to grid coordinates, such as: grid=trans_mat*mga

'''
def MGA2Grid(mga_cp_points,proj_cp_points,mga_points_2_trans):
    #normalize points
    [mga_cp_points_norm,trans_mat_mga]=normalize_coors(mga_cp_points)
    [proj_cp_points_norm,trans_mat_proj]=normalize_coors(proj_cp_points)


    num_of_points=proj_cp_points.shape[0]
    num_of_points_to_trans=mga_points_2_trans.shape[0]
    n=num_of_points*2 #number of measurements
    u=4 #number of variables
    
    A=np.zeros((n,u))
    L=np.zeros((n,1))
    for counter in range(0,num_of_points):
        A[2*counter,0]=mga_cp_points_norm[counter,0]
        A[2*counter,1]=-mga_cp_points_norm[counter,1]
        A[2*counter,2]=1
        L[2*counter,0]=proj_cp_points_norm[counter,0]
        
        A[2*counter+1,0]=mga_cp_points_norm[counter,1]
        A[2*counter+1,1]=mga_cp_points_norm[counter,0]
        A[2*counter+1,3]=1
        L[2*counter+1,0]=proj_cp_points_norm[counter,1]
        
    At=A.transpose()
    N=np.matmul(At,A)
    U=np.matmul(At,L)
    #var=np.matmul(np.linalg.inv(N),U)
    var=np.linalg.solve(N, U)
    trans_mat_norm=np.matrix([ [var[0,0], -var[1,0], var[2,0]] ,[var[1,0], var[0,0], var[3,0]] ,[0,0,1] ])
    #denormalize
    trans_mat=np.matmul(np.matmul(np.linalg.inv(trans_mat_proj),trans_mat_norm), trans_mat_mga)
    #project to grid system
    one_vec=np.ones((num_of_points_to_trans,1))
    mga_points_2_trans=np.concatenate((mga_points_2_trans, one_vec),axis=1)
    mga_points_2_trans=mga_points_2_trans.transpose()
    grid_points_trans=np.matmul(trans_mat,mga_points_2_trans)
    grid_points_trans=grid_points_trans.transpose()
    grid_points_trans=grid_points_trans[:,0:2]
    
    return [grid_points_trans,trans_mat]
