import numpy as np
import math as mt

from numpy.lib.shape_base import column_stack

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
LinMGA2Grid
--------
calculates linear transformation between MGA and projectgrid coordinates.

input:
1.mga_cp_points - MGA coordinates matrix n*2 [e,n;...]
2.proj_cp_points - grid coordinates matrix n*2 [e,n;...]
3.mga_points_2_trans - mga coordinates to transform matrix m*2 [e,n;...]

output:
1. ang_grid2mga_deg - adjusted value of rotation angle [degrees] from grid 2 mga. the angle is from north axis clockwise (azimuth)
2. mga_center_point - adjusted value of translation (grid origin in MGA) from grid to MGA system. 1*2 matrix [e,n]
3. resid - residuals 

'''
def LinMGA2Grid(mga_cp_points,proj_cp_points):
    #normalize points
    [mga_cp_points_norm,trans_mat_mga]=normalize_coors(mga_cp_points)
    [proj_cp_points_norm,trans_mat_proj]=normalize_coors(proj_cp_points)

    #adjustment params
    num_of_points=proj_cp_points.shape[0]
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

    #extract transformation params
    ang_grid2mga_deg=mt.atan(trans_mat[1,0]/trans_mat[0,0])/deg2rad
    scale=trans_mat[0,0]*trans_mat[0,0]+trans_mat[1,0]*trans_mat[1,0]
    rot_mat=trans_mat[0:2,0:2]/scale
    mga_center_point=np.matmul(-rot_mat.transpose(),trans_mat[0:2,2])
    
    return [ang_grid2mga_deg,mga_center_point.transpose()]


'''
NonLinMGA2Grid
---------------
calculates none linear transformation between MGA and projectgrid coordinates.

input:
1. mga_cp_points - MGA coordinates matrix n*2 [e,n;...]
2. grid_cp_points - grid coordinates matrix n*2 [e,n;...]
3. aprox_ang_grid2mga_deg - aproximate value of rotation angle [degrees] from grid 2 mga. the angle is from north axis clockwise (azimuth)
4. aprox_mga_center_point - aproximate value of translation (grid origin in MGA) from grid to MGA system. 1*2 matrix [e,n]

output:
1. ang_grid2mga_deg - adjusted value of rotation angle [degrees] from grid 2 mga. the angle is from north axis clockwise (azimuth)
2. mga_center_point - adjusted value of translation (grid origin in MGA) from grid to MGA system. 1*2 matrix [e,n]
3. converged - true if adjustment converged, else 0

'''
def NonLinMGA2Grid(mga_cp_points,grid_cp_points, aprox_ang_grid2mga_deg,aprox_mga_center_point):

    #adjustment params
    num_of_points=mga_cp_points.shape[0]
    n=2*num_of_points #number of measurements
    u=3 #number of variables
    conv_treshold=mt.pow(10,-10) #convergence threshold
    max_iter=15 #maximum iterations

    #build adjustment matrices
    aprox_ang_grid2mga_rad=deg2rad*aprox_ang_grid2mga_deg
    X0=np.matrix([-aprox_ang_grid2mga_rad,aprox_mga_center_point[0,0],aprox_mga_center_point[0,1]]).transpose() #aprox values vector
    Lb=np.reshape(mga_cp_points,(n,1)) #raw measurements vector
    A=np.zeros((n,u)) #partial derivitives matrix
    L0=np.zeros((n,1)) #aprox measurements vector
    
    converged=0
    iter=0
    difference=100

    while difference>conv_treshold and iter<max_iter:
        #calc adjustment matrices
        for counter in range(num_of_points):
            # renaming variables aprox values
            azi0=X0[0,0]
            te=X0[1,0]
            tn=X0[2,0]
            # grid coords (constants)
            e_grid=grid_cp_points[counter,0]
            n_grid=grid_cp_points[counter,1]

            cosa=mt.cos(azi0)
            sina=mt.sin(azi0)

            A[2*counter,0]=-e_grid*sina-n_grid*cosa
            A[2*counter,1]=1
            L0[2*counter,0]=e_grid*cosa-n_grid*sina+te

            A[2*counter+1,0]=e_grid*cosa-n_grid*sina
            A[2*counter+1,2]=1
            L0[2*counter+1,0]=e_grid*sina+n_grid*cosa+tn

        #solve
        L=Lb-L0
        At=A.transpose()
        N=np.matmul(At,A)
        U=np.matmul(At,L)
        X=np.linalg.solve(N,U)
        X0=X0+X

        #check convergence
        iter+=1
        difference=np.max(np.absolute(X))
    
    #check convergence
    if iter<max_iter:
        converged=1


    return [-X0[0,0]/deg2rad,X0[1:3,0].transpose(),converged]

'''
Project2Grid
------------
project MGA coordinates to a Project grid defined by translation and rotation relative to MGA

input:
1. rot_angle_deg - rotation angle from North axis clockwise (azimuth) of grid system relative to MGA.
2. MGA_base_point - The origin of the grid system in MGA coordinates ,1*2 matrix [e,n]  
3. mga_points_2_trans - mga coordinates to transform matrix m*2 [e,n;...]

output:
1.grid_points_trans - transformed mga coordinates matrix m*2 [e,n;...]

'''
def Project2Grid(rot_angle_deg,MGA_base_point,mga_points_2_trans):
    #setting homogenious coordinates
    num_of_points_to_trans=mga_points_2_trans.shape[0]
    one_vec=np.ones((num_of_points_to_trans,1))
    mga_points_2_trans=np.concatenate((mga_points_2_trans,one_vec),axis=1).transpose()
    ###building projection matrix###
    rot_mat=np.identity(3)
    trans_mat=np.identity(3)
    #2D rotation
    rot_angle_rad=rot_angle_deg*deg2rad
    cosa=mt.cos(rot_angle_rad)
    sina=mt.sin(rot_angle_rad)
    rot_mat[0,0]=cosa
    rot_mat[0,1]=-sina
    rot_mat[1,0]=sina
    rot_mat[1,1]=cosa
    #2D translation
    trans_mat[0,2]=-MGA_base_point[0,0]
    trans_mat[1,2]=-MGA_base_point[0,1]
    ###project###
    proj_mat=np.matmul(rot_mat,trans_mat)
    grid_points_trans=np.matmul(proj_mat,mga_points_2_trans)
    grid_points_trans=grid_points_trans.transpose()
    grid_points_trans=grid_points_trans[:,0:2]

    return grid_points_trans



'''================================================================================================================'''


'''testing functionality'''
if __name__ == "__main__":
    deg2rad=mt.pi/180

    ###test MGA2Grid###

    #read input
    grid_points_file=r'./Data/XYZ 211012 L1 SCAN TARGETS RK PG.txt'
    proj_cp_points=np.loadtxt(grid_points_file)
    proj_cp_points = proj_cp_points[np.argsort(proj_cp_points[:, 0])]
    proj_cp_points=proj_cp_points[:,1:3]

    mga_coords_file=r'./Data/XYZ 211012 L1 SCAN TARGETS RK.txt'
    mga_cp_points=np.loadtxt(mga_coords_file,dtype=float,comments='#',delimiter=',')
    mga_cp_points = mga_cp_points[np.argsort(mga_cp_points[:, 0])]
    mga_cp_points=mga_cp_points[:,1:3]

    #choose subset to calc transformation
    num_of_points=mga_cp_points.shape[0]
    ind=np.array(range(0,num_of_points))
    ind_cp=np.array([3,9,13,21,1])
    ind_ck=np.setdiff1d(ind, ind_cp)

    #calc transformation
    [ang_grid2mga_deg0,mga_center_point0]=LinMGA2Grid(mga_cp_points[ind_cp,:],proj_cp_points[ind_cp,:])

    print("adjusted angle[deg]: " + str(ang_grid2mga_deg0))
    print("adjusted center: " + str(mga_center_point0))

    ###test NonLinLS2DTrans###
    [ang_grid2mga_deg,mga_center_point,converged]=NonLinMGA2Grid(mga_cp_points[ind_cp,:],proj_cp_points[ind_cp,:], ang_grid2mga_deg0,mga_center_point0)
    converged_text="no"
    if converged:
        converged_text="yes"

    print("convergence: " + converged_text)
    print("adjusted angle[deg]: " + str(ang_grid2mga_deg))
    print("adjusted center: " + str(mga_center_point))

    ###test Project2Grid###
    grid_points_trans2=Project2Grid(ang_grid2mga_deg,mga_center_point,mga_cp_points[ind_ck,:])
    print("residuals: " + "\n" + str(grid_points_trans2-proj_cp_points[ind_ck,:]))