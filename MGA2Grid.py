import textwrap
import numpy as np
import math as mt

from numpy.lib.shape_base import column_stack


def normalize_coors(cp_points, use_scale=True):
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
    cp_points_norm=np.copy(cp_points)

    #reduce means
    means=np.mean(cp_points, axis=0)
    num_of_points=cp_points.shape[0]
    for counter in range(0,num_of_points):
        cp_points_norm[counter,0]-=means[0]
        cp_points_norm[counter,1]-=means[1]
    
    #scale so all elements will be in range of [0,1]
    if use_scale is True:
        cp_points_norm_abs=np.absolute(cp_points_norm)
        scale=1/np.amax(cp_points_norm_abs)
    else:
        scale=1

    cp_points_norm=cp_points_norm*scale

    trans_mat=np.matrix([[scale,0,-scale*means[0]],[0,scale,-scale*means[1]],[0,0,1]])

    return [cp_points_norm,trans_mat]



def LinMGA2Grid(mga_cp_points,proj_cp_points):
    '''
    LinMGA2Grid
    --------
    calculates linear transformation between MGA and projectgrid coordinates.

    input:
    1.mga_cp_points - MGA coordinates matrix n*2 [e,n;...]
    2.proj_cp_points - grid coordinates matrix n*2 [e,n;...]
    3.mga_points_2_trans - mga coordinates to transform matrix m*2 [e,n;...]

    output:
    1. proj_mat_mga2grid - projection matrix from mga system 2 grid system. 3*3 matrix, such as [e_grid;n_grid;1]=proj_mat_mga2grid*[e_mga;n_mga;1]
    '''
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
    var=np.linalg.solve(N, U)
    trans_mat_norm=np.matrix([ [var[0,0], -var[1,0], var[2,0]] ,[var[1,0], var[0,0], var[3,0]] ,[0,0,1] ])
    
    #denormalize
    proj_mat_mga2grid=np.matmul(np.matmul(np.linalg.inv(trans_mat_proj),trans_mat_norm), trans_mat_mga)
    
    return proj_mat_mga2grid



def NonLinMGA2Grid(mga_cp_points,grid_cp_points, aprox_ang_grid2mga_deg,aprox_mga_center_point):
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
    1. proj_mat_mga2grid - projection matrix from mga system 2 grid system. 3*3 matrix, such as [e_grid;n_grid;1]=proj_mat_mga2grid*[e_mga;n_mga;1]
    2. converged - true if adjustment converged, else 0
    '''

    #normalize points
    [mga_cp_points_norm,trans_mat_mga]=normalize_coors(mga_cp_points, use_scale=False)
    [grid_cp_points_norm,trans_mat_grid]=normalize_coors(grid_cp_points , use_scale=False)

    #adjustment params
    num_of_points=mga_cp_points_norm.shape[0]
    n=2*num_of_points #number of measurements
    u=3 #number of variables
    conv_treshold=mt.pow(10,-12) #convergence threshold
    max_iter=15 #maximum iterations

    #build adjustment matricespython
    aprox_ang_grid2mga_rad=deg2rad*aprox_ang_grid2mga_deg
    X0=np.matrix([-aprox_ang_grid2mga_rad,aprox_mga_center_point[0,0],aprox_mga_center_point[0,1]]).transpose() #aprox values vector
    Lb=np.reshape(mga_cp_points_norm,(n,1)) #raw measurements vector
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
            e_grid=grid_cp_points_norm[counter,0]
            n_grid=grid_cp_points_norm[counter,1]

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

    #denormalize
    azi=X0[0,0]
    te=X0[1,0]
    tn=X0[2,0]
    cosa=mt.cos(azi)
    sina=mt.sin(azi)
    trans_mat_norm=np.matrix([ [cosa,-sina,te] , [sina,cosa,tn] , [0,0,1] ])
    proj_mat_grid2mga=np.matmul(np.matmul(np.linalg.inv(trans_mat_mga),trans_mat_norm), trans_mat_grid)
    proj_mat_mga2grid=np.linalg.inv(proj_mat_grid2mga)

    return [proj_mat_mga2grid,converged]



def BuildProjectionMat2Grid(rot_angle_deg,MGA_base_point):
    '''
    BuildProjectionMat2Grid
    ------------------
    build projection matrix from mga system 2 grid system

    input:
    1. rot_angle_deg - rotation angle from North axis clockwise (azimuth) of grid system relative to MGA.
    2. MGA_base_point - The origin of the grid system in MGA coordinates ,1*2 matrix [e,n]  

    output:
    1. proj_mat_mga2grid - projection matrix from mga system 2 grid system. 3*3 matrix, such as [e_grid;n_grid;1]=proj_mat_mga2grid*[e_mga;n_mga;1]
    '''
    translation=np.matrix([ [1,0,-MGA_base_point[0,0]] , [0,1,-MGA_base_point[0,1]] , [0,0,1] ])
    rot_angle_rad=rot_angle_deg*deg2rad
    cosa=mt.cos(rot_angle_rad)
    sina=mt.sin(rot_angle_rad)
    rotation=np.matrix([ [cosa,-sina,0] , [sina,cosa,0] , [0,0,1] ])
    proj_mat_mga2grid=np.matmul(rotation,translation)

    return proj_mat_mga2grid


def ProjectionMat2Parts(proj_mat_mga2grid):
    '''
    ProjectionMat2Parts
    -------------------
    extract rotation and translation out of projection matrix from mga to grid system.

    input:
    1. proj_mat_mga2grid - projection matrix from mga system 2 grid system. 3*3 matrix, such as [e_grid;n_grid;1]=proj_mat_mga2grid*[e_mga;n_mga;1]

    output:
    1. rot_ang_deg - rotation angle from mga system to grid system
    2. mga_center_point - origin of grid system in mga coordinates. matrix 1*2 [e,n]
    '''

    #extract rotation angle
    rot_ang_rad=mt.atan(proj_mat_mga2grid[1,0]/proj_mat_mga2grid[0,0])
    rot_ang_deg=rot_ang_rad/deg2rad

    #extract translation
    mga_center_point=np.matmul(-np.linalg.inv(proj_mat_mga2grid[0:2,0:2]) , proj_mat_mga2grid[0:2,2])

    return [rot_ang_deg,mga_center_point.transpose()]


def Project2Grid(proj_mat_mga2grid,mga_points_2_trans):
    '''
    Project2Grid
    ------------
    project MGA coordinates to a Project grid defined by translation and rotation relative to MGA

    input:
    1. proj_mat_mga2grid - projection matrix from mga system 2 grid system. 3*3 matrix, such as [e_grid;n_grid;1]=proj_mat_mga2grid*[e_mga;n_mga;1]
    2. mga_points_2_trans - mga coordinates to transform matrix m*2 [e,n;...]

    output:
    1.grid_points_trans - transformed mga coordinates matrix m*2 [e,n;...]
    '''

    #setting homogenious coordinates
    num_of_points_to_trans=mga_points_2_trans.shape[0]
    one_vec=np.ones((num_of_points_to_trans,1))
    mga_points_2_trans=np.concatenate((mga_points_2_trans,one_vec),axis=1).transpose()

    #project to grid system
    grid_points_trans=np.matmul(proj_mat_mga2grid,mga_points_2_trans)
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
    lin_trans_mat=LinMGA2Grid(mga_cp_points[ind_cp,:],proj_cp_points[ind_cp,:])

    ###test NonLinLS2DTrans###
    [ang_grid2mga_deg0,mga_center_point0]=ProjectionMat2Parts(lin_trans_mat)
    [proj_mat_mga2grid,converged]=NonLinMGA2Grid(mga_cp_points[ind_cp,:],proj_cp_points[ind_cp,:], ang_grid2mga_deg0,mga_center_point0)
    converged_text="no"
    if converged:
        converged_text="yes"

    print("convergence: " + converged_text)

    ###test Project2Grid###
    grid_points_trans_lin=Project2Grid(lin_trans_mat,mga_cp_points[ind_ck,:])
    resid_lin=grid_points_trans_lin-proj_cp_points[ind_ck,:]
    print("residuals_lin: " + "\n" + str(resid_lin))
    V=np.reshape(resid_lin,(2*ind_ck.shape[0],1))
    g0_g_lin=mt.sqrt(np.matmul(V.transpose(),V)[0] / (ind_cp.shape[0]-4) ) 

    grid_points_trans_nonlin=Project2Grid(proj_mat_mga2grid,mga_cp_points[ind_ck,:])
    resid_nonlin=grid_points_trans_nonlin-proj_cp_points[ind_ck,:]
    print("residuals_nonlin: " + "\n" + str(resid_nonlin))
    V2=np.reshape(resid_nonlin,(2*ind_ck.shape[0],1))
    g0_g_nonlin=mt.sqrt(np.matmul(V2.transpose(),V2)[0] / (ind_cp.shape[0]-3) )
