from numpy import matrix as mat
from numpy import cross,concatenate,matmul
from numpy.linalg import norm
from math import cos,sin,pi

from Rotation import RotMat2D


def Rot2DFrom2Pt(xy1,xy2):
    '''
    Rot2DFrom2Pt
    ------------
    calculate 2D rotation matrix from 2 matching point pairs in rotated coordinate systems. p1<-p2

    input:
    1.xy1=2 points in 2D coordinate system. matrix 2*2 [x1,y1;x2,y2]
    2.xy2=2 matching points in 2D rotated coordinate system. matrix 2*2 [x1,y1;x2,y2]

    output:
    1. rotMat =2D rotation matrix from coordinate sytem 2 to system 1. matrix 2*2
    '''

    #calculate unit vectors in system 1
    p1=mat([xy1[0,0],xy1[0,1],0]).transpose()
    p2=mat([xy1[1,0],xy1[1,1],0]).transpose()
    zvec=mat([0,0,1]).transpose()
    xvec=p2-p1
    yvec=cross(zvec.transpose(),xvec.transpose()).transpose()
    xvec_unit=xvec/norm(xvec)
    yvec_unit=yvec/norm(yvec)
    R1=concatenate((xvec_unit[0:2,:],yvec_unit[0:2,:]),axis=1) #rotates from system 1 to the two points local system

    #calculate unit vectors in system 2
    p1r=mat([xy2[0,0],xy2[0,1],0]).transpose()
    p2r=mat([xy2[1,0],xy2[1,1],0]).transpose()
    xvecr=p2r-p1r
    yvecr=cross(zvec.transpose(),xvecr.transpose()).transpose()
    xvecr_unit=xvecr/norm(xvecr)
    yvecr_unit=yvecr/norm(yvecr)
    R2=concatenate((xvecr_unit[0:2,:],yvecr_unit[0:2,:]),axis=1) #rotates from system 1 to the two points local system

    rotMat=matmul(R1.transpose(),R2)
    return rotMat



def Rot3DFrom3Pt(xy1,xy2):
    '''
    Rot3DFrom3Pt
    ------------
    calculate 3D rotation matrix from 3 matching point pairs in rotated coordinate systems. p1<-p2

    input:
    1.xy1=3 points in 3D coordinate system. matrix 3*3 [x1,y1,z1;x2,y2,z2;x3,y3,z3]
    2.xy2=3 matching points in 3D rotated coordinate system. matrix 3*3 [x1,y1,z1;x2,y2,z2;x3,y3,z3]

    output:
    1. rotMat =3D rotation matrix from coordinate sytem 2 to system 1. matrix 3*3
    '''

    



'''testing functionality'''
if __name__ == "__main__":
    deg2rad=pi/180


    ###test Rot2DFrom2Pt###
    ang_deg=20
    rot_mat=RotMat2D(ang_deg*deg2rad)
    
    p1=mat([0,0]).transpose()
    p2=mat([5,5]).transpose()

    p1r=matmul(rot_mat,p1)
    p2r=matmul(rot_mat,p2)

    xy1=concatenate((p1.transpose(),p2.transpose()))
    xy2=concatenate((p1r.transpose(),p2r.transpose()))
    rot_mat_calc=Rot2DFrom2Pt(xy1,xy2)

    
    print (str(rot_mat))
    print (str(rot_mat_calc))
    print (str(norm(rot_mat_calc-rot_mat)))