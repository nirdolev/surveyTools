from numpy import matrix as mat
from numpy import multiply
from math import cos,sin,pi

def RotMat2D(ang_rad,direction='clockwise'):
    '''
    RotMat2D
    --------
    2D rotation matrix.

    input:
    1. ang_rad - rotation angle  [radians]
    2. direction - direction of rotation of the coordinate system (clockwise/counter_clockwise). if you want it to rotate a point it is contrary.

    output:
    1. rot_mat - 2D rotation matrix. 2*2 matrix
    '''

    cosa=cos(ang_rad)
    sina=sin(ang_rad)
    if direction=='clockwise':
        direct_mat=mat([ [1,-1] , [1,1] ])
    elif direction=='counter_clockwise':
        direct_mat=mat([ [1,1] , [-1,1] ])
    else:
        raise ValueError("direction argument can only get values of clockwise/counter_clockwise")
    
    rot_mat=multiply( mat([ [cosa,sina] , [sina,cosa] ]) , direct_mat)

    return rot_mat


'''testing functionality'''
if __name__ == "__main__":
    deg2rad=pi/180


    ###test RotMat2D###
    ang_deg=30
    rot_mat_clk=RotMat2D(ang_deg*deg2rad)
    print (str(rot_mat_clk))

    rot_mat_anti=RotMat2D(ang_deg*deg2rad,direction='counter_clockwise')
    print (str(rot_mat_anti))

    try:
        rot_mat_err=RotMat2D(ang_deg*deg2rad,direction='bla')
        print (str(rot_mat_err))
    except ValueError as e:
        print(e)

