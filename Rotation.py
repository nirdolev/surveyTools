from numpy import matrix as mat
from numpy import multiply
from math import cos,sin,pi

def sincos(ang_rad):
    '''
    sincos
    ------
    calculates sin and cos values of an angle [rad]

    input:
    1. ang_rad - rotation angle  [radians]

    output:
    1. asin - sin(angle)
    2. acos - cos(angle)
    '''
    asin=sin(ang_rad)
    acos=cos(ang_rad)
    return [asin,acos]


def RotMat2D(ang_rad,direction='counter_clockwise'):
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

    [asin,acos]=sincos(ang_rad)
    if direction=='clockwise':
        direct_mat=mat([ [1,-1] , [1,1] ])
    elif direction=='counter_clockwise':
        direct_mat=mat([ [1,1] , [-1,1] ])
    else:
        raise ValueError("direction argument can only get values of clockwise/counter_clockwise")
    
    rot_mat=multiply( mat([ [acos,asin] , [asin,acos] ]) , direct_mat)
    return rot_mat


def RotMat3X(ang_rad,direction='counter_clockwise'):
    '''
    RotMat3X
    --------
    3D rotation matrix around X axis

    input:
    1. ang_rad - rotation angle  [radians]
    2. direction - direction of rotation of the coordinate system (clockwise/counter_clockwise). if you want it to rotate a point it is contrary.

    output:
    1. rot_mat - 3D rotation matrix around X axis. 3*3 matrix
    '''

    [asin,acos]=sincos(ang_rad)
    if direction=='counter_clockwise':
        rot_mat=mat([ [1,0,0],[0,acos,asin],[0,-asin,acos] ])
    elif direction=='clockwise':
        rot_mat=mat([ [1,0,0],[0,acos,-asin],[0,asin,acos] ])
    else:
        raise ValueError("direction argument can only get values of clockwise/counter_clockwise")
    return rot_mat


def RotMat3Y(ang_rad,direction='counter_clockwise'):
    '''
    RotMat3Y
    --------
    3D rotation matrix around Y axis.

    input:
    1. ang_rad - rotation angle  [radians]
    2. direction - direction of rotation of the coordinate system (clockwise/counter_clockwise). if you want it to rotate a point it is contrary.

    output:
    1. rot_mat - 3D rotation matrix around Y axis. 3*3 matrix
    '''

    [asin,acos]=sincos(ang_rad)
    if direction=='counter_clockwise':
        rot_mat=mat([ [acos,0,-asin],[0,1,0],[asin,0,acos] ])
    elif direction=='clockwise':
        rot_mat=mat([ [acos,0,asin],[0,1,0],[-asin,0,acos] ])
    else:
        raise ValueError("direction argument can only get values of clockwise/counter_clockwise")
    return rot_mat


def RotMat3Z(ang_rad,direction='counter_clockwise'):
    '''
    RotMat3Z
    --------
    3D rotation matrix around Z axis.

    input:
    1. ang_rad - rotation angle  [radians]
    2. direction - direction of rotation of the coordinate system (clockwise/counter_clockwise). if you want it to rotate a point it is contrary.

    output:
    1. rot_mat - 3D rotation matrix around Z axis. 3*3 matrix
    '''

    [asin,acos]=sincos(ang_rad)
    if direction=='counter_clockwise':
        rot_mat=mat([ [acos,asin,0],[-asin,acos,0],[0,0,1] ])
    elif direction=='clockwise':
        rot_mat=mat([ [acos,-asin,0],[asin,acos,0],[0,0,1] ])
    else:
        raise ValueError("direction argument can only get values of clockwise/counter_clockwise")
    return rot_mat


def RotMat3D(ang_rad,axis='Z',direction='counter_clockwise'):
    '''
    RotMat3D
    --------
    3D rotation matrix
    
    input:
    1. ang_rad - rotation angle  [radians]
    2. axis - around which axis are we rotating. X/Y/Z
    3. direction - direction of rotation of the coordinate system (clockwise/counter_clockwise). if you want it to rotate a point it is contrary.

    output:
    1. rot_mat - 3D rotation matrix. 3*3 matrix
    '''
    
    if axis=='X':
        return RotMat3X(ang_rad,direction)
    elif axis=='Y':
        return RotMat3Y(ang_rad,direction)
    elif axis=='Z':
        return RotMat3Z(ang_rad,direction)
    else:
        raise ValueError("axis argument can only get values of X/Y/Z")

    
    



'''testing functionality'''
if __name__ == "__main__":
    deg2rad=pi/180


    ###test RotMat2D###
    ang_deg=30
    rot_mat_clk=RotMat2D(ang_deg*deg2rad)
    print (str(rot_mat_clk))

    rot_mat_anti=RotMat2D(ang_deg*deg2rad,direction='counter_clockwise')
    print (str(rot_mat_anti))


    #rotate a point clockwise (coordinate system rotates anti-clockwise)
    p2D=mat([0,50]).transpose()
    ang_deg=90
    rot_mat=RotMat2D(ang_deg*deg2rad,direction='counter_clockwise')
    rotP2D=rot_mat*p2D
    print (str(rotP2D.transpose()))

    #check wrong input
    try:
        rot_mat_err=RotMat2D(ang_deg*deg2rad,direction='bla')
        print (str(rot_mat_err))
    except ValueError as e:
        print(e)


    ###test RotMat3D###
    ang_rad=pi/2

    #check rotation around x
    rot_mat_x=RotMat3D(ang_rad,axis='X',direction='clockwise') #rotates a point counter-clockwise
    p3D_1=mat([0,50,0]).transpose()
    p3D_1_rot=rot_mat_x*p3D_1
    print (str(p3D_1_rot.transpose()))

    #check rotation around y
    rot_mat_y=RotMat3D(ang_rad,axis='Y',direction='counter_clockwise') #rotates a point clockwise
    p3D_2=mat([50,0,0]).transpose()
    p3D_2_rot=rot_mat_y*p3D_2
    print (str(p3D_2_rot.transpose()))

    #check rotation around z
    rot_mat_z=RotMat3D(ang_rad,axis='Z',direction='clockwise') #rotates a point counter-clockwise
    p3D_2_rot=rot_mat_z*p3D_2
    print (str(p3D_2_rot.transpose()))

    #check wrong input-axis
    try:
        rot_mat_err=RotMat3D(ang_rad,axis='bla',direction='clockwise')
        print (str(rot_mat_err))
    except ValueError as e:
        print(e)

    #check wrong input-direction
    try:
        rot_mat_err=RotMat3D(ang_rad,axis='X',direction='bla')
        print (str(rot_mat_err))
    except ValueError as e:
        print(e)




