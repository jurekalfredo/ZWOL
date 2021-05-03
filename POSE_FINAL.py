

from numpy import pi
from CD_robot import *
def MATRIX_(pose):
    """Calcula la matriz de (4x4) desde la posición (mm) y los ángulos de Euler (grados),
        dada POSE [x, y, z, r, p, w].
    """
    alfa = pose[3] * pi / 180
    beta = pose[4] * pi / 180
    gama = pose[5] * pi / 180

    Q10 = np.cos(alfa) * np.cos(beta)
    Q11 = np.cos(beta) * np.sin(alfa)
    Q12 = -np.sin(beta)
    Q13 = 0

    Q20 = np.cos(alfa) * np.sin(beta) * np.sin(gama) - np.cos(gama) * np.sin(alfa)
    Q21 = np.cos(alfa) * np.cos(gama) + np.sin(alfa) * np.sin(beta) * np.sin(gama)
    Q22 = np.cos(beta) * np.sin(gama)
    Q23 = 0

    Q30 = np.sin(alfa) * np.sin(gama) + np.cos(alfa) * np.cos(gama) * np.sin(beta)
    Q31 = np.cos(gama) * np.sin(alfa) * np.sin(beta) - np.cos(alfa) * np.sin(gama)
    Q32 = np.cos(beta) * np.cos(gama)
    Q33 = 0

    Q40 = pose[0]
    Q41 = pose[1]
    Q42 = pose[2]
    Q43 = 1

    T6r0_2 = np.eye(4)
    T6r0_2[0, :] = [Q10, Q20, Q30, Q40]
    T6r0_2[1, :] = [Q11, Q21, Q31, Q41]
    T6r0_2[2, :] = [Q12, Q22, Q32, Q42]
    T6r0_2[3, :] = [Q13, Q23, Q33, Q43]
    return T6r0_2


def POSE_(H):
    """ (H) refiere a matriz de (4x4)"""
    x = H[0, 3]
    y = H[1, 3]
    z = H[2, 3]

    if (H[2, 0]) > (1.0 - 1e-6):
        p = -pi / 2
        r = 0
        w = np.arctan2(-H[1, 2], H[1, 1])
    elif (H[2, 0]) < (-1.0 + 1e-6):
        p = pi / 2
        r = 0
        w = np.arctan2(H[1, 2], H[1, 1])
    else:
        p = np.arctan2(-H[2, 0], np.sqrt(H[0, 0] * H[0, 0] + H[1, 0] * H[1, 0]))
        w = np.arctan2(H[1, 0], H[0, 0])
        r = np.arctan2(H[2, 1], H[2, 2])
    #print([x, y, z, w * 180 / pi, p * 180 / pi, r * 180 / pi])    
    return [x, y, z, w * 180 / pi, p * 180 / pi, r * 180 / pi]

def Pose(AT60,tool,frame):
    
       tx = tool[0]
       ty = tool[1]
       tz = tool[2]
       tr = tool[3]
       tp = tool[4]
       tw = tool[5]

       fx = frame[0]
       fy = frame[1]
       fz = frame[2]
       fr = frame[3]
       fp = frame[4]
       fw = frame[5]

       pose_h = tx, ty, tz, tr, tp, tw
       Matrix_h = MATRIX_(pose_h)  
       AT_1=np.dot(AT60,Matrix_h)
       
       ###################################
       ### MATRIZ DE REFERECIA AGREGADO ##
       ###################################

       pose_f = fx, fy, fz, fr, fp, fw
       Matrix_f = MATRIX_(pose_f)

       inverse_1 = np.linalg.inv(Matrix_f)
       AT_2 =np.dot(inverse_1,AT_1)
       
       S = POSE_(AT_2)
       print ("X: "+str(S[0])+" Y: "+str(S[1])+" Z: "+str(S[2])+" Rz: "+str(S[3])+" Ry: "+str(S[4])+" Rz: "+str(S[5]))
       return S




MATRIX = (326.812 , -6.416, 399.531, 75.407, 7.102, 100.670)
POS = MATRIX_(MATRIX)

tool = 100, 70, 0.00, -50.00, 0.00, 0.00
frame = 0.00 , 0.00, 0.00, 0.00, 0.00, 0.00

POS_F = Pose(POS,tool,frame)    
    
    
