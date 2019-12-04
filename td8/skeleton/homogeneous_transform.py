import numpy as np

def rotX(alpha):
    c = np.cos(alpha)
    s = np.sin(alpha)
    return np.array([[1, 0, 0, 0],
                     [0, c, s, 0],
                     [0,-s, c, 0],
                     [0, 0, 0, 1]], dtype=np.double)
def rotY(alpha):
    c = np.cos(alpha)
    s = np.sin(alpha)
    return np.array([[c, 0,-s, 0],
                     [0, 1, 0, 0],
                     [s, 0, c, 0],
                     [0, 0, 0, 1]], dtype=np.double)

def rotZ(alpha):
    c = np.cos(alpha)
    s = np.sin(alpha)
    return np.array([[ c, s, 0, 0],
                     [-s, c, 0, 0],
                     [ 0, 0, 1, 0],
                     [ 0, 0, 0, 1]], dtype=np.double)

def translation(vec):
    return np.array([[1, 0, 0, -vec[0]],
                     [0, 1, 0, -vec[1]],
                     [0, 0, 1, -vec[2]],
                     [0, 0, 0, 1]], dtype=np.double)

def dRotX(alpha):
    c = np.cos(alpha)
    s = np.sin(alpha)
    return np.array([[0, 0, 0, 0],
                     [0,-s, c, 0],
                     [0,-c,-s, 0],
                     [0, 0, 0, 0]], dtype=np.double)

def dRotY(alpha):
    c = np.cos(alpha)
    s = np.sin(alpha)
    return np.array([[-s, 0,-c, 0],
                     [ 0, 0, 0, 0],
                     [ c, 0,-s, 0],
                     [ 0, 0, 0, 0]], dtype=np.double)

def dRotZ(alpha):
    c = np.cos(alpha)
    s = np.sin(alpha)
    return np.array([[-s, c, 0, 0],
                     [-c,-s, 0, 0],
                     [ 0, 0, 0, 0],
                     [ 0, 0, 0, 0]], dtype=np.double)

def dTranslation(vec):
    return np.array([[0, 0, 0, -vec[0]],
                     [0, 0, 0, -vec[1]],
                     [0, 0, 0, -vec[2]],
                     [0, 0, 0, 0]], dtype=np.double)

def invertTransform(T):
    I = T.copy()
    RI = T[:3,:3].transpose()
    I[:3,:3] = RI
    I[:3,3] = -RI.dot(T[:3,3])
    return I


if __name__ == "__main__":
    T = rotX(0.3).dot(translation(np.array([1,2,3])))
    print ("T: ", T)
    IT = invertTransform(T)
    print ("T: ", T)
    print ("T^-1: ", IT)
    print ("T*IT: ", T.dot(IT))
