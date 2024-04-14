import numpy as np
from math import sin, cos, acos, sqrt
from scipy.spatial.transform import Rotation as R


def normalize(v, tolerance=0.1):
    mag2 = sum(n * n for n in v)
    if mag2 == 0:
        return v 
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = tuple(n / mag for n in v)
    return np.array(v)

class Quaternion:

    def from_axisangle(theta, v):
        theta = theta
        v = normalize(v)

        new_quaternion = Quaternion()
        new_quaternion._axisangle_to_q(theta, v)
        return new_quaternion

    def from_value(value):
        new_quaternion = Quaternion()
        new_quaternion._val = value
        return new_quaternion
    def from_MATRIX(matrix):
        # print(matrix)
        new_quaternion = Quaternion()
        w = sqrt(1 + matrix[0][0] + matrix[1][1] + matrix[2][2])/2 + 0.000001;
        x = (matrix[2][1] - matrix[1][2])/(4*w)
        y = (matrix[0][2] - matrix[2][0])/(4*w)
        z = (matrix[1][0] - matrix[0][1])/(4*w)
        value = np.array([w,x,y,z])
        new_quaternion._val = value
        return new_quaternion
    def _axisangle_to_q(self, theta, v):
        x = v[0]
        y = v[1]
        z = v[2]

        w = cos(theta/2.)
        x = x * sin(theta/2.)
        y = y * sin(theta/2.)
        z = z * sin(theta/2.)

        self._val = np.array([w, x, y, z])

    def __mul__(self, b):

        if isinstance(b, Quaternion):
            return self._multiply_with_quaternion(b)
        elif isinstance(b, (list, tuple, np.ndarray)):
            if len(b) != 3:
                raise Exception("Input vector has invalid length {len(b)}")
            return self._multiply_with_vector(b)
        else:
            raise Exception("Multiplication with unknown type {type(b)}")

    def __sub__(self, b):

        if isinstance(b, Quaternion):
            return self._q_dist(b)
        elif isinstance(b, (list, tuple, np.ndarray)):
            if len(b) != 3:
                raise Exception("Input vector has invalid length {len(b)}")
            return self._q_dist(b)
        else:
            raise Exception("Multiplication with unknown type {type(b)}")
    def dist(self, b):
        A = self._val
        B = b._val
        # print(A,B)
        return 1 - np.dot(A,B)**2;
    def _q_dist(self,q2):
        return 1 - np.dot(self._val,q2._val)**2;
    def _multiply_with_quaternion(self, q2):
        w1, x1, y1, z1 = self._val
        w2, x2, y2, z2 = q2._val
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

        result = Quaternion.from_value(np.array((w, x, y, z)))
        return result

    def _multiply_with_vector(self, v):
        q2 = Quaternion.from_value(np.append((0.0), v))
        return (self * q2 * self.get_conjugate())._val[1:]

    def get_conjugate(self):
        w, x, y, z = self._val
        result = Quaternion.from_value(np.array((w, -x, -y, -z)))
        return result

    def __repr__(self):
        theta, v = self.get_axisangle()
        return f"((%.6f; %.6f, %.6f, %.6f))"%(theta, v[0], v[1], v[2])

    def get_axisangle(self):
        w, v = self._val[0], self._val[1:]
        theta = acos(w) * 2.0

        return theta, normalize(v)

    def tolist(self):
        return self._val.tolist()

    def vector_norm(self):
        w, v = self.get_axisangle()
        return np.linalg.norm(v)



class Haal(object):
    def __init__(self):
        self.rotation_Q = Quaternion.from_axisangle(0.0,(0,0,1))
        self.position_P = [0,0,0]
    def locate(self,x,y,z):
        self.position_P = [x,y,z]
    def return_list(self):
        P = self.position_P
        Q = self.rotation_Q.tolist()
        return P, Q 



class Transformation(object):
    def __init__(self):
        self.rotation = R.from_matrix([[1, 0, 0],
                                       [0, 1, 0],
                                       [0, 0, 1]])
        self.translation = [0,0,0]
        self.T = []
    def locate(self,x,y,z):
        self.translation = [x,y,z]
    def rotate(self,M):
        self.rotation = R.from_matrix(M)
    def from_value(T):

        new_t = Transformation();
        new_t.T = np.array(T);
        # print(T)
        new_t.rotation = R.from_matrix(new_t.T[:3,:3]);
        new_t.translation = list(np.ndarray.flatten(new_t.T[:-1,-1:]));
        return new_t;
    def T_matrix(self):
        T = [[1,0,0,0,],
             [0,1,0,0,],
             [0,0,1,0,],
             [0,0,0,1,],]
        # print("ROTATION",self.rotation)

        rot_mat = self.rotation.as_matrix();

        # print("ROTATION MATRIX",rot_mat)
        T[0][0] = rot_mat[0][0]
        T[0][1] = rot_mat[0][1]
        T[0][2] = rot_mat[0][2]

        T[1][0] = rot_mat[1][0]
        T[1][1] = rot_mat[1][1]
        T[1][2] = rot_mat[1][2]

        T[2][0] = rot_mat[2][0]
        T[2][1] = rot_mat[2][1]
        T[2][2] = rot_mat[2][2]

        T[0][3] = self.translation[0]
        T[1][3] = self.translation[1]
        T[2][3] = self.translation[2]

        self.T = np.array(T);

        return self.T 
    # def get_translation(self):
        # return 
    def __mul__(self, b):
        # print(self.T_matrix())
        # print(b.T_matrix())
        try:
            result = np.matmul(self.T_matrix(),b.T_matrix())
            # print(result)
            return_trans = Transformation.from_value(result)
            return  return_trans
        except Exception as e:
            raise e
            # raise e
