import array

import cv2
import math
import copy
import numpy

def mkmat(rows, cols, L):
    mat = numpy.matrix(L, dtype='float64')
    mat.resize((rows,cols))
    return mat

class CameraModel:

    def __init__(self):
        _D = [0.0045746891190022974, -0.02343381700044202, -0.0006267443584288398, 0.001170981545289317, 0.0]
        _K = [410.1407953364398, 0.0, 477.1751162124048, 0.0, 411.23094794597665, 286.0524304974213, 0.0, 0.0, 1.0]
        _R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        _P = [390.712890625, 0.0, 478.672438650392, 0.0, 0.0, 408.66998291015625, 284.83587628132955, 0.0, 0.0, 0.0, 1.0, 0.0]
        binning_x = 1
        binning_y = 1

        self.x_offset = 160
        self.y_offset = 100

        self.K = mkmat(3, 3, _K)
        self.D = mkmat(len(_D), 1, _D)
        self.R = mkmat(3, 3, _R)
        self.P = mkmat(3, 4, _P)
        self.full_K = mkmat(3, 3, _K)
        self.full_P = mkmat(3, 4, _P)
        self.width = 960
        self.height = 600
        self.binning_x = max(1, binning_x)
        self.binning_y = max(1, binning_y)
        self.resolution = (960, 600)


    def cx(self):
        """ Returns x center """
        return self.P[0,2]
    def cy(self):
        """ Returns y center """
        return self.P[1,2]
    def fx(self):
        """ Returns x focal length """
        return self.P[0,0]
    def fy(self):
        """ Returns y focal length """
        return self.P[1,1]

    def Tx(self):
        """ Return the x-translation term of the projection matrix """
        return self.P[0,3]

    def Ty(self):
        """ Return the y-translation term of the projection matrix """
        return self.P[1,3]


    def rectifyPoint(self, uv_raw):
        """
        :param uv_raw:    pixel coordinates
        :type uv_raw:     (u, v)

        Applies the rectification specified by camera parameters
        :math:`K` and and :math:`D` to point (u, v) and returns the
        pixel coordinates of the rectified point.
        """

        src = mkmat(1, 2, list(uv_raw))
        src.resize((1,1,2))
        dst = cv2.undistortPoints(src, self.K, self.D, R=self.R, P=self.P)
        return dst[0,0]

    def projectPixelTo3dRay(self, uv):
        """
        :param uv:        rectified pixel coordinates
        :type uv:         (u, v)

        Returns the unit vector which passes from the camera center to through rectified pixel (u, v),
        using the camera :math:`P` matrix.
        This is the inverse of :meth:`project3dToPixel`.
        """
        x = (uv[0] - self.cx()) / self.fx()
        y = (uv[1] - self.cy()) / self.fy()
        norm = math.sqrt(x*x + y*y + 1)
        x /= norm
        y /= norm
        z = 1.0 / norm
        return (x, y, z)