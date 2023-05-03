import numpy as np
from scipy.integrate import quad
from scipy.optimize import curve_fit


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 3, 2023
Code description: Contains different properties of the curve that is fit to a peduncle mask
"""


class Curve:

    def __init__(self, direction: str = None, curve_x: np.array = None, curve_y: np.array = None, params: np.array = None):
        self._parabola_direction = direction
        self._curve_x = curve_x
        self._curve_y = curve_y
        self._params = params

    @property
    def parabola_direction(self):
        return self._parabola_direction

    @parabola_direction.setter
    def parabola_direction(self, direction):
        self._parabola_direction = direction

    @property
    def curve_x(self):
        return self._curve_x

    @curve_x.setter
    def curve_x(self, value):
        self._curve_x = value

    @property
    def curve_y(self):
        return self._curve_y

    @curve_y.setter
    def curve_y(self, value):
        self._curve_y = value

    @property
    def params(self):
        return self._params

    @params.setter
    def params(self, params):
        self._params = params

    def curve_length_derivative(self, t):
        return (1 + (2*self._params[0]*t + self._params[1])**2)**0.5

    def full_curve_length(self):
        if self._parabola_direction == 'vertical':
            return abs(quad(self.curve_length_derivative, self._curve_y[0], self._curve_y[-1])[0])
        else:
            return abs(quad(self.curve_length_derivative, self._curve_x[0], self._curve_x[-1])[0])

    def curve_length(self, idx):
        if self._parabola_direction == 'vertical':
            return abs(quad(self.curve_length_derivative, self._curve_y[0], self._curve_y[idx])[0])
        else:
            return abs(quad(self.curve_length_derivative, self._curve_x[0], self._curve_x[idx])[0])

