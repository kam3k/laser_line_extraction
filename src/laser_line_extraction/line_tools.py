import math as m
import config as c

RANGES = None
BEARINGS = None
X = None
Y = None

################################################################################
# Line class
################################################################################

class Line(object):

    def __init__(self, indices):
        self.I = indices
        self.a = None
        self.r = None
        self.start = None
        self.end = None
        self.cov_aa = None
        self.cov_rr = None
        self.cov_ar = None

################################################################################
# Public functions
################################################################################

def dist_to_point(line, index):
    rad = m.sqrt(X[index]**2 + Y[index]**2)
    angle = m.atan2(Y[index], X[index])
    return abs(rad * m.cos(angle - line.a) - line.r)

def endpoint_fit(line):
    line.start = (X[line.I[0]], Y[line.I[0]])
    line.end = (X[line.I[-1]], Y[line.I[-1]])
    _angle_from_endpoints(line)
    _radius_from_endpoints(line)

def least_squares_fit(line):
    ranges = [RANGES[i] for i in line.I]
    bearings = [BEARINGS[i] for i in line.I]
    r_vars = [(c.RANGE_ERROR * r)**2 for r in ranges]
    b_vars = [c.BEARING_ERROR for b in bearings]
    old_rad, old_angle = 0.0, 0.0
    while abs(line.r - old_rad) > 1e-4 or abs(line.a - old_angle) > 1e-4:
        old_rad, old_angle = line.r, line.a
        P = _point_scalar_cov(line, ranges, bearings, r_vars, b_vars)
        _radius_from_leastsq(line, ranges, bearings, P)
        _angle_from_leastsq(line, ranges, bearings, r_vars, b_vars)
    _set_covariance(line, ranges, bearings, r_vars, b_vars)
    _set_endpoints(line)

def length(line):
    return m.sqrt((line.end[0] - line.start[0])**2 + 
                  (line.end[1] - line.start[1])**2)

def num_points(line):
    return len(line.I)

def set_data(ranges, bearings, xs, ys):
    global RANGES, BEARINGS, X, Y
    RANGES = ranges
    BEARINGS = bearings
    X = xs
    Y = ys

################################################################################
# Private utility functions
################################################################################

def _angle_from_endpoints(line):
    try:
        slope = (line.end[1] - line.start[1]) / (line.end[0] - line.start[0])
    except ZeroDivisionError:
        line.a = 0
    else:
        line.a = _pi_to_pi(m.atan(slope) + m.pi/2)

def _angle_from_leastsq(line, ranges, bearings, r_vars, b_vars):
    point_parameters = _point_parameters(line, ranges, bearings, r_vars, b_vars)
    angle_inc = _angle_increment(point_parameters)
    line.a += angle_inc

def _angle_increment(point_parameters):
    c, s, a, a_p, a_pp, b, b_p, b_pp = point_parameters
    num = sum((b[k] * a_p[k] - a[k] * b_p[k]) / b[k]**2 for k in range(len(c)))
    denom = sum(((a_pp[k] * b[k] - a[k] * b_pp[k]) * b[k] - 
                  2 * (a_p[k] * b[k] - a[k] * b_p[k]) * b_p[k]) / b[k]**3
                  for k in range(len(c)))
    return -(num/denom)

def _set_covariance(line, ranges, bearings, r_vars, b_vars):
        P = _point_scalar_cov(line, ranges, bearings, r_vars, b_vars)
        P_RR = 1.0 / sum(1.0/P_k for P_k in P)
        point_parameters = _point_parameters(line, ranges, bearings, r_vars, b_vars)
        c, s, a, a_p, a_pp, b, b_p, b_pp = point_parameters
        G = sum(((a_pp[k] * b[k] - a[k] * b_pp[k]) * b[k] - 
                      2 * (a_p[k] * b[k] - a[k] * b_p[k]) * b_p[k]) / b[k]**3
                      for k in range(len(c)))
        P_Ra = P_RR/G * sum(2 * d * m.sin(line.a - phi) / b_k
                             for (d, phi, b_k) in zip(ranges, bearings, b))
        P_aa = 1/G**2 * sum(4 * d**2 * m.sin(line.a - phi)**2 / b_k
                             for (d, phi, b_k) in zip(ranges, bearings, b))
        line.cov_aa = P_aa
        line.cov_rr = P_RR
        line.cov_ar = P_Ra


def _pi_to_pi(angle):
    """Converts and angle to the interval -PI < angle <= PI."""
    angle = angle % (2*m.pi)
    if angle > m.pi:
        angle -= 2*m.pi
    return angle

def _point_parameters(line, ranges, bearings, r_vars, b_vars):
    c = [m.cos(line.a - phi) for phi in bearings]
    s = [m.sin(line.a - phi) for phi in bearings]
    a = [(d * c_k - line.r)**2 for (d, c_k) in zip(ranges, c)]
    a_p = [-2 * d * s_k * (d * c_k - line.r) 
            for (d, c_k, s_k) in zip(ranges, c, s)]
    a_pp = [2 * d**2 * s_k**2 - 2 * d * c_k * (d * c_k - line.r)
            for (d, c_k, s_k) in zip(ranges, c, s)]
    b = [var_d * c_k**2 + var_phi * d**2 * s_k**2
            for (d, c_k, s_k, var_d, var_phi) 
            in zip(ranges, c, s, r_vars, b_vars)]
    b_p = [2 * (d**2 * var_phi - var_d) * c_k * s_k
            for (d, c_k, s_k, var_d, var_phi) 
            in zip(ranges, c, s, r_vars, b_vars)]
    b_pp = [2 * (d**2 * var_phi - var_d) * (c_k**2 - s_k**2)
            for (d, c_k, s_k, var_d, var_phi) 
            in zip(ranges, c, s, r_vars, b_vars)]
    return c, s, a, a_p, a_pp, b, b_p, b_pp

def _point_scalar_cov(line, ranges, bearings, r_vars, b_vars):
    Q_11 = [D**2 * var_phi * m.sin(Phi)**2 + var_d * m.cos(Phi)**2
            for (D, Phi, var_d, var_phi) in zip(ranges, bearings, r_vars, b_vars)]
    Q_12 = [-D**2 * var_phi * m.sin(2*Phi) / 2 + var_d * m.sin(2*Phi) / 2
            for (D, Phi, var_d, var_phi) in zip(ranges, bearings, r_vars, b_vars)]
    Q_22 = [D**2 * var_phi * m.cos(Phi)**2 + var_d * m.sin(Phi)**2
            for (D, Phi, var_d, var_phi) in zip(ranges, bearings, r_vars, b_vars)]
    P = [Q_11[k] * m.cos(line.a)**2 + 2 * Q_12[k] * m.sin(line.a) * m.cos(line.a)
            + Q_22[k] * m.sin(line.a)**2 for k in range(len(Q_11))]
    return P

def _radius_from_endpoints(line):
    line.r = line.start[0] * m.cos(line.a) + line.start[1] * m.sin(line.a)
    if line.r < 0:
        line.r = -line.r
        line.a = _pi_to_pi(line.a + m.pi)

def _radius_from_leastsq(line, ranges, bearings, P):
    P_RR = 1.0 / sum(1.0/P_k for P_k in P)
    R = P_RR * sum(d * m.cos(line.a - phi) / P_k for (d, phi, P_k) in
                   zip(ranges, bearings, P))
    line.r = R

def _set_endpoints(line):
    s = -1/m.tan(line.a)
    b = line.r / m.sin(line.a)
    x, y = line.start
    line.start = ((s * y + x - s * b) / (s**2 + 1), (s**2 * y + s * x + b) / (s**2 + 1))
    x, y = line.end
    line.end = ((s * y + x - s * b) / (s**2 + 1), (s**2 * y + s * x + b) / (s**2 + 1))
