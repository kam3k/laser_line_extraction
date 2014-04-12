import math as m


class LineExtractor(object):
    # --------------------------------------------------------------------------
    # Constructor
    # --------------------------------------------------------------------------
    def __init__(self, min_range=0.2, outlier_dist=0.05, min_split_dist=0.05,
                 min_line_length=0.5, min_line_points=9, max_line_gap=0.4,
                 least_sq_angle_thresh=1e-4, least_sq_radius_thresh=1e-4):
        self.ranges = None
        self.bearings = None
        self.range_vars = None
        self.bearing_vars = None
        self.min_range = min_range
        self.outlier_dist = outlier_dist
        self.min_split_dist = min_split_dist
        self.min_line_length = min_line_length
        self.min_line_points = min_line_points
        self.max_line_gap = max_line_gap
        Line.least_sq_angle_thresh = least_sq_angle_thresh
        Line.least_sq_radius_thresh = least_sq_radius_thresh
    # --------------------------------------------------------------------------
    # Public functions
    # --------------------------------------------------------------------------
    def run(self):
        if self.ranges is None:
            raise UnboundLocalError("Must set data before running.")
        self._filter_close_points()
        self._filter_outlier_points()
        lines = []
        self._split(self.indices[:], lines)
        lines = self._filter_short_lines(lines)
        lines = self._filter_sparse_lines(lines)
        for l in lines:
            l.least_sq_fit()
        lines = self._merge_lines(lines)
        return lines

    def set_data(self, ranges, bearings, range_vars, bearing_vars):
        # Instance data
        self.ranges = ranges
        self.bearings = bearings
        self.range_vars = range_vars
        self.bearing_vars = bearing_vars
        self.indices = range(len(self.ranges))
        # Line class variables
        Line.ranges = ranges
        Line.bearings = bearings
        Line.range_vars = range_vars
        Line.bearing_vars = bearing_vars
        Line.X = [r * m.cos(b) for r, b in zip(ranges, bearings)]
        Line.Y = [r * m.sin(b) for r, b in zip(ranges, bearings)]
    # --------------------------------------------------------------------------
    # Private functions
    # --------------------------------------------------------------------------
    def _dist_between_points(self, index_1, index_2):
        x_1 = self.ranges[index_1] * m.cos(self.bearings[index_1])
        y_1 = self.ranges[index_1] * m.sin(self.bearings[index_1])
        x_2 = self.ranges[index_2] * m.cos(self.bearings[index_2])
        y_2 = self.ranges[index_2] * m.sin(self.bearings[index_2])
        return m.sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2)

    def _filter_close_points(self):
        self.indices = [i for i in self.indices 
                        if self.ranges[i] >= self.min_range]

    def _filter_outlier_points(self):
        new_indices = []
        for i, p_i in enumerate(self.indices):
            if i == 0: # first point
                p_j = self.indices[i+1]
                p_k = self.indices[i+2]
            elif i == len(self.indices) - 1: # last point
                p_j = self.indices[i-1]
                p_k = self.indices[i-2]
            else: # middle points
                p_j = self.indices[i-1]
                p_k = self.indices[i+1]
            if (abs(self.ranges[p_i] - self.ranges[p_j]) > self.outlier_dist and
                abs(self.ranges[p_i] - self.ranges[p_k]) > self.outlier_dist): 
                line = Line([p_j, p_k])
                line.endpoint_fit()
                if line.dist_to_point(p_i) > self.min_split_dist:
                    continue # ignore this point
            new_indices.append(p_i)
        self.indices = new_indices

    def _filter_short_lines(self, lines):
        return [l for l in lines if l.length >= self.min_line_length]

    def _filter_sparse_lines(self, lines):
        return [l for l in lines if l.num_points >= self.min_line_points]

    def _merge_lines(self, lines):
        return lines

    def _split(self, indices, lines):
        # Don't split if only a single point (only occurs when orphaned by gap)
        if len(indices) <= 1:
            return
        line = Line(indices)
        line.endpoint_fit()
        dist_max = 0
        gap_max = 0
        # Find farthest point and largest gap
        for i in range(1, len(indices)-1):
            dist = line.dist_to_point(indices[i])
            if dist > dist_max:
                dist_max = dist
                i_max = i
            gap = self._dist_between_points(indices[i], indices[i+1])
            if gap > gap_max:
                gap_max = gap
                i_gap = i + 1
        # Check for gaps at endpoints
        gap_start = self._dist_between_points(indices[0], indices[1])
        if gap_start > gap_max:
            gap_max = gap_start
            i_gap = 1
        gap_end = self._dist_between_points(indices[-2], indices[-1])
        if gap_end > gap_max:
            gap_max = gap_end
            i_gap = len(indices) - 1
        # Check if line meets requirements or should be split
        if dist_max < self.min_split_dist and gap_max < self.max_line_gap:
            lines.append(line)
        else:
            i_split = i_max if dist_max >= self.min_split_dist else i_gap
            self._split(indices[:i_split], lines)
            self._split(indices[i_split:], lines)


class Line(object):
    # --------------------------------------------------------------------------
    # Class variables
    # --------------------------------------------------------------------------
    ranges = None
    bearings = None
    range_vars = None
    bearing_vars = None
    X = None
    Y = None
    least_sq_angle_thresh = None
    least_sq_radius_thresh = None
    # --------------------------------------------------------------------------
    # Constructor
    # --------------------------------------------------------------------------
    def __init__(self, indices):
        self.I = indices
        self.a = None
        self.r = None
        self.start = None
        self.end = None
        self.cov_aa = None
        self.cov_rr = None
        self.cov_ar = None
    # --------------------------------------------------------------------------
    # Public functions
    # --------------------------------------------------------------------------
    def dist_to_point(self, index):
        rad = m.sqrt(Line.X[index]**2 + Line.Y[index]**2)
        angle = m.atan2(Line.Y[index], Line.X[index])
        return abs(rad * m.cos(angle - self.a) - self.r)

    def endpoint_fit(self):
        self.start = (Line.X[self.I[0]], Line.Y[self.I[0]])
        self.end = (Line.X[self.I[-1]], Line.Y[self.I[-1]])
        self._angle_from_endpoints()
        self._radius_from_endpoints()

    def least_sq_fit(self):
        rs, bs, r_vars, b_vars = self._points_from_indices()
        Q = self._point_covariances(rs, bs, r_vars, b_vars)
        prev_radius, prev_angle = 0.0, 0.0
        while (abs(self.r - prev_radius) > Line.least_sq_radius_thresh or
               abs(self.a - prev_angle) > Line.least_sq_angle_thresh):
            prev_radius, prev_angle = self.r, self.a
            P = self._point_scalar_covariances(Q)
            self._radius_from_least_sq(rs, bs, P)
            self._angle_from_least_sq(rs, bs, r_vars, b_vars)
        self._covariance_from_least_sq(rs, bs, r_vars, b_vars)
        self._project_endpoints()

    @property
    def length(self):
        return m.sqrt((self.end[0] - self.start[0])**2 + 
                      (self.end[1] - self.start[1])**2)

    @property
    def num_points(self):
        return len(self.I)
    # --------------------------------------------------------------------------
    # Private functions
    # --------------------------------------------------------------------------
    def _angle_from_endpoints(self):
        try:
            slope = (self.end[1] - self.start[1]) / (self.end[0] - self.start[0])
        except ZeroDivisionError:
            self.a = 0
        else:
            self.a = _pi_to_pi(m.atan(slope) + m.pi/2)

    def _angle_from_least_sq(self, rs, bs, r_vars, b_vars):
        point_params = self._point_parameters(rs, bs, r_vars, b_vars)
        angle_inc = self._angle_increment(point_params)
        self.a += angle_inc

    def _angle_increment(self, point_parameters):
        c, s, a, a_p, a_pp, b, b_p, b_pp = point_parameters
        num = sum((b[k] * a_p[k] - a[k] * b_p[k]) / b[k]**2 for k in range(len(c)))
        denom = sum(((a_pp[k] * b[k] - a[k] * b_pp[k]) * b[k] - 
                      2 * (a_p[k] * b[k] - a[k] * b_p[k]) * b_p[k]) / b[k]**3
                      for k in range(len(c)))
        return -(num/denom)

    def _covariance_from_least_sq(self, rs, bs, r_vars, b_vars):
        Q = self._point_covariances(rs, bs, r_vars, b_vars)
        P = self._point_scalar_covariances(Q)
        self.cov_rr = 1.0 / sum(1.0/P_k for P_k in P)
        point_params = self._point_parameters(rs, bs, r_vars, b_vars)
        c, s, a, a_p, a_pp, b, b_p, b_pp = point_params
        G = sum(((a_pp[k] * b[k] - a[k] * b_pp[k]) * b[k] - 
                      2 * (a_p[k] * b[k] - a[k] * b_p[k]) * b_p[k]) / b[k]**3
                      for k in range(len(c)))
        self.cov_ar = self.cov_rr/G * sum(2 * d * m.sin(self.a - phi) / b_k 
                for (d, phi, b_k) in zip(rs, bs, b))
        self.cov_aa = 1/G**2 * sum(4 * d**2 * m.sin(self.a - phi)**2 / b_k 
                for (d, phi, b_k) in zip(rs, bs, b))

    def _point_covariances(self, rs, bs, r_vars, b_vars):
        Q = []
        for D, Phi, var_d, var_phi in zip(rs, bs, r_vars, b_vars):
            Q_11 = D**2 * var_phi * m.sin(Phi)**2 + var_d * m.cos(Phi)**2
            Q_12 = -D**2 * var_phi * m.sin(2*Phi) / 2 + var_d * m.sin(2*Phi) / 2
            Q_22 = D**2 * var_phi * m.cos(Phi)**2 + var_d * m.sin(Phi)**2
            Q.append((Q_11, Q_12, Q_22))
        return Q

    def _point_parameters(self, rs, bs, r_vars, b_vars):
        c = [m.cos(self.a - phi) for phi in bs]
        s = [m.sin(self.a - phi) for phi in bs]
        a = [(d * c_k - self.r)**2 for (d, c_k) in zip(rs, c)]
        a_p = [-2 * d * s_k * (d * c_k - self.r) 
                for (d, c_k, s_k) in zip(rs, c, s)]
        a_pp = [2 * d**2 * s_k**2 - 2 * d * c_k * (d * c_k - self.r)
                for (d, c_k, s_k) in zip(rs, c, s)]
        b = [var_d * c_k**2 + var_phi * d**2 * s_k**2
                for (d, c_k, s_k, var_d, var_phi) 
                in zip(rs, c, s, r_vars, b_vars)]
        b_p = [2 * (d**2 * var_phi - var_d) * c_k * s_k
                for (d, c_k, s_k, var_d, var_phi) 
                in zip(rs, c, s, r_vars, b_vars)]
        b_pp = [2 * (d**2 * var_phi - var_d) * (c_k**2 - s_k**2)
                for (d, c_k, s_k, var_d, var_phi) 
                in zip(rs, c, s, r_vars, b_vars)]
        return c, s, a, a_p, a_pp, b, b_p, b_pp

    def _point_scalar_covariances(self, Q):
        P = [Q_11 * m.cos(self.a)**2 + 2 * Q_12 * m.sin(self.a) * m.cos(self.a)
            + Q_22 * m.sin(self.a)**2 for (Q_11, Q_12, Q_22) in Q]
        return P

    def _points_from_indices(self):
        rs = [Line.ranges[i] for i in self.I]
        bs = [Line.bearings[i] for i in self.I]
        r_vars = [Line.range_vars[i] for i in self.I]
        b_vars = [Line.bearing_vars[i] for i in self.I]
        return rs, bs, r_vars, b_vars

    def _project_endpoints(self):
        s = -1/m.tan(self.a)
        b = self.r / m.sin(self.a)
        x, y = self.start
        self.start = ((s * y + x - s * b) / (s**2 + 1), 
                      (s**2 * y + s * x + b) / (s**2 + 1))
        x, y = self.end
        self.end = ((s * y + x - s * b) / (s**2 + 1),
                    (s**2 * y + s * x + b) / (s**2 + 1))

    def _radius_from_endpoints(self):
        self.r = self.start[0] * m.cos(self.a) + self.start[1] * m.sin(self.a)
        if self.r < 0:
            self.r = -self.r
            self.a = _pi_to_pi(self.a + m.pi)

    def _radius_from_least_sq(self, rs, bs, P):
        P_RR = 1.0 / sum(1.0/P_k for P_k in P)
        self.r = P_RR * sum(d * m.cos(self.a - phi) / P_k 
                            for (d, phi, P_k) in zip(rs, bs, P))


# ------------------------------------------------------------------------------
# Utility functions
# ------------------------------------------------------------------------------
def _pi_to_pi(angle):
    """Converts an angle to the interval -PI <= angle < PI."""
    angle = angle % (2*m.pi)
    if angle >= m.pi:
        angle -= 2*m.pi
    return angle
