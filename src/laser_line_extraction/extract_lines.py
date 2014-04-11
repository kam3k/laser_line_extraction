import math as m
import line_tools as lt
import config as c

RANGES = None
BEARINGS = None

################################################################################
# Public functions
################################################################################

def run():
    if RANGES is None:
        raise UnboundLocalError("Must call set_data(ranges, bearings) before " +
                "calling run()")
    # 'indices' is the list of unfiltered points
    indices = range(len(RANGES))
    # Filter points (minimum range, outliers)
    indices = _filter_close_points()
    indices = _filter_outlier_points(indices)
    # Group potential lines with recursive splitting
    index_groups = []
    _split(indices, index_groups)
    # Generate endpoint-fitted lines from index groups
    line_list_endpoint = []
    for group in index_groups:
        line = lt.Line(group)
        lt.endpoint_fit(line)
        line_list_endpoint.append(line)
    # Filter endpoint lines (minimum length, minimum number of points)
    line_list_endpoint = _filter_short_lines(line_list_endpoint)
    line_list_endpoint = _filter_sparse_lines(line_list_endpoint)
    # Fit least squares lines to endpoint lines
    line_list_leastsq = []
    for line in line_list_endpoint:
        lt.least_squares_fit(line)
        line_list_leastsq.append(line)
    # Merge colinear lines
    line_list_leastsq = _merge(line_list_leastsq)
    return line_list_leastsq

def set_data(ranges, bearings):
    global RANGES, BEARINGS
    RANGES = ranges
    BEARINGS = bearings
    # Set the ranges, bearings, x-coordinates, and y-coordinates in line tools
    X, Y = _rb_to_xy()
    lt.set_data(ranges, bearings, X, Y)

################################################################################
# Private functions
################################################################################

def _filter_close_points():
    return [i for i, r in enumerate(RANGES) if r >= c.MIN_POINT_RANGE]

def _filter_outlier_points(indices):
    new_indices = []
    for i, p_i in enumerate(indices):
        if i == 0: # first point
            p_j = indices[i+1]
            p_k = indices[i+2]
        elif i == len(indices) - 1: # last point
            p_j = indices[i-1]
            p_k = indices[i-2]
        else: # middle points
            p_j = indices[i-1]
            p_k = indices[i+1]
        if (abs(RANGES[p_i] - RANGES[p_j]) <= c.OUTLIER_RANGE_DIFF or
            abs(RANGES[p_i] - RANGES[p_k]) <= c.OUTLIER_RANGE_DIFF): 
            new_indices.append(p_i)
    return new_indices

def _filter_short_lines(line_list):
    return [line for line in line_list if lt.length(line) >= c.MIN_LINE_LENGTH]

def _filter_sparse_lines(line_list):
    return [line for line in line_list if lt.num_points(line) >= c.MIN_LINE_POINTS]

def _merge(line_list):
    return line_list

def _rb_to_xy():
    X = [r * m.cos(b) for r, b in zip(RANGES, BEARINGS)]
    Y = [r * m.sin(b) for r, b in zip(RANGES, BEARINGS)]
    return X, Y

def _split(indices, index_groups):
    line = lt.Line(indices)
    lt.endpoint_fit(line)
    dist_max = 0
    for i in range(1, len(indices)-1):
        dist = lt.dist_to_point(line, indices[i])
        if dist > dist_max:
            dist_max = dist
            i_max = i
    if dist_max < c.MAX_SPLIT_POINT_DIST:
        index_groups.append(indices)
    else:
        _split(indices[:i_max], index_groups)
        _split(indices[i_max:], index_groups)

################################################################################
# Visualize logged data if run as script
################################################################################

if __name__ == '__main__':
    import os
    import json
    import matplotlib.pyplot as plt

    this_directory = os.path.dirname(os.path.realpath(__file__))
    with open(os.path.join(this_directory, 'data', 'laser_scan_2.txt'), 'r') as f:
        scan = json.load(f)

    set_data(scan['ranges'], scan['bearings'])
    line_list = run()

    plt.plot(lt.X, lt.Y, 'kx')
    for line in line_list:
        plt.plot([line.start[0], line.end[0]], [line.start[1], line.end[1]], 'r')
    plt.show()
