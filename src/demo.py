import os
import json
import matplotlib.pyplot as plt
from laser_line_extraction.line_extractor import LineExtractor, Line

le = LineExtractor()

# Load data
this_directory = os.path.dirname(os.path.realpath(__file__))
scans = []
for i in range(1,7):
    filename = "laser_scan_" + str(i) + ".json"
    with open(os.path.join(this_directory, 'data', filename), 'r') as f:
        scans.append(json.load(f))


# Extract lines and plot
sub = 231
fig = plt.figure()
for scan in scans:
    # Make up some range and bearing variances
    r_vars = [(0.01 * r)**2 for r in scan['ranges']] # 1% of range
    b_vars = [0.0001**2 for b in scan['bearings']] 
    # Set the data and extract lines
    le.set_data(scan['ranges'], scan['bearings'], r_vars, b_vars)
    lines = le.run()
    ax = fig.add_subplot(sub)
    ax.axis('equal')
    for line in lines:
        ax.plot([line.start[0], line.end[0]],
                [line.start[1], line.end[1]], 'r')
    # Plot the raw points (different marker for filtered vs unfiltered)
    filtered_indices = list(set(range(len(scan['ranges']))) - 
                            set(le.indices))
    x_filtered = [Line.X[i] for i in filtered_indices]
    y_filtered = [Line.Y[i] for i in filtered_indices]
    ax.scatter(list(x_filtered), list(y_filtered), marker='o', 
            edgecolors='k', facecolors='none', s=30)
    x_unfiltered = [Line.X[i] for i in le.indices]
    y_unfiltered = [Line.Y[i] for i in le.indices]
    ax.scatter(list(x_unfiltered), list(y_unfiltered), marker='x',
            edgecolors='g', s=30)
    sub += 1
fig.tight_layout()

plt.show()

