import matplotlib.pyplot as plt
import re
 
# The sequence string from the PRINT command output
sequence = '''
50.0 0.0 0.0
SC_3_Cam
180.0
CC_3_Cam
50.0 100.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
50.0 200.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
150.0 0.0 0.0
SC_3_Cam
0.0
CC_3_Cam
150.0 100.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
150.0 200.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
150.0 0.0 0.0
SC_3_Cam
180.0
CC_3_Cam
150.0 100.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
150.0 200.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
250.0 0.0 0.0
SC_3_Cam
0.0
CC_3_Cam
250.0 100.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
250.0 200.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
250.0 0.0 0.0
SC_3_Cam
180.0
CC_3_Cam
250.0 100.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
250.0 200.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
350.0 0.0 0.0
SC_3_Cam
0.0
CC_3_Cam
350.0 100.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
350.0 200.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
350.0 0.0 0.0
SC_3_Cam
180.0
CC_3_Cam
350.0 100.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
350.0 200.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
450.0 0.0 0.0
SC_3_Cam
0.0
CC_3_Cam
450.0 100.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
450.0 200.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
450.0 0.0 0.0
SC_3_Cam
180.0
CC_3_Cam
450.0 100.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
450.0 200.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
550.0 0.0 0.0
SC_3_Cam
0.0
CC_3_Cam
550.0 100.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
550.0 200.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
550.0 0.0 0.0
SC_3_Cam
180.0
CC_3_Cam
550.0 100.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
550.0 200.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
650.0 0.0 0.0
SC_3_Cam
0.0
CC_3_Cam
650.0 100.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
650.0 200.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
650.0 0.0 0.0
SC_3_Cam
180.0
CC_3_Cam
650.0 100.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
650.0 200.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
750.0 0.0 0.0
SC_3_Cam
0.0
CC_3_Cam
750.0 100.0 0.0
VC_3_Cam
M_CAM_TAKE
CC_3_Cam
750.0 200.0 0.0
VC_3_Cam
M_CAM_TAKE
'''
 
# Extract coordinates from the sequence
coords = re.findall(r'CC_3_Cam\n([0-9.]+) ([0-9.]+) ([0-9.]+)', sequence)
 
# Convert to list of tuples with floats
coords = [(float(x), float(y), float(z)) for x, y, z in coords]
 
# Plotting
plt.figure(figsize=(12, 6))
x_coords = [x for x, y, z in coords]
y_coords = [y for x, y, z in coords]
plt.plot(x_coords, y_coords, '-o', markersize=5)
plt.scatter(x_coords, y_coords, color='red')
 
# Adding labels and titles
for i, (x, y, z) in enumerate(coords):
    plt.text(x, y, f'({x},{y})', fontsize=9, ha='right')
 
plt.title('Camera Movement Path')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.grid(True)
plt.show()