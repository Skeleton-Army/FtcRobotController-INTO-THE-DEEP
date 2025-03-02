class CameraConfig:
    z = 12.5
    halfImageWidth = 320
    halfImageHeight = 180

    # Initial parameters to optimize from for local optimization
    offsetHorizontal = 0
    offsetVertical = 18
    horizontalFOV = 60
    verticalFOV = 35

# Define parameter bounds for global optimization
bounds = [(-20, 20), # offsetHorizontal
          (0, 200), # offsetVertical
          (0, 200), # horizontalFOV
          (0, 200)] # verticalFOV

# Sample test data (Replace with actual data)
sample_data = [
    {"lowest": (150, 120), "actual_x": 5.2, "actual_y": 7.3},
    {"lowest": (200, 90), "actual_x": 3.1, "actual_y": 9.0},
    {"lowest": (100, 140), "actual_x": 6.7, "actual_y": 5.5}
]

# Local optimization searches for the best solution in a nearby region and stops when it finds a local minimum.
# Global optimization searches for the absolute best solution across the entire search space, avoiding local traps.
#
# Global optimization is slower but gives the best possible result.
# For our use, global is probably better.

optimization_type = "global" # local / global