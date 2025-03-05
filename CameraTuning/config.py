class CameraConfig:
    z = 12.5
    halfImageWidth = 320
    halfImageHeight = 180

    # Initial parameters to optimize from for local optimization
    offsetHorizontal = 0
    offsetVertical = 20
    horizontalFOV = 65.76
    verticalFOV = 39.88

# Define parameter bounds for global optimization
bounds = [
        (-20, 20), # offsetHorizontal
        (0, 200), # offsetVertical
        # (0, 200), # horizontalFOV
        # (0, 200) # verticalFOV
]

# Sample test data (Replace with actual data)
sample_data = [
    {"lowest": (334, 235), "actual_x": 0, "actual_y": 24},
    {"lowest": (90, 200), "actual_x": 14.5, "actual_y": 28.5},
    {"lowest": (205, 333), "actual_x": 5, "actual_y": 14.5},
    {"lowest": (66, 313), "actual_x": 11, "actual_y": 16.6},
    {"lowest": (81, 184), "actual_x": 16.7, "actual_y": 31},
    {"lowest": (190, 220), "actual_x": 7.5, "actual_y": 26},
]

# Local optimization searches for the best solution in a nearby region and stops when it finds a local minimum.
# Global optimization searches for the absolute best solution across the entire search space, avoiding local traps.
#
# Global optimization is slower but gives the best possible result.
# For our use, global is probably better.

optimization_type = "global" # local / global