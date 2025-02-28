import numpy as np
from scipy.optimize import minimize

class CameraConfig:
    z = 12.5
    halfImageWidth = 320
    halfImageHeight = 180

    # Initial parameters to optimize from
    offsetHorizontal = 0
    offsetVertical = 18
    horizontalFOV = 60
    verticalFOV = 35

# Sample test data (Replace with actual data)
sample_data = [
    {"lowest": (150, 120), "actual_x": 5.2, "actual_y": 7.3},
    {"lowest": (200, 90), "actual_x": 3.1, "actual_y": 9.0},
    {"lowest": (100, 140), "actual_x": 6.7, "actual_y": 5.5}
]

# Function to calculate sample position based on lowest point
def calculate_sample_position(offset_horizontal, offset_vertical, horizontal_fov, vertical_fov, lowest):
    h_over_width = horizontal_fov / (CameraConfig.halfImageWidth * 2)
    v_over_height = vertical_fov / (CameraConfig.halfImageHeight * 2)

    horizontal_angle = np.radians((CameraConfig.halfImageWidth - lowest[0]) * h_over_width + offset_horizontal)
    sample_y = CameraConfig.z / np.tan(np.radians((lowest[1] - CameraConfig.halfImageHeight) * v_over_height + offset_vertical))
    sample_x = np.tan(horizontal_angle) * sample_y

    return sample_x, sample_y

# Error function: Difference between calculated and actual positions
def error_function(params, data):
    offset_horizontal, offset_vertical, horizontal_fov, vertical_fov = params
    total_error = 0

    for sample in data:
        calc_x, calc_y = calculate_sample_position(offset_horizontal, offset_vertical, horizontal_fov, vertical_fov, sample["lowest"])
        actual_x, actual_y = sample["actual_x"], sample["actual_y"]

        error = np.sqrt((calc_x - actual_x) ** 2 + (calc_y - actual_y) ** 2)  # Euclidean distance error
        total_error += error

    return total_error  # Goal is to minimize this error

# Initial guesses for parameters
initial_params = np.array([CameraConfig.offsetHorizontal, CameraConfig.offsetVertical, CameraConfig.horizontalFOV, CameraConfig.verticalFOV], dtype=float)

# Run optimization
result = minimize(error_function, initial_params, args=(sample_data,), method='Nelder-Mead')

# Print optimized values
optimal_offsetHorizontal, optimal_offsetVertical, optimal_horizontalFOV, optimal_verticalFOV = result.x

print("Optimized Parameters:")
print(f"offsetHorizontal: {optimal_offsetHorizontal}")
print(f"offsetVertical: {optimal_offsetVertical}")
print(f"horizontalFOV: {optimal_horizontalFOV}")
print(f"verticalFOV: {optimal_verticalFOV}")