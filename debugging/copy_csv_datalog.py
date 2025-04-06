import shutil
import os
from datetime import datetime

# Prompt the user to enter the path to the CSV file
input_file = input("Enter the path to the CSV file: ")

# Check if the file exists
if not os.path.isfile(input_file):
    print("File not found!")
    exit(1)

# Get today's date in YYYY-MM-DD format
date_str = datetime.now().strftime("%Y-%m-%d")

# Build new filename in the same directory
dir_name = os.path.dirname(input_file)
new_filename = os.path.join(dir_name, f"{date_str}.csv")

# Copy the file
shutil.copy2(input_file, new_filename)

print(f"File copied to: {new_filename}")
