import shutil
import os
from datetime import datetime

class CopyCsvFile:
    def __init__(self, input_file):
        self.input_file = input_file

    def run(self):
        if os.path.isfile(self.input_file):
            # Get today's date in YYYY-MM-DD format
            date_str = datetime.now().strftime("%Y-%m-%d")

            # Build new filename in the same directory
            dir_name = os.path.dirname(self.input_file)
            new_filename = os.path.join(dir_name, f"{date_str}.csv")

            # Copy the file
            shutil.copy2(self.input_file, new_filename)

            print(f"File copied to: {new_filename}")

        print(f"data log {self.input_file} was not found!")

print("copying...")
copy_file = CopyCsvFile("C:/Users/User/Desktop/datalog_1.csv")
copy_file.run()
