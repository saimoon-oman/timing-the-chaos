import os
import pandas as pd

# Function to calculate average and standard deviation of rows
def calculate_stats(file_path):
    # Read the CSV file without header
    df = pd.read_csv(file_path, header=None)

    # Calculate average and standard deviation for each column
    averages = df.mean(axis=0)
    std_devs = df.std(axis=0)

    # Transpose the result to have the statistics as new rows
    result_df = pd.concat([averages, std_devs], axis=1).transpose()

    # Append the result as a new row
    df = pd.concat([df, result_df], axis=0, ignore_index=True)

    return df


def rewrite_stats():
    # Folder containing CSV files
    folder_path = 'D:\\Projects\\DhakaSim-runner\\statistics'

    # Output folder
    output_folder = 'D:\\Projects\\DhakaSim-runner\\stat_avg\\'

    # Iterate through each file in the folder
    for filename in os.listdir(folder_path):
        if filename.endswith(".csv"):
            file_path = os.path.join(folder_path, filename)

            # Calculate statistics for the current file
            result_df = calculate_stats(file_path)

            # Save the result to a new CSV file in the output folder
            os.makedirs(output_folder, exist_ok = True)
            output_path = os.path.join(output_folder, f"stats_{filename}")
            result_df.to_csv(output_path, index=False)

    print("Processing complete.")

if __name__ == "__main__":
    rewrite_stats()
    