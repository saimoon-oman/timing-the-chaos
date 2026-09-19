import os
import pandas as pd

def write_to_csv(output_path, result_df):
    if not os.path.isfile(output_path):
        result_df.to_csv(output_path, index=False)
    else:
        avg_df = pd.read_csv(output_path, header=None)
        avg_df = pd.concat([avg_df, result_df], axis=0, ignore_index=True)
        avg_df.to_csv(output_path, index=False, header=False)


def aggregate_data_across_folders(parent_folder, output_folder, list_of_files):
    # Initialize an empty dictionary to store aggregated data for each file
    aggregated_data = {}

    # Iterate through all folders in the parent folder
    for folder in os.listdir(parent_folder):
        folder_path = os.path.join(parent_folder, folder)

        # Skip if it's not a folder
        if not os.path.isdir(folder_path):
            continue

        # Iterate through all CSV files in the folder
        for file_name in os.listdir(folder_path):
            if file_name.endswith('.csv') and file_name in list_of_files:
                file_path = os.path.join(folder_path, file_name)

                # Read the CSV file into a DataFrame
                df = pd.read_csv(file_path, header=None)

                # Compute the average for each column
                average_values = df.mean(axis=0)
                std_devs = df.std(axis=0)
                # print(file_path)
                # print(average_values)
                # print(std_devs)
                
                result_df = pd.concat([average_values], axis=1, ignore_index=True).transpose()
                output_path = os.path.join(output_folder, f"avg_{file_name}")
                write_to_csv(output_path, result_df)

                result_df = pd.concat([std_devs], axis=1, ignore_index=True).transpose()
                output_path = os.path.join(output_folder, f"stddev_{file_name}")
                write_to_csv(output_path, result_df)





parent_folder =  'D:\\Projects\\DhakaSim-runner\\stat_agg_input'
output_folder = 'D:\\Projects\\DhakaSim-runner\\stat_agg_output\\'

if __name__ == "__main__":
    list_of_files = ["accident.csv", "avg_tt0.csv", "avg_tt1.csv", "avg_tt10.csv", "avg_tt11.csv", "avg_tt2.csv", "avg_tt3.csv", "avg_tt4.csv", 
                     "avg_tt5.csv", "avg_tt6.csv", "avg_tt7.csv", "avg_tt8.csv", "avg_tt9.csv", "collisions0.csv", "collisions1.csv", 
                     "collisions10.csv", "collisions11.csv", "collisions2.csv", "collisions3.csv", "collisions4.csv", "collisions5.csv",
                     "collisions6.csv", "collisions7.csv", "collisions8.csv", "collisions9.csv", "fuel0.csv", "fuel1.csv", "fuel10.csv", 
                     "fuel11.csv", "fuel2.csv", "fuel3.csv", "fuel4.csv", "fuel5.csv", "fuel6.csv", "fuel7.csv", "fuel8.csv", "fuel9.csv", 
                     "trip_complete0.csv", "trip_complete1.csv", "trip_complete10.csv", "trip_complete11.csv", "trip_complete2.csv", 
                     "trip_complete3.csv", "trip_complete4.csv", "trip_complete5.csv", "trip_complete6.csv", "trip_complete7.csv", 
                     "trip_complete8.csv", "trip_complete9.csv"]
    # for file in list_of_files:
    #     output_path = os.path.join(output_folder, f"avg_{file}")


    aggregate_data_across_folders(parent_folder, output_folder, list_of_files)


