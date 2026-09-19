from datetime import datetime
import subprocess
import shutil
import stats_avg_calc

base_stat_folder = "stat_avg"

primes = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109, 113, 
          127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193, 197, 199, 211, 223, 227, 229, 233, 239, 241, 251, 
          257, 263, 269, 271, 277, 281, 283, 293, 307, 311, 313, 317, 331, 337, 347, 349, 353, 359, 367, 373, 379, 383, 389, 397, 
          401, 409, 419, 421, 431, 433, 439, 443, 449, 457, 461, 463, 467, 479, 487, 491, 499, 503, 509, 521, 523, 541, 547, 557, 
          563, 569, 571, 577, 587, 593, 599]

def run_jar_once():
    now = datetime.now()

    start_time = now.strftime("%H:%M:%S")
    print("Start Time =", start_time)

    result = subprocess.run(['java', '-jar', 'Dhakasim.jar'], capture_output=True,text=True)

    now = datetime.now()

    end_time = now.strftime("%H:%M:%S")
    print("End Time =", end_time)
    return result

def change_parameter(parameter_name, parameter_value):
    temp = []
    with open("input/parameter.txt", "r") as inputf:
        for line in inputf:
            line = line.strip("\n")
            match = line.startswith(parameter_name)
            if match:
                list1 = line.split(" ")
                # current_seed = int(list1[1])
                # list1[1] = current_seed + 1
                line = str(list1[0]) + " " + str(parameter_value)
            temp.append(line)
    inputf.close()

# overwrite original file

    with open("input/parameter.txt", "w") as outputf:
        for item in temp:
            outputf.write(item + "\n")
    outputf.close()
    
def run_changing_seed():
    results = []
    seeds = primes[0:50]

    for seed in seeds:
        change_parameter("RandomSeed", seed)
        print("Seed " + str(seed))
        results.append(run_jar_once())

    change_parameter("RandomSeed", 0)

    with open("statistics/logs.txt", "w") as outputf:
            outputf.write(str(results))
    outputf.close()

def change_CF_model():
    for i in range(9, 12):
        change_parameter("CF_model", i)
        run_changing_seed()
        stats_avg_calc.rewrite_stats()
        new_stat_folder = "all_stats/CF_" + str(i) + "/"
        shutil.move(base_stat_folder, new_stat_folder)
        # shutil.copy("input", new_stat_folder)
        shutil.move("statistics", "statistics_" + str(i))
        print("CF_" + str(i) + " done")



if __name__ == "__main__":
    # run_changing_seed()
    change_CF_model()
    
