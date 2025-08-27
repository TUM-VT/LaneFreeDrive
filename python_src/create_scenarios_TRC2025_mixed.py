import pandas as pd
from itertools import product
import configparser
from pathlib import Path
from math import ceil, floor

experiments_name = "trc_2025_experiments"

# Path of the sumo config file to be used for the experiment
config_file = Path("SUMO_Runs\\ring_road_trc_2025\\ring.sumocfg")

# The default ini configuration to be used as the base configuration
default_ini_path = "SUMO_Runs\\ring_road_trc_2025\\trc_2025_default_settings.ini"

parent_folder = config_file.parent.joinpath("Experiments", experiments_name)
if parent_folder.exists() is False:
    parent_folder.mkdir(parents=True)
d_config = configparser.ConfigParser()
d_config.optionxform = str
d_config.read(default_ini_path)
with open(parent_folder.joinpath('default_lft_config.ini'), 'w') as configfile:
    d_config.write(configfile)

##############################################

# All Humans
params = {"RANDOM Init:vehicle_types": ["Human_1,Human_2,Human_3,Human_5,Human_7"],
          "RANDOM Init:vehicle_counts": [5*[10], 5*[20], 5*[30], 5*[40], 5*[50], 5*[60], 5*[70], 5*[80]],
          # This is penetration rate of CAVs
          "Penetration Rate": [0],
          "General Parameters:seed": ["1", "2", "3", "4", "5"],
          # Following parameters don't matter for HDVs
          "Vehicle Models:AV": ["PotentialLines"],
          "Adaptive Potential Lines Parameters:AdaptiveAlgorithm": ["ConstantMargin"],
          "Adaptive Potential Lines Parameters:ConstantMargin": ["40"],
          "Adaptive Potential Lines Parameters:AdaptiveFollowerDistance": ["40"],
          }

variable_combinations = list(product(*params.values()))
df_human = pd.DataFrame(variable_combinations, columns=params.keys())
df_human["Total Vehicles"] = df_human["RANDOM Init:vehicle_counts"].apply(lambda x: sum(x))

# Enter the variation of the variables for each section of the configuration file.
params = {"RANDOM Init:vehicle_types": ["AV_1,AV_2,AV_3,AV_5,AV_7"],
          "Penetration Rate": [1.0],
          "Total Vehicles": [50, 100, 150, 200, 250, 300, 350, 400],
          "General Parameters:seed": ["1", "2", "3", "4", "5"],
          "Vehicle Models:AV": ["PotentialLines"],
          "Adaptive Potential Lines Parameters:AdaptiveAlgorithm": ["ConstantMargin"],
          "Adaptive Potential Lines Parameters:ConstantMargin": ["40"],
          "Adaptive Potential Lines Parameters:AdaptiveFollowerDistance": ["40"],
          }

variable_combinations = list(product(*params.values()))
df_1 = pd.DataFrame(variable_combinations, columns=params.keys())

# Enter the variation of the variables for each section of the configuration file.
params = {"RANDOM Init:vehicle_types": ["AV_1,AV_2,AV_3,AV_5,AV_7,Human_1,Human_2,Human_3,Human_5,Human_7"],
          "Penetration Rate": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.95, 0.97, 0.99],
          "Total Vehicles": [50, 100, 150, 200, 250, 300, 350, 400],
          "General Parameters:seed": ["1", "2", "3", "4", "5"],
          "Vehicle Models:AV": ["PotentialLines"],
          "Adaptive Potential Lines Parameters:AdaptiveAlgorithm": ["ConstantMargin"],
          "Adaptive Potential Lines Parameters:ConstantMargin": ["40"],
          "Adaptive Potential Lines Parameters:AdaptiveFollowerDistance": ["40"],
          }

variable_combinations = list(product(*params.values()))
df_2 = pd.DataFrame(variable_combinations, columns=params.keys())

# Enter the variation of the variables for each section of the configuration file.
params = {"RANDOM Init:vehicle_types": ["AV_1,AV_2,AV_3,AV_5,AV_7,Human_1,Human_2,Human_3,Human_5,Human_7"],
          "Penetration Rate": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.95, 0.99],
          "Total Vehicles": [100, 200, 250, 300, 400],
          "General Parameters:seed": ["1", "2", "3", "4", "5"],
          "Vehicle Models:AV": ["AdaptivePotentialLines"],
          "Adaptive Potential Lines Parameters:AdaptiveAlgorithm": ["SVAM", "FAM"],
          "Adaptive Potential Lines Parameters:ConstantMargin": ["20"],
          "Adaptive Potential Lines Parameters:AdaptiveFollowerDistance": ["20", "40", "60"],
          }

variable_combinations = list(product(*params.values()))
df_3 = pd.DataFrame(variable_combinations, columns=params.keys())

# Enter the variation of the variables for each section of the configuration file.
params = {"RANDOM Init:vehicle_types": ["AV_1,AV_2,AV_3,AV_5,AV_7,Human_1,Human_2,Human_3,Human_5,Human_7"],
          "Penetration Rate": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.95, 0.99],
          "Total Vehicles": [100, 200, 250, 300, 400],
          "General Parameters:seed": ["1", "2", "3", "4", "5"],
          "Vehicle Models:AV": ["AdaptivePotentialLines"],
          "Adaptive Potential Lines Parameters:AdaptiveAlgorithm": ["ConstantMargin", "NSCM"],
          "Adaptive Potential Lines Parameters:ConstantMargin": ["20" ,"40", "60"],
          "Adaptive Potential Lines Parameters:AdaptiveFollowerDistance": ["40"],
          }

variable_combinations = list(product(*params.values()))
df_4 = pd.DataFrame(variable_combinations, columns=params.keys())

##############################################

df = pd.concat([df_human, df_1, df_2, df_3, df_4], ignore_index=True)

veh_counts = []
for _, row in df.iterrows():
    if row["Penetration Rate"] == 0:
        veh_counts.append(row["RANDOM Init:vehicle_counts"])
        continue

    p = row["Penetration Rate"]
    v = row["Total Vehicles"]
    v_count_av = 5*[0]
    max_av = floor(p*v)
    i = 0
    while sum(v_count_av) < max_av:
        v_count_av[i] += 1
        i = (i + 1) % 5

    v_count_human = 5 * [0]
    max_human = ceil(round((1.0 - p) * v, 2))
    i = 0
    while sum(v_count_human) < max_human:
        v_count_human[i] += 1
        i = (i + 1) % 5
    veh_counts.append(v_count_av + v_count_human)
    s = sum(veh_counts[-1])
    if sum(veh_counts[-1]) != v:
        # raise ValueError("The vehicle counts are not correct")
        print("Vehicle counts problem: ", v, p, s, max_av, max_human, max_human/v *100)
    if max_human == 0:
        print("Warning: No human vehicles in the scenario with penetration rate: ", p, " and total vehicles: ", v)

df["Total Vehicles"] = df["Total Vehicles"]
df["RANDOM Init:vehicle_counts"] = veh_counts
df["Total Humans"] = df["RANDOM Init:vehicle_counts"].apply(lambda x: sum(x[5:]))
df["Total AV"] = df["RANDOM Init:vehicle_counts"].apply(lambda x: sum(x[:5]))
df["RANDOM Init:vehicle_counts"] = df["RANDOM Init:vehicle_counts"].apply(lambda x: ",".join(map(str, x)))

scenario_names = []
config_paths = []
sync_file_names = []
for _, row in df.iterrows():
    d_config = configparser.ConfigParser()
    d_config.optionxform = str
    d_config.read(default_ini_path)
    scenario = (str(row["Penetration Rate"])
                + "_" + str(row["Total Vehicles"])
                + "_" + str(row["Vehicle Models:AV"])
                + "_" + str(row["General Parameters:seed"])
                + "_" + str(row["Adaptive Potential Lines Parameters:AdaptiveAlgorithm"])
                + "_" + str(row["Adaptive Potential Lines Parameters:AdaptiveFollowerDistance"])
                + "_" + str(row["Adaptive Potential Lines Parameters:ConstantMargin"])
                )
    for key, value in row.items():
        if key in ["Total Vehicles", "Penetration Rate", "Total Humans", "Total AV"]:
            continue
        with_comma = value.replace(",", "")
        with_comma = with_comma.replace("Human", "H")

        section, var = key.split(":")
        d_config[section][var] = value
    collision_file = str(parent_folder.joinpath(scenario, "collisions_file.csv"))
    d_config["General Parameters"]["collisions_file"] = collision_file

    trajectory_file = str(parent_folder.joinpath(scenario, "trajectory.csv"))
    #d_config["General Parameters"]["trajectory_file"] = trajectory_file
    d_config["General Parameters"]["trajectory_file"] = ""

    veh_info_file = str(parent_folder.joinpath(scenario, "vehicle_info.csv"))
    d_config["General Parameters"]["vehicle_info_file"] = veh_info_file

    lateral_speed_file = str(parent_folder.joinpath(scenario, "lateral_speed.csv"))
    d_config["General Parameters"]["lateral_speed_file"] = lateral_speed_file

    #strip_changes_file = str(parent_folder.joinpath(scenario, "strips_change.csv"))
    d_config["Strip Based Human Parameters"]["StripsChangeFile"] = ""

    sync_file = str(parent_folder.joinpath(scenario, "sync_file.csv"))
    d_config["Adaptive Potential Lines Parameters"]["SyncFile"] = ""
    sync_file_names.append(sync_file)

    scenario_names.append(scenario)

    config_path = parent_folder.joinpath(scenario, "config.ini")
    if config_path.parent.exists() is False:
        config_path.parent.mkdir(parents=True)
    with open(config_path, 'w') as configfile:
        d_config.write(configfile)
    config_paths.append(str(config_path))

df["end_time"] = 3600
df["default_config"] = str(parent_folder.joinpath('default_lft_config.ini'))
df["config"] = config_paths
df["sumo_config"] = str(config_file)
df["scenario_name"] = scenario_names
df["sync_file"] = sync_file_names
print("Total scenarios: ", len(df))
assert len(df) == len(df.scenario_name.unique()), "The scenario names are not unique"
df.to_csv(parent_folder.parent.joinpath(experiments_name + ".csv"))
