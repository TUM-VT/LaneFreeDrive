import numpy as np
import pandas as pd
from itertools import product
import configparser
from pathlib import Path
from math import ceil, floor

experiments_name = "example_study"

# Path of the sumo config file to be used for the experiment. Change it for a different SUMO configuration.
sumo_config_file = Path("SUMO_Runs\\example_ring_road\\ring.sumocfg")

# The default ini configuration to be used as the base configuration. By default, it takes it from the src folder.
# You can change it if a different default configuration is needed.
default_ini_path = Path().absolute().parent.joinpath("src", "lft_plugin_config_files", "default_config.ini")

# Create the main study folder and copy the default config file.
parent_folder = sumo_config_file.parent.joinpath("Experiments", experiments_name)
if parent_folder.exists() is False:
    parent_folder.mkdir(parents=True)
d_config = configparser.ConfigParser()
d_config.optionxform = str
d_config.read(default_ini_path)
with open(parent_folder.joinpath('default_config.ini'), 'w') as configfile:
    d_config.write(configfile)


######## Scenario creation ########
# Create scenarios for the experiment. Each scenario will have different values for some simulation parameters and hence
# a separate config.ini in its folder.

# Define the parameters that are different from the default configuration file.

# Note1: The vehicle types, for example, AV_1, AV_2, Human_1 etc, are defined in the sumo route (i.e. xyz.rou.xml) file.
# You can find it in the same folder as the sumo_config_file

# Note2: In the following dictionary keys for parameters, the values before ":" refer to the section in the configu.ini
# file.
params = {"RANDOM Init:vehicle_types": ["AV_1,AV_2,AV_3,AV_4", "AV_1,AV_2,Human_1,Human_2"],
          "RANDOM Init:vehicle_counts": [(15,15,15,15), (30,30,30,30)],
          "General Parameters:seed": ["1"],
          "General Parameters:circular_movement": ["1"],
          "General Parameters:vehicle_init": ["RANDOM"],
          "General Parameters:speed_dist": ["UNIFORM"],
          "Potential Lines Parameters:PLForceModel": ["UNIFORM"],
          "Strip Based Human Parameters:LaneChangeThreshold": ["10"],
          }

variable_combinations = list(product(*params.values()))
df = pd.DataFrame(variable_combinations, columns=params.keys())
df["Total Vehicles"] = df["RANDOM Init:vehicle_counts"].apply(lambda x: sum(x))
# Convert the vehicle counts to comma separated string
df["RANDOM Init:vehicle_counts"] = df["RANDOM Init:vehicle_counts"].apply(lambda x: ",".join(map(str, x)))

scenario_names = []
config_paths = []
for _, row in df.iterrows():
    d_config = configparser.ConfigParser()
    d_config.optionxform = str
    d_config.read(default_ini_path)
    # Scenario name should be unique as the individual experiment folders are created based on it. One way of doing it
    # is to use some parameter value that changes in each scene
    scenario = (str(row["Total Vehicles"])
                + "_" + str(row["RANDOM Init:vehicle_types"])
                + "_" + str(row["General Parameters:seed"])
                )
    for key, value in row.items():
        if ":" not in key:
            continue
        section, var = key.split(":")
        d_config[section][var] = value
    collision_file = str(parent_folder.joinpath(scenario, "collisions_file.csv"))
    d_config["General Parameters"]["collisions_file"] = collision_file

    trajectory_file = str(parent_folder.joinpath(scenario, "trajectory.csv"))
    d_config["General Parameters"]["trajectory_file"] = ""

    veh_info_file = str(parent_folder.joinpath(scenario, "vehicle_info.csv"))
    d_config["General Parameters"]["vehicle_info_file"] = veh_info_file

    lateral_speed_file = str(parent_folder.joinpath(scenario, "lateral_speed.csv"))
    d_config["General Parameters"]["lateral_speed_file"] = lateral_speed_file

    scenario_names.append(scenario)

    # Wrtie the modified config.ini for the scenario
    config_path = parent_folder.joinpath(scenario, "config.ini")
    if config_path.parent.exists() is False:
        config_path.parent.mkdir(parents=True)
    with open(config_path, 'w') as configfile:
        d_config.write(configfile)
    config_paths.append(str(config_path))

df["end_time"] = 3600
df["default_config"] = str(parent_folder.joinpath('default_config.ini'))
df["config"] = config_paths
df["sumo_config"] = str(sumo_config_file)
df["scenario_name"] = scenario_names
print("Total scenarios: ", len(df))
assert len(df) == len(df.scenario_name.unique()), "The scenario names are not unique"
df.to_csv(parent_folder.parent.joinpath(experiments_name + ".csv"))
