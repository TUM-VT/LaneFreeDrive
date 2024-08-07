import pandas as pd
from itertools import product
import configparser
from pathlib import Path

experiments_name = "human_new"

# Path of the sumo config file to be used for the experiment
config_file = Path("SUMO_Runs\\ring_road_single_edge\\ring.sumocfg")

# The default ini configuration to be used as the base configuration
# default_ini_path = "D:\SUMO Work\Human in LFT\\src\\default_config\\default_config.ini"
default_ini_path = "trb_const_config.ini"

parent_folder = config_file.parent.joinpath("Experiments", experiments_name)
if parent_folder.exists() is False:
    parent_folder.mkdir(parents=True)
d_config = configparser.ConfigParser()
d_config.optionxform = str
d_config.read(default_ini_path)
with open(parent_folder.joinpath('default_lft_config.ini'), 'w') as configfile:
    d_config.write(configfile)

# Enter the variation of the variables for each section of the configuration file.
params = {"RANDOM Init:vehicle_types": ["Human_1,Human_2,Human_3,Human_5,Human_7"],
          "RANDOM Init:vehicle_counts": [5*[10], 5*[20], 5*[30], 5*[40], 5*[50], 5*[60], 5*[70], 5*[80]],
          "Strip Based Human Parameters:LaneChangeThreshold": ["0.1", "0.5", "1", "10", "20"],
          "General Parameters:seed": ["1", "2", "3"],
          }

variable_combinations = list(product(*params.values()))
df = pd.DataFrame(variable_combinations, columns=params.keys())
df["Total Vehicles"] = df["RANDOM Init:vehicle_counts"].apply(lambda x: sum(x))
df["Total Humans"] = df["Total Vehicles"]
df["Total AV"] = 0
df["RANDOM Init:vehicle_counts"] = df["RANDOM Init:vehicle_counts"].apply(lambda x: ",".join(map(str, x)))

scene = [f"df[{x}].astype(str)" for x in ["RANDOM Init:vehicle_types", "RANDOM Init:vehicle_counts",
                                         "Strip Based Human Parameters:ReactionTime",
                                         "Strip Based Human Parameters:LaneChangeThreshold"]]

scenario_names = []
config_paths = []
for _, row in df.iterrows():
    d_config = configparser.ConfigParser()
    d_config.optionxform = str
    d_config.read(default_ini_path)
    scenario = ""
    for key, value in row.items():
        if key in ["Total Vehicles", "Total Humans", "Total AV"]:
            continue
        with_comma = value.replace(",", "")
        with_comma = with_comma.replace("Human", "H")
        scenario = scenario + with_comma + "_"

        section, var = key.split(":")
        d_config[section][var] = value
    collision_file = str(parent_folder.joinpath(scenario, "collisions_file.csv"))
    d_config["General Parameters"]["collisions_file"] = collision_file

    trajectory_file = str(parent_folder.joinpath(scenario, "trajectory.csv"))
    d_config["General Parameters"]["trajectory_file"] = trajectory_file

    veh_info_file = str(parent_folder.joinpath(scenario, "vehicle_info.csv"))
    d_config["General Parameters"]["vehicle_info_file"] = veh_info_file

    strip_changes_file = str(parent_folder.joinpath(scenario, "strips_change.csv"))
    d_config["Strip Based Human Parameters"]["StripsChangeFile"] = strip_changes_file

    scenario_names.append(scenario)

    config_path = parent_folder.joinpath(scenario, "config.ini")
    if config_path.parent.exists() is False:
        config_path.parent.mkdir(parents=True)
    with open(config_path, 'w') as configfile:
        d_config.write(configfile)
    config_paths.append(str(config_path))

df["end_time"] = 4000
df["default_config"] = str(parent_folder.joinpath('default_lft_config.ini'))
df["config"] = config_paths
df["sumo_config"] = str(config_file)
df["scenario_name"] = scenario_names
assert len(df) == len(df.scenario_name.unique()), "The scenario names are not unique"
df.to_csv(parent_folder.parent.joinpath(experiments_name + ".csv"))
