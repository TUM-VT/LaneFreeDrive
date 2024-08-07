import pandas as pd
from itertools import product
import configparser
from pathlib import Path
from math import ceil, floor

experiments_name = "mixed_new"

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
params = {"RANDOM Init:vehicle_types": ["AV_1,AV_2,AV_3,AV_5,AV_7,Human_1,Human_2,Human_3,Human_5,Human_7"],
          "Penetration Rate": [0.1, 0.2, 0.4, 0.5, 0.6, 0.8, 0.9, 0.95, 0.97, 0.99],
          "Total Vehicles": [50, 100, 150, 200, 250, 300, 350, 400],
          "Strip Based Human Parameters:LaneChangeThreshold": ["10"],
          "General Parameters:seed": ["1", "2", "3"],
          }

variable_combinations = list(product(*params.values()))
df = pd.DataFrame(variable_combinations, columns=params.keys())

veh_counts = []
for _, row in df.iterrows():
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
    print(v, s, max_av, max_human, max_human/v *100)
    if sum(veh_counts[-1]) != v:
        raise ValueError("The vehicle counts are not correct")
    if max_human == 0:
        raise ValueError("The human count is zero")

df["Total Vehicles"] = df["Total Vehicles"]
df["RANDOM Init:vehicle_counts"] = veh_counts
df["Total Humans"] = df["RANDOM Init:vehicle_counts"].apply(lambda x: sum(x[5:]))
df["Total AV"] = df["RANDOM Init:vehicle_counts"].apply(lambda x: sum(x[:5]))
df["RANDOM Init:vehicle_counts"] = df["RANDOM Init:vehicle_counts"].apply(lambda x: ",".join(map(str, x)))

scenario_names = []
config_paths = []
for _, row in df.iterrows():
    d_config = configparser.ConfigParser()
    d_config.optionxform = str
    d_config.read(default_ini_path)
    scenario = (str(row["Penetration Rate"])
                + "_" + str(row["Total Vehicles"])
                + "_" + str(row["Strip Based Human Parameters:LaneChangeThreshold"])
                + "_" + str(row["General Parameters:seed"])
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
