import subprocess
from pathlib import Path
import configparser
import pandas as pd
import traci
import os

def set_margin_file(config_path, traci):
    """ Sets the python margin file for Python RL if it is not found at the given location. """
    config = configparser.ConfigParser()
    config.optionxform = str
    config.read(config_path)
    margin_file_path = config["Adaptive Potential Lines Parameters"]["PythonMarginInputFile"]
    if margin_file_path != "":
        margin_file_path = Path(margin_file_path)
        if margin_file_path.exists() is False:
            print(f"APL RL Margin file {margin_file_path} not found. Generating a default margin file.")
            vehicle_ids = traci.vehicle.getIDList()
            df = pd.DataFrame(vehicle_ids, columns=["VehicleID"])
            df["Margin"] = 40
            df.to_csv(margin_file_path, index=False, header=False)

def start_sumo_simulation(sumo_cfg_path, default_lft_config_path,  lft_config_path, end_time,
                          relative_output_path, port=8813) -> subprocess.Popen:

    python_src_path = Path().absolute().parent
    # Set your SUMO binary path and configuration file
    sumo_binary = python_src_path.joinpath("SUMO\\bin\\sumo-gui.exe")
    lft_plugin_path = python_src_path.joinpath("x64-Debug")

    # Create a dictionary for custom environment variables
    my_env = os.environ.copy()
    my_env["PATH"] += ";" + str(lft_plugin_path)
    my_env["CONFIG_FILE"] = str(python_src_path.absolute().joinpath(lft_config_path))
    my_env["DEFAULT_CONFIG_FILE"] = str(python_src_path.absolute().joinpath(default_lft_config_path))
    my_env["SUMO_HOME"] = str(python_src_path.absolute().joinpath("SUMO"))

    # Compose the command line
    sumo_cmd = [str(sumo_binary), "-c", sumo_cfg_path, "--end", str(end_time),
                "--remote-port", str(port),
                "--start",
                "--output-prefix", relative_output_path + "\\"]

    # Start the SUMO process with custom environment
    output_file_path = Path(sumo_cfg_path).parent.joinpath(relative_output_path, "lft_output.txt")
    if output_file_path.parent.exists() is False:
        output_file_path.parent.mkdir()
    with open(output_file_path, 'w') as f:
        process = subprocess.Popen(sumo_cmd, env=my_env, stdout=f)


    traci.init(port=port)
    traci.simulationStep()
    current_time = traci.simulation.getTime()
    set_margin_file(my_env["CONFIG_FILE"], traci)
    print(current_time)
    while current_time < end_time:
        traci.simulationStep()
        current_time = traci.simulation.getTime()
    # Close the TraCI connection and stop the simulation
    traci.close()
    return process


if __name__ == "__main__":

    python_src_path = Path().absolute().parent
    csv_path = python_src_path.joinpath("SUMO_Runs", "example_ring_road", "Experiments", "example_rl_study.csv")
    row_num = 1
    sumo_port = 8814

    csv_df = pd.read_csv(csv_path)
    row = csv_df.iloc[row_num]

    lft_default_config = row["default_config"]
    lft_config = python_src_path.joinpath(row["config"])
    sumo_config = python_src_path.joinpath(row["sumo_config"])
    end_time = int(row["end_time"])
    results_relative_path = lft_config.relative_to(sumo_config.parent).parent
    process = start_sumo_simulation(str(sumo_config), lft_default_config, str(lft_config), end_time,
                                    str(results_relative_path), sumo_port)
    # kill the process to close sumo
    process.kill()
