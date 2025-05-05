import os
import subprocess
from pathlib import Path
from tqdm import tqdm
from time import sleep
import pandas as pd
import sys
import xml.etree.ElementTree as ET
import traceback


def start_sumo_simulation(sumo_cfg_path, default_lft_config_path,  lft_config_path, end_time,
                          relative_output_path) -> subprocess.Popen:
    # Set your SUMO binary path and configuration file
    sumo_binary = Path("SUMO\\bin\\sumo-gui.exe")
    lft_plugin_path = "x64-Debug"

    # Create a dictionary for custom environment variables
    my_env = os.environ.copy()
    my_env["PATH"] += ";" + lft_plugin_path
    my_env["CONFIG_FILE"] = str(Path().absolute().joinpath(lft_config_path))
    my_env["DEFAULT_CONFIG_FILE"] = str(Path().absolute().joinpath(default_lft_config_path))
    my_env["SUMO_HOME"] = str(Path().absolute().joinpath("SUMO"))

    # Compose the command line
    sumo_cmd = [str(sumo_binary), "-c", sumo_cfg_path, "--end", str(end_time), "--start",
                "--output-prefix", relative_output_path + "\\"]

    # Start the SUMO process with custom environment
    output_file_path = Path(sumo_cfg_path).parent.joinpath(relative_output_path, "lft_output.txt")
    if output_file_path.parent.exists() is False:
        output_file_path.parent.mkdir()
    with open(output_file_path, 'w') as f:
        process = subprocess.Popen(sumo_cmd, env=my_env, stdout=f)
    return process


def run_from_csv(csv_path, num_parallel=1):
    df = pd.read_csv(csv_path)
    remaining_inx = df.index.to_list()
    completed = []
    running_processes: dict[int, subprocess.Popen] = {}
    with tqdm(total=len(df)) as pbar:
        while (len(completed) < len(df)):
            if len(running_processes) < num_parallel and len(remaining_inx) > 0:
                inx = remaining_inx.pop(0)
                row = df.iloc[inx]
                lft_default_config = row["default_config"]
                lft_config = Path(row["config"])
                sumo_config = Path(row["sumo_config"])
                end_time = int(row["end_time"])
                results_relative_path = lft_config.relative_to(sumo_config.parent).parent
                process = start_sumo_simulation(str(sumo_config), lft_default_config, str(lft_config), end_time,
                                                str(results_relative_path))
                running_processes[inx] = process

            to_be_removed = []
            for inx, p in running_processes.items():
                if p is None:
                    to_be_removed.append(inx)
                    completed.append(inx)
                    pbar.update(1)
                elif p.poll() is not None:
                    to_be_removed.append(inx)
                    completed.append(inx)
                    pbar.update(1)
            for inx in to_be_removed:
                del running_processes[inx]
            sleep(0.001)
            pbar.set_postfix({"running": len(running_processes)})


if __name__ == "__main__":
    # Get the csv file path and the number of parallel simulations from command line arguments
    if len(sys.argv) != 3:
        print("Usage: python parallel_run.py <csv_path> <num_parallel>")
        exit(1)
    csv_path = Path(sys.argv[1])
    num_parallel = int(sys.argv[2])

    # Start the simulations
    run_from_csv(str(csv_path), num_parallel=num_parallel)
