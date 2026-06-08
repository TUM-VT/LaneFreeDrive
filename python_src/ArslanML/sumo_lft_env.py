import subprocess
from pathlib import Path
import sys
import configparser
import os
from ray.rllib.env.multi_agent_env import MultiAgentEnv
import numpy as np
from helper_functions import detector_xml_to_dataframe
import pandas as pd

sumo_path = r"D:\Work\SUMO Work\Human in LFT\python_src\SUMO"
sys.path.append(sumo_path + r"\tools")

import traci
import traci.constants as tc


class Vehicle:
    def __init__(self, veh_id, length, width):
        """ A class to save the attributes of each vehicle via traci. """
        self.veh_id = veh_id
        self.length = length
        self.width = width
        self.x = None
        self.y = None
        self.vx = None
        self.vy = None
        self.ax = None
        self.ay = None

class SUMOMultiAgentEnv(MultiAgentEnv):

    next_index = 1
    def __init__(self, config):
        super().__init__()
        self.episode_num = -1

        self.sumo_home_path = config["sumo_home_path"]
        self.sumo_cfg_path = config["sumo_cfg_path"]
        self.end_time = config["end_time"]

        self.my_env = os.environ.copy()
        self.my_env["PATH"] += ";" + str(config["lft_plugin_path"])
        self.my_env["CONFIG_FILE"] = str(config["lft_config_path"])
        self.my_env["DEFAULT_CONFIG_FILE"] = str(config["default_lft_config_path"])
        self.my_env["SUMO_HOME"] = str(config["sumo_home_path"])

        self.run_index = SUMOMultiAgentEnv.next_index
        SUMOMultiAgentEnv.next_index += 1
        self.worker_string = f"worker_{config.worker_index}_{self.run_index}"

        self.lft_config = self._read_config()
        self.output_folder = str(Path(config["lft_config_path"]).parent) + "_" + self.worker_string
        self.output_folder = Path(self.output_folder)
        self.relative_output_path = str(self.output_folder.relative_to(Path(self.sumo_cfg_path).parent))
        # Adjust the configuration parameter to be in the output folder
        for section in self.lft_config.sections():
            for key, value in self.lft_config.items(section):
                if ("file" in key or "File" in key) and value != "":
                    file_name = Path(value).name
                    new_file = str(self.output_folder.joinpath(file_name))
                    self.lft_config.set(section, key, new_file)
                    # print(f"Worker {config.worker_index}: seting {key} to {new_file}")
        self.config_path = self.output_folder.joinpath("config.ini")
        self._write_config()
        self.my_env["CONFIG_FILE"] = str(self.config_path)

        self.traci_port = int(config["traci_port"]) + config.worker_index * 20 + self.run_index
        self.traci_label = f"sim_{self.traci_port}"
        self.traci_conn = None
        # Dictionary for vehicle information
        self.veh_attributes: dict[str, Vehicle] = {}
        self.time_step_size = None
        self.detector_ids = []

        self.running_process = None

    def _read_config(self):
        """ Reads the LFT config file """
        d_config = configparser.ConfigParser()
        d_config.optionxform = str
        d_config.read(self.my_env["DEFAULT_CONFIG_FILE"])

        cur_config = configparser.ConfigParser()
        cur_config.optionxform = str
        cur_config.read(self.my_env["CONFIG_FILE"])

        # Overwrite the default config parameters with the ones in the current config file
        for section in d_config.sections():
            for key, value in d_config.items(section):
                if cur_config.has_option(section, key):
                    d_config.set(section, key, cur_config.get(section, key))
        return d_config

    def read_detector_files(self) -> dict[str, pd.DataFrame]:
        """ Reads the xml files of detectors from the output folder """
        detector_dfs = {}
        for detector in self.detector_ids:
            file_path = self.output_folder.joinpath(f"{detector}.xml")
            if file_path.exists():
                detector_dfs[detector] = detector_xml_to_dataframe(file_path)
            else:
                print(f"Warning: Detector file {file_path} does not exist.")
        return detector_dfs

    def _write_config(self):
        """ Writes the LFT config file to the given path. """
        if not Path(self.config_path).exists():
            Path(self.config_path).parent.mkdir(parents=True)
        with open(self.config_path, 'w') as configfile:
            self.lft_config.write(configfile)

    def reset_sim_track(self):
        self.time_step_size = None
        self.veh_attributes = {}
        self.detector_ids = []

    def update_veh_info(self):
        if self.time_step_size is None:
            self.time_step_size = traci.simulation.getDeltaT()

        """ Gets the necessary information from traci to construct the observations for each agent. """
        all_vehicles = traci.vehicle.getIDList()
        # Remove the vehicles
        rem_veh = set(self.veh_attributes.keys()) - set(all_vehicles)
        for veh_id in rem_veh:
            del self.veh_attributes[veh_id]

        for veh_id in all_vehicles:
            if veh_id not in self.veh_attributes:
                length = traci.vehicle.getLength(veh_id)
                width = traci.vehicle.getWidth(veh_id)
                self.veh_attributes[veh_id] = Vehicle(veh_id, length, width)
                # subscribe to vehicle to get the information
                traci.vehicle.subscribe(veh_id, [tc.VAR_POSITION, tc.VAR_SPEED, tc.VAR_SPEED_LAT, tc.VAR_ACCELERATION])

        # Get the subscription information and update the vehicle attributes
        all_info = traci.vehicle.getAllSubscriptionResults()
        for vid, veh in self.veh_attributes.items():
            veh.x, veh.y = all_info[vid][tc.VAR_POSITION]
            old_vy = veh.vy
            veh.vx, veh.vy = all_info[vid][tc.VAR_SPEED], all_info[vid][tc.VAR_SPEED_LAT]
            veh.ax = all_info[vid][tc.VAR_ACCELERATION]
            veh.ay = 0 if old_vy is None else (veh.vy - old_vy) / self.time_step_size

        return rem_veh

    def modify_lft_config(self):
        sumo_seed = str(np.random.randint(1e6))
        self.lft_config["General Parameters"]["seed"] = sumo_seed
        print(f"{self.worker_string} Episode {self.episode_num}: setting sumo lft seed {sumo_seed}")
        self._write_config()

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self.episode_num += 1
        self.modify_lft_config()
        self.reset_sim_track()
        self.start_sumo_simulation()
        self.detector_ids = self.traci_conn.inductionloop.getIDList()
        return {}, {}

    def start_sumo_simulation(self):
        sumo_binary_path = Path(self.sumo_home_path).joinpath("bin", "sumo.exe")
        sumo_cmd = [str(sumo_binary_path), "-c", self.sumo_cfg_path, "--end", str(self.end_time),
                    "--remote-port", str(self.traci_port),
                    "--start",
                    "--quit-on-end",
                    "--output-prefix", self.relative_output_path + "\\"]

        output_file_path = Path(self.sumo_cfg_path).parent.joinpath(self.relative_output_path, "lft_output.txt")
        if output_file_path.parent.exists() is False:
            output_file_path.parent.mkdir()
        with open(output_file_path, 'w') as f:
            self.running_process = subprocess.Popen(sumo_cmd, env=self.my_env, stdout=f)
        traci.init(port=self.traci_port, label=self.traci_label)
        self.traci_conn = traci.getConnection(self.traci_label)
        self.traci_conn.simulationStep()

    def step(self, action_dict):
        current_time = self.traci_conn.simulation.getTime()
        print(current_time)
        done = False
        if current_time >= self.end_time:
            self.close()
            done = True
        else:
            self.traci_conn.simulationStep()
        return {}, {}, done, {}, {}

    def close(self) -> None:
        self.traci_conn.close()
        try:
            self.running_process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.running_process.kill()
            print(f"sumo process of {self.worker_string} timed out and was killed.")
