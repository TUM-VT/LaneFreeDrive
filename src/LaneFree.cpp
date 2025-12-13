#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <SimpleIni.h>
#include <map>
#include <set>
#include <filesystem>
#include <iomanip>
#include <unordered_set>
#include <regex>
#include <random>
#include <fstream>
#include "Controller.h"
#include "PotentialLines.h"
#include "AdaptivePotentialLines.h"
#include "StripBasedHuman.h"
#define DEFINE_VARIABLES

#ifdef __unix__
#include "LaneFree_linux.h"
#include <dlfcn.h>
#elif defined(WIN32)
#include <LaneFree.h>
#endif

using std::map;
using std::string;
using std::vector;
using iniMap = map<string, map<string, string>>;
namespace fs = std::filesystem;

map<NumericalID, Car*> carsMap;
map<string, LFTStrategy*> strategies;
std::unordered_set<LFTStrategy*> used_strategies;
iniMap config;
std::ofstream collision_file;
std::ofstream trajectory_file;
map<string, std::tuple<double, double>> initial_positions;
map<std::tuple<Car*, Car*>, double> collisionStartTimes;
std::set<std::tuple<Car*, Car*>> collisionPairs;
std::string lateral_speed_file;
map<Car*, double> abs_lateral_speeds;
std::mt19937 speed_rand;

iniMap readConfigFile(char* file_name) {
	// Get path of the dll plugin for the lanefree traffic
	#ifdef __unix__
		Dl_info dl_info;
		dladdr((void*)readConfigFile, &dl_info);
		std::string dllPath = dl_info.dli_fname;
	#elif defined(WIN32)
		HMODULE hModule = GetModuleHandle("libLaneFreePlugin.dll");
		char dllPath[MAX_PATH];
		GetModuleFileName(hModule, dllPath, MAX_PATH);
	#endif

	fs::path myPath(dllPath);
	// Replace the dll name with the name of the file
	myPath.replace_filename(file_name);
	if (!fs::exists(myPath)) {
		throw std::runtime_error("The configuration file " + string(file_name) + " not found\n");
	}
	iniMap data;
	printf("Reading the configuration file %s\n", file_name);
	CSimpleIniA ini;
	ini.LoadFile(myPath.string().c_str());
	CSimpleIniA::TNamesDepend sections;
	ini.GetAllSections(sections);
	for (const auto& section : sections)
	{
		CSimpleIniA::TNamesDepend keys;
		ini.GetAllKeys(section.pItem, keys);
		for (const auto& key : keys)
		{
			string value = ini.GetValue(section.pItem, key.pItem);
			for (string s : {"#", "\t"}) {
				size_t commentPos = value.find_first_of(s);
				if (commentPos != string::npos) {
					value = value.substr(0, commentPos);
				}
			}
			data[section.pItem][key.pItem] = value;
		}
	}
	return data;
}

iniMap readConfigFileFallback(char* config_file, char* default_file) {
	iniMap defaultVals = readConfigFile(default_file);
	iniMap data;
	try {
		data = readConfigFile(config_file);
	}
	catch (const std::runtime_error& e) {
		printf("Using default parameters from file %s\n", default_file);
		return defaultVals;
	}

	for (auto x = defaultVals.begin(); x != defaultVals.end(); ++x) {
		string defSec = x->first;
		map<string, string> defKeyVal = x->second;
		auto itSec = data.find(defSec);
		if (itSec != data.end()) {
			map<string, string>* KVMap = &itSec->second;
			for (auto y = defKeyVal.begin(); y != defKeyVal.end(); ++y) {
				string param = y->first;
				if (KVMap->count(param) == 0) {
					printf("Parameter %s not found in config. Setting to default value %s\n", param.c_str(), y->second.c_str());
					KVMap->insert(std::make_pair(param, y->second));
				}
			}
		}
		else {
			printf("section %s does not exist in %s. Using default values.\n", defSec.c_str(), config_file);
			data[defSec] = x->second;
		}
	}
	return data;
}

void insert_vehicles() {
	auto gen_config = config["General Parameters"];
	std::string vehicle_init = gen_config["vehicle_init"];
	std::regex reg(",");
	if (vehicle_init.compare("RANDOM") == 0) {
		auto rand_config = config["RANDOM Init"];
		std::string route = rand_config["routes"];

		std::mt19937 rng(std::stoi(gen_config["seed"]));
		std::uniform_real_distribution<double> uni_x(std::stoi(rand_config["lower_x"]), std::stoi(rand_config["upper_x"]));
		std::uniform_real_distribution<double> uni_y(std::stoi(rand_config["lower_y"]), std::stoi(rand_config["upper_y"]));
		double min_distance_x = std::stod(rand_config["min_distance_x"]);
		double min_distance_y = std::stod(rand_config["min_distance_y"]);

		std::string str = rand_config["vehicle_types"];
		std::vector<std::string> vehicle_types{ std::sregex_token_iterator(str.begin(), str.end(), reg, -1), {} };

		str = rand_config["vehicle_counts"];
		std::vector<std::string> vehicle_counts{ std::sregex_token_iterator(str.begin(), str.end(), reg, -1), {} };

		// Store vehicle positions to check for overlaps
		std::vector<std::pair<double, double>> vehicle_positions;

		for (int i = 0; i < vehicle_types.size(); i++){
			int count = std::stoi(vehicle_counts[i]);
			std::vector<std::string> lon_positions, lat_positions;
			// Check if vehicle_types[i] is a key in rand_config
			if ((rand_config.count(vehicle_types[i] + "_x") != 0) && (rand_config.count(vehicle_types[i] + "_y") != 0)){
				std::string str = rand_config[vehicle_types[i] + "_x"];
				lon_positions = { std::sregex_token_iterator(str.begin(), str.end(), reg, -1), {} };
				str = rand_config[vehicle_types[i] + "_y"];
				lat_positions = { std::sregex_token_iterator(str.begin(), str.end(), reg, -1), {} };
				std::cout << "Fixed positions of " << vehicle_types[i] << " are given. Skipping random generation for " << vehicle_types[i] << "." << std::endl;
				for (int j = 0; j < lon_positions.size(); j++) {
					std::string veh_name = vehicle_types[i] + "_" + std::to_string(j);
					#ifdef __unix__
						insert_new_vehicle((char*)veh_name.c_str(), (char*)route.c_str(), (char*)vehicle_types[i].c_str(), std::stod(lon_positions[j]), std::stod(lat_positions[j]), 0, 0);
					#elif defined(WIN32)
						insert_new_vehicle((char*)veh_name.c_str(), (char*)route.c_str(), (char*)vehicle_types[i].c_str(), std::stod(lon_positions[j]), std::stod(lat_positions[j]), 0, 0, 0, 0);
					#endif
				}
				continue;
			}

			for (int j = 0; j < count; j++) {
				std::string veh_name = vehicle_types[i] + "_" + std::to_string(j);
				double x_val = uni_x(rng);
				double y_val = uni_y(rng);

				// Check if the new vehicle overlaps with existing ones
				bool veh_remains = true;
				while (veh_remains) {
					x_val = uni_x(rng);
					y_val = uni_y(rng);
					bool overlaps = false;
					for (const auto& pos : vehicle_positions) {
						double dist_x = std::abs(x_val - pos.first);
						double dist_y = std::abs(y_val - pos.second);
						if (dist_x < min_distance_x && dist_y < min_distance_y) {
							overlaps = true;
							break;
						}
					}

					if (!overlaps) {
						vehicle_positions.push_back(std::make_pair(x_val, y_val));
						initial_positions[veh_name] = std::make_tuple(x_val, y_val);
						#ifdef __unix__
							insert_new_vehicle((char*)veh_name.c_str(), (char*)route.c_str(), (char*)vehicle_types[i].c_str(), x_val, y_val, 0, 0);
						#elif defined(WIN32)
							insert_new_vehicle((char*)veh_name.c_str(), (char*)route.c_str(), (char*)vehicle_types[i].c_str(), x_val, y_val, 0, 0, 0, 0);
						#endif
						veh_remains = false;
					}
				}

			}
		}
	}
}

void simulation_initialize() {
	auto config_path = getenv("CONFIG_FILE");
	auto default_path = getenv("DEFAULT_CONFIG_FILE");
	if (config_path == NULL) {
		config_path = "config.ini";
	}
	if (default_path == NULL) {
		default_path = "default_config.ini";
	}
	config = readConfigFileFallback(config_path, default_path);
	insert_vehicles();
	LFTStrategy::setCircular(config);
	strategies["PotentialLines"] = new PotentialLines(config);
	strategies["AdaptivePotentialLines"] = new AdaptivePotentialLines(config);
	strategies["StripBasedHuman"] = new StripBasedHuman(config);

	// File to store collisions
	auto gen_config = config["General Parameters"];
	string collisions_path = gen_config["collisions_file"];
	if (collisions_path.compare("") != 0) {
		collision_file.open(collisions_path);
		collision_file << "StartTime, EndTime,Vehicle1,Vehicle2\n";
	}

	string trajectory_path = gen_config["trajectory_file"];
	if (trajectory_path.compare("") != 0) {
		trajectory_file.open(trajectory_path);
		trajectory_file << "Time,Vehicle1,x,y,speed_x,speed_y,acceleration_x,acceleration_y,jerk_x,jerk_y,potential_line\n";
	}
	lateral_speed_file = gen_config["lateral_speed_file"];

	//initialize the same seed
	speed_rand = std::mt19937(stoi(gen_config["seed"]));
	string speed_dist = gen_config["speed_dist"];
	// Check if the speed dist is NORMAL, UNIFORM or Empty string and print an error message if it is not
	if (speed_dist.compare("NORMAL") != 0 && speed_dist.compare("UNIFORM") != 0 && speed_dist.compare("") != 0) {
		printf("Speed distribution %s not recognized. Setting it to empty field.\n", speed_dist.c_str());
		config["General Parameters"]["speed_dist"] = "";
	}
	carsMap.clear();
	abs_lateral_speeds.clear();
	printf("\nInitializiation over!!!\n");
}

void simulation_step() {

	NumericalID* allVehIDs = get_all_ids();
	// Update the vehicle class instances
	for (int i = 0; i < get_all_ids_size(); i++) {
		NumericalID numID = allVehIDs[i];
		if (carsMap.count(numID) == 0) {
			Car* car = new Car(numID, config, strategies);
			carsMap[numID] = car;
			used_strategies.insert(car->getLFTStrategy());
			abs_lateral_speeds[car] = 0.0;
		}
		carsMap[numID]->update();
	}

	// Update the strategies
	for (const auto& strategy : used_strategies) {
		strategy->setCarsMap(carsMap);
		strategy->update();
	}

	// Apply the acceleration
	for (int i = 0; i < get_all_ids_size(); i++) {
		NumericalID numID = allVehIDs[i];
		Car* car = carsMap[numID];
		abs_lateral_speeds[car] += std::abs(car->getSpeedY());
		double old_accx = car->getAccX();
		double old_accy = car->getAccY();
		auto[ax, ay] = car->applyAcceleration();
		// Record the trajectory for car
		if (trajectory_file.is_open()) {
			double time = get_time_step_length() * get_current_time_step();
			double jerk_x = (ax - old_accx) / get_time_step_length();
			double jerk_y = (ay - old_accy) / get_time_step_length();
			double pl = 0;
			if (car->getLFTStrategy() == strategies["PotentialLines"] || car->getLFTStrategy() == strategies["AdaptivePotentialLines"]) {
				pl = ((PotentialLines*)car->getLFTStrategy())->getAssignedPL(car);
			}
			trajectory_file << time << "," << car->getVehName() << "," << std::fixed << std::setprecision(2) 
				<< car->getX() << "," << car->getY() << "," << car->getSpeedX() << "," << car->getSpeedY() << "," << ax << "," << ay << "," << jerk_x << "," <<jerk_y << "," << pl << std::endl;
		}
	}

	// Finish the time step
	for (const auto& strategy : used_strategies) {
		strategy->finish_time_step();
	}

	// Record the collisions
	double current_time = get_time_step_length() * get_current_time_step();
	std::set<std::tuple<Car*, Car*>> delete_pairs;
	for (const auto& entry : collisionStartTimes) {
		auto pair = entry.first;
		if (collisionPairs.count(pair) == 0) {
			double start_time = entry.second;
			auto [car1, car2] = pair;
			collision_file << start_time << "," << current_time << "," << car1->getVehName() << "," << car2->getVehName() << "\n";
			delete_pairs.insert(pair);
		}
	}
	for (const auto& pair : delete_pairs) {
		collisionStartTimes.erase(pair);
	}
	collisionPairs.clear();

	// Record the vehicle information in the first time step

	if (get_current_time_step() == 1) {
		auto gen_config = config["General Parameters"];
		string vehicle_info_path = gen_config["vehicle_info_file"];
		if (vehicle_info_path.compare("") != 0) {
			std::ofstream vehicle_info_file;
			vehicle_info_file.open(vehicle_info_path);
			vehicle_info_file << "Vehicle,desired_speed,initial_x,initial_y\n";
			for (const auto& entry : carsMap) {
				Car* car = entry.second;
				string veh_name = car->getVehName();
				double desired_speed = car->getDesiredSpeed();
				auto [x, y] = initial_positions[veh_name];
				vehicle_info_file << veh_name << "," << desired_speed << "," << x << "," << y << "\n";
			}
			vehicle_info_file.close();
		}
	}
}

void simulation_finalize() {
	if (collision_file.is_open()) {
		collision_file.close();
	}
	if (trajectory_file.is_open()) {
		trajectory_file.close();
	}
	if (lateral_speed_file.compare("") != 0) {
		std::ofstream speed_file;
		speed_file.open(lateral_speed_file);
		speed_file << "Vehicle,mean_lateral_speed\n";
		for (const auto& [car, speed] : abs_lateral_speeds) {
			double mean_speed = speed / get_current_time_step();
			speed_file << car->getVehName() << "," << mean_speed << "\n";
		}
		speed_file.close();
	}

	for (const auto& strategy : used_strategies) {
		strategy->finalize_simulation();
	}
	used_strategies.clear();
}

// Function to generate a single random sample
double generate_combined_normal_sample() {
	auto norm_config = config["Desired Speed: Normal"];
	std::vector<std::string> mu_vector = LFTStrategy::splitString(norm_config["mu"], ",");
	std::vector<std::string> sigma_vector = LFTStrategy::splitString(norm_config["sigma"], ",");

	std::uniform_real_distribution<double> distribution(0.0, 1.0);
	double random_number = distribution(speed_rand);

	// Determine which normal distribution to sample from based on the coefficients
	double coeff = 1.0 / mu_vector.size();
	double cumulative_probability = 0.0;
	for (int i = 0; i < mu_vector.size(); ++i) {
		cumulative_probability += coeff;
		if (random_number <= cumulative_probability) {
			std::normal_distribution<double> norm_dist(std::stod(mu_vector[i]), std::stod(sigma_vector[i]));
			return norm_dist(speed_rand);
		}
	}

	// This point should never be reached
	return 0.0;
}

void event_vehicle_enter(NumericalID veh_id) {
	auto gen_config = config["General Parameters"];
	string dist_type = gen_config["speed_dist"];
	double min_speed = std::stod(gen_config["min_desired_speed"]);
	double max_speed = std::stod(gen_config["max_desired_speed"]);
	if (dist_type.compare("UNIFORM") == 0) {
		std::uniform_real_distribution<double> uni(min_speed, max_speed);
		double desired_speed = uni(speed_rand);
		set_desired_speed(veh_id, desired_speed);
	}
	else if (dist_type.compare("NORMAL") == 0) {
		auto norm_config = config["Desired Speed: Normal"];
		double desired_speed = generate_combined_normal_sample();
		set_desired_speed(veh_id, desired_speed);
	}
	set_circular_movement(veh_id, LFTStrategy::isCircular());

}

void event_vehicle_exit(NumericalID veh_id, int has_arrived) {
	if (has_arrived == 1) {
		delete carsMap[veh_id];
		carsMap.erase(veh_id);
	}
}

void event_vehicles_collide(NumericalID veh_id1, NumericalID veh_id2) {
	if (carsMap.size() > 0) {
		Car* car1 = carsMap[veh_id1];
		Car* car2 = carsMap[veh_id2];
		std::tuple<Car*, Car*> combination = std::make_tuple(car1, car2);
		double time = get_time_step_length() * get_current_time_step();
		collisionPairs.insert(combination);

		if (collisionStartTimes.count(combination) == 0) {
			collisionStartTimes[combination] = time;
		}
	}
}

void event_vehicle_out_of_bounds(NumericalID veh_id) {
}