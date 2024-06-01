#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <SimpleIni.h>
#include <map>
#include <filesystem>
#include <unordered_set>
#include <regex>
#include <random>
#include <fstream>
#include "Controller.h"
#include "PotentialLines.h"
#include "StripBasedHuman.h"
#define DEFINE_VARIABLES

#ifdef __unix__
#include "LaneFree_linux.h"
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

iniMap readConfigFile(char* file_name) {
	// Get path of the dll plugin for the lanefree traffic
	HMODULE hModule = GetModuleHandle("libLaneFreePlugin.dll");
	char dllPath[MAX_PATH];
	GetModuleFileName(hModule, dllPath, MAX_PATH);
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
	auto it = config.find("General Parameters");
	std::string vehicle_init = it->second["vehicle_init"];
	std::regex reg(",");
	if (vehicle_init.compare("RANDOM") == 0) {
		auto it = config.find("RANDOM Init");
		std::string route = it->second["routes"];

		std::mt19937 rng(std::stoi(it->second["seed"]));
		std::uniform_real_distribution<double> uni_x(std::stoi(it->second["lower_x"]), std::stoi(it->second["upper_x"]));
		std::uniform_real_distribution<double> uni_y(std::stoi(it->second["lower_y"]), std::stoi(it->second["upper_y"]));
		double min_distance_x = std::stod(it->second["min_distance_x"]);
		double min_distance_y = std::stod(it->second["min_distance_y"]);

		std::string str = it->second["vehicle_types"];
		std::vector<std::string> vehicle_types{ std::sregex_token_iterator(str.begin(), str.end(), reg, -1), {} };

		str = it->second["vehicle_counts"];
		std::vector<std::string> vehicle_counts{ std::sregex_token_iterator(str.begin(), str.end(), reg, -1), {} };

		// Store vehicle positions to check for overlaps
		std::vector<std::pair<double, double>> vehicle_positions;

		for (int i = 0; i < vehicle_types.size(); i++){
			int count = std::stoi(vehicle_counts[i]);
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
						insert_new_vehicle((char*)veh_name.c_str(), (char*)route.c_str(), (char*)vehicle_types[i].c_str(), x_val, y_val, 0, 0, 0, 0);
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
		default_path = "default_config\\default_config.ini";
	}
	config = readConfigFileFallback(config_path, default_path);
	insert_vehicles();
	LFTStrategy::setCircular(config);
	strategies["PotentialLines"] = new PotentialLines(config);
	strategies["StripBasedHuman"] = new StripBasedHuman(config);

	// File to store collisions
	auto it = config.find("General Parameters");
	string collisions_path = it->second["collisions_file"];
	if (collisions_path.compare("") != 0) {
		collision_file.open(collisions_path);
		collision_file << "Time,Vehicle1,Vehicle2\n";
	}

	//initialize srand with the same seed as sumo
	srand(get_seed());
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
		}
		carsMap[numID]->update();
	}

	// Update the strategies
	for (const auto& strategy : used_strategies) {
		strategy->setCarsMap(carsMap);
		strategy->update();
	}

	NumericalID* myedges = get_all_edges();  //returns an array with all the edges
	NumericalID n_myedges = get_all_edges_size(); //returns the size of all the edges
	for (int i = 0; i < n_myedges; i++) {

		NumericalID n_edge_ids = get_all_ids_in_edge_size(myedges[i]);
		NumericalID* ids_in_edge = get_all_ids_in_edge(myedges[i]);

		for (int j = 0; j < n_edge_ids; j++) {
			carsMap[ids_in_edge[j]]->applyAcceleration();
		}

	}
}

void simulation_finalize() {
	if (collision_file.is_open()) {
		collision_file.close();
	}
}

double box_muller(double mu, double sigma) {
	double u1 = rand() / (double)RAND_MAX;
	double u2 = rand() / (double)RAND_MAX;

	double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * 3.141 * u2);
	return z0 * sigma + mu;
}

double generate_desired_speed() {

	double random_num;
	double speed;
	auto it = config.find("General Parameters");
	double mu1 = std::stod(it->second["mu1"]);
	double mu2 = std::stod(it->second["mu2"]);
	double sigma1 = std::stod(it->second["sigma1"]);
	double sigma2 = std::stod(it->second["sigma2"]);

	do {
		if (rand() % 2 == 0) {
			random_num = box_muller(mu1, sigma1);
		}
		else {
			random_num = box_muller(mu2, sigma2);
		}

		speed = random_num;
	} while (speed < 25 || speed > 35);

	return speed;
}

void event_vehicle_enter(NumericalID veh_id) {
	char* v_type = get_veh_type_name(veh_id);
	int desired_speed = generate_desired_speed();
	set_desired_speed(veh_id, desired_speed);
	set_circular_movement(veh_id, LFTStrategy::isCircular());

}

void event_vehicle_exit(NumericalID veh_id, int has_arrived) {
	if (has_arrived == 1) {
		delete carsMap[veh_id];
		carsMap.erase(veh_id);
	}
}

void event_vehicles_collide(NumericalID veh_id1, NumericalID veh_id2) {
	Car* car1 = carsMap[veh_id1];
	Car* car2 = carsMap[veh_id2];
	double time = get_time_step_length() * get_current_time_step();
	if (collision_file.is_open()) {
		collision_file << time << "," << car1->getVehName() << "," << car2->getVehName() << "\n";
	}
}

void event_vehicle_out_of_bounds(NumericalID veh_id) {
}