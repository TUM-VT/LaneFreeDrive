#pragma once
#include "AdaptivePotentialLines.h"
#include <cmath>
#include <iostream>
#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#include "LaneFree.h"
#endif

using std::map;
using std::string;

double check_parameter_in_pl(iniMap config, string pl_param, string apl_param) {
	map<string, string> secParam = config["Adaptive Potential Lines Parameters"];
	string apl = secParam[apl_param];
	if (apl.compare("") == 0) {
		std::cout << apl_param << " not set in the configuration file. Using " << pl_param << " from PotentialLines" << std::endl;
		return stod(config["Potential Lines Parameters"][pl_param]);
	}
	else {
		return stod(apl);
	}
}

AdaptivePotentialLines::AdaptivePotentialLines(iniMap config): PotentialLines(config) {
	printf("Setting parameters for Adaptive Potential Lines strategy\n");

	map<string, string> secParam = config["Adaptive Potential Lines Parameters"];
	AdaptiveAlgorithm = secParam["AdaptiveAlgorithm"];
	ConstantMargin = stod(secParam["ConstantMargin"]);
	AdaptiveFollowerDistance = stod(secParam["AdaptiveFollowerDistance"]);
	apl_corridor_force_index = check_parameter_in_pl(config, "pl_force_index", "APL_pl_force_index");

	std::set<std::string> validAlgorithms = { "ConstantMargin", "SVAM", "FAM", "NSCM", "NAM", "AM", "PythonRL"};
	if (validAlgorithms.find(AdaptiveAlgorithm) == validAlgorithms.end()) {
		printf("The value of AdaptiveAlgorithm can only be ConstantMargin, SVAM, FAM, NSCM, NAM, AM or PythonRL. Using the default value ConstantMargin\n");
		AdaptiveAlgorithm = "ConstantMargin";
	}
	syncfile_name = secParam["SyncFile"];
	python_margin_filename = secParam["PythonMarginInputFile"];
}

void AdaptivePotentialLines::update() {
	PotentialLines::update();
	follower_map.clear();
	std::map <Car*, Car*> leader_map_temp;
	for (const auto& [key, car] : carsMap) {
		follower_map[car] = nullptr;
		std::vector<Car*> front_neighbors = getNeighbours(car, AdaptiveFollowerDistance);
		leader_map_temp[car] = calculateLeader(car, front_neighbors);
	}
	for (const auto& [follower, leader] : leader_map_temp) {
		if (leader != nullptr) {
			follower_map[leader] = follower;
		}
	}

	if (AdaptiveAlgorithm.compare("PythonRL") == 0) {
		// Update the APL margin from the csv file, set by Python
		std::ofstream margin_file;
		std::ifstream file(python_margin_filename);
		if (file.is_open()) {
			python_margin_map.clear();
			std::string line;
			while (std::getline(file, line)) {
				auto line_vector = splitString(line, ",");
				if (line_vector[0].compare("") != 0) {
					python_margin_map[line_vector[0]] = std::stod(line_vector[1]);
				}
			}
			file.close();
		}
		else {
            std::cout << "Unable to open the python margin sync file " << python_margin_filename << std::endl;
		}
	}

	buildHumanOccupiedMap();
	cars_with_modified_pl.clear();
	for (const auto& [key, car] : carsMap) {
		std::vector<Car*> front_neighbors = getNeighbours(car, this->FrontDistnce);
		double car_x = car->getX();
		for (const auto& [range, lat_info] : human_free_space) {
			auto [x1, x2] = range;
			if (x1 <= car_x && car_x <= x2) {
				cars_with_modified_pl.insert(car);
				break;
			}
		}
	}
	if (syncfile_name.compare("") != 0) {
		sync_file.open(syncfile_name);
		sync_file << "Step,Vehicle,Modified_PL,x1,x2,pl_y\n";
	}
}

void AdaptivePotentialLines::finish_time_step() {
	if (sync_file.is_open()) {
		sync_file.close();
	}
}

double AdaptivePotentialLines::calculateSurroundingSpeed(Car* car) {
	double avg_speed{ 0 }, count{ 0 };
	double y1 = car->getY() - car->getWidth() / 2.0;
	double y2 = car->getY() + car->getWidth() / 2.0;
	for (Car* neighbour : getNeighbours(car, -20.0)) {
		// Only count the neigbours that have different lateral positions
		if ((neighbour->getY() < y1) || (neighbour->getY() > y2)) {
			avg_speed += neighbour->getSpeedX();
			count++;
		}
	}
	if (count > 0) {
		avg_speed /= count;
	}
	else avg_speed = std::nan("");
	return avg_speed;
}

double AdaptivePotentialLines::calculateAPLMargin(Car* car) {
	double margin = std::nan("");

	if (AdaptiveAlgorithm.compare("ConstantMargin") == 0) {
		margin = ConstantMargin;
		return margin;
	}

	if (AdaptiveAlgorithm.compare("PythonRL") == 0) {
		margin = python_margin_map[car->getVehName()];
		return margin;
	}

	if (AdaptiveAlgorithm.compare("NSCM") == 0) {
		double avg_speed = calculateSurroundingSpeed(car);
		// If there are no neigbour, then also consider the space for modifying the PL
		if (std::isnan(avg_speed) || car->getSpeedX() < avg_speed) {
			margin = ConstantMargin;
		}
		return margin;
	}

	// Check if there is a follower
	Car* follower = follower_map[car];

	if (AdaptiveAlgorithm.compare("AM") == 0) {
		if (follower != nullptr) {
			double dist_to_follower = car->getX() - car->getLength() - follower->getX();
			margin = dist_to_follower + follower->getLength();
		}
		else {
			margin = ConstantMargin;
		}
		return margin;
	}

	if (follower != nullptr) {
		double safe_vel = calculateSafeVelocity(follower, car);
		double avg_speed = calculateSurroundingSpeed(car);
		// If there are no neigbour, then also consider the space for modifying the PL
		if (std::isnan(avg_speed)) {
			avg_speed = 1000;
		}
		if ((AdaptiveAlgorithm.compare("SVAM") == 0 && follower->getSpeedX() < 1.05 * safe_vel && car->getSpeedX() < avg_speed)
		|| (AdaptiveAlgorithm.compare("FAM") == 0 && car->getSpeedX() < avg_speed )
		|| (AdaptiveAlgorithm.compare("NAM") == 0)){
			double dist_to_follower = car->getX() - car->getLength() - follower->getX();
			margin = dist_to_follower + follower->getLength();
		}
	}
	return margin;
}


void AdaptivePotentialLines::buildHumanOccupiedMap() {
	human_free_space.clear();

	double x1_last{ std::nan("") }, x2_last{ std::nan("") };
	std::map<double, double> occupied_lats;
	NumericalID* myedges = get_all_edges();
	NumericalID n_myedges = get_all_edges_size();
	for (int i = 0; i < n_myedges; i++) {
		double edge_width = get_edge_width(myedges[i]);
		NumericalID n_edge_ids = get_all_ids_in_edge_size(myedges[i]);
		// Vehicles in ids_in_edge are already arranged according to logitudinal positions
		NumericalID* ids_in_edge = get_all_ids_in_edge(myedges[i]);
		for (int j = 0; j < n_edge_ids; j++) {
			Car* car = carsMap[ids_in_edge[j]];
			if (car->getModelName().compare("Human") == 0) {
				double margin = calculateAPLMargin(car);
				if (std::isnan(margin)) {
					continue;
				}
				double x1 = car->getX() - car->getLength() / 2.0 - margin;
				double x2 = car->getX() + car->getLength() / 2.0;

				double y1 = car->getY() - car->getWidth() / 2.0;
				double y2 = car->getY() + car->getWidth() / 2.0;

				if (std::isnan(x1_last)) {
					x1_last = x1;
					x2_last = x2;
					occupied_lats[y1] = y2;
				}
				else if (x1 <= x2_last) {
					x2_last = x2;
					occupied_lats[y1] = y2;
				}
				else {
					human_free_space[std::make_tuple(x1_last, x2_last)] = calculateAvailableLateralFromOccupied(occupied_lats, edge_width);
					occupied_lats.clear();
					x1_last = x1;
					x2_last = x2;
					occupied_lats[y1] = y2;
				}
			}
		}
		human_free_space[std::make_tuple(x1_last, x2_last)] = calculateAvailableLateralFromOccupied(occupied_lats, edge_width);
		occupied_lats.clear();
		x1_last = std::nan("");
		x2_last = std::nan("");
	}
}

std::map<double, double> AdaptivePotentialLines::calculateAvailableLateralFromOccupied(std::map<double, double> lateral, double edge_width) {
	std::map<double, double> new_lats;
	double y1_last{ std::nan("") }, y2_last{ std::nan("") };

	for (const auto& [y1, y2] : lateral) {
		if (std::isnan(y1_last)) {
			new_lats[0] = y1;
			y1_last = y1;
			y2_last = y2;
		}
		else if (y1 <= y2_last) {
			y2_last = y2;
		}
		else {
			new_lats[y2_last] = y1;
			y1_last = y1;
			y2_last = y2;
		}
	}
	new_lats[y2_last] = edge_width;
	return new_lats;
}

double AdaptivePotentialLines::calculatePLForce(Car* ego) {
	double car_x = ego->getX();
	double pl_force = std::nan("");
	for (const auto& [range, lat_info] : human_free_space) {
		auto [x1, x2] = range;
		if (x1 <= car_x && car_x <= x2) {

			std::map<double, double> possible_lats;
			double available_space = 0.0;
			double max_vehicle_width = 1.9;
			for (const auto& [y1, y2] : lat_info) {
				double pl_space = y2 - y1 - max_vehicle_width;
				if (pl_space > 0) {
					available_space += pl_space;
					possible_lats[y1 + 0.5 * max_vehicle_width] = y2 - 0.5 * max_vehicle_width;
				}
			}
			double co = ego->getDesiredSpeed() - MINDesiredSpeed;
			double areas = MAXDesiredSpeed - MINDesiredSpeed;
			double rel_line = (available_space / areas) * co;

			double space_count = 0;
			double passed_space = 0.0;
			for (const auto& [y1, y2] : possible_lats) {
				double pl_space = y2 - y1;
				space_count += pl_space;
				if (rel_line < space_count) {
					double target_line = y1 + rel_line - passed_space;
					Car* leader = leader_map[ego];
					assigned_pl[ego] = target_line;
					pl_force = apl_corridor_force_index * (target_line - ego->getY());
					if (sync_file.is_open()) {
						sync_file << get_current_time_step() << "," << ego->getVehName() << "," << "Y" << ","
							<< x1 << "," << x2 << "," << target_line << "\n";
					}
					car_in_apl[ego] = true;
					break;
				}
				passed_space += pl_space;
			}
			break;
		}
	}
	if (std::isnan(pl_force)) {
		if (sync_file.is_open()){
			sync_file << get_current_time_step() << "," << ego->getVehName() << "," << "N" << ",,,\n";
		}
		pl_force = PotentialLines::calculatePLForce(ego);
		car_in_apl[ego] = false;
	}
	return pl_force;
}