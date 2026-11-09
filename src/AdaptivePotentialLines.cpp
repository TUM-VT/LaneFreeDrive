#pragma once
#include "AdaptivePotentialLines.h"
#include <cmath>
#include <algorithm>
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
		auto leaders = calculateLeader(car, front_neighbors, 1);
		leader_map_temp[car] = (leaders.size() > 0) ? leaders[0] : nullptr;
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
			apl_force_index_map.clear();
			std::string line;
			while (std::getline(file, line)) {
				auto line_vector = splitString(line, ",");
				if (line_vector[0].compare("") != 0) {
					python_margin_map[line_vector[0]] = std::stod(line_vector[1]);
					apl_force_index_map[line_vector[0]] = std::stod(line_vector[2]);
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

	Car* mainline_car = nullptr;
	for (const auto& [id, car] : carsMap) {
		if (!car->getIfOnRampVeh() && !car->getIfOffRampVeh()) {
			mainline_car = car;
			break;
		}
	}
	if (mainline_car == nullptr) {
		return;
	}
	auto [upper_bound, lower_bound] = mainline_car->calBoundary(0);

	std::vector<Car*> human_cars;
	for (const auto& [id, car] : carsMap) {
		if (car->getModelName() == "Human") {
			human_cars.push_back(car);
		}
	}
	std::sort(human_cars.begin(), human_cars.end(), [](Car* a, Car* b) {
			return std::get<0>(a->getGlobalPosition()) < std::get<0>(b->getGlobalPosition());
		});

	std::map<double, double> occupied_lats;
	double x1_last{ std::nan("") }, x2_last{ std::nan("") };
	for (Car* car : human_cars) {
		double margin = calculateAPLMargin(car);
		if (std::isnan(margin)) {
			continue;
		}
		
		auto [global_x, global_y] = car->getGlobalPosition();

		double x1 = global_x - car->getLength() / 2.0 - margin;
		double x2 = global_x + car->getLength() / 2.0;

		double y1 = global_y - car->getWidth() / 2.0;
		double y2 = global_y + car->getWidth() / 2.0;

		if (std::isnan(x1_last)) {
			x1_last = x1;
			x2_last = x2;
			occupied_lats[y1] = y2;
		}
		else if (x1 <= x2_last) {
			x2_last = std::max(x2_last, x2);
			occupied_lats[y1] = y2;
		}
		else {
			human_free_space[std::make_tuple(x1_last, x2_last)] = calculateAvailableLateralFromOccupied(occupied_lats, lower_bound, upper_bound);
			occupied_lats.clear();
			x1_last = x1;
			x2_last = x2;
			occupied_lats[y1] = y2;
		}

	}

	if (!std::isnan(x1_last)) {
		human_free_space[std::make_tuple(x1_last, x2_last)] = calculateAvailableLateralFromOccupied(occupied_lats, lower_bound, upper_bound);
	}

}

std::map<double, double> AdaptivePotentialLines::calculateAvailableLateralFromOccupied(std::map<double, double> lateral, double lower_bound, double upper_bound) {
	std::map<double, double> new_lats;
	double last_y = lower_bound;

	for (const auto& [y1, y2] : lateral) {
		// Clip occupied intervals to road boundaries
		double start = std::max(y1, lower_bound);
		double end = std::min(y2, upper_bound);

		if (end <= start)
			continue;

		// Free space before the occupied interval
		if (start > last_y) {
			new_lats[last_y] = start;
		}

		// Handle overlapping occupied intervals
		last_y = std::max(last_y, end);
	}

	// Free space after the last occupied interval
	if (last_y < upper_bound) {
		new_lats[last_y] = upper_bound;
	}

	return new_lats;
}


double AdaptivePotentialLines::calculatePLForce(Car* ego) {
	auto [car_x, car_y] = ego->getGlobalPosition();
	auto [upper_bound, lower_bound] = ego->calBoundary(BoundaryControlLookAhead);

	double pl_force = std::nan("");
	for (const auto& [range, lat_info] : human_free_space) {
		// If it is off-ramp vehicle or on-ramp vehicle on the ramp, then don't use APL
		if (ego->getIfOffRampVeh() || (ego->getIfOnRampVeh() && ego->getIfOnRampEdge())) {
			break;
		}

		auto [x1, x2] = range;
		if (x1 <= car_x && car_x <= x2) {

			std::map<double, double> possible_lats;
			double available_space = 0.0;
			double max_vehicle_width = 1.9;

			for (const auto& [y1, y2] : lat_info) {
				double start = std::max(y1, lower_bound);
				double end = std::min(y2, upper_bound);

				double pl_space = end - start - max_vehicle_width;

				if (pl_space > 0) {
					available_space += pl_space;
					possible_lats[start + 0.5 * max_vehicle_width] = end - 0.5 * max_vehicle_width;
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
				if (rel_line <= space_count) {
					double target_line = y1 + rel_line - passed_space;
					assigned_pl[ego] = target_line;
					double apl_force = apl_force_index_map[ego->getVehName()];
					pl_force = apl_force * (target_line - car_y);
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