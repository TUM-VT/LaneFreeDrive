#pragma once
#include <algorithm>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <SimpleIni.h>
#include <map>
#include <filesystem>
#include <regex>
#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#include <LaneFree.h>
#include "libLaneFreePlugin_Export.h"
#endif

#define CONTROLLER_H
#include "Controller.h"

#define MAX(a, b) (((a) > (b))?(a):(b))
#define MIN(a, b) (((a) <= (b))?(a):(b))


namespace fs = std::filesystem;
using std::map;
using std::string;

std::unordered_set<std::string> vehTypesBoundaryError;

LFTStrategy::LFTStrategy(iniMap config) {
	// Set ramp parameters
	map<string, string> ramp_info = config["Ramp Information"];
	on_ramp_edges = splitString(ramp_info["OnRampEdges"], ",");
	off_ramp_edges = splitString(ramp_info["OffRampEdges"], ",");
}

bool LFTStrategy::isOnRampSituation(Car* car) {
	bool on_ramp_situation = false;
	if (car->getIfOnRampVeh()) {
		string current_edge = car->getCurrentEdgeName();
		for (string edge : on_ramp_edges) {
			if (current_edge.compare(edge) == 0) {
				on_ramp_situation = true;
				break;
			}
		}
	}
	return on_ramp_situation;
}

bool LFTStrategy::isOffRampSituation(Car* car) {
	bool off_ramp_situation = false;
	if (car->getIfOffRampVeh()) {
		string current_edge = car->getCurrentEdgeName();
		for (string edge : off_ramp_edges) {
			if (current_edge.compare(edge) == 0) {
				off_ramp_situation = true;
				break;
			}
		}
	}
	return off_ramp_situation;
}

void LFTStrategy::setCircular(iniMap config) {
	auto it = config.find("General Parameters");
	int circular_int = std::stoi(it->second["circular_movement"]);
	LFTStrategy::circular = (circular_int > 0)? true: false;
}

map<string, map<string, string>> LFTStrategy::extractModelSpecificParams(iniMap config, string prefix) {
	map<string, map<string, string>> params;
	for (auto const& [key, val] : config) {
		if (key.find(prefix) == 0) {
			string mstring = key.substr(prefix.length());
			std::vector<string> models = splitString(mstring, ",");
			for (string model : models) {
				params[model] = val;
			}
		}
	}
	return params;
}

std::vector<string> LFTStrategy::splitString(const string& s, string delimiter) {
	std::regex reg(delimiter);
	std::vector<string> split{ std::sregex_token_iterator(s.begin(), s.end(), reg, -1), {} };
	return split;
}

std::tuple<double, double> LFTStrategy::applyAccAndJerkConstraints(double ax, double ay, Car* car) {

	// Limit the acceleration values based on the constraints
	if (acc_jerk_limits.count("longitudinal_acceleration_limits") > 0) {
		auto [min_lon_acc, max_lon_acc] = acc_jerk_limits["longitudinal_acceleration_limits"];
		ax = MIN(max_lon_acc, MAX(min_lon_acc, ax));
	}

	if (acc_jerk_limits.count("lateral_acceleration_limits") > 0) {
		auto [min_lat_acc, max_lat_acc] = acc_jerk_limits["lateral_acceleration_limits"];
		ay = MIN(max_lat_acc, MAX(min_lat_acc, ay));
	}

	// Limit the jerk values based on the constraints
	if (acc_jerk_limits.count("longitudinal_jerk_limits") > 0) {
		auto [min_long_jerk, max_long_jerk] = acc_jerk_limits["longitudinal_jerk_limits"];
		// Calculate max acceleration possible based on acceleration and time step length
		double acc_diff = ax - car->getAccX();
		if (acc_diff > 0) {
			double possible_acc = car->getAccX() + max_long_jerk * get_time_step_length();
			ax = MIN(ax, possible_acc);
		}
		else if (acc_diff < 0) {
			double possible_acc = car->getAccX() + min_long_jerk * get_time_step_length();
			ax = MAX(ax, possible_acc);
		}
	}

	if (acc_jerk_limits.count("lateral_jerk_limits") > 0) {
		auto [min_lat_jerk, max_lat_jerk] = acc_jerk_limits["lateral_jerk_limits"];
		// Calculate max acceleration possible based on acceleration and time step length
		double acc_diff = ay - car->getAccY();
		if (acc_diff > 0) {
			double possible_acc = car->getAccY() + max_lat_jerk * get_time_step_length();
			ay = MIN(ay, possible_acc);
		}
		else if (acc_diff < 0) {
			double possible_acc = car->getAccY() + min_lat_jerk * get_time_step_length();
			ay = MAX(ay, possible_acc);
		}
	}

	return std::make_tuple(ax, ay);
}

void LFTStrategy::setAccAndJerkConstraints(map<string, string> config) {
	for (string key: { "lateral_acceleration_limits", "longitudinal_acceleration_limits", "lateral_jerk_limits", "longitudinal_jerk_limits"}) {
		if (config[key].compare("") != 0) {
			std::vector<string> limits = splitString(config[key], ",");
			if (limits.size() == 2) {
				double min_limit = std::stod(limits[0]);
				double max_limit = std::stod(limits[1]);
				acc_jerk_limits[key] = std::make_tuple(min_limit, max_limit);
			} else {
				throw std::invalid_argument("Invalid format for " + key + " in config file. Expected format: min,max");
			}
		}
	}
}

bool LFTStrategy::circular = false;

Car::Car(NumericalID numID, iniMap config, map<string, LFTStrategy*> strategies) {
	width =  get_veh_width(numID);
	length = get_veh_length(numID);
	this->numID = numID;
	typeName = get_veh_type_name(numID);
	vehName = get_vehicle_name(numID);

	auto itm = config.find("Vehicle Models");
	map<string, string> usedModelsMap = itm->second;

	for (const auto& entry : usedModelsMap) {
		string object_type = entry.first;
		string model_type = usedModelsMap[object_type];
		if (typeName.compare(0, object_type.length(), object_type) == 0) {
			modelName = object_type;
			try {
				lftstrategy = strategies.at(model_type);
			}
			catch (const std::out_of_range& e) {
				throw std::invalid_argument("No suitable LFT strategy found for vtype " + typeName + ". Check Vehicle Models in config file.");
			}
		}
	}

	auto ramp_info = config["Ramp Information"];
	if (vehName.find(ramp_info["on_ramp_veh_ident_key"]) != std::string::npos) {
		isOnRampVeh = true;
	}
	if (vehName.find(ramp_info["off_ramp_veh_ident_key"]) != std::string::npos) {
		isOffRampVeh = true;
	}

}

Car::Car(const Car& car) {
	width = car.width;
	length = car.length;
	numID = car.numID;
	typeName = car.typeName;
	vehName = car.vehName;
	modelName = car.modelName;
	x = car.x;
	y = car.y;
	speedX = car.speedX;
	speedY = car.speedY;
	accX = car.accX;
	accY = car.accY;
	desiredSpeed = car.desiredSpeed;
	currentEdge = car.currentEdge;
	lftstrategy = car.lftstrategy;
}

std::string Car::getCurrentEdgeName() { 
	return get_edge_name(currentEdge); 
}

std::tuple<double, double> Car::applyAcceleration() {
	auto [ax, ay] = lftstrategy->calculateAcceleration(this);
	accX = ax;
	accY = ay;
	apply_acceleration(numID, ax, ay);
	return std::make_tuple(ax, ay);
}

std::tuple<double, double> Car::calDistanceToBoundary(double offset_x, double offset_y) {
	double distance_left{ 0 }, distance_right{ 0 };
	get_distance_to_road_boundaries_at(numID, offset_x, offset_y, &distance_left, &distance_right, NULL, NULL, NULL);
	return std::make_tuple(distance_left, distance_right);
}

std::tuple<double, double> Car::calBoundary(double offset_x) {
	double left_boundary_y{ 0 }, right_boundary_y{ 0 };
	get_global_position_of_road_boundaries_at(numID, offset_x, &left_boundary_y, &right_boundary_y);
	return std::make_tuple(left_boundary_y, right_boundary_y);
}

double Car::getRelativeDistanceX(Car* other) {
	return get_relative_distance_x(numID, other->getNumId());
}

double Car::getRelativeDistanceY(Car* other) {
	return get_relative_distance_y(numID, other->getNumId());
}

std::tuple<double, double> Car::getGlobalPosition() {
	return std::make_tuple(get_global_position_x(numID), get_global_position_y(numID));
}

double Car::getCurrentEdgeWidth() {
	return get_edge_width(currentEdge);
}

double Car::getCurrentEdgeLength() {
	return get_edge_length(currentEdge);
}

void Car::update() {
	speedX = get_speed_x(numID);
	speedY = get_speed_y(numID);
	x = get_position_x(numID);
	circularX = x;
	y = get_position_y(numID);
	currentEdge = get_edge_of_vehicle(numID);
	desiredSpeed = get_desired_speed(numID);
}

std::vector<Car*> LFTStrategy::getNeighbours(Car* ego, double distance) {
	size_t size;
	NumericalID* neighbors;
	std::vector<Car*> neighborCars = {};
	if (distance >= 0) {
		neighbors = get_all_neighbor_ids_front(ego->getNumId(), distance, 0, &size);
	}
	else {
		neighbors = get_all_neighbor_ids_back(ego->getNumId(), -distance, 0, &size);
	}

	for (int i = 0; i < size; i++) {
		NumericalID numID = neighbors[i];
		Car* car = carsMap[numID];
		neighborCars.push_back(car);
		car->setCircularX(car->getX());
	}
	
	if (circular) {
		map<int, double> new_car_posx;
		double edge_len = get_edge_length(ego->getCurrentEdge());
		
		if (distance > 0 && ego->getX() > (edge_len - distance - 10)) {
			for (Car* car: neighborCars) {
				if (car->getX() <= distance) {
					car->setCircularX(car->getX() + edge_len);
				}
			}
		}

		else if (distance < 0 && ego->getX() < -distance) {
			for (Car* car: neighborCars) {
				if (car->getX() > edge_len + distance) {
					car->setCircularX(car->getX() - edge_len);
				}
			}
		}
	}
	
	return neighborCars;
}

