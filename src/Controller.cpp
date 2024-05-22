#pragma once
#include <stdio.h>
#include <stdlib.h>
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


namespace fs = std::filesystem;
using std::map;
using std::string;

std::unordered_set<std::string> vehTypesBoundaryError;

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

bool LFTStrategy::circular = false;

Car::Car(NumericalID numID, iniMap config, map<string, LFTStrategy*> strategies) {
	width =  get_veh_width(numID);
	length = get_veh_length(numID);
	this->numID = numID;
	typeName = get_veh_type_name(numID);
	vehName = get_vehicle_name(numID);

	auto itm = config.find("Vehicle Models");
	map<string, string> usedModelsMap = itm->second;
	auto itb = config.find("Vehicle Boundaries");
	map<string, string> usedModelsBoundaryMap = itb->second;

	for (const auto& entry : usedModelsMap) {
		string object_type = entry.first;
		string model_type = usedModelsMap[object_type];
		if (typeName.compare(0, object_type.length(), object_type) == 0) {
			try {
				lftstrategy = strategies.at(model_type);
			}
			catch (const std::out_of_range& e) {
				throw std::invalid_argument("No suitable LFT strategy found for vtype " + typeName + ". Check Vehicle Models in config file.");
			}

			try {
				// Set the boundary object
				string boundary_name = usedModelsBoundaryMap.at(object_type);
				if (boundary_name == "RectangularHardBoundary") {
					boundary = new RectangularHardBoundary(config, this);
				}
				else {
					throw std::invalid_argument("The boundary type " + boundary_name + " does not exist for vtype " + typeName);
				}
			}
			catch (const std::out_of_range& e) {
				if (vehTypesBoundaryError.find(typeName) == vehTypesBoundaryError.end()) {
					std::cerr << "\nNo Boundary type given for vtype " + typeName + ". Check Vehicle Boundaries in config file.";
					vehTypesBoundaryError.insert(typeName);
				}
				
			}
		}
	}
}

Car::Car(const Car& car) {
	width = car.width;
	length = car.length;
	numID = car.numID;
	typeName = car.typeName;
	vehName = car.vehName;
	x = car.x;
	y = car.y;
	speedX = car.speedX;
	speedY = car.speedY;
	desiredSpeed = car.desiredSpeed;
	currentEdge = car.currentEdge;
	lftstrategy = car.lftstrategy;
	boundary = car.boundary;
}

void Car::applyAcceleration() {
	auto [ax, ay] = lftstrategy->calculateAcceleration(this);
	apply_acceleration(numID, ax, ay);
}

void Car::update() {
	speedX = get_speed_x(numID);
	speedY = get_speed_y(numID);
	x = get_position_x(numID);
	circularX = x;
	y = get_position_y(numID);
	currentEdge = get_edge_of_vehicle(numID);
	desiredSpeed = get_desired_speed(numID);
	if (boundary != nullptr) {
		boundary->updateBoundary();
	}
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
					car->setCircularX(std::abs(car->getX()) + edge_len);
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

