#pragma once
#include "PotentialLines.h"
#include <set>
#include <fstream>


class AdaptivePotentialLines : public PotentialLines {

public:

	AdaptivePotentialLines(iniMap config);

protected:

	double ConstantMargin;
	double AdaptiveFollowerDistance;
	std::string AdaptiveAlgorithm;
	std::map <Car*, Car*> follower_map;
	std::map <std::tuple<double, double>, std::map<double, double>> human_free_space;
	std::set <Car*> cars_with_modified_pl;
	std::ofstream sync_file;
	std::string syncfile_name;
	std::string python_margin_filename;
	std::map<std::string, double> python_margin_map;

	void update() override;

	void finish_time_step() override;

	double calculatePLForce(Car* ego, double lower_bound, double upper_bound) override;
	std::tuple<double, double> calculateForces(Car* ego, Car* neighbour, double a, double b) override;

	double calculateAPLMargin(Car* car);

	void buildHumanOccupiedMap();

	std::map<double, double> calculateAvailableLateralFromOccupied(std::map<double, double> lateral, double edge_width);

	double calculateSurroundingSpeed(Car* car);

};