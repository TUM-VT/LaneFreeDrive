#pragma once
#include "PotentialLines.h"
#include <set>


class AdaptivePotentialLines : public PotentialLines {

public:

	AdaptivePotentialLines(iniMap config);
	std::tuple<double, double> calculateAcceleration(Car* ego) override;

protected:

	double ConstantMargin;
	double AdaptiveFollowerDistance;
	std::string AdaptiveAlgorithm;
	std::map <Car*, Car*> follower_map;
	std::map <std::tuple<double, double>, std::map<double, double>> human_free_space;
	std::set <Car*> cars_with_modified_pl;

	void update() override;

	std::tuple<double, double> calculateForces(Car* ego, Car* neighbour, double a, double b) override;

	double calculatePLForceUniformAdaptive(Car* ego, double lower_bound, double upper_bound);

	double calculateAPLMargin(Car* car);

	void buildHumanOccupiedMap();

	std::map<double, double> calculateAvailableLateralFromOccupied(std::map<double, double> lateral, double edge_width);

	double calculateSurroundingSpeed(Car* car);

};