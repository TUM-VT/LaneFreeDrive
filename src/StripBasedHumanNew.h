/** 
* @brief Stip-based Human Driver Model.
* 
* This file models a human driver in lane-less traffic. It is based on the following paper:
* https://ascelibrary.org/doi/10.1061/%28ASCE%29CP.1943-5487.0000378
*
* @author Arslan Ali Syed <arslan.syed@tum.de>
* @date 17-01-2024
*/

#pragma once
#include "Controller.h"
#include <vector>
#include <random>
#include <fstream>
#include <unordered_map>

class NetworkStrips {
public:
	NetworkStrips() {};
	NetworkStrips(double strip_width);
	void addStripsForNewCar(Car* car);
	double getYFromInx(NumericalID edge, int index);
	int getInxFromY(NumericalID edge, double y);
	void shiftAssignedStrip(Car* car, int delta_strip);
	int calculateStripLimit(Car* car);

	double getStripsCount(NumericalID edge) { return edge_strip_counts[edge]; }
	int getAssignedStripInx(Car* car) { return std::get<1>(carAssignedStrip[car]); }

private:
	double strip_width;
	std::map<Car*, std::tuple<NumericalID, int>> carAssignedStrip;
	std::map<NumericalID, int> edge_strip_counts;
	std::map<NumericalID, double> edge_widths;
};

class StripBasedHumanNew : public LFTStrategy {

public:
	StripBasedHumanNew(iniMap config);
	std::tuple<double, double> calculateAcceleration(Car* ego) override;
	void update() override;
	void finalize_simulation() override;

private:
	double ReactionTime;
	double Deccelerate;
	double MaxBrakeDeceleration;
	double Accelerate;
	double MinSafeGap;
	double StripWidth;
	double FrontDistance;
	double LaneChangeThreshold;
	double Lambda;
	double numStripsConsidered;
	std::ofstream StripsChangeFile;
	std::map<Car*, std::vector<double>> driverMemory;
	std::map<Car*, double> ReactionTimesMap;
	std::mt19937 rng;
	std::normal_distribution<double> reaction_distribution;
	NetworkStrips network_strips;

	std::unordered_map<int, std::tuple<double, Car*>> calculateSafeVelocities(Car* ego, std::vector<Car*> front_cars);

	Car* calculateLeader(Car* ego, std::vector<Car*> front_neighbors);

	void updateStripChangeBenefit(Car* ego, std::unordered_map<int, std::tuple<double, Car*>> safeVelMap, double vsafe_current);

	bool isSufficientGap(Car* ego, double x, double y, std::vector<Car*> front_cars, std::vector<Car*> back_cars);

	// Calculate the safe velocity for the ego vehicle with the leader at a certain gap
	double calculateSafeVelocity(Car* ego, Car* leader, double gap);

};