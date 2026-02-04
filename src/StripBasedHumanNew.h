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

typedef struct
{
	int mainInx = 0;
	int numOccupied = 0;
}StripInfoNew;

class EdgeStripsNew {
public:
	EdgeStripsNew(NumericalID edge_id, double edge_width, double strip_width);
	void updateOccupiedMap(std::map<NumericalID, Car*>& carsMap);
	StripInfoNew getVehicleStripInfo(Car* car);
	int getTotalNoStrips() { return total_strips; }
	double getYFromInx(int index);
	double getEdgeWidth() { return edge_width; }

private:
	NumericalID edge_id;
	double edge_width;
	double strip_width;
	int total_strips;
	std::map<Car*, StripInfoNew> carOccupancyMap;
	std::tuple<int, int> calculateStripInx(Car* car);
	
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
	std::map<Car*, StripInfoNew> occupancyMap;
	std::map<Car*, std::vector<double>> driverMemory;
	std::map<NumericalID, EdgeStripsNew*> edgeStrips;
	std::map<Car*, double> ReactionTimesMap;
	std::mt19937 rng;
	std::normal_distribution<double> reaction_distribution;

	std::unordered_map<int, std::tuple<double, Car*>> calculateSafeVelocities(Car* ego, std::vector<Car*> front_cars);

	void updateStripChangeBenefit(Car* ego, std::unordered_map<int, std::tuple<double, Car*>> safeVelMap);

	bool isSufficientGap(Car* ego, double x, double y, std::vector<Car*> front_cars, std::vector<Car*> back_cars);

	bool isCrossingRoadBoundary(Car* car, int strip_inx, EdgeStripsNew* strip);

	// Calculate the safe velocity for the ego vehicle with the leader at a certain gap
	double calculateSafeVelocity(Car* ego, Car* leader, double gap);

};