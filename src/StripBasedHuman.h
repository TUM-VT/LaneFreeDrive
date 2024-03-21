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

typedef struct
{
	int mainInx = 0;
	int numOccupied = 0;
}StripInfo;

class EdgeStrips {
public:
	EdgeStrips(NumericalID edge_id, double edge_width, double strip_width);
	/*Calculates the closest vehicle among front neighbours that overlap with the strips of the ego vehicle.Returns nullptr if found none.
	The front_neighbours are assumed to sorted according to logitudinal distance from ego vehicle.*/
	Car* calculateClosestVehicleOnStrip(Car* ego, std::vector<Car*> front_neighbours);
	void updateOccupiedMap(std::map<NumericalID, Car*>& carsMap);
	std::vector<Car*> getStripVehicles(int index);
	StripInfo getVehicleStripInfo(Car* car);
	int getTotalNoStrips() { return total_strips; }
	double getYFromInx(int index);

private:
	NumericalID edge_id;
	double edge_width;
	double strip_width;
	int total_strips;
	std::map<Car*, StripInfo> carOccupancyMap;
	std::map<int, std::vector<Car*>> stripOccupancyMap;
	// Calculates the position of the main and upper strip of the vehicle
	int calulcateStripPos(Car* car);
	std::tuple<int, int> calculateStripInx(Car* car);
	std::vector<std::tuple<Car*, int>> calculateLeaders(Car* ego, std::vector<Car*> front_neighbours);
	
};

class StripBasedHuman : public LFTStrategy {

public:
	StripBasedHuman(iniMap config);
	std::tuple<double, double> calculateAcceleration(Car* ego) override;
	void update() override;

private:
	double ReactionTime;
	double Deccelerate;
	double Accelerate;
	double StripWidth;
	double FrontDistance;
	double LaneChangeThreshold;
	double Lambda;
	std::map<Car*, StripInfo> occupancyMap;
	std::map<Car*, double> driverMemory;
	std::map<NumericalID, EdgeStrips*> edgeStrips;

	std::map<int, std::tuple<double, Car*>> calculateSafeVelocities(Car* ego);

	std::tuple<int, double, Car*> calculateLeaderFromSafeVelMap(Car* ego, std::map<int, std::tuple<double, Car*>> safeVelMap);

	std::tuple<std::vector<Car*>, std::vector<Car*>> calculateFollowerLeader(Car* ego, std::vector<int> strip_indices);

	void updateStripChangeBenefit(Car* ego, std::map<int, std::tuple<double, Car*>> safeVelMap);

	bool isSufficientGap(Car* ego, int strip_inx);

	std::tuple<bool, bool> StripBasedHuman::isLaneChangePossible(Car* ego);

	// Calculate the safe velocity for the ego vehicle
	double calculateSafeVelocity(Car* ego, Car* leader);

};