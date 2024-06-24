#pragma once
#include "Controller.h"
#include <set>


class PotentialLines : public LFTStrategy {

public:

	PotentialLines(iniMap config);
	std::tuple<double, double> calculateAcceleration(Car* ego) override;

private:

	double FrontDistnce;
	double BackDistance;
	double ForceIndex, LowerLong, UpperLong;
	double Li, Wi, wx1, wx2, wy, verordnungsindex;
	double Kp1, Kp2;
	double MINDesiredSpeed, MAXDesiredSpeed;
	double nudge_index, repulse_index;
	int n, p, q;
	double ReactionTime;
	double Deccelerate;
	std::set<std::string> VSafeVehModels;
	std::map<std::string, std::map<std::string, std::string>> modelParams;

	std::tuple<double, double> calculateNeighbourForces(Car* ego, std::vector<Car*> neighbours);

	std::tuple<double, double> calculatePotentialFunMajorMinorAxis(Car* ego, Car* neighbour);

	std::tuple<double, double> calculateForces(Car* ego, Car* neighbour, double a, double b);

	std::tuple<double, double> calculateTargetSpeedForce(Car* car);

	double calculateSafeAcc(Car* ego, std::vector<Car*> front_neighbors);

	// Calculates new lateral accelerations such that the ego vehicle does not cross road boundary 
	double controlRoadBoundary(Car* ego, double ay);

	double PotentialLines::calculatePLForce(Car* ego, double lower_bound, double upper_bound);

	/* The Porbability Integral Transform (PIT) of the pdf used for generating the potential lines.
	* A good link to understand PIT: https://matthewfeickert.github.io/Statistics-Notes/notebooks/Introductory/probability-integral-transform.html
	* It uses the Trapezoidal rule for integration.
	*/
	double mixed_normal_cdf(double x);

	/* The underlying probability density function used for generating potential lines.
	* At the moment, it uses a combintation of two guassian distributions, representing fast- and slow-moving vehicles
	*/
	double mixed_normal_pdf(double x);

};