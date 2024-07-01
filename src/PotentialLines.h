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
	double Accelerate;
	std::string PLForceModel;
	std::vector<std::string> speed_mu;
	std::vector<std::string> speed_sigma;
	std::map<int, double> cdf_map;
	std::set<std::string> VSafeVehModels;
	std::map<std::string, std::map<std::string, std::string>> modelParams;

	std::tuple<double, double> calculateNeighbourForces(Car* ego, std::vector<Car*> neighbours);

	std::tuple<double, double> calculatePotentialFunMajorMinorAxis(Car* ego, Car* neighbour);

	std::tuple<double, double> calculateForces(Car* ego, Car* neighbour, double a, double b);

	std::tuple<double, double> calculateTargetSpeedForce(Car* car);

	double calculateSafeAcc(Car* ego, std::vector<Car*> front_neighbors);

	// Calculates new lateral accelerations such that the ego vehicle does not cross road boundary 
	double controlRoadBoundary(Car* ego, double ay);

	double PotentialLines::calculatePLForceCDF(Car* ego, double lower_bound, double upper_bound);

	double PotentialLines::calculatePLForceUniform(Car* ego, double lower_bound, double upper_bound);

	/* The Porbability Integral Transform (PIT) of the pdf used for generating the potential lines.
	* A good link to understand PIT: https://matthewfeickert.github.io/Statistics-Notes/notebooks/Introductory/probability-integral-transform.html
	* It uses the midpoint rule for integration.
	*/
	std::map<int, double> calculate_cdf_vector(double min_speed, double max_speed);

	// Calculates the pdf of the normal distribution at point x
	double normal_pdf(double x, double mu, double sigma);

};