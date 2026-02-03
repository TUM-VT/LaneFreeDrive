#pragma once
#include "Controller.h"


class Flocking : public LFTStrategy {

public:

	Flocking(iniMap config);
	std::tuple<double, double> calculateAcceleration(Car* ego) override;

protected:
	// This is an example parameter for the flocking model. Add other parameters as needed.
	double example_flock_param;
};