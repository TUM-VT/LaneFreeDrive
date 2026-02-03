#include "Flocking.h"
#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#include <LaneFree.h>
#endif

Flocking::Flocking(iniMap config) : LFTStrategy(config) {
	// Initialize flocking-specific parameters from the config
	auto flocking_param = config["Flocking Parameters"];
	example_flock_param = std::stod(flocking_param["example_flock_param"]);
}

std::tuple<double, double>  Flocking::calculateAcceleration(Car* ego) {
	// This method return the acceleration values (ax, ay) for the ego vehicle based on the flocking model.

	double ax = 0.0;
	double ay = 0.0;

	return std::make_tuple(ax, ay);
}