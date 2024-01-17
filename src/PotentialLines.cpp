#include "PotentialLines.h"
#include "LaneFree.h"

#define SAMPLE_UNIFORM(min, max) ((double)min + ((double)random()/RAND_MAX)*(max - min))
#define MAX(a, b) (((a) > (b))?(a):(b))
#define MIN(a, b) (((a) <= (b))?(a):(b))

#define PI 3.14159265358979323846

#define mu1 28.0
#define sigma1 1.0
#define mu2 32.0
#define sigma2 1.0

using std::map;
using std::string;

PotentialLines::PotentialLines() {
	iniMap parameters = readConfigFileFallback("config.ini", "default_config\\default_config.ini");
	auto it = parameters.find("Potential Lines Parameters");
	map<string, string> secParam = it->second;
	FrontDistnce = stod(secParam["FrontDistance"]);
	BackDistance = stod(secParam["BackDistance"]);
	ForceIndex = stod(secParam["ForceIndex"]);
	LowerLong = stod(secParam["LowerLong"]);
	UpperLong = stod(secParam["UpperLong"]);
	wx1 = stod(secParam["wx1"]);
	wx2 = stod(secParam["wx2"]);
	wy = stod(secParam["wy"]);
	n = stoi(secParam["n"]);
	p = stoi(secParam["p"]);
	q = stoi(secParam["q"]);
	Li = stod(secParam["Li"]);
	Wi = stod(secParam["Wi"]);
	Kp1 = stod(secParam["kp1"]);
	Kp2 = stod(secParam["kp1"]);
	kpBoundary = stod(secParam["kp_boundary"]);
	kdBoundary = stod(secParam["kd_boundary"]);
	MINDesiredSpeed = stod(secParam["min_desired_speed"]);
	MAXDesiredSpeed = stod(secParam["max_desired_speed"]);
	verordnungsindex = stod(secParam["pl_force_index"]);
	nudge_index = stod(secParam["nudge_index"]);
	repulse_index = stod(secParam["repulse_index"]);
}

std::tuple<double, double>  PotentialLines::calculateAcceleration(Car* ego) {
	int cross_edge = 0;
	size_t front_neighbors_size;
	size_t back_neighbors_size;
	NumericalID* front_neighbors = get_all_neighbor_ids_front(ego->getNumId(), this->FrontDistnce, cross_edge, &front_neighbors_size);
	NumericalID* back_neighbors = get_all_neighbor_ids_back(ego->getNumId(), this->BackDistance, cross_edge, &back_neighbors_size);

	auto [fx_nudge, fy_nudge] = calculateNeighbourForces(ego, back_neighbors, back_neighbors_size);
	auto [fx_repluse, fy_repluse] = calculateNeighbourForces(ego, front_neighbors, front_neighbors_size);
	fx_repluse = -fx_repluse;
	auto [ax_desired, ay_desired] = calculateTargetSpeedForce(ego);
	double fy_pl = calculatePLForce(ego, LowerLong, UpperLong);
	auto [fy_lower_boundary, fy_upper_boundary] = calculateBoundaryForces(ego, LowerLong, UpperLong);

	// Calculate combined force
	double fx{ 0 }, fy{ 0 };
	fx = ax_desired + nudge_index * fx_nudge + repulse_index * fx_repluse;
	fy = ay_desired + nudge_index * fy_nudge + repulse_index * fy_repluse + fy_pl;
	// Consider the boundary control
	double mid = (UpperLong + LowerLong) * 0.5;
	if (ego->getY() >= mid) {
		fy = MIN(fy, fy_upper_boundary);
	}
	else {
		fy = MAX(fy, fy_lower_boundary);
	};
	return std::make_tuple(fx, fy);
}

std::tuple<double, double> PotentialLines::calculateNeighbourForces(Car* ego, NumericalID* other_ids, size_t n_others) {
	double totalFX{ 0 }, totalFy{ 0 };
	for (int f = 0; f < n_others; f++) {
		NumericalID neighbour_id = other_ids[f];
		Car* neighbour = carsMap[neighbour_id];

		auto [a, b] = calculateAB(ego, neighbour);
		auto [fx, fy] = calculateForces(ego, neighbour, a, b);
		totalFX += fx;
		totalFy += fy;
	}
	return std::make_tuple(totalFX, totalFy);
}

std::tuple<double, double> PotentialLines::calculateAB(Car* ego, Car* neighbour) {
	double a{ 0 }, b{ 0 };
	a = Li * (ego->getLength() + neighbour->getLength())
		+ wx1 * (ego->getSpeedX() + neighbour->getSpeedX())
		+ wx2 * fabs(ego->getSpeedX() - neighbour->getSpeedX());
	double wi = Wi * ego->getWidth() + Wi * neighbour->getWidth();

	double item0 = (neighbour->getSpeedY() - ego->getSpeedY()) / (ego->getY() - neighbour->getY() + 0.0001);
	double p0 = sqrt(pow(item0, 2.0) + 0.001);
	b = wi + wy * (tanh(item0) + sqrt(tanh(item0 * item0) + 0.0001));
	return std::make_tuple(a, b);
}

std::tuple<double, double> PotentialLines::calculateForces(Car* ego, Car* neighbour, double a, double b) {
	double rel_dist_x = fabs(ego->getX() - neighbour->getX());
	double rel_dist_y = fabs(ego->getY() - neighbour->getY());

	double item1 = pow((2 * rel_dist_x / a), n);
	double item2 = pow((2 * rel_dist_y / b), p);

	double f = ForceIndex / (pow((item1 + item2), q) + 1);

	double theta = atan(rel_dist_y / rel_dist_x);
	double fx{ 0 }, fy{ 0 };

	fx = f * cos(theta);
	if (neighbour->getY() >= ego->getY()) {
		fy = -f * sin(theta);
	}
	else {
		fy = f * sin(theta);
	}
	return std::make_tuple(fx, fy);
}


std::tuple<double, double> PotentialLines::calculateTargetSpeedForce(Car* car) {
	double vd = get_desired_speed(car->getNumId());
	double target_speed = 1.1 * car->getSpeedX();
	double control_speed = MIN(target_speed, vd);
	double ax_desired = Kp1 * (control_speed - car->getSpeedX());
	double ay_desired = -Kp2 * car->getSpeedY();
	return std::make_tuple(ax_desired, ay_desired);
}

double PotentialLines::calculatePLForce(Car* ego, double lower_bound, double upper_bound) {
	double vd = get_desired_speed(ego->getNumId());
	double ky = 0.05;

	double cdf_value = mixed_normal_cdf(vd);
	double co = vd - MINDesiredSpeed;
	double areas = MAXDesiredSpeed - MINDesiredSpeed;
	double target_line = lower_bound + cdf_value * (upper_bound - lower_bound);
	double plForce = verordnungsindex * (target_line - ego->getY()) - ky * ego->getSpeedY();
	return plForce;
}

std::tuple<double, double> PotentialLines::calculateBoundaryForces(Car* ego, double lower_bound, double upper_bound) {
	double upper_diff = upper_bound - ego->getY();
	double upper_bound_force = kpBoundary * upper_diff - kdBoundary * ego->getSpeedY();

	double lower_diff = lower_bound - ego->getY();
	double lower_bound_force = kpBoundary * lower_diff - kdBoundary * ego->getSpeedY();
	return std::make_tuple(lower_bound_force, upper_bound_force);
}

double PotentialLines::mixed_normal_cdf(double x) {
	double lower_speed = 25.0;
	double upper_speed = x;
	int num_intervals = 1000;
	double interval_width = (upper_speed - lower_speed) / num_intervals;

	double cdf = 0.0;
	for (int i = 0; i < num_intervals; i++) {
		double x1 = lower_speed + i * interval_width;
		double x2 = x1 + interval_width;
		double y1 = mixed_normal_pdf(x1);
		double y2 = mixed_normal_pdf(x2);

		cdf += 0.5 * (y1 + y2) * interval_width;
	}
	return cdf;
}

double PotentialLines::mixed_normal_pdf(double x) {
	double coef1 = 1.0 / (sigma1 * sqrt(2.0 * PI));
	double coef2 = 1.0 / (sigma2 * sqrt(2.0 * PI));
	double exp1 = exp(-0.5 * pow((x - mu1) / sigma1, 2));
	double exp2 = exp(-0.5 * pow((x - mu2) / sigma2, 2));

	return 0.5 * (coef1 * exp1 + coef2 * exp2);
}