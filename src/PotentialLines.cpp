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

PotentialLines::PotentialLines(iniMap config) {
	printf("Setting parameters for Potential Lines strategy");
	auto it = config.find("Potential Lines Parameters");
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
	Kp2 = stod(secParam["kp2"]);
	MINDesiredSpeed = stod(secParam["min_desired_speed"]);
	MAXDesiredSpeed = stod(secParam["max_desired_speed"]);
	verordnungsindex = stod(secParam["pl_force_index"]);
	nudge_index = stod(secParam["nudge_index"]);
	repulse_index = stod(secParam["repulse_index"]);
	ReactionTime = stod(secParam["ReactionTime"]);
	Deccelerate = stod(secParam["Deceleration"]);
	auto vsafe = splitString(secParam["VSafeVehModels"], ",");
	if (vsafe[0].compare("") != 0) {
		VSafeVehModels = std::set(vsafe.begin(), vsafe.end());
	}

	// find the key in the variable that starts with "PL:"
	modelParams = extractModelSpecificParams(config, "PL:");
}

std::tuple<double, double>  PotentialLines::calculateAcceleration(Car* ego) {
	int cross_edge = 0;
	std::vector<Car*> front_neighbors = getNeighbours(ego, this->FrontDistnce);
	std::vector<Car*> back_neighbors = getNeighbours(ego, -this->BackDistance);

	auto [fx_nudge, fy_nudge] = calculateNeighbourForces(ego, back_neighbors);
	auto [fx_repluse, fy_repluse] = calculateNeighbourForces(ego, front_neighbors);
	auto [ax_desired, ay_desired] = calculateTargetSpeedForce(ego);
	double fy_pl = calculatePLForce(ego, LowerLong, UpperLong);

	// Calculate combined force
	double fx{ 0 }, fy{ 0 };
	fx = ax_desired + fx_nudge + fx_repluse;
	fy = ay_desired + fy_nudge + fy_repluse + fy_pl;
	// Consider the boundary control
	fy = controlRoadBoundary(ego, fy);

	// Limit the x-axis acceleration according to safe velocity
	if (VSafeVehModels.size() > 0){
		double ax_safe = calculateSafeAcc(ego, front_neighbors);
		fx = std::min(fx, ax_safe);
	}

	return std::make_tuple(fx, fy);
}

std::tuple<double, double> PotentialLines::calculateNeighbourForces(Car* ego, std::vector<Car*> neighbours) {
	double totalFX{ 0 }, totalFy{ 0 };
	for (Car* neighbour : neighbours){
		auto [major, minor] = calculatePotentialFunMajorMinorAxis(ego, neighbour);
		auto [fx, fy] = calculateForces(ego, neighbour, major, minor);
		totalFX += fx;
		totalFy += fy;
	}
	return std::make_tuple(totalFX, totalFy);
}

double PotentialLines::calculateSafeAcc(Car* ego, std::vector<Car*> front_neighbors) {
	double lower_ego_y = ego->getY() - ego->getWidth() / 2.0;
	double upper_ego_y = ego->getY() + ego->getWidth() / 2.0;

	Car* leader = nullptr;
	for (Car* car : front_neighbors) {
		double delta_x = car->getX() - ego->getX();
		if (delta_x < FrontDistnce && VSafeVehModels.find(car->getModelName()) != VSafeVehModels.end()) {
			double lower_car_y = car->getY() - car->getWidth() / 2.0;
			double upper_car_y = car->getY() + car->getWidth() / 2.0;
			if ((lower_ego_y < upper_car_y && upper_car_y < upper_ego_y) || (lower_ego_y < lower_car_y && lower_car_y < upper_ego_y)) {
				leader = car;
				break;
			}
		}
	}
	
	double desired_speed = get_desired_speed(ego->getNumId());
	double time_step = get_time_step_length();
	double ax{1000};
	if (leader != nullptr) {
		double a = ReactionTime * Deccelerate;
		double gap = leader->getX() - leader->getLength() / 2.0 - (ego->getX() + ego->getLength() / 2.0);
		double vsafe = -a + sqrt(pow(a, 2) + pow(leader->getSpeedX(), 2) + 2 * Deccelerate * gap);

		double diff_vel_x = vsafe - ego->getSpeedX();
		if (diff_vel_x < 0) {
			ax = -std::min(std::abs(diff_vel_x / time_step), Deccelerate);
		}
		else {
			ax = diff_vel_x / time_step;
		}
	}

	return ax;
}

std::tuple<double, double> PotentialLines::calculatePotentialFunMajorMinorAxis(Car* ego, Car* neighbour) {
	double lon_axis{ 0 }, lat_axis{ 0 };
	double li {Li}, w{Wi};
	if (modelParams.find(neighbour->getModelName()) != modelParams.end()){
		auto param = modelParams[neighbour->getModelName()];
		li = std::stod(param["Li"]);
		w = std::stod(param["Wi"]);
	}

	lon_axis = li * (ego->getLength() + neighbour->getLength())
		+ wx1 * (ego->getSpeedX() + neighbour->getSpeedX())
		+ wx2 * fabs(ego->getSpeedX() - neighbour->getSpeedX());
	double wi = w * ego->getWidth() + w * neighbour->getWidth();
	// double item0 = (ego->getY() - neighbour->getY()) / (neighbour->getSpeedY() - ego->getSpeedY() + 0.0001);
	// lat_axis = wi + wy * (tanh(item0) + sqrt(pow(tanh(item0), 2) + 0.0001));

	double t1 = neighbour->getY() - ego->getY();
	double t2 = ego->getSpeedY() - neighbour->getSpeedY();
	lat_axis = wi + wy * (tanh(t1) * t2 + sqrt(pow(tanh(t1) * t2, 2) + 0.0001));

	return std::make_tuple(lon_axis, lat_axis);
}

std::tuple<double, double> PotentialLines::calculateForces(Car* ego, Car* neighbour, double major_axis, double minor_axis) {
	double neighbour_x = neighbour->getCircularX();
	double rel_dist_x = fabs(ego->getX() - neighbour_x);
	double rel_dist_y = fabs(ego->getY() - neighbour->getY());

	double item1 = pow((2.0 * rel_dist_x / major_axis), n);
	double item2 = pow((2.0 * rel_dist_y / minor_axis), p);

	double f = ForceIndex / (pow((item1 + item2), q) + 1);
	double fx{ 0 }, fy{ 0 };
	if (f > 0.001) {
		double theta = atan((ego->getY() - neighbour->getY()) / (ego->getX() - neighbour_x));
		fx = f * cos(theta);
		fy = f * sin(theta);

		double fx_sign = (ego->getX() < neighbour_x) ? -1 : 1;
		double fy_sign = (ego->getY() < neighbour->getY()) ? -1 : 1;

		fx = fx_sign * fabs(fx);
		fy = fy_sign * fabs(fy);
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
	double plForce = verordnungsindex * (target_line - ego->getY()) + ky * ego->getSpeedY();
	return plForce;
}

double PotentialLines::controlRoadBoundary(Car* ego, double ay) {
	double step = get_time_step_length();
	double road_width = get_edge_width(ego->getCurrentEdge());

	double upper_diff = road_width - (ego->getY() + ego->getWidth() / 2.0);
	double speed_to_reach_up = upper_diff / step;
	double upper_lim = (speed_to_reach_up - ego->getSpeedY()) / step;

	double lower_diff = ego->getY() - ego->getWidth() / 2.0;
	double speed_to_reach_down = -lower_diff / step;
	double lower_lim = (speed_to_reach_down - ego->getSpeedY()) / step;

	double fy = MIN(ay, upper_lim);
	fy = MAX(fy, lower_lim);
	return fy;
}

double PotentialLines::mixed_normal_cdf(double x) {
	double lower_speed = MINDesiredSpeed;
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