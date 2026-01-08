#include "PotentialLines.h"
#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#include <LaneFree.h>
#endif
#include <iostream>
#include<iomanip>

#define SAMPLE_UNIFORM(min, max) ((double)min + ((double)random()/RAND_MAX)*(max - min))
#define MAX(a, b) (((a) > (b))?(a):(b))
#define MIN(a, b) (((a) <= (b))?(a):(b))

#define PI 3.14159265358979323846

using std::map;
using std::string;

PotentialLines::PotentialLines(iniMap config): LFTStrategy(config) {
	printf("Setting parameters for Potential Lines strategy\n");
	map<string, string> secParam = config["Potential Lines Parameters"];
	FrontDistnce = stod(secParam["FrontDistance"]);
	BackDistance = stod(secParam["BackDistance"]);
	ForceIndex = stod(secParam["ForceIndex"]);
	DesireVelocityFI = stod(secParam["DesireVelocityForceIndex"]);
	PLBoundaryMargin = stod(secParam["PLBoundaryMargin"]);
	PLBoundaryMarginOffsetX = stod(secParam["PLBoundaryMarginOffsetX"]);
	wx1 = stod(secParam["wx1"]);
	wy = stod(secParam["wy"]);
	n = stoi(secParam["n"]);
	p = stoi(secParam["p"]);
	q = stoi(secParam["q"]);
	Li = stod(secParam["Li"]);
	Wi = stod(secParam["Wi"]);
	Kp1 = stod(secParam["kp1"]);
	Kp2 = stod(secParam["kp2"]);
	verordnungsindex = stod(secParam["pl_force_index"]);
	nudge_index = stod(secParam["nudge_index"]);
	repulse_index = stod(secParam["repulse_index"]);
	ReactionTime = stod(secParam["ReactionTime"]);
	Deccelerate = stod(secParam["Deceleration"]);
	Accelerate = stod(secParam["Acceleration"]);
	MinSafeGap = stod(secParam["MinSafeGap"]);
	BoundaryControlLookAhead = stod(secParam["BoundaryControlLookAhead"]);
	k1_boundary = stod(secParam["k1_boundary"]);
	k2_boundary = stod(secParam["k2_boundary"]);
	setAccAndJerkConstraints(secParam);

	auto vsafe = splitString(secParam["VSafeVehModels"], ",");
	if (vsafe[0].compare("") != 0) {
		VSafeVehModels = std::set(vsafe.begin(), vsafe.end());
	}

	// find the key in the variable that starts with "PL:"
	modelParams = extractModelSpecificParams(config, "PL:");

	MINDesiredSpeed = stod(config["General Parameters"]["min_desired_speed"]);
	MAXDesiredSpeed = stod(config["General Parameters"]["max_desired_speed"]);
	speed_mu = splitString(config["Desired Speed: Normal"]["mu"], ",");
	speed_sigma = splitString(config["Desired Speed: Normal"]["sigma"], ",");

	PLForceModel = secParam["PLForceModel"];
	if (PLForceModel.compare("CDF") != 0 && PLForceModel.compare("UNIFORM") != 0) {
		printf("Invalid PLForceModel for Potential Lines. Simulation will use default CDF model\n");
		PLForceModel = "CDF";
	}
	if (PLForceModel.compare("CDF") == 0) {
		cdf_map = calculate_cdf_vector(MINDesiredSpeed, MAXDesiredSpeed);
	}

	string ttc_file_string = secParam["ttcFile"];
	if (ttc_file_string.compare("") != 0) {
		ttc_file.open(ttc_file_string);
		ttc_file << "Time,Vehicle,Leader,gap,speed_x,leader_speed_x,TTC\n";
	}
}

void PotentialLines::finalize_simulation() {
	if (ttc_file.is_open()) {
		ttc_file.close();
	}
}

void PotentialLines::update() {
	leader_map.clear();
	assigned_pl.clear();
	for (const auto& [key, car] : carsMap) {
		std::vector<Car*> front_neighbors = getNeighbours(car, this->FrontDistnce);
		leader_map[car] = calculateLeader(car, front_neighbors);
	}
}

std::tuple<double, double>  PotentialLines::calculateAcceleration(Car* ego) {
	int cross_edge = 0;
	std::vector<Car*> front_neighbors = getNeighbours(ego, this->FrontDistnce);
	std::vector<Car*> back_neighbors = getNeighbours(ego, -this->BackDistance);
	
	double fy_pl = calculatePLForce(ego);
	auto [fx_nudge, fy_nudge] = calculateNeighbourForces(ego, back_neighbors);
	auto [fx_repluse, fy_repluse] = calculateNeighbourForces(ego, front_neighbors);
	auto [ax_desired, ay_desired] = calculateTargetSpeedForce(ego);

	// Calculate combined force
	double fx{ 0 }, fy{ 0 };
	fx = ax_desired + fx_nudge + fx_repluse;
	fy = ay_desired + fy_nudge + fy_repluse + fy_pl;
	// Consider the boundary control
	fy = controlRoadBoundary(ego, fy);

	// Limit the x-axis acceleration according to safe velocity
	if (VSafeVehModels.size() > 0){
		Car* leader = leader_map[ego];
		double ax_safe = calculateSafeAcc(ego, leader);
		fx = std::min(fx, ax_safe);

		if (ttc_file.is_open()) {
			double sim_time = get_current_time_step() * get_time_step_length();
			if (leader != nullptr) {
				double relative_speed = ego->getSpeedX() - leader->getSpeedX();
				if (relative_speed > 0) {
					double lead_gap = ego->getRelativeDistanceX(leader) - leader->getLength() / 2.0 - ego->getLength() / 2.0;
					double TTC = lead_gap / relative_speed;
					ttc_file << sim_time << "," << ego->getVehName() << "," << leader->getVehName() << std::fixed << std::setprecision(2) << "," << lead_gap << "," << ego->getSpeedX() << "," << leader->getSpeedX() << "," << TTC << std::endl;
				}
			}
		}
	}

	auto [fxC, fyC] = applyAccAndJerkConstraints(fx, fy, ego);

	return std::make_tuple(fxC, fyC);
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

Car* PotentialLines::calculateLeader(Car* ego, std::vector<Car*> front_neighbors) {
	double lower_ego_y = ego->getY() - ego->getWidth() / 2.0;
	double upper_ego_y = ego->getY() + ego->getWidth() / 2.0;
	Car* leader = nullptr;
	for (Car* car : front_neighbors) {
		double delta_x = car->getX() - ego->getX();
		if (delta_x < FrontDistnce && VSafeVehModels.find(car->getModelName()) != VSafeVehModels.end()) {
			double lower_car_y = car->getY() - car->getWidth() / 2.0;
			double upper_car_y = car->getY() + car->getWidth() / 2.0;

			if (std::max(lower_ego_y, lower_car_y) < std::min(upper_ego_y, upper_car_y)) {
				leader = car;
				break;
			}
		}
	}
	return leader;
}

double PotentialLines::calculateSafeVelocity(Car* ego, Car* leader) {
	double a = ReactionTime * Deccelerate;
	double gap = ego->getRelativeDistanceX(leader) - leader->getLength() / 2.0 - ego->getLength() / 2.0;
	double vsafe = -a + sqrt(pow(a, 2) + pow(leader->getSpeedX(), 2) + 2 * Deccelerate * (gap - MinSafeGap));
	if (gap - MinSafeGap < 0) {
		vsafe = 0;
	}
	return vsafe;
}

double PotentialLines::calculateSafeAcc(Car* ego, Car* leader) {
	double desired_speed = ego->getDesiredSpeed();
	double time_step = get_time_step_length();
	double ax{1000};
	if (leader != nullptr) {
		double vsafe = calculateSafeVelocity(ego, leader);

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

	lon_axis = li * (ego->getLength() + neighbour->getLength()) + wx1 * (ego->getSpeedX() + neighbour->getSpeedX());

	double wi = w * ego->getWidth() + w * neighbour->getWidth();
	double t1 = neighbour->getY() - ego->getY();
	double t2 = ego->getSpeedY() - neighbour->getSpeedY();
	lat_axis = wi + wy * (tanh(t1) * t2 + sqrt(pow(tanh(t1) * t2, 2) + 0.0001));

	return std::make_tuple(lon_axis, lat_axis);
}

std::tuple<double, double> PotentialLines::calculateForces(Car* ego, Car* neighbour, double major_axis, double minor_axis) {
	// The following relative distances are "neigbour" - "ego"
	double rel_dist_x = ego->getRelativeDistanceX(neighbour);
	double rel_dist_y = ego->getRelativeDistanceY(neighbour);

	double item1 = pow((2.0 * fabs(rel_dist_x) / major_axis), n);
	double item2 = pow((2.0 * fabs(rel_dist_y) / minor_axis), p);

	double desire_ego = (ego->getDesiredSpeed() - ego->getSpeedX() ) / (ego->getDesiredSpeed());
	double desire_neighbour = (neighbour->getDesiredSpeed() - neighbour->getSpeedX()) / (neighbour->getDesiredSpeed());
	double desire_factor = DesireVelocityFI * (desire_neighbour - desire_ego);
	double f = (ForceIndex + desire_factor) / (pow((item1 + item2), q) + 1);
	double fx{ 0 }, fy{ 0 };
	if (f > 0.001) {
		double theta = atan(rel_dist_y / rel_dist_x);
		fx = f * cos(theta);
		fy = f * sin(theta);

		double fx_sign = (rel_dist_x > 0) ? -1 : 1;
		double fy_sign = (rel_dist_y > 0) ? -1 : 1;

		double force_index = (rel_dist_x > 0) ? repulse_index : nudge_index;
		if (modelParams.find(neighbour->getModelName()) != modelParams.end()) {
			auto param = modelParams[neighbour->getModelName()];
			force_index = (rel_dist_x > 0) ? std::stod(param["repulse_index"]) : std::stod(param["nudge_index"]);
		}

		fx = force_index * fx_sign * fabs(fx);
		fy = force_index * fy_sign * fabs(fy);
	}
	return std::make_tuple(fx, fy);
}


std::tuple<double, double> PotentialLines::calculateTargetSpeedForce(Car* car) {
	double vd = car->getDesiredSpeed();
	double step = get_time_step_length();
	double target_speed = car->getSpeedX() + Accelerate * step;
	double control_speed = MIN(target_speed, vd);
	double ax_desired = Kp1 * (control_speed - car->getSpeedX());
	double ay_desired = -Kp2 * car->getSpeedY();
	return std::make_tuple(ax_desired, ay_desired);
}

double PotentialLines::calculatePLForce(Car* ego) {
	double edge_width = ego->getCurrentEdgeWidth();
	double lower_bound = PLBoundaryMargin;
	double upper_bound = edge_width - PLBoundaryMargin;
	auto [global_left_boundary_y, global_right_boundary_y] = ego->calBoundary(BoundaryControlLookAhead);
	lower_bound = global_right_boundary_y + PLBoundaryMargin;
	upper_bound = global_left_boundary_y - PLBoundaryMargin;

	double target_line;
	if (PLForceModel.compare("UNIFORM") == 0) {
		target_line = calculateTargetLineUniform(ego, lower_bound, upper_bound);
	}
	else {
		target_line = calculateTargetLineCDF(ego, lower_bound, upper_bound);
	}
	assigned_pl[ego] = target_line;
	auto [global_x, global_y] = ego->getGlobalPosition();
	double fy_pl = verordnungsindex * (target_line - global_y);
	return fy_pl;
}

double PotentialLines::calculateTargetLineCDF(Car* ego, double lower_bound, double upper_bound) {
	double vd = get_desired_speed(ego->getNumId());
	int vd_key = 1000 * std::floor(vd * 1000.0);
	// Find the vd_key in cdf_map otherwise throw an error
	if (cdf_map.find(vd_key) == cdf_map.end()) {
		printf("Desired speed %f not found in cdf_map\n", vd);
		return 0;
	}
	double cdf_value = cdf_map[vd_key];
	double co = vd - MINDesiredSpeed;
	double areas = MAXDesiredSpeed - MINDesiredSpeed;
	double target_line = lower_bound + cdf_value * (upper_bound - lower_bound);
	return target_line;
}



double PotentialLines::calculateTargetLineUniform(Car* ego, double lower_bound, double upper_bound) {
	double vd = ego->getDesiredSpeed();

	double co = vd - MINDesiredSpeed;
	double areas = MAXDesiredSpeed - MINDesiredSpeed;
	double target_line = lower_bound + ((upper_bound - lower_bound) / areas) * co;
	return target_line;
}

double PotentialLines::controlRoadBoundary(Car* ego, double ay) {
	double step = get_time_step_length();
	auto [left_boundary_dist, right_boundary_dist] = ego->calDistanceToBoundary(BoundaryControlLookAhead, 0);
	
	double left_bound = left_boundary_dist - ego->getWidth() / 2.0;
	double left_boundary_acc_limit = k1_boundary * left_bound - k2_boundary * ego->getSpeedY();

	// The right boundary is below the vehicle so use the negative sign.
	double right_bound = - right_boundary_dist + ego->getWidth() / 2.0;
	double right_boundary_acc_limit = k1_boundary * right_bound - k2_boundary * ego->getSpeedY();

	double fy = MIN(ay, left_boundary_acc_limit);
	fy = MAX(fy, right_boundary_acc_limit);
	return fy;
}

std::map<int, double> PotentialLines::calculate_cdf_vector(double min_speed, double max_speed) {
	double step_size = 0.001;
	int last_key = (int)min_speed * 100000;
	std::map<int, double> cdfs;
	cdfs[last_key] = 0;
	double x = min_speed;
	double weight = 1.0 / speed_mu.size();
	double y_last = 0;

	while (x < max_speed) {
		x += step_size;
		double y = 0;
		for (int j = 0; j < speed_mu.size(); j++) {
			double mu = stod(speed_mu[j]);
			double sigma = stod(speed_sigma[j]);
			y += weight * normal_pdf(x, mu, sigma);
		}
		int key = 1000 * std::floor(x * 1000.0);
		cdfs[key] = cdfs[last_key] + 0.5 * (y_last + y) * step_size;
		y_last = y;
		last_key = key;
	}
	return cdfs;
}

double PotentialLines::normal_pdf(double x, double mu, double sigma) {
	double coef = 1.0 / (sigma * sqrt(2.0 * PI));
	double expn = exp(-0.5 * pow((x - mu) / sigma, 2));
	return coef * expn;
}