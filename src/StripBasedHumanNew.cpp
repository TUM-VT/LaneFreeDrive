#include "StripBasedHumanNew.h"
#include "Controller.h"
#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#include <LaneFree.h>
#endif
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <numeric>
#include <set>
#include <random>

using std::map;
using std::string;
using std::vector;
using std::tuple;


NetworkStrips::NetworkStrips(double strip_width, double look_ahead_distance) {
	this->strip_width = strip_width;
	this->look_ahead_distance = look_ahead_distance;
}

void NetworkStrips::update(Car* car) {
	// For simplicity, strips are calculated according to the road boundaries which can be different from the actual road boundaries.
	// This assumes that the first strip starts at the lateral location of the the right most boundary.
	auto [left_boundary_dist, right_boundary_dist] = car->calDistanceToBoundary(look_ahead_distance, 0);
	boundary_widths[car] = left_boundary_dist + right_boundary_dist;
	boundary_strip_counts[car] = floor((left_boundary_dist + right_boundary_dist) / strip_width);
	// also track the lateral location of the zero strip relative to the right side of the current edge.
	zero_strip_y[car] = car->getY() - right_boundary_dist;

	if (carAssignedStrip.find(car) == carAssignedStrip.end()) {
		double car_start_y = right_boundary_dist - car->getWidth() / 2.0;
		carAssignedStrip[car] = floor(car_start_y / strip_width);
	}
	// Due to changing boundary values it is possible that the vehicle position is now out of the boundary strip limits. 
	// In this case, we assign the vehicle to the closest strip within the limits.
	else if (left_boundary_dist < 0) {
		carAssignedStrip[car] = calculateStripLimit(car);
	}
	else if (right_boundary_dist < 0) {
		carAssignedStrip[car] = 0;
	}
}

void NetworkStrips::vehicle_exit(Car* car) {
	carAssignedStrip.erase(car);
	boundary_widths.erase(car);
	boundary_strip_counts.erase(car);
	zero_strip_y.erase(car);
}

double NetworkStrips::getYFromInx(Car* car, int index) {
	if (index > boundary_strip_counts[car]) {
		throw std::invalid_argument("received out of index value for the boundaries of car");
	}
	return strip_width * index + zero_strip_y[car];
}

int NetworkStrips::getInxFromY(Car* car, double y) {
	double y_relative_to_zero_strip = y - zero_strip_y[car];
	int strip_inx = floor(y_relative_to_zero_strip / strip_width);
	if (strip_inx < 0 || strip_inx > boundary_strip_counts[car]) {
		strip_inx = std::max(0, strip_inx);
		strip_inx = std::min(strip_inx, boundary_strip_counts[car]);
	}
	return strip_inx;
}

void NetworkStrips::shiftAssignedStrip(Car* car, int delta_strip) {
	int strip_inx = carAssignedStrip[car];
	int new_strip_inx = strip_inx + delta_strip;
	if (new_strip_inx < 0 || new_strip_inx > boundary_strip_counts[car]) {
		throw std::invalid_argument("received out of index value for strip");
	}
	carAssignedStrip[car] = new_strip_inx;
}

int NetworkStrips::calculateStripLimit(Car* car) {
	double max_y = boundary_widths[car] - car->getWidth();
	int max_strip = floor(max_y / strip_width);
	return max_strip;
}

void StripBasedHumanNew::update() {
	for (auto& [car_id, car] : carsMap) {
		network_strips.update(car);
	}
	
}

void StripBasedHumanNew::vehicle_exit(Car* car) {
	driverMemory.erase(car);
	ReactionTimesMap.erase(car);
	network_strips.vehicle_exit(car);
}

StripBasedHumanNew::StripBasedHumanNew(iniMap config): LFTStrategy(config) {
	printf("Setting parameters for Strip Based Human Driver strategy\n");
	map<string, string> secParam = config["Strip Based Human Parameters"];

	map<Car*, double> ReactionTimeMap;
	if (secParam["ReactionTime"].compare("RANDOM") == 0) {
		ReactionTime = std::nan("");
		rng = std::mt19937(std::stoi(config["General Parameters"]["seed"]));
		std::vector<string> reaction_range = splitString(secParam["ReactionTimeRange"], ",");
		reaction_distribution = std::normal_distribution<double>(std::stod(reaction_range[0]), std::stod(reaction_range[1]));
	}
	else {
		ReactionTime = stod(secParam["ReactionTime"]);
	}
	k1 = stod(secParam["k1"]);
	k2 = stod(secParam["k2"]);
	off_ramp_desire_mid_point = stod(secParam["off_ramp_desire_mid_point"]);
	off_ramp_desire_spread = stod(secParam["off_ramp_desire_spread"]);
	off_ramp_desire_lambda = stod(secParam["off_ramp_desire_lambda"]);
	StripWidth = stod(secParam["StripWidth"]);
	FrontDistance = stod(secParam["FrontDistance"]);
	Deccelerate = stod(secParam["Deceleration"]);
	MaxBrakeDeceleration = stod(secParam["MaxBrakeDeceleration"]);
	Accelerate = stod(secParam["Acceleration"]);
	MinSafeGap = stod(secParam["MinSafeGap"]);
	Lambda = stod(secParam["Lambda"]);
	numStripsConsidered = -std::log(0.01) / Lambda;
	LaneChangeThreshold = stod(secParam["LaneChangeThreshold"]);
	string file_path = secParam["StripsChangeFile"];
	if (file_path.compare("") != 0) {
		StripsChangeFile.open(file_path);
		StripsChangeFile << "Time,Vehicle,From,To,Willingness Right,Willingness Left\n";
	}
	network_strips = NetworkStrips(StripWidth, stod(secParam["StripsLookAheadDistance"]));
}

double StripBasedHumanNew::calculateSafeVelocity(Car* ego, Car* leader, double gap) {
	double reaction_time = ReactionTime;
	if (std::isnan(ReactionTime)) {
		if (ReactionTimesMap.find(ego) == ReactionTimesMap.end()) {
			reaction_time = reaction_distribution(rng);
			ReactionTimesMap[ego] = reaction_time;
		}
		else {
			reaction_time = ReactionTimesMap[ego];
		}
	}
	double a = reaction_time * Deccelerate;
	double vsafe = -a + sqrt(pow(a, 2) + pow(leader->getSpeedX(), 2) + 2 * Deccelerate * (gap-MinSafeGap));
	return vsafe;
}

Car* StripBasedHumanNew::calculateFirstLateralOverlap(Car* ego, std::vector<Car*> neighbors) {
	double lower_ego_y = ego->getY() - ego->getWidth() / 2.0;
	double upper_ego_y = ego->getY() + ego->getWidth() / 2.0;
	Car* leader = nullptr;
	for (Car* car : neighbors) {
		double lower_car_y = car->getY() - car->getWidth() / 2.0;
		double upper_car_y = car->getY() + car->getWidth() / 2.0;

		if (std::max(lower_ego_y, lower_car_y) < std::min(upper_ego_y, upper_car_y)) {
			leader = car;
			break;
		}
	}
	return leader;
}

std::unordered_map<int, tuple<double, Car*>> StripBasedHumanNew::calculateSafeVelocities(Car* ego, vector<Car*> front_cars) {

	int edge_strips_count = network_strips.getStripsCount(ego);
	int ego_lw_strip = network_strips.getAssignedStripInx(ego);
	int ego_up_strip = network_strips.getInxFromY(ego, ego->getY() + ego->getWidth() / 2.0) + 1;
	int numocc = ego_up_strip - ego_lw_strip;
	int count_indices = 0;

	std::unordered_map<int, tuple<double, Car*>> safe_velocity_map;
	safe_velocity_map.reserve(edge_strips_count);
	for (int i = 0; i < edge_strips_count; i++) {
		safe_velocity_map[i] = std::make_tuple(std::nan(""), nullptr);
	}

	for (Car* car : front_cars) {
		int car_lw_inx = network_strips.getInxFromY(ego, car->getY() - car->getWidth() / 2.0);
		int car_up_inx = network_strips.getInxFromY(ego, car->getY() + car->getWidth() / 2.0);
		car_up_inx += 1;

		int overlap_lower = std::max(car_lw_inx - numocc, 0);
		int overlap_upper = std::min(car_up_inx, edge_strips_count - 1);

		double gap = ego->getRelativeDistanceX(car) - car->getLength() / 2.0 - ego->getLength() / 2.0;
		double safe_vel = calculateSafeVelocity(ego, car, gap);
		if (gap < 0) {
			safe_vel = 0;
		}

		for (int k = overlap_lower; k <= overlap_upper; k++) {
			if (std::get<1>(safe_velocity_map[k]) == nullptr) {
				count_indices++;
				safe_velocity_map[k] = std::make_tuple(safe_vel, car);
			}
		}
		if (count_indices == edge_strips_count) {
			break;
		}
	}
	return safe_velocity_map;
}

void StripBasedHumanNew::updateStripChangeBenefit(Car* ego, std::unordered_map<int, tuple<double, Car*>> safeVelMap, double vsafe_current) {
	int total_strips = network_strips.getStripsCount(ego);
	int current_strip = network_strips.getAssignedStripInx(ego);
	int upper_current_strip = network_strips.getInxFromY(ego, ego->getY() + ego->getWidth() / 2.0) + 1;
	int numOccupied = upper_current_strip - current_strip;

	int left_most = std::min(current_strip + (int)numStripsConsidered, total_strips - 1);
	int right_most = std::max(current_strip - (int)numStripsConsidered, 0);

	double right = 0;
	double left = 0;
	for (int inx = right_most; inx < left_most + 1; inx++) {
		double vsafe_neighbour = ego->getDesiredSpeed();
		auto [vsafe, car] = safeVelMap[inx];
		if (car != nullptr) {
			vsafe_neighbour = vsafe;
		}

		double diff_inx = std::abs(current_strip - inx);
		double benefit = ((vsafe_neighbour - vsafe_current) / ego->getDesiredSpeed()) * std::exp(-Lambda * diff_inx);
		if (inx < current_strip) {
			right += benefit;
		}
		else if (inx > current_strip) {
			left += benefit;
		}
	}

	// If it is an off-ramp vehicle, then add additional benefit to the right side
	if (ego->getIfOffRampVeh()) {
		double dist_to_off_ramp = ego->calDistanceToRampEnd();
		if (dist_to_off_ramp > -1) {
			double benefit = 1 / (1 + std::exp(off_ramp_desire_lambda * (dist_to_off_ramp - off_ramp_desire_mid_point) / off_ramp_desire_spread));
			double lateral_ratio = ego->getY() / (network_strips.getBoundaryWidth(ego) - ego->getWidth());
			benefit = LaneChangeThreshold * lateral_ratio * benefit;
			right += benefit;
		}
	}

	// Initiate driver memory for new cars
	if (driverMemory.find(ego) == driverMemory.end()) {
		driverMemory[ego] = { 0, 0 };
	}

	if (left > 0) {
		driverMemory[ego][0] += left;
	}
	else {
		driverMemory[ego][0] /= 2.0;
	}

	if (right > 0) {
		driverMemory[ego][1] += right;
	}
	else {
		driverMemory[ego][1] /= 2.0;
	}

	// If the driver is on the edges of the road, then set the corresponding driver memory to 0
	if (current_strip <= 1) {
		driverMemory[ego][1] = 0;
	}
	else if (current_strip + numOccupied >= total_strips) {
		driverMemory[ego][0] = 0;
	}

}

bool StripBasedHumanNew::isSufficientGap(Car* ego, double x, double y, vector<Car*> neighbours) {

	bool sufficient_gap = true;
	Car* overlap_car = calculateFirstLateralOverlap(ego, neighbours);
	if (overlap_car != nullptr) {
		double diff_vel_x = ego->getSpeedX() - overlap_car->getSpeedX();
		double brake_distance = pow(diff_vel_x, 2) / (2 * Deccelerate) + MinSafeGap;
		double gap;
		if (overlap_car->getRelativeDistanceX(ego) > 0) {
			gap = overlap_car->getRelativeDistanceX(ego);
		}
		else {
			gap = ego->getRelativeDistanceX(overlap_car);
		}
		gap = gap - overlap_car->getLength() / 2.0 - ego->getLength() / 2.0;

		if (gap < brake_distance) {
			sufficient_gap = false;
		}
	}
	return sufficient_gap;

}

tuple<double, double> StripBasedHumanNew::calculateAcceleration(Car* ego) {
	vector<Car*> front_cars = getNeighbours(ego, FrontDistance);
	vector<Car*> back_cars = getNeighbours(ego, -FrontDistance);

	/* Calculate the longitudinal acceleration */
	double desired_speed = ego->getDesiredSpeed();
	double time_step = get_time_step_length();

	Car* leader = calculateFirstLateralOverlap(ego, front_cars);
	double vsafe_x{ desired_speed }, gap;
	if (leader != nullptr) {
		gap = ego->getRelativeDistanceX(leader) - leader->getLength() / 2.0 - ego->getLength() / 2.0;
		vsafe_x = calculateSafeVelocity(ego, leader, gap);
		if (std::isnan(vsafe_x)) {
			vsafe_x = 0;
		}
	};
	double max_vel_x = ego->getSpeedX() + time_step * Accelerate;		// The max velocity possible in the next time step
	double next_vel_x = std::min({ vsafe_x, desired_speed, max_vel_x });
	
	double diff_vel_x = next_vel_x - ego->getSpeedX();
	double ax = 0;
	if (diff_vel_x < 0) {
		ax = -std::min(std::abs(diff_vel_x / time_step), Deccelerate);
		// Check if the allowed deceleration is sufficient to reach the desired speed with given normal deceleration
		if (leader != nullptr) {
			double brake_distance = pow(diff_vel_x, 2) / (2 * Deccelerate) + MinSafeGap;
			if (gap < brake_distance) {
				ax = -std::min({ std::abs(diff_vel_x / time_step), MaxBrakeDeceleration });
			}
		}
	}
	else {
		ax = std::min(diff_vel_x / time_step, Accelerate);
	}

	std::unordered_map<int, tuple<double, Car*>> safe_vel_map = calculateSafeVelocities(ego, front_cars);

	/* Calculate lateral acceleration */
	updateStripChangeBenefit(ego, safe_vel_map, vsafe_x);

	double left_benefit = driverMemory[ego][0];
	double right_benefit = driverMemory[ego][1];

	int current_strip = network_strips.getAssignedStripInx(ego);
	double target_y = network_strips.getYFromInx(ego, current_strip) + ego->getWidth() / 2.0;

	if ((left_benefit > LaneChangeThreshold) || (right_benefit > LaneChangeThreshold)) {
		int delta_inx = (left_benefit > right_benefit) ? 1 : -1;
		if (current_strip + delta_inx <= network_strips.calculateStripLimit(ego)) {

			double new_y = network_strips.getYFromInx(ego, current_strip + delta_inx) + ego->getWidth() / 2.0;
			bool is_sufficient_gap_front = isSufficientGap(ego, ego->getX(), new_y, front_cars);
			bool is_sufficient_gap_back = isSufficientGap(ego, ego->getX(), new_y, back_cars);
			if (is_sufficient_gap_front && is_sufficient_gap_back) {
				target_y = new_y;
				network_strips.shiftAssignedStrip(ego, delta_inx);
				if (StripsChangeFile.is_open()) {
					double time = get_time_step_length() * get_current_time_step();
					StripsChangeFile << time << "," << ego->getVehName() << "," << current_strip << "," << current_strip + delta_inx << "," << right_benefit << "," << left_benefit << "\n";
				}
			}
		}
	}
	double ay = -k1 * (ego->getY() - target_y) - k2 * ego->getSpeedY();

	return std::make_tuple(ax, ay);
}

void StripBasedHumanNew::finalize_simulation() {
	if (StripsChangeFile.is_open()) {
		StripsChangeFile.close();
	}
}