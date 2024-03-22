#include "StripBasedHuman.h"
#include "Controller.h"
#include "LaneFree.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <numeric>

using std::map;
using std::string;
using std::vector;
using std::tuple;

EdgeStrips::EdgeStrips(NumericalID edge_id, double edge_width, double strip_width) {
	this->edge_id = edge_id;
	this->strip_width = strip_width;
	this->edge_width = edge_width;
	total_strips = floor(edge_width / strip_width);
}

StripInfo EdgeStrips::getVehicleStripInfo(Car* car) {
	return carOccupancyMap[car];
}

double EdgeStrips::getYFromInx(int index) {
	if (index > total_strips) {
		throw std::invalid_argument("received out of index value for strip");
	}
	return strip_width * index;
}

tuple<int, int> EdgeStrips::calculateStripInx(Car* car) {
	double lower_bound = car->getY() - car->getWidth() / 2.0;
	double upper_bound = car->getY() + car->getWidth() / 2.0;
	if (upper_bound > edge_width || lower_bound < 0) {
		printf("\nWarning: vehicle %s of type %s is crossing the edge boundary", car->getVehName(), car->getTypeName());
	}
	int main_strip_inx = floor(lower_bound / strip_width);
	int num_occupied = ceil(upper_bound / strip_width) - main_strip_inx;
	return std::make_tuple(main_strip_inx, num_occupied);
}

vector<Car*> EdgeStrips::getStripVehicles(int index) {
	vector<Car*> strip_cars;
	auto it = stripOccupancyMap.find(index);
	if (it != stripOccupancyMap.end()) {
		strip_cars = it->second;
	}
	return strip_cars;
}

void EdgeStrips::updateOccupiedMap(std::map<NumericalID, Car*>& carsMap) {
	carOccupancyMap.clear();
	stripOccupancyMap.clear();
	NumericalID n_edge_ids = get_all_ids_in_edge_size(edge_id);
	NumericalID* ids_in_edge = get_all_ids_in_edge(edge_id);

	for (int j = 0; j < n_edge_ids; j++) {
		Car* ego = carsMap[ids_in_edge[j]];
		StripInfo strip_info;
		auto [mainInx, upperInx] = calculateStripInx(ego);
		strip_info.mainInx = mainInx;
		strip_info.numOccupied = upperInx;
		carOccupancyMap[ego] = strip_info;
		stripOccupancyMap[mainInx].push_back(ego);
	}
}

int EdgeStrips::calulcateStripPos(Car* car) {
	double y = car->getY();
	double strip_y = strip_width * floor(y / strip_width);
	return strip_y;
}

void StripBasedHuman::update() {
	NumericalID* edges = get_all_edges();
	NumericalID n_edges = get_all_edges_size();

	for (int i = 0; i < n_edges; i++) {
		NumericalID edge_id = edges[i];
		edgeStrips[edge_id]->updateOccupiedMap(carsMap);
	}
}

StripBasedHuman::StripBasedHuman(iniMap config) {
	printf("\nSetting parameters for Strip Based Human Driver strategy");
	auto it = config.find("Strip Based Human Parameters");
	map<string, string> secParam = it->second;

	ReactionTime = stod(secParam["ReactionTime"]);
	StripWidth = stod(secParam["StripWidth"]);
	FrontDistance = stod(secParam["FrontDistance"]);
	Deccelerate = stod(secParam["Deceleration"]);
	Accelerate = stod(secParam["Acceleration"]);
	Lambda = stod(secParam["Lambda"]);
	LaneChangeThreshold = stod(secParam["LaneChangeThreshold"]);

	NumericalID* all_edges = get_all_edges();
	int num_edges = get_all_edges_size();
	for (int i = 0; i < num_edges; i++) {
		NumericalID edge_id = all_edges[i];
		double width = get_edge_width(edge_id);
		EdgeStrips* strip_edge = new EdgeStrips(edge_id, width, StripWidth);
		edgeStrips[edge_id] = strip_edge;
	}
}

double StripBasedHuman::calculateSafeVelocity(Car* ego, Car* leader) {
	double a = ReactionTime * Deccelerate;
	double gap = leader->getX() - leader->getLength() - ego->getX();
	double vsafe = -a + sqrt(pow(a, 2) + pow(leader->getSpeedX(), 2) + 2 * Accelerate * gap);
	return vsafe;
}

bool compareCar(Car* car1, Car* car2) { return car1->getX() < car2->getX(); }

tuple<vector<Car*>, vector<Car*>> StripBasedHuman::calculateFollowerLeader(Car* ego, vector<int> strip_indices) {
	EdgeStrips* strip = edgeStrips[ego->getCurrentEdge()];
	double ego_x = ego->getX();
	vector<Car*> leaders;
	vector<Car*> followers;

	for (int i : strip_indices) {
		vector<Car*> strip_cars = strip->getStripVehicles(i);
		Car* lead_car = nullptr;
		Car* follow_car = nullptr;
		for (Car* car : strip_cars) {
			double diff_x = car->getX() - car->getLength() - ego_x;
			// The vector strip_cars is already arranged in order of longitudinal distance
			if (diff_x > 0 && car->getNumId() != ego->getNumId()) {
				lead_car = car;
				break;
			}
			if (diff_x < 0 && car->getNumId() != ego->getNumId()) {
				follow_car = car;
			}
		}
		leaders.push_back(lead_car);
		followers.push_back(lead_car);
	}
	return std::make_tuple(leaders, followers);
}

std::map<int, tuple<double, Car*>> StripBasedHuman::calculateSafeVelocities(Car* ego) {
	EdgeStrips* strip = edgeStrips[ego->getCurrentEdge()];
	double ego_x = ego->getX();
	std::map<int, tuple<double, Car*>> safe_velocity_map;
	vector<int> indices(strip->getTotalNoStrips());
	std::iota(indices.begin(), indices.end(), 0);

	auto[followers, leaders] = calculateFollowerLeader(ego, indices);
	for (int i : indices) {
		Car* leader = leaders[i];
		if (leader != nullptr) {
			double safe_velocity = calculateSafeVelocity(ego, leader);
			safe_velocity_map[i] = std::make_tuple(safe_velocity, leader);
		}
	}
	return safe_velocity_map;
}

tuple<int, double, Car*> StripBasedHuman::calculateLeaderFromSafeVelMap(Car* ego, std::map<int, tuple<double, Car*>> safeVelMap) {
	EdgeStrips* ego_strip = edgeStrips[ego->getCurrentEdge()];
	StripInfo ego_strip_info = ego_strip->getVehicleStripInfo(ego);
	double ego_x = ego->getX();
	double closest_diff = 1e12;
	Car* leader = nullptr;
	double closest_vsafe, closest_inx;
	for (int i = 0; i < ego_strip_info.numOccupied; i++) {
		int inx = ego_strip_info.mainInx + i;
		if (safeVelMap.find(inx) != safeVelMap.end()) {
			auto [vsafe, car] = safeVelMap[inx];
			double diff = car->getX() - ego_x;
			if (diff < closest_diff && diff < FrontDistance) {
				closest_diff = diff;
				leader = car;
				closest_vsafe = vsafe;
				closest_inx = inx;
			}
		}
	}
	return std::make_tuple(closest_inx, closest_vsafe, leader);
}

void StripBasedHuman::updateStripChangeBenefit(Car* ego, std::map<int, tuple<double, Car*>> safeVelMap) {
	EdgeStrips* ego_strip = edgeStrips[ego->getCurrentEdge()];
	StripInfo ego_strip_info = ego_strip->getVehicleStripInfo(ego);
	int total_strips = ego_strip->getTotalNoStrips();
	double vel_max = get_desired_speed(ego->getNumId());

	double vsafe_current = vel_max;
	int main_inx = ego_strip_info.mainInx;
	if (safeVelMap.find(main_inx) != safeVelMap.end()) {
		auto [vsafe, car] = safeVelMap[main_inx];
		vsafe_current = vsafe;
	}
	
	double right = 0;
	double left = 0;
	for (int inx = 0; inx < total_strips; inx++) {
		double vsafe_neighbour = vel_max;
		if (safeVelMap.find(inx) != safeVelMap.end()) {
			auto [vsafe, car] = safeVelMap[inx];
			vsafe_neighbour = vsafe;
		}
		double diff_inx = std::abs(main_inx - inx);
		double benefit = ((vsafe_neighbour - vsafe_current) / vel_max) * std::exp(-Lambda * diff_inx);
		if (inx < main_inx) { right += benefit; }
		else if (inx > main_inx) { left += benefit; }
	}

	if (left > 0) {
		driverMemory[ego] += left;
	}
	else if (right > 0) {
		driverMemory[ego] += right;
	}
	else {
		driverMemory[ego] /= 2.0;
	}

}

bool StripBasedHuman::isSufficientGap(Car* ego, int strip_inx) {
	auto [followers, leaders] = calculateFollowerLeader(ego, { strip_inx });
	double delta_t = get_time_step_length();
	bool sufficient_gap = true;
	double leader_gap, follow_gap;

	double ego_nextx = ego->getX() + delta_t * ego->getSpeedX();
	if (leaders[0] != nullptr) {
		double leader_nextx = leaders[0]->getX() + delta_t * leaders[0]->getSpeedX();
		if (std::abs(ego_nextx - leader_nextx - leaders[0]->getLength()) < 0) {
			sufficient_gap = false;
		}
	}
	if (followers[0] != nullptr) {
		double follower_nextx = followers[0]->getX() + delta_t * followers[0]->getSpeedX();
		if (std::abs(ego_nextx - follower_nextx - ego->getLength()) < 0) {
			sufficient_gap = false;
		}
	}
	return sufficient_gap;
}

bool StripBasedHuman::isCrossingRoadBoundary(Car* car, int strip_inx, EdgeStrips* strip) {
	double y = strip->getYFromInx(strip_inx);
	double car_width = car->getWidth();
	double road_width = strip->getEdgeWidth();
	bool violation = false;
	if (y < 0 || (y + car_width > road_width)) {
		violation = true;
	}
	return violation;
}

std::tuple<bool, bool> StripBasedHuman::isLaneChangePossible(Car* ego) {
	EdgeStrips* ego_strip = edgeStrips[ego->getCurrentEdge()];
	int ego_inx = ego_strip->getVehicleStripInfo(ego).mainInx;
	int total_strips = ego_strip->getTotalNoStrips();
	
	bool right_side = false;
	if (ego_inx > 0) {
		right_side = isSufficientGap(ego, ego_inx - 1);
		right_side = right_side && !isCrossingRoadBoundary(ego, ego_inx - 1, ego_strip);
	}

	bool left_side = false;
	if (ego_inx < total_strips - 1) {
		left_side = isSufficientGap(ego, ego_inx + 1);
		left_side = left_side && !isCrossingRoadBoundary(ego, ego_inx + 1, ego_strip);
	}

	return std::make_tuple(right_side, left_side);
	
}

double StripBasedHuman::calculateStopLatAcc(Car* car) {
	double time_step = get_time_step_length();
	double current_speed = car->getSpeedY();
	int sign = (current_speed > 0) ? 1 : -1;
	double ay = -sign * current_speed / time_step;
	return ay;
}


tuple<double, double> StripBasedHuman::calculateAcceleration(Car* ego) {
	std::map<int, tuple<double, Car*>> safe_vel_map = calculateSafeVelocities(ego);
	EdgeStrips* ego_strip = edgeStrips[ego->getCurrentEdge()];
	StripInfo ego_strip_info = ego_strip->getVehicleStripInfo(ego);

	/* Calculate the longitudinal acceleration */
	auto [strip_inx, vsafe_x, leader] = calculateLeaderFromSafeVelMap(ego, safe_vel_map);
	double desired_speed = get_desired_speed(ego->getNumId());
	double time_step = get_time_step_length();
	// The max velocity possible in the next time step
	double max_vel_x = ego->getSpeedX() + time_step * Accelerate;

	double next_vel_x = std::min(desired_speed, max_vel_x);
	if (leader != nullptr) {
		double next_vel_x = std::min({ vsafe_x, desired_speed, max_vel_x });
	};
	
	double diff_vel_x = next_vel_x - ego->getSpeedX();
	double ax = 0;
	if (diff_vel_x < 0) {
		ax = -std::min(std::abs(diff_vel_x / time_step), Deccelerate);
	}
	else {
		ax = std::min(diff_vel_x / time_step, Accelerate);
	}

	/* Calculate lateral acceleration */
	double ay = 0;
	updateStripChangeBenefit(ego, safe_vel_map);
	double benefit = driverMemory[ego];

	if (abs(benefit) > LaneChangeThreshold) {
		int delta_inx = (benefit > 0) ? 1 : -1;
		double new_y = ego_strip->getYFromInx(ego_strip_info.mainInx + delta_inx);
		double current_y = ego->getY();
		double width = ego->getWidth();
		double diff_y = new_y - (ego->getY() - ego->getWidth() / 2);

		double req_speed_y = diff_y / time_step;
		double speed_diff = req_speed_y - ego->getSpeedY();
		ay = speed_diff / time_step;

		// Check if the lane change is possible
		if (ay != 0) {
			auto [bool_right, bool_left] = isLaneChangePossible(ego);
			if ((benefit < 0 && !bool_right) || (benefit > 0 && !bool_left)) {
				ay = calculateStopLatAcc(ego);
			}
		}
	}
	else {
		ay = calculateStopLatAcc(ego);
	}

	return std::make_tuple(ax, ay);
}
