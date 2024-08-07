#include "StripBasedHuman.h"
#include "Controller.h"
#include "LaneFree.h"
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
	int main_strip_inx = floor(lower_bound / strip_width);
	if (main_strip_inx < 0) {
		main_strip_inx = 0;
	}
	int num_occupied = ceil(upper_bound / strip_width) - main_strip_inx;
	return std::make_tuple(main_strip_inx, num_occupied);
}

void EdgeStrips::updateOccupiedMap(std::map<NumericalID, Car*>& carsMap) {
	carOccupancyMap.clear();
	NumericalID n_edge_ids = get_all_ids_in_edge_size(edge_id);
	NumericalID* ids_in_edge = get_all_ids_in_edge(edge_id);

	for (int j = 0; j < n_edge_ids; j++) {
		Car* ego = carsMap[ids_in_edge[j]];
		StripInfo strip_info;
		auto [mainInx, upperInx] = calculateStripInx(ego);
		strip_info.mainInx = mainInx;
		strip_info.numOccupied = upperInx;
		carOccupancyMap[ego] = strip_info;
	}
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
	StripWidth = stod(secParam["StripWidth"]);
	FrontDistance = stod(secParam["FrontDistance"]);
	Deccelerate = stod(secParam["Deceleration"]);
	Accelerate = stod(secParam["Acceleration"]);
	Lambda = stod(secParam["Lambda"]);
	LaneChangeThreshold = stod(secParam["LaneChangeThreshold"]);
	string file_path = secParam["StripsChangeFile"];
	if (file_path.compare("") != 0) {
		StripsChangeFile.open(file_path);
		StripsChangeFile << "Time,Vehicle,From,To,Willingness Right,Willingness Left\n";
	}

	NumericalID* all_edges = get_all_edges();
	int num_edges = get_all_edges_size();
	for (int i = 0; i < num_edges; i++) {
		NumericalID edge_id = all_edges[i];
		double width = get_edge_width(edge_id);
		EdgeStrips* strip_edge = new EdgeStrips(edge_id, width, StripWidth);
		edgeStrips[edge_id] = strip_edge;
	}
}

double StripBasedHuman::calculateSafeVelocity(Car* ego, Car* leader, double gap) {
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
	double vsafe = -a + sqrt(pow(a, 2) + pow(leader->getSpeedX(), 2) + 2 * Deccelerate * gap);
	return vsafe;
}

std::unordered_map<int, tuple<double, Car*>> StripBasedHuman::calculateSafeVelocities(Car* ego, vector<Car*> front_cars) {
	EdgeStrips* strip = edgeStrips[ego->getCurrentEdge()];
	vector<int> indices(strip->getTotalNoStrips());
	std::iota(indices.begin(), indices.end(), 0);
	int numocc = strip->getVehicleStripInfo(ego).numOccupied;
	int count_indices = 0;

	std::unordered_map<int, tuple<double, Car*>> safe_velocity_map;
	safe_velocity_map.reserve(indices.size());
	for (int i : indices) {
		safe_velocity_map[i] = std::make_tuple(std::nan(""), nullptr);
	}

	for (Car* car : front_cars) {
		StripInfo* info = &strip->getVehicleStripInfo(carsMap[car->getNumId()]);
		int car_lw = info->mainInx;
		int car_up = car_lw + info->numOccupied - 1;

		int overlap_lower = std::max(car_lw - numocc - 1, 0);
		int overlap_upper = std::min(car_up + 1, (int)indices.size() - 1);
		double gap = car->getCircularX() - car->getLength() / 2.0 - (ego->getX() + ego->getLength() / 2.0);
		double safe_vels = calculateSafeVelocity(ego, car, gap);
		if (gap < 0) {
			safe_vels = 0;
		}
		for (int k = overlap_lower; k <= overlap_upper; k++) {
			if (std::get<1>(safe_velocity_map[k]) == nullptr){
				count_indices++;
				safe_velocity_map[k] = std::make_tuple(safe_vels, car);
			}
		}
		if (count_indices == indices.size()) {
			break;
		}
	}
	return safe_velocity_map;
}

void StripBasedHuman::updateStripChangeBenefit(Car* ego, std::unordered_map<int, tuple<double, Car*>> safeVelMap) {
	EdgeStrips* ego_strip = edgeStrips[ego->getCurrentEdge()];
	StripInfo ego_strip_info = ego_strip->getVehicleStripInfo(ego);
	int total_strips = ego_strip->getTotalNoStrips();
	double vel_max = get_desired_speed(ego->getNumId());

	double vsafe_current = vel_max;
	int main_inx = ego_strip_info.mainInx;
	auto [vsafe, car] = safeVelMap[main_inx];
	if (car != nullptr) {
		vsafe_current = vsafe;
	}
	
	double right = 0;
	double left = 0;
	for (int inx = 0; inx < total_strips; inx++) {
		double vsafe_neighbour = vel_max;
		auto [vsafe, car] = safeVelMap[inx];
		if (car != nullptr) {
			vsafe_neighbour = vsafe;
		}

		double diff_inx = std::abs(main_inx - inx);
		double benefit = ((vsafe_neighbour - vsafe_current) / vel_max) * std::exp(-Lambda * diff_inx);
		if (inx < main_inx) { 
			right += benefit; 
		}
		else if (inx > main_inx) { 
			left += benefit; 
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
	if (ego_strip_info.mainInx <= 1) {
		driverMemory[ego][1] = 0;
	}
	else if (ego_strip_info.mainInx + ego_strip_info.numOccupied >= total_strips) {
		driverMemory[ego][0] = 0;
	}

}

bool StripBasedHuman::isSufficientGap(Car* ego, double x, double y, vector<Car*> front_cars, vector<Car*> back_cars) {
	double delta_t = get_time_step_length();
	double ego_nextx = ego->getX() + delta_t * ego->getSpeedX();

	double ego_lw_x = x - ego->getLength() / 2.0;
	double ego_up_x = x + ego->getLength() / 2.0;
	double ego_lw_y = y - ego->getWidth() / 2.0;
	double ego_up_y = y + ego->getWidth() / 2.0;

	bool sufficient_gap = true;
	for (vector<Car*> neighbours : { front_cars, back_cars }) {
		for (Car* car : neighbours) {
			if (std::abs(ego->getX() - car->getX()) > 3 * ego->getLength()) {
				break;
			}

			double car_lw_x = car->getCircularX() + delta_t * car->getSpeedX() - car->getLength() / 2.0;
			double car_up_x = car->getCircularX() + delta_t * car->getSpeedX() + car->getLength() / 2.0;

			if ((ego_lw_x <= car_lw_x && car_lw_x <= ego_up_x) || (ego_lw_x <= car_up_x && car_up_x <= ego_up_x) 
				|| (car_lw_x <= ego_lw_x && ego_lw_x <= car_up_x) || (car_lw_x <= ego_up_x && ego_up_x <= car_up_x)) {
				double car_lw_y = car->getY() + delta_t * car->getSpeedY() - 1.2 * car->getWidth() / 2.0;
				double car_up_y = car->getY() + delta_t * car->getSpeedY() + 1.2 * car->getWidth() / 2.0;
				if ((ego_lw_y <= car_lw_y && car_lw_y <= ego_up_y) || (ego_lw_y <= car_up_y && car_up_y <= ego_up_y)) {
					sufficient_gap = false;
					break;
				}
			}
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


tuple<double, double> StripBasedHuman::calculateAcceleration(Car* ego) {
	vector<Car*> front_cars = getNeighbours(ego, FrontDistance);
	vector<Car*> back_cars = getNeighbours(ego, -FrontDistance);

	EdgeStrips* ego_strip = edgeStrips[ego->getCurrentEdge()];
	StripInfo ego_strip_info = ego_strip->getVehicleStripInfo(ego);

	std::unordered_map<int, tuple<double, Car*>> safe_vel_map = calculateSafeVelocities(ego, front_cars);

	/* Calculate the longitudinal acceleration */
	auto [vsafe_x, leader] = safe_vel_map[ego_strip_info.mainInx];

	double desired_speed = get_desired_speed(ego->getNumId());
	double time_step = get_time_step_length();
	// The max velocity possible in the next time step
	double max_vel_x = ego->getSpeedX() + time_step * Accelerate;

	double next_vel_x = std::min(desired_speed, max_vel_x);
	if (leader != nullptr) {
		next_vel_x = std::min({ vsafe_x, desired_speed, max_vel_x });
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
	updateStripChangeBenefit(ego, safe_vel_map);

	double left_benefit = driverMemory[ego][0];
	double right_benefit = driverMemory[ego][1];

	double ay = - ego->getSpeedY() / time_step;
	if ((left_benefit > LaneChangeThreshold) || (right_benefit > LaneChangeThreshold)) {
		int delta_inx = (left_benefit > right_benefit) ? 1 : -1;
		double new_y = ego_strip->getYFromInx(ego_strip_info.mainInx + delta_inx) + ego->getWidth() / 2;
		double diff_y = new_y - ego->getY();

		double req_speed_y = diff_y / time_step;
		double speed_diff = req_speed_y - ego->getSpeedY();
		ay = speed_diff / time_step;

		// Check if the lane change is possible
		if (ay != 0) {
			double new_x = ego->getX() + next_vel_x * time_step;
			bool boundary_cross = isCrossingRoadBoundary(ego, ego_strip_info.mainInx + delta_inx, ego_strip);
			bool sufficient_gap = isSufficientGap(ego, new_x, new_y, front_cars, back_cars);
			if (boundary_cross || !sufficient_gap) {
				ay = -ego->getSpeedY() / time_step;
			}
			else {
				if (StripsChangeFile.is_open()) {
					double time = get_time_step_length() * get_current_time_step();
					StripsChangeFile << time << "," << ego->getNumId() << "," << ego_strip_info.mainInx << "," << ego_strip_info.mainInx + delta_inx << "," << right_benefit << "," << left_benefit << "\n";
				}
			}
		}
	}

	return std::make_tuple(ax, ay);
}

void StripBasedHuman::finalize_simulation() {
	if (StripsChangeFile.is_open()) {
		StripsChangeFile.close();
	}
}