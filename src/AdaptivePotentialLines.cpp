#include "AdaptivePotentialLines.h"
#include "LaneFree.h"

using std::map;
using std::string;

AdaptivePotentialLines::AdaptivePotentialLines(iniMap config): PotentialLines(config) {
	printf("Setting parameters for Adaptive Potential Lines strategy\n");

	map<string, string> secParam = config["Adaptive Potential Lines Parameters"];
	AdaptiveMargin = stod(secParam["AdaptiveMargin"]);
	PLForceModel = secParam["PLForceModel"];
	if (PLForceModel.compare("UNIFORM_ADAPTIVE") != 0) {
		printf("Invalid PLForceModel for Adaptive Potential Lines. Simulation will use default UNIFORM_ADAPTIVE model\n");
		PLForceModel = "UNIFORM_ADAPTIVE";
	}
}

void AdaptivePotentialLines::update() {
	PotentialLines::update();
	buildHumanOccupiedMap();
	cars_with_modified_pl.clear();
	for (const auto& [key, car] : carsMap) {
		std::vector<Car*> front_neighbors = getNeighbours(car, this->FrontDistnce);
		double car_x = car->getX();
		for (const auto& [range, lat_info] : human_free_space) {
			auto [x1, x2] = range;
			if (x1 <= car_x && car_x <= x2) {
				cars_with_modified_pl.insert(car);
				break;
			}
		}
	}
}

std::tuple<double, double>  AdaptivePotentialLines::calculateAcceleration(Car* ego) {
	std::vector<Car*> front_neighbors = getNeighbours(ego, this->FrontDistnce);
	std::vector<Car*> back_neighbors = getNeighbours(ego, -this->BackDistance);
	Car* leader = leader_map[ego];

	auto [fx_nudge, fy_nudge] = calculateNeighbourForces(ego, back_neighbors);
	auto [fx_repluse, fy_repluse] = calculateNeighbourForces(ego, front_neighbors);
	auto [ax_desired, ay_desired] = calculateTargetSpeedForce(ego);
	double fy_pl;
	
	fy_pl = calculatePLForceUniformAdaptive(ego, LowerLong, UpperLong);

	// Calculate combined force
	double fx{ 0 }, fy{ 0 };
	fx = ax_desired + fx_nudge + fx_repluse;
	fy = ay_desired + fy_nudge + fy_repluse + fy_pl;
	// Consider the boundary control
	fy = controlRoadBoundary(ego, fy);

	// Limit the x-axis acceleration according to safe velocity
	if (VSafeVehModels.size() > 0){
		double ax_safe = calculateSafeAcc(ego, leader);
		fx = std::min(fx, ax_safe);
	}

	return std::make_tuple(fx, fy);
}

std::tuple<double, double> AdaptivePotentialLines::calculateForces(Car* ego, Car* neighbour, double major_axis, double minor_axis) {
	double neighbour_x = neighbour->getCircularX();
	// double rel_dist_x = fabs(ego->getX() - neighbour_x);
	double dneigh = neighbour_x - wx1 * (ego->getSpeedX() - neighbour->getSpeedX()) / 2.0;
	double rel_dist_x = fabs(ego->getX() - dneigh);
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

		double force_index = (ego->getX() < neighbour_x) ? repulse_index : nudge_index;
		if (modelParams.find(neighbour->getModelName()) != modelParams.end()) {
			auto param = modelParams[neighbour->getModelName()];
			force_index = (ego->getX() < neighbour_x) ? std::stod(param["repulse_index"]) : std::stod(param["nudge_index"]);
		}

		fx = force_index * fx_sign * fabs(fx);
		fy = force_index * fy_sign * fabs(fy);
	}
	return std::make_tuple(fx, fy);
}


void AdaptivePotentialLines::buildHumanOccupiedMap() {
	human_free_space.clear();

	double x1_last{ std::nan("") }, x2_last{ std::nan("") };
	std::map<double, double> occupied_lats;
	NumericalID* myedges = get_all_edges();
	NumericalID n_myedges = get_all_edges_size();
	for (int i = 0; i < n_myedges; i++) {
		double edge_width = get_edge_width(myedges[i]);
		NumericalID n_edge_ids = get_all_ids_in_edge_size(myedges[i]);
		// Vehicles in ids_in_edge are already arranged according to logitudinal positions
		NumericalID* ids_in_edge = get_all_ids_in_edge(myedges[i]);
		for (int j = 0; j < n_edge_ids; j++) {
			Car* car = carsMap[ids_in_edge[j]];
			if (car->getModelName().compare("Human") == 0) {
				double x1 = car->getX() - car->getLength() / 2.0 - AdaptiveMargin;
				double x2 = car->getX() + car->getLength() / 2.0; //+ AdaptiveMargin;

				double y1 = car->getY() - car->getWidth() / 2.0;
				double y2 = car->getY() + car->getWidth() / 2.0;

				if (std::isnan(x1_last)) {
					x1_last = x1;
					x2_last = x2;
					occupied_lats[y1] = y2;
				}
				else if (x1 <= x2_last) {
					x2_last = x2;
					occupied_lats[y1] = y2;
				}
				else {
					human_free_space[std::make_tuple(x1_last, x2_last)] = calculateAvailableLateralFromOccupied(occupied_lats, edge_width);
					occupied_lats.clear();
					x1_last = x1;
					x2_last = x2;
					occupied_lats[y1] = y2;
				}
			}
		}
		human_free_space[std::make_tuple(x1_last, x2_last)] = calculateAvailableLateralFromOccupied(occupied_lats, edge_width);
		occupied_lats.clear();
		x1_last = std::nan("");
		x2_last = std::nan("");
	}
}

std::map<double, double> AdaptivePotentialLines::calculateAvailableLateralFromOccupied(std::map<double, double> lateral, double edge_width) {
	std::map<double, double> new_lats;
	double y1_last{ std::nan("") }, y2_last{ std::nan("") };

	for (const auto& [y1, y2] : lateral) {
		if (std::isnan(y1_last)) {
			new_lats[0] = y1;
			y1_last = y1;
			y2_last = y2;
		}
		else if (y1 <= y2_last) {
			y2_last = y2;
		}
		else {
			new_lats[y2_last] = y1;
			y1_last = y1;
			y2_last = y2;
		}
	}
	new_lats[y2_last] = edge_width;
	return new_lats;
}

double AdaptivePotentialLines::calculatePLForceUniformAdaptive(Car* ego, double lower_bound, double upper_bound) {
	double car_x = ego->getX();
	double pl_force = std::nan("");
	for (const auto& [range, lat_info] : human_free_space) {
		auto [x1, x2] = range;
		if (x1 <= car_x && car_x <= x2) {

			std::map<double, double> possible_lats;
			double available_space = 0.0;
			double max_vehicle_width = 1.9;
			for (const auto& [y1, y2] : lat_info) {
				double pl_space = y2 - y1 - max_vehicle_width;
				if (pl_space > 0) {
					available_space += pl_space;
					possible_lats[y1 + 0.5 * max_vehicle_width] = y2 - 0.5 * max_vehicle_width;
				}
			}
			double co = ego->getDesiredSpeed() - MINDesiredSpeed;
			double areas = MAXDesiredSpeed - MINDesiredSpeed;
			double rel_line = (available_space / areas) * co;

			double space_count = 0;
			for (const auto& [y1, y2] : possible_lats) {
				double pl_space = y2 - y1;
				space_count += pl_space;
				if (rel_line < space_count) {
					double target_line = y1 + rel_line;
					Car* leader = leader_map[ego];
					double factor = verordnungsindex;
					if (leader != nullptr && leader->getModelName().compare("Human") == 0) {
						double factor = verordnungsindex;
					}
					pl_force = factor * (target_line - ego->getY());
					break;
				}
			}
			break;
		}
	}
	if (std::isnan(pl_force)) {
		pl_force = calculatePLForceUniform(ego, lower_bound, upper_bound);
	}
	return pl_force;
}