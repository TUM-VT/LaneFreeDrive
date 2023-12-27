#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "Controller.h"
#define DEFINE_VARIABLES

#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#include <LaneFree.h>
#endif

#define Location_detector 760

#define NUDGING_INDEX 1
#define REPULSIVE_INDEX 1

#define NUDGING_INDEX_Emergency 1.3
#define REPULSIVE_INDEX_Emergency 0.8

#define MIN_DESIRED_SPEED 25
#define MAX_DESIRED_SPEED 35
#define MIN_DESIRED_SPEED_E 40
#define MAX_DESIRED_SPEED_E 45
#define CIRCULAR_MOVEMENT 0

#define wx1 0.7
#define wx2 0.7
#define wy  0.5

#define n 2
#define p 2
#define q 6

sim_t sim_params;

Record* all_records = NULL;
WidthRecord* flow_records = NULL;
EmergencyVehicleLocation* location_records = NULL;
EmergencyVehicleLocation* speed_records = NULL;

double emergency_location{};
double emergency_speed{};
double total_width{};

void simulation_initialize() {

	wth = fopen("C:/Users/Hanwen/Desktop/SimulationPlot/wth.csv", "w+");
	tarfet_line = fopen("C:/Users/Hanwen/Desktop/SimulationPlot/wth.csv", "w+");

	//initialize srand with the same seed as sumo
	srand(get_seed());

	int n_init[5] = {0, 0, 0, 0, 0};

	int col_car = 4;

	double x_incr = 5.6, y_incr = 3, vx_incr = 0.005;
	double x_val = x_incr, y_val = y_incr, vx_val = vx_incr;

	int virtual_lanes = 2;
	double width = 10.2;

	char veh_name[40];

	//route_id and type_id should be defined in the scenario we are running
	char route_id[40] = "route0";
	char type_id[40] = "lane_free_car_";
	NumericalID v_id;

	for (int j = 0; j <= col_car; j++)
	{
		sprintf(type_id, "lane_free_car_%d", j);
		
		for (int i = 0; i < n_init[j]; i++) {

			sprintf(veh_name, "%s_plugin_%d", type_id, (i + 1));
			printf("try insert!\n");
			
			v_id = insert_new_vehicle(veh_name, route_id, type_id, x_val++, y_val, vx_val, 0, 0, 0);
			printf("%s inserted\n", veh_name);

			y_val = y_val + y_incr;
			if (i % virtual_lanes == (virtual_lanes - 1)) {
				x_val += x_incr;

				if (vx_val < 35) {
					vx_val += vx_incr;
				}
				y_val = y_incr;
			}
		}
	}
	printf("Initializiation over!!!\n");
}

void simulation_step() {

	double  length, width, TS = get_time_step_length();
	
	int t = get_current_time_step();

	NumericalID* myedges = get_all_edges();  //returns an array with all the edges
	NumericalID n_myedges = get_all_edges_size(); //returns the size of all the edges

	NumericalID* ids_in_edge;
	NumericalID n_edge_ids;
	char* ego_veh_name;
	char* front_veh_name;
	char* back_veh_name;

	NumericalID* neighbor_size;
	NumericalID* front_neighbors;
	NumericalID* back_neighbors;
	size_t front_neighbors_size; //store here the number of front neighbors for each vehicle
	size_t back_neighbors_size; //store here the number of front neighbors for each vehicle
	double front_distance = 50;
	double back_distance = 50;//get vehicles up to 200 meters ahead
	int cross_edge = 0;  //1: get neighbors that are beyond the current road segment, 0: get only vehicles within the same road edge

	for (int i = 0; i < n_myedges; i++) {

		n_edge_ids = get_all_ids_in_edge_size(myedges[i]); //returns the numbera of all ids in a given edge, based on the edge's id
		length = get_edge_length(myedges[i]);
		width = get_edge_width(myedges[i]);

		char* edge_name = get_edge_name(myedges[i]);

		double fx{};
		double fy{};

		if (n_edge_ids > 0)
		{
			if (all_records == NULL)
			{
				all_records = new Record[80000];
				for (int g = 0; g < 80000; g++)
				{
					all_records[g].count = 1;
					all_records[g].up = 0.0;
					all_records[g].first = false;
				}
			}

			if (flow_records == NULL)
			{
				flow_records = new WidthRecord[80000];
				for (int f = 0; f < 80000; f++)
				{
					flow_records[f].last = 0.0;
					flow_records[f].first = false;
				}
			}

			ids_in_edge = get_all_ids_in_edge(myedges[i]); 

			Node** power = new Node * [n_edge_ids];

			for (int kk = 0; kk < n_edge_ids; kk++)
			{
				power[kk] = new Node[n_edge_ids];
			}


			if (location_records == NULL)
			{
				location_records = new EmergencyVehicleLocation[80000];
				for (int f = 0; f < 80000; f++)
				{
					location_records[f].current_location = 0.0;
				}
			}

			if (speed_records == NULL)
			{
				speed_records = new EmergencyVehicleLocation[80000];
				for (int f = 0; f < 80000; f++)
				{
					speed_records[f].current_speed = 0.0;
				}
			}

			for (int j = 0; j < n_edge_ids; j++)
			{

				char* id = get_veh_type_name(ids_in_edge[j]);
				double errorVd, errorVy, errorAccX, errorAccY;
				double position_y = get_position_y(ids_in_edge[j]);
				double position_x = get_global_position_x(ids_in_edge[j]);
				int vd;
				double vx, vy;
				double ax_desired{ 0 }, ay_desired{ 0 }, fx_repulsive{ 0 }, fy_repulsive{ 0 }, fx_nudging{ 0 }, fy_nudging{ 0 };
				double bound_force{ 0 };
				double cur_speed_y = get_speed_y(ids_in_edge[j]);
				double cur_speed_x = get_speed_x(ids_in_edge[j]);

				if (!CIRCULAR_MOVEMENT)
				{
					front_neighbors = get_all_neighbor_ids_front(ids_in_edge[j], front_distance, cross_edge, &front_neighbors_size);
					back_neighbors = get_all_neighbor_ids_back(ids_in_edge[j], back_distance, cross_edge, &back_neighbors_size);
				}
				else
				{
					neighbor_size = ids_in_edge;
				}

				for (int h = 0, u = j - 1; h < back_neighbors_size; h++, u--)
				{	
					double fxi_nudging = -1 * power[u][h].x;
					double fyi_nudging = -1 * power[u][h].y;
					fx_nudging += fxi_nudging;
					fy_nudging += fyi_nudging;
				}

				for (int f = 0; f < front_neighbors_size; f++)
				{
					double fi_repulsive{ 0 }, fxi_repulsive{ 0 }, fyi_repulsive{ 0 };
					front_veh_name = get_vehicle_name(front_neighbors[f]);
					double a_ego{}, b_ego{};
    				calculation_ab(a_ego, b_ego, ids_in_edge[j], front_neighbors[f], wx1, wx2, wy);
					calculation_forces(fi_repulsive, ids_in_edge[j], front_neighbors[f], n, p, q, a_ego, b_ego);   //Repulsive force
					determin_forces_repulsive(fxi_repulsive, fyi_repulsive, fi_repulsive, ids_in_edge[j], front_neighbors[f]);

					power[j][f].x = fxi_repulsive;
					power[j][f].y = fyi_repulsive;

					fx_repulsive += fxi_repulsive;
					fy_repulsive += fyi_repulsive;
				}

				char* veh_name = get_vehicle_name(ids_in_edge[j]);
				char* veh_type = get_veh_type_name(ids_in_edge[j]);
				double UPPER = UPPER_boundary(ids_in_edge[j], myedges[i]);
				double LOWER = LOWER_boundary(ids_in_edge[j], myedges[i]);		
				double mid = (UPPER + LOWER) * 0.5;
				resetvd(ids_in_edge[j], myedges[i]);

				vd = get_desired_speed(ids_in_edge[j]);
				vx = get_speed_x(ids_in_edge[j]);
				vy = get_speed_y(ids_in_edge[j]);

				target_speed_forces(ax_desired, ay_desired, ids_in_edge[j], vd);

				bool emergency = emergency_range(emergency_location, emergency_speed, position_x, ids_in_edge[j]);
				double ordnungskraft = pl_calculation(ids_in_edge[j], myedges[i], LOWER, UPPER, emergency);

				if (strcmp(veh_type, "lane_free_car_12") == 0) {
					fx = ax_desired + NUDGING_INDEX_Emergency * fx_nudging + REPULSIVE_INDEX_Emergency * fx_repulsive;
					fy = ay_desired + NUDGING_INDEX_Emergency * fy_nudging + REPULSIVE_INDEX_Emergency * fy_repulsive + ordnungskraft;
				}
				else {
					fx = ax_desired + NUDGING_INDEX * fx_nudging + REPULSIVE_INDEX * fx_repulsive;
					fy = ay_desired + NUDGING_INDEX * fy_nudging + REPULSIVE_INDEX * fy_repulsive + ordnungskraft;
				}


				double fy_control{ 0 };
				double fy_control_upper = upper_boundary_forces(fy, ids_in_edge[j], mid, UPPER);
				double fy_control_lower = lower_boundary_forces(fy, ids_in_edge[j], mid, LOWER);

				double pos_x = get_global_position_x(ids_in_edge[j]);
				double pos_y = get_position_y(ids_in_edge[j]);

				if (pos_y >= mid) {
					fy_control = fy_control_upper;
				}
				else {
					fy_control = fy_control_lower;
				};

				if (strcmp(edge_name, "R2") == 0 || strcmp(edge_name, "R3") == 0 || strcmp(edge_name, "R6") == 0) 
				{
					apply_acceleration(ids_in_edge[j], 0, 0);
				}
				else
				{
					apply_acceleration(ids_in_edge[j], fx, fy_control);
				}

				if (pos_x < all_records[ids_in_edge[j]].up)
				{
					++(all_records[ids_in_edge[j]].count);
				}

				all_records[ids_in_edge[j]].up = pos_x;
				double global_counter = all_records[ids_in_edge[j]].count * 1000.0;

				if (all_records[ids_in_edge[j]].first && global_counter + pos_x > 100)
				{
					all_records[ids_in_edge[j]].count -= 1;
					global_counter = all_records[ids_in_edge[j]].count * 1000.0;
				}
				all_records[ids_in_edge[j]].first = false;

				flow_records[ids_in_edge[j]].first = false;

				double veh_width = get_veh_width(ids_in_edge[j]);

				if (strcmp(veh_type, "lane_free_car_12") == 0) {
					location_records[ids_in_edge[j]].current_location = pos_x;
					emergency_location = location_records[ids_in_edge[j]].current_location;

					location_records[ids_in_edge[j]].current_speed = vx;
					emergency_speed = speed_records[ids_in_edge[j]].current_speed;

				}

				double global_x = get_global_position_x(ids_in_edge[j]);
				double global_y = get_global_position_y(ids_in_edge[j]);

				if (pos_x > Location_detector && flow_records[ids_in_edge[j]].last < Location_detector)
				{
					char* na = get_vehicle_name(ids_in_edge[j]);
					char* ty = get_veh_type_name(ids_in_edge[j]);
					double dv = get_desired_speed(ids_in_edge[j]);

					fprintf(wth, "%d, %s, %f, %f, %f\n", t, na, pos_y, vx, dv);
				}
				flow_records[ids_in_edge[j]].last = pos_x;
			}

			for (int kk = 0; kk < n_edge_ids; kk++)
			{
				delete[]power[kk];
			}
			delete[]power;
		}
	}
}

void simulation_finalize() {

}

void event_vehicle_enter(NumericalID veh_id) {
	char* v_type = get_veh_type_name(veh_id);
	int desired_speed = generate_desired_speed();
	set_desired_speed(veh_id, desired_speed);
	set_circular_movement(veh_id, false);

}

void event_vehicle_exit(NumericalID veh_id, int has_arrived) {
	int num = 0;
}

void event_vehicles_collide(NumericalID veh_id1, NumericalID veh_id2) {

	char vname1[40];
	sprintf(vname1,"%s",get_vehicle_name(veh_id1));
	char* vname2 = get_vehicle_name(veh_id2);	
	double t = get_current_time_step();
}

void event_vehicle_out_of_bounds(NumericalID veh_id) {
}