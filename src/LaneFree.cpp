#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include "Controller.h"
#define DEFINE_VARIABLES

#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#include <LaneFree_win.h>
#endif

#define Location_detector 939.09

#define UPPER 9.1
#define LOWER 1.1

#define NUDGING_INDEX 1.0
#define REPULSIVE_INDEX 1.0

#define MIN_DESIRED_SPEED 25
#define MAX_DESIRED_SPEED 35
#define CIRCULAR_MOVEMENT 0

#define wx1 0.5
#define wx2 0.5
#define wy  0.2

#define n 2
#define p 2
#define q 4

sim_t sim_params;
/*FILE* acc;
FILE* vol;
FILE* wth;
FILE* trj;
FILE* spc;*/
Record* all_records = NULL;
WidthRecord* flow_records = NULL;

double total_width{};

void simulation_initialize() {

	/*acc = fopen("C:\\Users\\Hanwen\\Desktop\\SimulationPlot\\acc.csv", "w+");
	vol = fopen("C:\\Users\\Hanwen\\Desktop\\SimulationPlot\\vol.csv", "w+");
	wth = fopen("C:\\Users\\Hanwen\\Desktop\\SimulationPlot\\wth.csv", "w+");
	trj = fopen("C:\\Users\\Hanwen\\Desktop\\SimulationPlot\\trj.csv", "w+");
	spc = fopen("C:\\Users\\Hanwen\\Desktop\\SimulationPlot\\spc.csv", "w+");

	fprintf(acc, "Simulation step, Vehicle name, Acc X, Acc Y\n");
	fprintf(vol, "Simulation step, Vehicle name, Desired Speed, Speed X, Speed Y\n");
	fprintf(wth, "Simulation step, Vehicle name, Vehicle width, Total width\n");
	fprintf(trj, "Simulation step, Vehicle name, Coordinate Y, Coordinate X\n");
	fprintf(spc, "Simulation step, Vehicle name, Speed X, Position Y\n");*/
	
	//initialize srand with the same seed as sumo
	srand(get_seed()); // The seeds are used for creating different random value

	int n_init[5] = {0, 0, 0, 0, 0 };

	int col_car = 3;
	double desired_gap = 66.0;

	double x_incr = desired_gap, y_incr = 1.3, vx_incr = 0.01;
	double x_val = x_incr, y_val = y_incr, vx_val = vx_incr;

	int virtual_lanes = 4; //initially be 3
	double width = 10.2;

	char veh_name[40]; //

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
			
			v_id = insert_new_vehicle(veh_name, route_id, type_id, x_val, y_val, vx_val, 0, 0);
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
	
	//if (t == 7202) { system("pause"); }

	NumericalID* myedges = get_all_edges();  //returns an array with all the edges
	NumericalID n_myedges = get_all_edges_size(); //returns the size of all the edges sums the number of edges retrieved above

	printf("List of all edges:\n");
	for (NumericalID i = 0; i < n_myedges; i++) {
		printf("Edge %lld: %lld\n", i, myedges[i]);
	}

	// Print the total number of edges
	printf("Total number of edges: %lld\n", n_myedges);

	NumericalID* ids_in_edge;
	NumericalID n_edge_ids;
	NumericalID ego_distance;
	char* veh_name;
	char* ego_veh_name;
	NumericalID front_ego_veh_id;
	char* back_veh_name;
	NumericalID ego_vehicle_id = 0;
	double target_gap = 66.0;

	NumericalID* neighbor_size;
	NumericalID* front_neighbors;
	NumericalID* front_ego_neighbor;
	NumericalID* back_neighbors;
	size_t front_neighbors_size; //store here the number of front neighbors for each vehicle
	size_t back_neighbors_size; //store here the number of front neighbors for each vehicle
	size_t front_ego_size;
	double front_distance = 100;
	double front_ego_distance = 66;
	double back_distance = 100;//get vehicles up to 200 meters ahead
	int cross_edge = 0;  //1: get neighbors that are beyond the current road segment, 0: get only vehicles within the same road edge

	for (int i = 0; i < n_myedges; i++) {

		n_edge_ids = get_all_ids_in_edge_size(myedges[i]); //returns the numbera of all ids in a given edge, based on the edge's id
		
		length = get_edge_length(myedges[i]);
		printf("edge_length for edge %d: %I64d\n", i, length);
		width = get_edge_width(myedges[i]);
		printf("edge_width for edge %d: %I64d\n", i, width);
		ids_in_edge = get_all_ids_in_edge(myedges[i]);
		printf("ids in edge %d: %I64d\n", i, ids_in_edge);

		double fx{};
		double fy{};

		if (n_edge_ids > 0)
		{

			if (all_records == NULL)
			{
				all_records = new Record[2000];
				for (int g = 0; g < 2000; g++)
				{
					all_records[g].count = 1;
					all_records[g].up = 0.0;
					all_records[g].first = false;
				}
			}

			if (flow_records == NULL)
			{
				flow_records = new WidthRecord[2000];
				for (int f = 0; f < 2000; f++)
				{
					flow_records[f].last = 0.0;
					flow_records[f].first = false;
				}
			}

			

			Node** power = new Node * [n_edge_ids];

			for (int kk = 0; kk < n_edge_ids; kk++)
			{
				power[kk] = new Node[n_edge_ids];
			}

			for (int j = 0; j < n_edge_ids; j++)
			{
				printf("Vehicle IDs in the Current Edge ID %d: %lld\n", j, ids_in_edge[j]);
				char* id = get_veh_type_name(ids_in_edge[j]);
				veh_name = get_vehicle_name(ids_in_edge[j]);


				//printf("Vehicle Type: %s\n", id);
				//printf("Vehicle Name: %s\n", veh_name);

				double errorVd, errorVy, errorAccX, errorAccY;
				
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
					//neighbors_size = -1;
				}

				if (front_neighbors == 0 && back_neighbors == 0)
				{
					//j += 1;
					continue;
				}



				for (int h = 0, u = j - 1; h < back_neighbors_size; h++, u--)
				{	

					double fxi_nudging = -1 * power[u][h].x;
					double fyi_nudging = -1 * power[u][h].y;

					fx_nudging += fxi_nudging;
					fy_nudging += fyi_nudging;
				}


				
				//NumericalID* ego_front_neighbors = get_all_neighbor_ids_front(0, front_distance, 1, &front_neighbors_size);

				/*printf("Front Neighbors of the Ego Vehicle (ID: %lld):\n", ego_vehicle_id);
				for (int i = 0; i < front_neighbors_size; i++) {
					printf("Neighbor of ego %d: %lld\n", i, ego_front_neighbors[i]);
				}*/



				/* NumericalID front_neighbor_id = ego_front_neighbors[j];
				double current_distance_to_front_vehicle = get_relative_distance_x(0, front_neighbor_id);
				printf("Distance of ego to Front Neighbor %d: %lf meters\n", i, current_distance_to_front_vehicle);

				if (i == 0 || current_distance_to_front_vehicle < target_gap)
				{
					// Calculate the acceleration needed to maintain the desired gap
					double acceleration = 1.0; /* Calculate the acceleration needed based on the current distance and desired gap ;

					// Apply the calculated acceleration to the ego vehicle
					apply_acceleration(ego_vehicle_id, acceleration, 0);
				}*/
				


				for (int f = 0; f < front_neighbors_size; f++)
				{
					double fi_repulsive{ 0 }, fxi_repulsive{ 0 }, fyi_repulsive{ 0 };
					double a_ego{}, b_ego{};
					//double wx1, wx2, wy;

					//wx1 = 0.5; wx2 = 0.5; wy = 0.3;

					//maintain_gap(ids_in_edge[f], front_neighbors[f], target_gap);
    				calculation_ab(a_ego, b_ego, ids_in_edge[j], front_neighbors[f], wx1, wx2, wy);
					calculation_forces(fi_repulsive, ids_in_edge[j], front_neighbors[f], n, p, q, a_ego, b_ego);//Repulsive force
					determin_forces_repulsive(fxi_repulsive, fyi_repulsive, fi_repulsive, ids_in_edge[j], front_neighbors[f]);

					//printf("a_ego: %lf, b_ego: %lf\n", a_ego, b_ego);

					power[j][f].x = fxi_repulsive;	
					power[j][f].y = fyi_repulsive;  

					fx_repulsive += fxi_repulsive;
					fy_repulsive += fyi_repulsive;
				}

				vd = get_desired_speed(ids_in_edge[j]);
				vx = get_speed_x(ids_in_edge[j]);
				vy = get_speed_y(ids_in_edge[j]);

				target_speed_forces(ax_desired, ay_desired, ids_in_edge[j], vd);
				
				double ordnungskraft = verkehrsordnungskraft(ids_in_edge[j]);
				
				fx = ax_desired + NUDGING_INDEX * fx_nudging + REPULSIVE_INDEX * fx_repulsive;
				fy = ay_desired + NUDGING_INDEX * fy_nudging + REPULSIVE_INDEX * fy_repulsive + ordnungskraft;

				double fy_control{ 0 };
				double fy_control_upper = upper_boundary_forces(fy, ids_in_edge[j]);
				double fy_control_lower = lower_boundary_forces(fy, ids_in_edge[j]);

				double mid = (UPPER + LOWER) * 0.5;
				double pos_x = get_position_x(ids_in_edge[j]);
				double pos_y = get_position_y(ids_in_edge[j]);

				if (pos_y >= mid) {
					fy_control = fy_control_upper;
				}
				else {
					fy_control = fy_control_lower;
				}

				apply_acceleration(ids_in_edge[j], fx, fy_control);

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

				//printf("id: %I64d\n", ids_in_edge[j]);


				//if (t >= 0 && t <= 6000)
				//{
				//	fprintf(acc, "%d, %I64d, %lf, %lf\n", t, ids_in_edge[j], fx, fy_control);
				//	fprintf(vol, "%d, %I64d, %d, %lf, %lf\n", t, ids_in_edge[j], vd, cur_speed_x, cur_speed_y);
				//	fprintf(trj, "%d, %I64d, %lf, %lf\n", t, ids_in_edge[j], global_counter + pos_x, pos_y);

				//	if (pos_x > Location_detector && flow_records[ids_in_edge[j]].last < Location_detector)
				//	{
				//		total_width += veh_width;
				//		fprintf(wth, "%d, %I64d, %f, %lf\n", t, ids_in_edge[j], veh_width, total_width);
				//		if (pos_y > 5.2) 
				//		{
				//			fprintf(spc, "%d, %I64d, %lf, %lf\n", t, ids_in_edge[j], vx, pos_y);
				//		}
				//	}
				//}

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
	//fclose(acc);
	/*if (all_records != NULL)
	{
		delete[] all_records;
	}*/
}

void event_vehicle_enter(NumericalID veh_id) {

	//set_desired_speed(veh_id, rand()%(MAX_DESIRED_SPEED - MIN_DESIRED_SPEED + 1) + MIN_DESIRED_SPEED);

	//char* vname1 = get_vehicle_name(veh_id);
	//printf("Vehicle %s entered with speed %f.\n",vname1,get_speed_x(veh_id));
	//make the vehicles emulate a ring road scenario
	//set_desired_speed(veh_id, vd);

	set_desired_speed(veh_id, (double)(rand() % (int)((double)MAX_DESIRED_SPEED - (double)MIN_DESIRED_SPEED + 1) + (double)MIN_DESIRED_SPEED));
	set_circular_movement(veh_id, false);

	//double y_init = get_position_y(veh_id);
	//double road_width = get_edge_width( get_edge_of_vehicle(veh_id) );
	//double v_width = get_veh_width(veh_id);
	//double v_desired = (double)MIN_DESIRED_SPEED + ((y_init - (v_width / 2)) / (road_width - (v_width))) * ((double)MAX_DESIRED_SPEED - (double)MIN_DESIRED_SPEED);
	//set_desired_speed(veh_id, v_desired);
}

void event_vehicle_exit(NumericalID veh_id) {

	//char* vname1 = get_vehicle_name(veh_id);

	//printf("Vehicle %s existed at time %.2f, at pos:%f.\n",vname1, get_current_time_step()*get_time_step_length(), get_position_x(veh_id));

	int num = 0;
	add(&num);

}

void event_vehicles_collide(NumericalID veh_id1, NumericalID veh_id2) {

	char vname1[40];
	sprintf(vname1,"%s",get_vehicle_name(veh_id1));
	char* vname2 = get_vehicle_name(veh_id2);
	printf("Collision between %s and %s at timestep: %d, and time: %.1f.\n",vname1, vname2, get_current_time_step(), get_current_time_step()*get_time_step_length());
	
	double t = get_current_time_step();

	//if (t >= 14400 && t <= 19200) { system("pause"); } //3600-4800 
	//system("pause");
}

void event_vehicle_out_of_bounds(NumericalID veh_id) {
	//char* vname1 = get_vehicle_name(veh_id);
	//printf("Vehicle %s is out of bounds at time %.2f, at pos:%f,%f.\n", vname1, get_current_time_step() * get_time_step_length(), get_position_x(veh_id),get_position_y(veh_id));
}