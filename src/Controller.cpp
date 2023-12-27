#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
// 23:42 2022 07 23
#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#include <LaneFree_win.h>
#include "libLaneFreePlugin_Export.h"
#endif

#define CONTROLLER_H
#include "Controller.h"

#define SAMPLE_UNIFORM(min, max) ((double)min + ((double)random()/RAND_MAX)*(max - min))
#define MAX(a, b) (((a) > (b))?(a):(b))
#define MIN(a, b) (((a) <= (b))?(a):(b))

#define UPPER 9.1
#define LOWER 1.1

#define MIN_DESIRED_SPEED 25
#define MAX_DESIRED_SPEED 35
#define verordnungsindex 0.07

#define PI 3.1415926

#define kp1 0.15 //The Kp for longitudinal desired speed force
#define kp2 0.4 //The Kp for lateral deceleration
#define kp_boundary 0.7 //The Kp for boundary forces
#define FORCE_INDEX 1 //The index for controlling strength of forces
#define kd_boundary 0.7

#define WALL_DN(point, safety, v, yi, wi) U_lemma3( -(safety)*v, MAX(0, yi-0.5*wi - (point)), -sim->uymax_hard)
#define WALL_UP(point, safety, v, yi, wi) U_lemma3( +(safety)*v, MAX(0, (point) - (yi+0.5*wi)), -sim->uymax_hard)

static double U_lemma3
(double v, double d, double ubar) {
	double nom, den;
	double T = get_time_step_length();
	double ret;

	nom = T * ubar - 2 * v + sqrt(pow(T, 2) * pow(ubar, 2) - (4 * T * ubar * v) + 8 * ubar * (T * v - d));
	den = 2 * T;

	if (fpclassify((ret = nom / den)) == FP_NAN) {
		ret = ubar;
	}

	return ret;
}

void squre
(double &parameter) {
	parameter = pow(parameter, 2.0);
}

void random_desired_speed(int &vd ,int min_value, int max_value) {
	int j;
	srand((int)time(0));

	int range = max_value - min_value;
	j = min_value + (int)(range * rand() / (RAND_MAX+1));

	vd = j;
}

void calculation_ab
(double& a, double& b, NumericalID ids_in_edge, NumericalID front_neighbors, double wx1,double wx2, double wy ) {
	double veh_length = get_veh_length(ids_in_edge);
	double veh_width = get_veh_width(ids_in_edge);
	double obs_length = get_veh_length(front_neighbors);
	double obs_width = get_veh_width(front_neighbors);
	double ego_speed_x = get_speed_x(ids_in_edge);
	double ego_speed_y = get_speed_x(ids_in_edge);
	double obs_speed_x = get_speed_x(front_neighbors);
	double obs_speed_y = get_speed_x(front_neighbors);

	int Li = 1.8;
	a = Li * (veh_length + obs_length) + wx1 * (ego_speed_x + obs_speed_x) + wx2 * fabs(ego_speed_x - obs_speed_x);

	double we = 1.3 * veh_width;
	double woi = 1.3 * obs_width;
	double Wi = we + woi;

	double ego_postion_y = get_position_y(ids_in_edge);
	double obs_postion_y = get_position_y(front_neighbors);

	double item0 = (obs_speed_y - ego_speed_y) / (ego_postion_y - obs_postion_y + 0.0001);

	double p0 = sqrt(pow(item0, 2.0) + 0.001);

	b = Wi + wy * ( tanh(item0) + sqrt( tanh(item0* item0)+0.0001 ) );
}

double r_two_ellipses(double a1, double b1, double a2, double b2, NumericalID ego_id, NumericalID neighbor_id) {
	double x1 = get_position_x(ego_id);
	double y1 = get_position_y(ego_id);;
	double x2 = get_position_x(neighbor_id);
	double y2 = get_position_y(neighbor_id);

	double theta = atan(fabs(y2 - y1)/fabs(x2 - x1));

	double r1 = sqrt(pow(a1 * cos(theta), 2) + pow(b1 * sin(theta), 2));
	double r2 = sqrt(pow(a2 * cos(theta), 2) + pow(b2 * sin(theta), 2));

	return r1 + r2;
};

double relative_distance(NumericalID ego_id, NumericalID neighbor_id) {
	double x1 = get_position_x(ego_id);
	double y1 = get_position_y(ego_id);;
	double x2 = get_position_x(neighbor_id);
	double y2 = get_position_y(neighbor_id);

	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
};

void calculation_forces
(double &fi, NumericalID ids_in_edge, NumericalID front_neighbors,int n, int p, int q, double a, double b) {
	double pos_x = get_position_x(ids_in_edge);
	double pos_y = get_position_y(ids_in_edge);
	double obs_x = get_position_x(front_neighbors);
	double obs_y = get_position_y(front_neighbors);

	double relative_distance_x = fabs(get_position_x(ids_in_edge) - get_position_x(front_neighbors));
	double relative_distance_y = fabs(get_position_y(ids_in_edge) - get_position_y(front_neighbors));

	double item1 = pow((2 * relative_distance_x / a), n);
	double item2 = pow((2 * relative_distance_y / b), p);

	fi = FORCE_INDEX / (pow((item1 + item2), q) + 1);
}

void determin_forces_nudging
(double& fxi, double& fyi, double fi, NumericalID ids_in_edge, NumericalID back_neighbors) {

	double position_x = get_position_x(ids_in_edge);
	double position_y = get_position_y(ids_in_edge);
	double ob_position_x = get_position_x(back_neighbors);
	double ob_position_y = get_position_y(back_neighbors);

	double x_diff = fabs(position_x - ob_position_x);
	double y_diff = fabs(position_y - ob_position_y);

	double theta = atan(y_diff / x_diff);


	//printf("ego-obs postion: (%f,%f) degree!\n", x_diff, y_diff);
	//printf("The position angle: %f degree!\n", theta * 180 / PI);
	//double theta = atan2(y_diff, x_diff) * 180 / PI;
	if (ob_position_y >= position_y) {
		fxi = fi * cos(theta);
		fyi = -fi * sin(theta);
	}
	else {
		fxi = fi * cos(theta);
		fyi = fi * sin(theta);
	}

}

void determin_forces_repulsive
(double& fxi, double& fyi, double fi, NumericalID ids_in_edge, NumericalID front_neighbors) {
	
	double position_x = get_position_x(ids_in_edge);
	double position_y = get_position_y(ids_in_edge);
	double ob_position_x = get_position_x(front_neighbors);
	double ob_position_y = get_position_y(front_neighbors);

	double x_diff = ob_position_x - position_x;
	double y_diff = ob_position_y - position_y;

	double theta = atan(y_diff/x_diff);

	//printf("The position angle: %f degree!\n", theta * 180 / PI);
	
	//double theta = atan2(y_diff, x_diff) * 180 / PI;
	//printf("The value of theta: %f degree!\n", theta * 180 / PI);

	fxi = -fi * cos(theta);
	fyi = -fi * sin(theta);
}

void target_speed_forces
(double &ax_desired, double & ay_desired, NumericalID ids_in_edge, double vd)
{
	double cur_speed_x = get_speed_x(ids_in_edge);
	double cur_speed_y = get_speed_y(ids_in_edge);

	double target_speed = 1.1 * cur_speed_x;

	double control_speed = MIN(target_speed, vd);

	ax_desired = kp1 * (control_speed - cur_speed_x);
	ay_desired = -kp2 * cur_speed_y;
}

double upper_boundary_forces
(double fy, NumericalID ids_in_edge) 
{
	double fy_control_upper{ 0 }, error_bound{ 0 }, bound_force{ 0 };

	double mid_point = (UPPER + LOWER) * 0.5;
	double position_y = get_position_y(ids_in_edge);

	char* veh_name = get_vehicle_name(ids_in_edge);
	//printf("The lateral position of %s: %f\n", veh_name, position_y);

	error_bound = UPPER - position_y;
	bound_force = kp_boundary * error_bound;
	//printf("The boundary force and fy of %s: (%f , %f)\n", veh_name, bound_force, fy);

	fy_control_upper = MIN(fy, bound_force);
	//printf("The control force of %s: %f\n", veh_name, fy_control_upper);

	return fy_control_upper;
}

double lower_boundary_forces
(double fy, NumericalID ids_in_edge)
{
	double fy_control_lower{ 0 }, error_bound{ 0 }, bound_force{ 0 };

	double mid_point = (UPPER + LOWER) * 0.5;
	double position_y = get_position_y(ids_in_edge);

	char* veh_name = get_vehicle_name(ids_in_edge);
	//printf("The lateral position of %s: %f\n", veh_name, position_y);

	error_bound = LOWER - position_y;
	bound_force = kp_boundary * error_bound;
	//printf("The boundary force and fy of %s: (%f , %f)\n", veh_name, bound_force, fy);

	fy_control_lower = MAX(fy, bound_force);
	//printf("The control force of %s: %f\n", veh_name, fy_control_lower);

	return fy_control_lower;
}

void add
(int *p) 
{
	*p = *p * +1;
}

void overtake
(NumericalID ids_ego, NumericalID ids_obs) {
	double mid_point = (UPPER + LOWER) * 0.5;
	double pos_ego_y = get_position_y(ids_ego);
	double pos_obs_y = get_position_y(ids_obs);

	double pos_ego_x = get_position_x(ids_ego);
	double pos_obs_x = get_position_x(ids_obs);

	int desired_speed_ego = get_desired_speed(ids_ego);
	int desired_speed_obs = get_desired_speed(ids_obs);
	
	int diff_desired = desired_speed_ego - desired_speed_obs;
	double fabs_y = fabs(pos_ego_y - pos_obs_y);
	double fabs_x = fabs(pos_ego_x - pos_obs_x);
	
		
	//if (diff_desired < 0) {printf("1. In!!!\n");}
	//if (fabs_y < 1) { printf("2. In!!!\n"); }
	//if (fabs_x < 40) { printf("3. In!!!\n"); }

	if (desired_speed_ego < desired_speed_obs && fabs(pos_ego_y - pos_obs_y) < 1 && fabs(pos_ego_x - pos_obs_x) < 40) {
		if (pos_ego_x > mid_point) {
			char* name_ego = get_vehicle_name(ids_ego);
			//printf("The vehicle %s will be pushed downward!\n", name_ego);
			apply_acceleration(ids_ego, 0, -1);
		}
		else {
			char* name_obs = get_vehicle_name(ids_ego);
			//printf("The vehicle %s will be pushed upward!\n", name_obs);
			apply_acceleration(ids_obs, 0, 1);
		}
	}
}

double verkehrsordnungskraft
(NumericalID ids_ego) {
	double vd = get_desired_speed(ids_ego);

	double co = vd - (double)MIN_DESIRED_SPEED;
	double areas = (double)MAX_DESIRED_SPEED - (double)MIN_DESIRED_SPEED;

	double target_line = LOWER + ((UPPER - LOWER) / areas) * co;

	//printf("Co: %f\nAreas number: %f\n", co, areas);

	double pos_y = get_position_y(ids_ego);
	char* name = get_vehicle_name(ids_ego);
	double ordnungskraft = verordnungsindex * (target_line - pos_y);
	//printf("The desired speed and target line of vehicle %s are: (%f, %f) \nOrdnungskraft: %f\n", name, vd, target_line, ordnungskraft);
	return ordnungskraft;
}

/*void maintain_gap(NumericalID ego_id, NumericalID front_neighbors, double target_gap)
{

	double current_gap = relative_distance(ego_id, front_neighbors);

	double gap_error = current_gap = relative_distance(ego_id, front_neighbors);
	double force = kp_boundary * gap_error;

	double a_desired = force;
	apply_acceleration(ego_id, a_desired, 0);
}*/

void maintain_gap(NumericalID ego_id, NumericalID front_neighbor, double target_gap) {
	double current_gap = get_relative_distance_x(ego_id, front_neighbor);
	double ego_speed = get_speed_x(ego_id);
	double front_speed = get_speed_x(front_neighbor);

	// Calculate the relative velocity
	double relative_velocity = ego_speed - front_speed;

	// Calculate the desired acceleration based on the gap error and relative velocity
	double gap_error = target_gap - current_gap;
	double acceleration = kp_boundary * gap_error - kd_boundary * relative_velocity;

	// Apply the calculated acceleration to the ego vehicle
	apply_acceleration(ego_id, acceleration, 0);
}

/*void initialize_y_speed
(double speed_y, double TS, NumericalID front_neighbors) {
	printf("Initializing the lateral speed...\n");
	double fyii = -(speed_y)/TS;
	printf("The speed_y, TS and initializing acceleration are: (%f,%f,%f) \n", speed_y, TS, fyii);
	
	apply_acceleration(0, fyii, front_neighbors);

	double ini_speed = get_speed_y(front_neighbors);
	
	if (ini_speed != 0) {
		printf("The speed_y cannot be initialized!\n");
	}
	else {
		printf("The speed_y was successfully initialized!\n");
	}
}
*/

/*
void regulate_forces
(sim_t* sim, NumericalID edge_id, NumericalID veh_id, double* fx, double* fy) {
	// y wall 
	double fxi = *fx, fyi = *fy;
	if (sim->dynamic_y_walls) {
		double uy_max = sim->walls.y.bound_up;
		double uy_min = sim->walls.y.bound_dn;
		if (uy_max >= -uy_min) {
			fyi = MAX(fyi, -uy_min);
			fyi = MIN(fyi, +uy_max);
		}
	}

	double vx = get_speed_x(veh_id), vd = get_desired_speed(veh_id), vy = get_speed_y(veh_id);
	double yi = get_position_y(veh_id), wi = get_veh_width(veh_id);
	double roadwid_meters = get_edge_width(edge_id);
	double T = get_time_step_length();

	//keeping vehicles within the road
	fyi = MAX(fyi, -WALL_DN(0, 1.05, vy, yi, wi));
	fyi = MIN(fyi, +WALL_UP(roadwid_meters, 1.05, vy, yi, wi));

	//x wall 
	if (sim->dynamic_x_walls && sim->walls.x.leader != -1) {
		if (sim->walls.x.bound < fxi) {
			fxi = sim->walls.x.bound;
			sim->reg++;
		}
	}

	//[umin, umax] ranges
	fxi = MIN(fxi, sim->uxmax_hard);
	fxi = MAX(fxi, sim->uxmin_hard);

	fyi = MIN(fyi, +sim->uymax_hard);
	fyi = MAX(fyi, -sim->uymax_hard);

	//non-negative speed
	fxi = MAX(fxi, -vx / T);

	//non-excessive speed
	fxi = MIN(fxi, (sim->vd_alpha * vd - vx) / T);

	// lat. speed never exceeds 0.5 x lon. speed, i.e.
	// enforcing: |vy[k+1]| <= 0.5 vx[k+1]
	//
	fyi = MIN(fyi, +(0.5 * vx - vy) / T);
	fyi = MAX(fyi, -(0.5 * vx + vy) / T);

	//single-lane exceptions
	if (sim->single_lane)
		fyi = 0;

	*fx = fxi;
	*fy = fyi;
}
*/

/*

double x_acceleration(double forces_i, double pos_x, double pos_y, double ob_position_x, double ob_position_y) {
	double x_diff = fabs(pos_x - ob_position_x);
	double y_diff = fabs(pos_y - ob_position_y);
	double PI = 3.1415926;
	double x_acceleration;
	double theta = atan2(y_diff, x_diff) * 180 / PI;
	if (pos_x <= ob_position_x) {
		return fabs(forces_i * cos(theta));
	}
	else {
		return -fabs(forces_i * cos(theta));
	}
}

double y_acceleration(double forces_i, double pos_x, double pos_y, double ob_position_x, double ob_position_y) {
	double x_diff = fabs(pos_x - ob_position_x);
	double y_diff = fabs(pos_y - ob_position_y);
	double PI = 3.1415926;
	double y_acceleration;
	double theta = atan2(y_diff, x_diff) * 180 / PI;
	if (pos_y <= ob_position_y) {
		return fabs(forces_i * sin(theta));
	}
	else {
		return -fabs(forces_i * sin(theta));
	}
}
*/