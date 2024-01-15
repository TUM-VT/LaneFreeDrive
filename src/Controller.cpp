#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#include <LaneFree.h>
#include "libLaneFreePlugin_Export.h"
#endif

#define CONTROLLER_H
#include "Controller.h"
#include <iostream>

#define SAMPLE_UNIFORM(min, max) ((double)min + ((double)random()/RAND_MAX)*(max - min))
#define MAX(a, b) (((a) > (b))?(a):(b))
#define MIN(a, b) (((a) <= (b))?(a):(b))

#define MIN_DESIRED_SPEED 25
#define MAX_DESIRED_SPEED 35
#define MIN_DESIRED_SPEED_E 40
#define MAX_DESIRED_SPEED_E 42
#define verordnungsindex_normal 0.12
#define verordnungsindex_emergency 0.05

#define kp_pl 0.3
#define kp_pl0 0.12

#define PI 3.14159265358979323846

#define kp1 0.3
#define kp2 0.65
#define kp_boundary 0.55
#define kd_boundary 0.2
#define FORCE_INDEX 2

#define WALL_DN(point, safety, v, yi, wi) U_lemma3( -(safety)*v, MAX(0, yi-0.5*wi - (point)), -sim->uymax_hard)
#define WALL_UP(point, safety, v, yi, wi) U_lemma3( +(safety)*v, MAX(0, (point) - (yi+0.5*wi)), -sim->uymax_hard)

#define mu1 28.0
#define sigma1 1.0
#define mu2 32.0
#define sigma2 1.0

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
(double fy, NumericalID ids_in_edge, double mid_point, double UPPER)
{
	double fy_control_upper{ 0 }, error_bound{ 0 }, bound_force{ 0 };
	double vy = get_speed_y(ids_in_edge);
	double position_y = get_position_y(ids_in_edge);
	char* veh_name = get_vehicle_name(ids_in_edge);
	error_bound = UPPER - position_y;
	bound_force = kp_boundary * error_bound - kd_boundary * vy;
	fy_control_upper = MIN(fy, bound_force);
	return fy_control_upper;
}

double lower_boundary_forces
(double fy, NumericalID ids_in_edge, double mid_point, double LOWER)
{
	double fy_control_lower{ 0 }, error_bound{ 0 }, bound_force{ 0 };
	double vy = get_speed_y(ids_in_edge);
	double position_y = get_position_y(ids_in_edge);
	char* veh_name = get_vehicle_name(ids_in_edge);
	error_bound = LOWER - position_y;
	bound_force = kp_boundary * error_bound - kd_boundary * vy;
	fy_control_lower = MAX(fy, bound_force);
	return fy_control_lower;
}

void overtake
(NumericalID ids_ego, NumericalID ids_obs, int UPPER, int LOWER) {
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

	if (desired_speed_ego < desired_speed_obs && fabs(pos_ego_y - pos_obs_y) < 1 && fabs(pos_ego_x - pos_obs_x) < 40) {
		if (pos_ego_x > mid_point) {
			char* name_ego = get_vehicle_name(ids_ego);
			apply_acceleration(ids_ego, 0, -1);
		}
		else {
			char* name_obs = get_vehicle_name(ids_ego);
			apply_acceleration(ids_obs, 0, 1);
		}
	}
}

bool emergency_range(double emergency_location, double emergency_speed, double position_x, NumericalID id_ego) {
	bool index{};
	double safty_range_front = 250;
	double safty_range_back = 20;

	char* vtype = get_veh_type_name(id_ego);
	double pos_diff = abs(emergency_location - position_x);
	
	index = false;
	return index;
}

double box_muller(double mu, double sigma) {
	double u1 = rand() / (double)RAND_MAX;
	double u2 = rand() / (double)RAND_MAX;

	double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
	return z0 * sigma + mu;
}

double generate_desired_speed() {

	double random_num;
	double speed;

	do {
		if (rand() % 2 == 0) {
			random_num = box_muller(mu1, sigma1);
		}
		else {
			random_num = box_muller(mu2, sigma2);
		}

		speed = random_num;
	} while (speed < 25 || speed > 35);

	return speed;
}

/* The underlying probability density function used for generating potential lines.
* At the moment, it uses a combintation of two guassian distributions, representing fast- and slow-moving vehicles
*/
double mixed_normal_pdf(double x) {
	double coef1 = 1.0 / (sigma1 * sqrt(2.0 * PI));
	double coef2 = 1.0 / (sigma2 * sqrt(2.0 * PI));
	double exp1 = exp(-0.5 * pow((x - mu1) / sigma1, 2));
	double exp2 = exp(-0.5 * pow((x - mu2) / sigma2, 2));

	return 0.5 * (coef1 * exp1 + coef2 * exp2);
}

/* The Porbability Integral Transform (PIT) of the pdf used for generating the potential lines.
* A good link to understand PIT: https://matthewfeickert.github.io/Statistics-Notes/notebooks/Introductory/probability-integral-transform.html
* It uses the Trapezoidal rule for integration.
*/
double mixed_normal_cdf(double x) {
	double lower_bound = 25.0;
	double upper_bound = x;
	int num_intervals = 1000;
	double interval_width = (upper_bound - lower_bound) / num_intervals;

	double cdf = 0.0;
	for (int i = 0; i < num_intervals; i++) {
		double x1 = lower_bound + i * interval_width;
		double x2 = x1 + interval_width;
		double y1 = mixed_normal_pdf(x1);
		double y2 = mixed_normal_pdf(x2);

		cdf += 0.5 * (y1 + y2) * interval_width;
	}
	return cdf;
}

/* Currently, the upperand lower bounds for different vehiclesand edge types are hard coded.
The target line for emergency vehicle is also hard-coded.
*/
double pl_calculation(NumericalID ids_ego, NumericalID ids_edge, double LOWER, double UPPER, bool emergency) {
	double target_line{};
	double UPPER_lower{};
	double LOWER_higher{};
	double vd = get_desired_speed(ids_ego);
	double vy = get_speed_y(ids_ego);
	double ky = 0.05;
	double pos_x = get_global_position_x(ids_ego);
	double pos_y = get_position_y(ids_ego);
	double verordnungsindex = verordnungsindex_normal;

	double cdf_value = mixed_normal_cdf(vd);

	double co = vd - (double)MIN_DESIRED_SPEED;
	double areas = (double)MAX_DESIRED_SPEED - (double)MIN_DESIRED_SPEED;

	char* edge_name = get_edge_name(ids_edge);
	char* type_name = get_veh_type_name(ids_ego);

	if (emergency) {
		LOWER_higher = LOWER;
		UPPER_lower = UPPER - 2.5;
	}
	else {
		LOWER_higher = LOWER;
		UPPER_lower = UPPER;
	}

	// Following condition forces specific target line for emergency vehicle
	if (strcmp(type_name, "lane_free_car_12") == 0)
	{
		verordnungsindex = verordnungsindex_emergency;
		target_line = 12.9;
	}
	else {
		double UPPER_new = MIN(UPPER_lower, UPPER);
		double LOWER_new = MAX(LOWER_higher, LOWER);
		target_line = LOWER_new + cdf_value * (UPPER_new - LOWER_new);
	}

	char* name = get_vehicle_name(ids_ego);
	double ordnungskraft = verordnungsindex * (target_line - pos_y) - ky * vy;
	return ordnungskraft;
}