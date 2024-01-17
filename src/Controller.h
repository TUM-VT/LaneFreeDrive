#pragma once
#include <map>
#include <string>
#ifdef CONTROLLER_H
#define EXTERN_C /* nothing */
#else
#define EXTERN_C extern
#endif 

typedef long long int NumericalID;
typedef enum { FCA_0, FCA_1, FCA_2, FCA_3 } fca_method_t;
using iniMap = std::map<std::string, std::map<std::string, std::string>>;

class Car;

class LFTStrategy {
public:
	LFTStrategy(){};
	virtual std::tuple<double, double> calculateAcceleration(Car* car) = 0;
	void setCarsMap(std::map<NumericalID, Car*> &cars) { carsMap = cars; }

protected:
	std::map<NumericalID, Car*> carsMap;
	iniMap LFTStrategy::readConfigFileFallback(char* config_file, char* default_config);
	iniMap readConfigFile(char* file_name);
};

class Car {
public:
	Car(NumericalID numID);
	Car() {};
	void update();
	void applyAcceleration();

	double getWidth() { return width; }
	double getLength() { return length; }
	std::string getTypeName() { return typeName; }
	std::string getVehName() { return vehName; }
	NumericalID getNumId() { return numID; }
	double getX() { return x; }
	double getY() { return y; }
	double getSpeedX() { return speedX; }
	double getSpeedY() { return speedY; }

	void setLFTStrategy(LFTStrategy* lftstrategy) { this->lftstrategy = lftstrategy; }

protected:
	double width;
	double length;
	double speedX;
	double speedY;
	double x;
	double y;
	std::string typeName;
	std::string vehName;
	NumericalID numID;
	LFTStrategy* lftstrategy;
};

typedef struct 
{
	double up = 0;
	long count = 0;
	bool first = true;
}Record;

typedef struct
{
	double last = 0;
	long count = 0;
	bool first = true;
}WidthRecord;

typedef struct 
{
	double x = 0;
	double y = 0;
}Node;

////typedef struct 
//{
//	double posx = 0;
//}Position;

typedef struct
{
	long current_location = 0;
	double current_speed = 0;
}EmergencyVehicleLocation;

typedef struct {
	int j;
	double mag;
	double fcax, fcay;
}nbor_t;

typedef struct {
	double vd;
	double accX;
	double accY;
}Error;

typedef	struct { 
	struct {
		double position;
		double distance;
		double velocity;
		long double bound;
		int leader;
		double degree;
	}x;
	struct {
		double up, degree_up;
		double dn, degree_dn;
		int up_j, dn_j;
		long double bound_up, bound_dn;
	}y;
}walls_t;

typedef struct { 
	double alpha;
	double frame_timegap;
	int ftsx_disabled;
	int fca_nbors, fca_nbors_nudge;
	int fca_sat;
	double duet, duet_vd, duet_vd_slow, duet_dy, duet_v0;

	int reg;
	int fca_max, four_lane_init;
	int barrier;
	double single_lane_space, single_lane_front_vd;
	int single_lane;
	int dynamic_y_walls, dynamic_x_walls;
	double vd_throttling_horizon;
	int warmup, vd_throttling;
	double influence_radius_meters;
	double fwd_force_max_x, fwd_force_max_y;
	double bwd_force_max_x, bwd_force_max_y;

	fca_method_t fca_method;
	double time_gap_x, time_gap_y;
	int zero_controls, zero_initial_speed, lanedrop, virtual_lanes;
	int n, K;
	double uxmax_hard, uxmin_hard, uymax_hard;
	double T, vd_meters_per_sec_lo, vd_meters_per_sec_hi;
	double vd_meters_per_sec_lo_truck, vd_meters_per_sec_hi_truck;
	double roadlen_meters, roadwid_meters;
	double ftsx_zeta, ftsx_hi, ftsx_lo;
	double ftsy_zeta, ftsy_hi;
	int* truck;
	double** x, ** y, ** vx, ** vy, ** ux, ** uy, * w, * l, * vd;
	double wall_y_up, wall_y_dn, wall_y_up_j, wall_y_dn_j;
	walls_t walls;
	double** fx, ** fy;
	int** crash;
	double* flow;
	double* vdx_effective, * vdy_effective;
	double coeff_fcax, coeff_fcay, fcax_zeta, fcay_zeta;
	double coeff_fmdl;
	double safety_level_x, safety_level_y;

	double*** degree_x;
	double*** degree_y;
	int crashes, crashes_on_leaders;
	double* class_w, * class_l;
	int ring_network;
	double vd_alpha;
}sim_t;

static double U_lemma3(double v, double d, double ubar);

void squre(double& parameter);

void random_desired_speed(int& vd, int min_value, int max_value);

void calculation_ab(double& a, double& b, NumericalID ids_in_edge, NumericalID front_neighbors, double wx1, double wx2, double wy);

double r_two_ellipses(double a1, double b1, double a2, double b2, NumericalID ego_id, NumericalID neighbor_id);

double relative_distance(NumericalID ego_id, NumericalID neighbor_id);

void calculation_forces(double& fi, NumericalID ids_in_edge, NumericalID front_neighbors, int n, int p, int q, double a, double b);

void determin_forces_nudging
(double& fxi, double& fyi, double fi, NumericalID ids_in_edge, NumericalID back_neighbors);

void determin_forces_repulsive
(double& fxi, double& fyi, double fi, NumericalID ids_in_edge, NumericalID front_neighbors);

void target_speed_forces (double& ax_desired, double& ay_desired, NumericalID ids_in_edge, double vd);

// Implements the forces for the upper boundary
double upper_boundary_forces(double fy, NumericalID ids_in_edge, double mid_point, double UPPER);
//void initialize_y_speed(double speed_y, double TS, NumericalID front_neighbors);

// Implements the forces for the lower boundary
double lower_boundary_forces(double fy, NumericalID ids_in_edge, double mid_point, double LOWER);

//void add(int* p);

//void overtake(NumericalID ids_ego, NumericalID ids_obs);
//void regulate_forces(sim_t* sim, NumericalID edge_id, NumericalID veh_id, double* fx, double* fy);

// Function for calculating the potential lines
double pl_calculation(NumericalID ids_ego, NumericalID ids_edge, double LOWER, double UPPER, bool emergency);

bool emergency_range(double emergency_location, double emergency_speed, double position_x, NumericalID id_ego);

//double rand_normal(double mean, double stddev);

double box_muller(double mu, double sigma);

double generate_desired_speed();

// this is how you can define static variables
EXTERN_C int example_extern_variable_static;