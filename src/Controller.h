#pragma once
#include <map>
#include <vector>
#include <string>
#include <unordered_set>
#ifdef CONTROLLER_H
#define EXTERN_C /* nothing */
#else
#define EXTERN_C extern
#endif 

typedef long long int NumericalID;
using iniMap = std::map<std::string, std::map<std::string, std::string>>;

class Car;

class LFTStrategy {
public:
	LFTStrategy(iniMap config);
	virtual std::tuple<double, double> calculateAcceleration(Car* car) = 0;
	void setCarsMap(std::map<NumericalID, Car*> &cars) { carsMap = cars; }
	// This method is called at the end of the simulation
	virtual void finalize_simulation() {};
	// Returns the neighbours of the ego vehicle within a certain distance (front or back). For the case of circular movement, after calling getNeighbours, use getCircularX of Car to get the corrected x position of the vehicle.
	std::vector<Car*> getNeighbours(Car* ego, double distance);
	virtual void update() {};
	virtual void finish_time_step() {};
	static bool isCircular() { return circular; }
	static void setCircular(iniMap config);
	// Extracts the parameters of section name with the prefix from the config file
	std::map<std::string, std::map<std::string, std::string>> extractModelSpecificParams(iniMap config, std::string prefix);
	// Splits a string s by the delimiter
	static std::vector<std::string> splitString(const std::string& s, std::string delimiter);
	std::vector<std::string> getOnRampEdges() { return on_ramp_edges; }
	std::vector<std::string> getOffRampEdges() { return off_ramp_edges; }

protected:
	std::map<NumericalID, Car*> carsMap;
	static bool circular;
	std::map<std::string, std::tuple<double, double>> acc_jerk_limits;
	std::tuple<double, double> applyAccAndJerkConstraints(double ax, double ay, Car* car);
	void setAccAndJerkConstraints(std::map<std::string, std::string> config);
	// Checks if the vehicle is under on-ramp situation
	bool isOnRampSituation(Car* car);
	// Checks if the vehicle is under off-ramp situation
	bool isOffRampSituation(Car* car);

private:
	std::vector<std::string> on_ramp_edges, off_ramp_edges;
};

class Car {
public:
	Car(NumericalID numID, iniMap config, std::map<std::string, LFTStrategy*> strategies);
	// Copy constructor of the class
	Car(const Car& car);
	void update();
	std::tuple<double, double> applyAcceleration();
	std::tuple<double, double> calDistanceToBoundary(double offset_x, double offset_y);
	std::tuple<double, double> calBoundary(double offset_x);
	double getRelativeDistanceX(Car* other);
	double getRelativeDistanceY(Car* other);
	std::tuple<double, double> getGlobalPosition();

	double getWidth() { return width; }
	double getLength() { return length; }
	std::string getTypeName() { return typeName; }
	std::string getVehName() { return vehName; }
	std::string getModelName() { return modelName; }
	NumericalID getNumId() { return numID; }
	double getX() { return x; }
	double getY() { return y; }
	double getSpeedX() { return speedX; }
	double getSpeedY() { return speedY; }
	double getAccX() { return accX; }
	double getAccY() { return accY; }
	double getDesiredSpeed() { return desiredSpeed; }
	double getCircularX() { return circularX; }
	double getCurrentEdgeWidth();
	double getCurrentEdgeLength();
	bool getIfOnRampVeh() { return isOnRampVeh; }
	bool getIfOffRampVeh() { return isOffRampVeh; }

	NumericalID getCurrentEdge() { return currentEdge; }
	std::string getCurrentEdgeName();
	LFTStrategy* getLFTStrategy() { return lftstrategy; }

	void setLFTStrategy(LFTStrategy* lftstrategy) { this->lftstrategy = lftstrategy; }
	void setX(double x) { this->x = x; }
	void setCircularX(double circularX) { this->circularX = circularX; }

protected:
	double width;
	double length;
	double speedX;
	double speedY;
	double accX = 0;
	double accY = 0;
	double x;			// This is the x position within the current edge
	double y;			// This is the current y position relative to the current edge
	double desiredSpeed;
	bool isOnRampVeh{ false }, isOffRampVeh{ false };
	// This attribute is used to store the corrected x position of the vehicle after a call to the getNeighbours method of the LFTStrategy class.
	double circularX;
	std::string typeName;
	std::string vehName;
	std::string modelName;
	NumericalID numID;
	NumericalID currentEdge;
	LFTStrategy* lftstrategy;
};
// this is how you can define static variables
EXTERN_C int example_extern_variable_static;