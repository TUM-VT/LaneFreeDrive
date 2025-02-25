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
	LFTStrategy() {};
	LFTStrategy(iniMap config){};
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

protected:
	std::map<NumericalID, Car*> carsMap;
	static bool circular;
};

class Car {
public:
	Car(NumericalID numID, iniMap config, std::map<std::string, LFTStrategy*> strategies);
	// Copy constructor of the class
	Car(const Car& car);
	void update();
	std::tuple<double, double> applyAcceleration();

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
	double getDesiredSpeed() { return desiredSpeed; }
	double getCircularX() { return circularX; }
	NumericalID getCurrentEdge() { return currentEdge; }

	void setLFTStrategy(LFTStrategy* lftstrategy) { this->lftstrategy = lftstrategy; }
	void setX(double x) { this->x = x; }
	void setCircularX(double circularX) { this->circularX = circularX; }
	LFTStrategy* getLFTStrategy() { return lftstrategy; }

protected:
	double width;
	double length;
	double speedX;
	double speedY;
	double x;
	double y;
	double desiredSpeed;
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