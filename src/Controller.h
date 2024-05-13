#pragma once
#include <map>
#include <string>
#include <unordered_set>
#include "Boundary.h"
#ifdef CONTROLLER_H
#define EXTERN_C /* nothing */
#else
#define EXTERN_C extern
#endif 

typedef long long int NumericalID;
using iniMap = std::map<std::string, std::map<std::string, std::string>>;

class Car;
class CarBoundary;

class LFTStrategy {
public:
	LFTStrategy() {};
	LFTStrategy(iniMap config){};
	virtual std::tuple<double, double> calculateAcceleration(Car* car) = 0;
	void setCarsMap(std::map<NumericalID, Car*> &cars) { carsMap = cars; }
	// Returns the neighbours of the ego vehicle within a certain distance (front or back). For the case of circular movement, after calling getNeighbours, use getCircularX of Car to get the corrected x position of the vehicle.
	std::vector<Car*> getNeighbours(Car* ego, double distance);
	virtual void update() {};
	static bool isCircular() { return circular; }
	static void setCircular(iniMap config);
	// Extracts the parameters of section name with the prefix from the config file
	std::map<std::string, std::map<std::string, std::string>> extractModelSpecificParams(iniMap config, std::string prefix);

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
	double getDesiredSpeed() { return desiredSpeed; }
	double getCircularX() { return circularX; }
	NumericalID getCurrentEdge() { return currentEdge; }

	void setLFTStrategy(LFTStrategy* lftstrategy) { this->lftstrategy = lftstrategy; }
	void setBoundary(CarBoundary* boundary) { this->boundary = boundary; }
	void setX(double x) { this->x = x; }
	void setCircularX(double circularX) { this->circularX = circularX; }
	CarBoundary* getBoundary() { return this->boundary; }
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
	NumericalID numID;
	NumericalID currentEdge;
	LFTStrategy* lftstrategy;
	CarBoundary* boundary = nullptr;
};
// this is how you can define static variables
EXTERN_C int example_extern_variable_static;