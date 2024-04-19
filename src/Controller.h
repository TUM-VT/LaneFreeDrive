#pragma once
#include <map>
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
	std::vector<Car*> getNeighbours(Car* ego, double distance);
	virtual void update() {};
	static bool isCircular() { return circular; }
	static void setCircular(iniMap config);

protected:
	std::map<NumericalID, Car*> carsMap;
	static bool circular;
};

class Car {
public:
	Car(NumericalID numID, iniMap config, std::map<std::string, LFTStrategy*> strategies);
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
	NumericalID getCurrentEdge() { return currentEdge; }

	void setLFTStrategy(LFTStrategy* lftstrategy) { this->lftstrategy = lftstrategy; }
	LFTStrategy* getLFTStrategy() { return lftstrategy; }

protected:
	double width;
	double length;
	double speedX;
	double speedY;
	double x;
	double y;
	double desiredSpeed;
	std::string typeName;
	std::string vehName;
	NumericalID numID;
	NumericalID currentEdge;
	LFTStrategy* lftstrategy;
};
// this is how you can define static variables
EXTERN_C int example_extern_variable_static;