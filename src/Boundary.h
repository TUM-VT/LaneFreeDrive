#pragma once
#include "Controller.h"
#include "LaneFree.h"
#include <vector>

using iniMap = std::map<std::string, std::map<std::string, std::string>>;

class Car;

class CarBoundary {
public:
	CarBoundary() {};
	CarBoundary(iniMap config, Car* ego);
	// For the car object, it calculates the max acceleration possible in x and y axis to avoid crossing into the boundary in the next time step
	virtual std::tuple<double, double> calculateSafeAcc(Car* car) = 0;
	// If the car is within boundary, it calculates the acceleration required to leave the boudary
	virtual std::tuple<double, double> calculateAccToLeave(Car* car) = 0;
	// Updates the boundary according current location of the ego vehicle
	virtual void updateBoundary() = 0;

protected:
	Car* ego;
};

class RectangularHardBoundary : public CarBoundary {
public:
	RectangularHardBoundary(iniMap config, Car* ego);
	std::tuple<double, double> calculateSafeAcc(Car* car) override;
	std::tuple<double, double> calculateAccToLeave(Car* car) override;
	std::tuple<double, double> calculateDistanceToBoundary(Car* car, double x, double y);
	std::tuple<double, double> calculateDistanceToLeaveBoundary(Car* car, double x, double y);
	void updateBoundary() override;

private:
	double RearX;
	double FrontX;
	double LeftY;
	double RightY;
	double lower_x;
	double upper_x;
	double lower_y;
	double upper_y;
};

