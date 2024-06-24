#include "Boundary.h"
#include <cmath>

using std::map;
using std::string;
using std::vector;
using std::tuple;

RectangularHardBoundary::RectangularHardBoundary(iniMap config, Car* ego) {
	this->ego = ego;
	auto it = config.find("RectangularHardBoundary Parameters");
	map<string, string> secParam = it->second;
	RearX = stod(secParam["RearX"]);
	FrontX = stod(secParam["FrontX"]);
	LeftY = stod(secParam["LeftY"]);
	RightY = stod(secParam["RightY"]);
}

void RectangularHardBoundary::updateBoundary() {
	lower_x = ego->getX() - ego->getLength() / 2.0 - RearX;
	upper_x = ego->getX() + ego->getLength() / 2.0 + FrontX;

	lower_y = ego->getY() - ego->getWidth() / 2.0 - RightY;
	upper_y = ego->getY() + ego->getWidth() / 2.0 + LeftY;
}

std::tuple<double, double> RectangularHardBoundary::calculateDistanceToLeaveBoundary(Car* car, double x, double y) {
	double del_x = std::nan("");
	double del_y = std::nan("");

	double car_upper_x = x + car->getLength() / 2.0;
	double car_lower_x = x - car->getLength() / 2.0;
	double car_upper_y = y + car->getWidth() / 2.0;
	double car_lower_y = y - car->getWidth() / 2.0;

	if ((car_lower_x > lower_x && car_lower_x < upper_x) || (car_upper_x > lower_x && car_upper_x < upper_x)) {
		if (car->getX() > ego->getX()) {
			del_x = upper_x - car_lower_x;
		}
		else {
			del_x = lower_x - car_upper_x;
		}
	}

	if ((car_lower_y > lower_y && car_lower_y < upper_y) || (car_upper_y > lower_y && car_upper_y < upper_y)) {
		if (car->getY() > ego->getY()) {
			del_y = upper_y - car_lower_y;
		}
		else {
			del_y = lower_y - car_upper_y;
		}
	}
	return std::make_tuple(del_x, del_y);
}

std::tuple<double, double> RectangularHardBoundary::calculateDistanceToBoundary(Car* car, double x, double y) {
	double car_upper_x = x + car->getLength() / 2.0;
	double car_lower_x = x - car->getLength() / 2.0;
	double car_upper_y = y + car->getWidth() / 2.0;
	double car_lower_y = y - car->getWidth() / 2.0;
	double del_x = std::nan("");
	double del_y = std::nan("");

	if (car_upper_x < lower_x) {
		del_x = lower_x - car_upper_x;
	}
	else if (car_lower_x > upper_x){
		del_x = upper_x - car_lower_x;
	}

	if (car_upper_y < lower_y) {
		del_y = lower_y - car_upper_y;
	}
	else if (car_lower_y > upper_y) {
		del_y = upper_y - car_lower_y;
	}
	return std::make_tuple(del_x, del_y);
}

std::tuple<double, double> RectangularHardBoundary::calculateSafeAcc(Car* car) {
	double step = get_time_step_length();
	double speedX = car->getSpeedX();
	double speedY = car->getSpeedY();
	double x = car->getX(); // + speedX * step;
	double y = car->getY(); //+ speedY * step;
	double acc_x = std::nan("");
	double acc_y = std::nan("");

	auto [off_x, off_y] = calculateDistanceToBoundary(car, x, y);

	if (!std::isnan(off_x)) {
		double req_speed_x = off_x / step;
		acc_x = (req_speed_x - speedX) / step;
	}

	if (!std::isnan(off_y)) {
		double req_speed_y = off_y / step;
		acc_y = (req_speed_y - speedY) / step;
	}
	return std::make_tuple(acc_x, acc_y);
}

std::tuple<double, double> RectangularHardBoundary::calculateAccToLeave(Car* car) {
	double step = get_time_step_length();
	double speedX = car->getSpeedX();
	double speedY = car->getSpeedY();
	double x = car->getX(); // + speedX * step;
	double y = car->getY(); //+ speedY * step;
	double acc_x = std::nan("");
	double acc_y = std::nan("");

	auto [off_x, off_y] = calculateDistanceToLeaveBoundary(car, x, y);

	if (!std::isnan(off_x)) {
		double req_speed_x = off_x / step;
		acc_x = (req_speed_x - speedX) / step;
	}

	if (!std::isnan(off_y)) {
		double req_speed_y = off_y / step;
		acc_y = (req_speed_y - speedY) / step;
	}
	return std::make_tuple(acc_x, acc_y);
}
