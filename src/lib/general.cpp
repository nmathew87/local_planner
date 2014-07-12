
/*************************************************************************//**
 *****************************************************************************
 * @file        containers.cpp
 * @brief       Provide methods for sweeping a partion
 * @author      Frank Imeson
 * @date        2012-01-02
 *****************************************************************************
 ****************************************************************************/

// http://www.ros.org/wiki/CppStyleGuide


#include "local_planner/general.hpp"
#define DEBUG

/***********************************************************************************************
 * Vector2D Class Methods
 *
 *
 ***********************************************************************************************/	

Vector2D::Vector2D ()
	: x(0)
	, y(0)
{}

Vector2D::Vector2D (double _x, double _y) {
	x = _x;
	y = _y;
}

Vector2D::Vector2D (const Vector2D &other) {
	x = other.x;
	y = other.y;
}


Vector2D::Vector2D (const Vector2f &other) {
	x = other(0);
	y = other(1);
}


Vector2D& Vector2D::operator = (const Vector2D &other) {
	x = other.x;
	y = other.y;
	return *this;
}

Vector2D Vector2D::operator + (const Vector2D &other) {
	Vector2D result;
	result.x = x + other.x;
	result.y = y + other.y;
	return result;
}

Vector2D Vector2D::operator - (const Vector2D &other) {
	Vector2D result;
	result.x = x - other.x;
	result.y = y - other.y;
	return result;
}

Vector2D Vector2D::operator * (const double &other) {
	Vector2D result;
	result.x = x*other;
	result.y = y*other;
	return result;
}

Vector2D Vector2D::operator / (const double &other) {
	Vector2D result;
	result.x = x/other;
	result.y = y/other;
	return result;
}

bool Vector2D::operator == (const Vector2D &other) {
	return ((float)x == (float)other.x) && ((float)y == (float)other.y);
}


Vector2f Vector2D::vector2f () {
	return Vector2f(x,y);
}

Vector2D& Vector2D::operator = (const Vector2f &other) {
	x = other(0);
	y = other(1);
	return *this;
}

Vector2D Vector2D::operator + (const Vector2f &other) {
	Vector2D result;
	result.x = x + other(0);
	result.y = y + other(1);
	return result;
}

Vector2D Vector2D::operator - (const Vector2f &other) {
	Vector2D result;
	result.x = x - other(0);
	result.y = y - other(1);
	return result;
}


bool Vector2D::operator == (const Vector2f &other) {
	return ((float)x == other(0)) && ((float)y == other(1));
}


Vector2d Vector2D::vector2d () {
	return Vector2d(x,y);
}

Vector2D& Vector2D::operator = (const Vector2d &other) {
	x = other(0);
	y = other(1);
	return *this;
}

Vector2D Vector2D::operator + (const Vector2d &other) {
	Vector2D result;
	result.x = x + other(0);
	result.y = y + other(1);
	return result;
}

Vector2D Vector2D::operator - (const Vector2d &other) {
	Vector2D result;
	result.x = x - other(0);
	result.y = y - other(1);
	return result;
}


bool Vector2D::operator == (const Vector2d &other) {
	return ((float)x == other(0)) && ((float)y == other(1));
}


double Vector2D::norm() {
	return 	Vector2d(x,y).norm();
}

Vector2D Vector2D::rotate(const double &angle) {
	Vector2D result;
	Vector2d temp(x,y);
	Rotation2D<double> rotation(angle);
	result = rotation * temp;
	return result;
}














/*****************************************************************************
 * Prototype Functions
 ****************************************************************************/




double urand () {
	return (double)rand()/(double)RAND_MAX;
}


double erand (double lambda) {
	// F(x) = 1 - e^{-lambda x}
	// x = F_inv(x) = - ln( 1 - p ) / lambda
	return - log( 1 - urand() ) / lambda;
}


