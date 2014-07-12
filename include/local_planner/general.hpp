
/*************************************************************************//**
 *****************************************************************************
 * @file        containers.hpp
 * @brief       Provide methods for sweeping a partion
 * @author      Frank Imeson
 * @date        2012-01-02
 *****************************************************************************
 ****************************************************************************/

#ifndef GENERAL_H		// guard
#define GENERAL_H


/*****************************************************************************
 * INCLUDE
 ****************************************************************************/

#include <ros/ros.h>
#include <math.h>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;




/*****************************************************************************
 * DEFINE
 ****************************************************************************/






class Vector2D {
	public:
		double x;
		double y;

		Vector2D ();
		Vector2D (double,double);
		Vector2D (const Vector2D &other);
		Vector2D (const Vector2f &other);

		Vector2D& operator = (const Vector2D &other);
		Vector2D operator + (const Vector2D &other);
		Vector2D operator - (const Vector2D &other);
		Vector2D operator * (const double &other);
		Vector2D operator / (const double &other);
		bool operator == (const Vector2D &other);

		Vector2f vector2f ();
		Vector2D& operator = (const Vector2f &other);
		Vector2D operator + (const Vector2f &other);
		Vector2D operator - (const Vector2f &other);
		bool operator == (const Vector2f &other);

		Vector2d vector2d ();
		Vector2D& operator = (const Vector2d &other);
		Vector2D operator + (const Vector2d &other);
		Vector2D operator - (const Vector2d &other);
		bool operator == (const Vector2d &other);

		double norm();
		Vector2D rotate(const double &angle);
};













/*****************************************************************************
 * Prototype Functions
 ****************************************************************************/

template < class T >
string ROS_DEBUG_VECTOR ( vector<T> row_vector ) {
	stringstream text;
	if (row_vector.size() > 0) {
		text << "[ " << row_vector[0];
		for(unsigned int i=1; i<row_vector.size(); i++)
			text << " , " << row_vector[i];
		text << " ]";
	}
	return text.str();
}

/*
template < double T >
string ROS_DEBUG_VECTOR ( vector<T> row_vector ) {
	stringstream text;
	text << "[ " << row_vector[0];
	for(unsigned int i=1; i<row_vector.size(); i++)
		text << " , " << round(row_vector[i]*100)/100;
	text << " ]";
	return text.str();
}
*/





double urand ();
double erand ();


/***********************************************************************************************
 * Tree Class
 ***********************************************************************************************/	

template <class T>
struct TreeNode {
	private:
	public:
		int depth;
		T key;
		TreeNode* parent;
		vector<TreeNode*> child;

		TreeNode ();
		TreeNode<T>* insert (T key);
		void abortion ();
};


template <class T>
TreeNode<T>::TreeNode () {
	depth = 0;
	parent = NULL;
}


template <class T>
TreeNode<T>* TreeNode<T>::insert (T key) {
	TreeNode<T>* new_node = new TreeNode<T>;
	new_node->depth = depth + 1;
	new_node->key = key;
	new_node->parent = this;
	child.push_back(new_node);
	return new_node;
}


template <class T>
void TreeNode<T>::abortion () {
	for (unsigned int i=0; i<child.size(); i++) {
		child[i]->abortion();
		delete child[i];
	}
	child.clear();
}






/***********************************************************************************************
 * RouletteWheel Class
 ***********************************************************************************************/	


template <class T>
class RouletteWheel {
	private:
		vector<T> key_wheel;
		vector<double> cost_wheel;
		double sum, max,min;
	public:
		RouletteWheel ();
		void add (T key, double cost);
		void normalize ();
		void inv_normalize ();
		T spin ();
		void clear ();
};



template <class T>
RouletteWheel<T>::RouletteWheel ()
	: sum(0)
	, max(0)
	, min(9999)
{
}

template <class T>
void RouletteWheel<T>::add (T key, double cost) {
	key_wheel.push_back(key);
	cost_wheel.push_back(cost);
	sum += abs(cost);

	if ( cost > max )
		max = cost;

	if ( cost < min )
		min = cost;
}

template <class T>
void RouletteWheel<T>::normalize () {
	ROS_DEBUG_STREAM("Roulette Norm: " << sum << ", " << ROS_DEBUG_VECTOR(cost_wheel) );
	for (unsigned int i=0; i<cost_wheel.size(); i++)
		cost_wheel[i] = cost_wheel[i]/sum;

	sum = 0;
	for (unsigned int i=0; i<cost_wheel.size(); i++) {
		sum += cost_wheel[i];
	}

	ROS_DEBUG_STREAM("Roulette Norm: " << sum << ", " << ROS_DEBUG_VECTOR(cost_wheel) );
	sum = 1.0;
}

template <class T>
void RouletteWheel<T>::inv_normalize () {

	double avg = (max-min)/2;
	//ROS_DEBUG("Roulette Norm: Max(%.2f), Min(%.2f), Avg(%.2f)", max, min, avg );
	//ROS_DEBUG_STREAM("Roulette Norm: " << ROS_DEBUG_VECTOR(cost_wheel) );
	for (unsigned int i=0; i<cost_wheel.size(); i++) {

		if ( cost_wheel[i] > avg ) {
			double delta = cost_wheel[i] - avg;
			cost_wheel[i] = avg - delta;
		} else {
			double delta = avg - cost_wheel[i];
			cost_wheel[i] = avg + delta;
		}

	}

	sum = 0;
	for (unsigned int i=0; i<cost_wheel.size(); i++) {
		sum += cost_wheel[i];
	}
	//ROS_DEBUG_STREAM("Roulette Norm: " << ROS_DEBUG_VECTOR(cost_wheel) );

	for (unsigned int i=0; i<cost_wheel.size(); i++) {
		cost_wheel[i] = cost_wheel[i]/sum;
	}

	//ROS_DEBUG_STREAM("Roulette Norm: " << ROS_DEBUG_VECTOR(cost_wheel) );
	sum = 1.0;

}


template <class T>
T RouletteWheel<T>::spin () {
	if (sum != 1.0)
		normalize();

	double spin = urand();
	for (unsigned int i=0; i<cost_wheel.size(); i++) {
		if ( spin < cost_wheel[i] ) {
			//ROS_DEBUG_STREAM("RouletteSpin = " << i << "/" << key_wheel.size() << ", Cost Wheel: " << ROS_DEBUG_VECTOR(cost_wheel) );
			return key_wheel[i];
		} else {
			spin -= cost_wheel[i];
		}
	}

	//ROS_DEBUG_STREAM("RouletteSpin = " << key_wheel.size() << "/" << key_wheel.size() << ", Cost Wheel: " << ROS_DEBUG_VECTOR(cost_wheel) );
	return key_wheel.back();
}


template <class T>
void RouletteWheel<T>::clear () {
	sum = 0;
	max = 0;
	min = 9999;
	key_wheel.clear();
	cost_wheel.clear();
}


/***********************************************************************************************
 * 
 ***********************************************************************************************/


#endif		// guard


