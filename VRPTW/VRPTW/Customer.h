#ifndef CUSTOMER
#define CUSTOMER

#include<iostream>
using namespace std;

#include<vector>

class Customer {
private:
	/*最初拥有的信息*/
	double num;       //Customer NO.
	double x;         //XCOORD.
	double y;         //YCOORD.
	double d;         //DEMAND	
	double e_t;       //EARLIEST TIME
	double l_t;       //LATEST TIME
	double s_t;       //SERVICE TIME

	/*分格后拥有的信息*/
	double box_x;
	double box_y;

public:
	Customer() {}
	Customer(vector<double>* data);

	friend class Data;
	friend class Node;
	friend class Route;
	friend class Solution;
	friend class Solve;
};
#endif