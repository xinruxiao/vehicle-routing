#ifndef DATA
#define DATA

#include<iostream>
#include<fstream>
using namespace std;

#include"Customer.h"

class Data {
private:
	vector<Customer*> data;					//记录节点的信息
	vector<vector<double>> dis;         //记录每两个节点之间的距离

public:
	Data() {}
	Data(string s);
	void buildGrid(double DIV);
	void computeDis();
	double distance(double cus_i, double cus_j);   //返回客户cus_i和cus_j之间的距离

	void printDistance();

	friend class Node;
	friend class Route;
	friend class Solution;
	friend class Solve;
};
#endif