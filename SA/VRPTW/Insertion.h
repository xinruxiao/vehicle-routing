#ifndef INSERTION
#define INSERTION

#include<iostream>
using namespace std;

#include<vector>
#include"Node.h"

class Insertion {
private:
	/*插入条目要存储的信息*/
	double u_num;			//节点编号
	
	Node* u_pred;
	Node* u_succ;

	double cost;

public:
	Insertion() {}

	friend class Solve;
};
#endif
