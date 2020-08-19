#ifndef NODE
#define NODE

#include<iostream>
using namespace std;

#include<vector>
#include"Data.h"

class Node {
private:
	/*路径中的节点需要存储的信息*/
	double num;			//节点编号

	double a_t;       //ARRIVE TIME
	double w_t;       //WAIT TIME
	double b_t;       //开始服务时间 b_t = a_t + d_t
	double d_t;       //DEPART TIME

	double z_t;       //最晚到达时间

	Node* pred;
	Node* succ;

public:
	Node(){}

	void doCopy(Node* n)
	{
		num = n->num;

		a_t = n->a_t;
		w_t = n->w_t;
		b_t = n->b_t;
		d_t = n->d_t;

		z_t = n->z_t;
	}

	friend class Route;
	friend class Solution;
	friend class Solve;
};
#endif