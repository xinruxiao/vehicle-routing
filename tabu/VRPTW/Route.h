#ifndef ROUTE
#define ROUTE

#include"Node.h"
#include"Customer.h"
#include"Data.h"

class Route {
private:
	double demand;   //路径上的总需求
	int size;        //路径上节点的数目
	Node* head;      //路径上第一个节点
	Node* tail;      //路径上最后一个节点

public:
	Route(){}
	Route(double u, Data* d);      //用编号为num的节点初始化一条路径

	Node* searchNode(double num);

	double computeNewAT(double u, double dt_u, Node* p_j, Data* d);

	//判断路径上节点p_j以及其后面的节点是否可行
	//u为前一个节点的编号，dt_u为节点u的离开时间
	//适用于路径的方向没有发生改变的情况
	bool isFeasible(double u, double dt_u, Node* p_j, Data* d);
	//测试一般情况下是否可行，可以应用于任何情况
	bool isFeasibleGC(double u, double dt_u, Node* p_j, Data* d);

	//将路径r添加到当前路径后面是否可行
	//当当前路径是有反转得到的时候，使用这一函数判定可行性
	bool isFeasibleAddRoute(Route* r, Data* d);

	//更新路径上从p_j开始的节点信息
	void updateRoute(Node* p_j, Data* d);

	void reverseRoute();	//反转全部路径
	//节点pi在节点pj前面
	void reversePartialRoute(Node* pi, Node* pj); //反转路径中的一部分,即反转节点pi到节点pj的路径

	//将路径j上的节点合并到路径i上
	void mergeRoute(Route* p_j, Data* d);

	void printRoute();

	friend class Solution;
	friend class Solve;
};
#endif
