#ifndef SOLUTION
#define SOLUTION

#include"Customer.h"
#include"Data.h"
#include"Node.h"
#include"Route.h"

class Solution {
private:
	double allTime;   //解决方案的总持续时间,一段路径的旅行时间在这里与这段路径的距离数值相等
	double allLength;  //解决方案中所有路径上的距离之和
	vector<Route*> routeSet; //解决方案中的路径集

public:
	Solution() {}
	//自定义拷贝函数
	void printSolution();

	//使用s新构造一个解决方案
	void doCopy(Solution* s);

	void computeLength(Data* data);  //计算所有路径的总长度
	void computeTime();   //计算所有路径的总持续时间

	bool isRegular(Data* d);

	/*寻找编号为u的客户在哪一条路径上*/
	void search(double u, int* route_u, Node** node_u);

	void do2Exchange(Route* r, Node* i, Node* i_succ, Node* j, Node* j_succ);
	void restore2Exchange(Route* r, Node* i, Node* i_succ, Node* j, Node* j_succ);

	void doOrExchange(Node* i_pred, Node* i, Node* i_succ, Node* j, Node* j_succ);
	void restoreOrExchange(Node* i_pred, Node* i, Node* i_succ, Node* j, Node* j_succ);

	void doRelocation(Node* i_pred, Node* i, Node* i_succ, Node* j, Node* j_succ);
	void restoreRelocation(Node* i_pred, Node* i, Node* i_succ, Node* j, Node* j_succ);

	void doExchange(Node* i_pred, Node* i, Node* i_succ, Node* j_pred, Node* j, Node* j_succ);
	void restoreExchange(Node* i_pred, Node* i, Node* i_succ, Node* j_pred, Node* j, Node* j_succ);

	void doCrossover(Node* i, Node* i_succ, Node* j, Node* j_succ);
	void restoreCrossover(Node* i, Node* i_succ, Node* j, Node* j_succ);

	friend class Solve;
};
#endif
