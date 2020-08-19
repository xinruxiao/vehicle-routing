#ifndef SOLVE
#define SOLVE

#include<fstream>
#include<iostream>
using namespace std;

#include"Node.h"
#include"Route.h"
#include"Solution.h"
#include"Insertion.h"

#include<vector>
#include<ctime>

class Solve {
public:
	Solve() {};
	Solve(Data* d);

	/*插入启发式算法*/
	void useInsertion();
	void initRoute(int flag);
	void initInfor();
	void updateInfor(Route* route);
	void printInfor();

	/*生成邻居解决方案的集合*/
	void generateNeighbour(int operate, double num, Solution* n_s);
	void generate2Exchange(double num, Solution* n_s);
	void generateOrExchange(double num, Solution* n_s);
	void generateRelocation(double num, Solution* n_s);  //只有这种情况有可能会减少路径数
	void generateExchange(double num, Solution* n_s);
	void generateCrossover(double num, Solution* n_s);

	void changeToNeighbour(vector<double>* n, Solution* temp_s);


	/*禁忌搜索算法*/
	void tabuSearch();
	/*禁忌搜索*/
    //判断是否在禁忌表中，是则返回true，否则返回false
	bool isTabu(vector<double>* n);
	void updateTabuList(int iteration);
	void addToTabuList(int iteration, vector<double>* n);
	void reverseOperate(vector<double>** r_n, vector<double>* n, Solution* n_s);

	void record(Solution* temp_s);

private:
	//从文件中读取的数据
	vector<Customer*>* cus;
	Data* data;

	//获得的解决方案
	Solution s;

	/*插入启发式算法需要存储的信息*/
	vector<Insertion> infor; /*未加入路径的节点编号；最好的插入位置；插入该节点后增加的开销*/

	/*禁忌搜索算法*/
	vector<vector<double>*> tabu_list;
	int tenure;  //禁忌周期

	//最好的邻居解决方案的开销
	vector<double> best_neighbour;
	double g_best_neighbour;

	friend class Solution;
};

#endif
