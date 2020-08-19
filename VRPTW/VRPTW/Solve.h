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

	/*节省启发式算法*/
	void useSavings();  //使用节省启发式算法产生初始解决方案
	void buildSavings();	//计算节省
	void deleteSavings(int i = 0, int j = 0);	//删除边的顶点为i或者j的saving值，没有参数则默认只删除savings的第一个元素
	void downAdjust(int i);	//调整最大树，把要删除的节点用最后面的值覆盖，并向下调整
	void printSaving(); 

	/*插入启发式算法*/
	void useInsertion();
	void initRoute(int flag);
	void initInfor();
	void updateInfor(Route* route);
	void printInfor();

	/*or-opt算法*/
	/*既包含单条路径上的Or交换
	又包含两条路径之间的Or交换*/
	bool useOrOpt();
	void OrExchangeNeighbour(Node* node_u, int r_num);
	void localSearch();

	/*Russell(1995)*/
	/*使用局部搜索算法对已有的解决方案进行提升*/
	void localSearch1();
	void computeU();
	void printU();

	/*Russell(2004)*/
	/*模拟退火算法*/
	void minimizeRoute();
	void simulatedAnneal();
	//判断e1<e2,返回true
	bool compareE(vector<double>* e1, vector<double>* e2);

	/*生成邻居解决方案的集合*/
	void generateNeighbour(int operate, double num, Solution* n_s);
	void generate2Exchange(double num, Solution* n_s);
	void generateOrExchange(double num, Solution* n_s);
	void generateRelocation(double num, Solution* n_s);  //只有这种情况有可能会减少路径数
	void generateExchange(double num, Solution* n_s);
	void generateCrossover(double num, Solution* n_s);

	void changeToNeighbour(vector<double>* n, Solution* temp_s);

	/*计算目标函数值*/
	void computeEvaluation(vector<double>** e,vector<double>* n, Solution* n_s);
	
	/*计算最小化路径数目的目标函数值，其中包含三个数值*/
	/*计算第一个目标函数值*/
	double computeFirstEvaluation(vector<double>* n, Solution* n_s);
	/*计算第二个目标函数值*/
	double computeSecondEvaluation(vector<double>* n, Solution* n_s);
	/*计算第三个目标函数值*/
	double computeThirdEvaluation(vector<double>* n, Solution* n_s);
	//计算解决方案n_s的最小时间延迟
	double computeMinimalDelay(Solution* n);
	double computeLatestArriveTime(Node* node_j_succ); //计算路径上最晚到达节点node_j_succ的时间

	/*根据目标函数值对邻居进行升序排序*/
	void computeOrder();

	void minimizeTravelCost();
	void selectCustomers(vector<double>* cus_set, int n);
	void orderRelatedness(vector<vector<double>>* relatedness);

	/*禁忌搜索算法*/
	void tabuSearch();
	void generateCross(Solution* n_s);
	/*禁忌搜索*/
    //判断是否在禁忌表中，是则返回true，否则返回false
	bool isTabu(vector<vector<double>*>* tabu_list, vector<double>* n);
	void updateTabuList(vector<vector<double>*>* tabu_list, int iteration, int tenure);
	void addToTabuList(vector<vector<double>*>* tabu_list, int iteration, vector<double>* n);
	void reverseOperate(vector<double>** r_n, vector<double>* n, Solution* n_s);

	void record(Solution* temp_s);

private:
	//从文件中读取的数据
	vector<Customer*>* cus;
	Data* data;

	//获得的解决方案
	Solution s;

	/*节省启发式算法需要存储的信息*/
	vector<vector<double> *> savings;	//记录节省的信息

	/*插入启发式算法需要存储的信息*/
	vector<Insertion> infor; /*未加入路径的节点编号；最好的插入位置；插入该节点后增加的开销*/

	/*Russell(1995)提出的算法所需要存储的信息*/
	vector<vector<double>> u;

	/*Russell(2004)提出的算法所需要存储的信息*/
	/*以最小化路径数目为目标所需要存储的信息*/
	vector<vector<double>> evaluation;
	vector<vector<double>*> neighbour;
	vector<int> order;   //按照evaluation大小对邻居进行排序

	//目标函数值为第二种类型
	vector<vector<double>> g;   //存储目标函数值

	friend class Solution;
};

#endif
