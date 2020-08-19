#include <iostream>
using namespace std;

#include "Data.h"
#include "Solve.h"
#include<string>
#include<time.h>

int main()
{
	clock_t start, finish;

	ofstream out;
	out.open("C:\\Users\\MiaoAYao\\Desktop\\SA.csv", std::ios::app);

	string dir = "C:\\Users\\MiaoAYao\\Desktop\\tabu\\VRPTW\\Data\\";
	string file;
	cout << "输入文件名：" << endl;
	getline(cin, file);

	string file_name;
	ifstream in(file);

	//out << "**************************************************" << endl;
	//out << "*************模拟退火算法**************************" << endl;
	//out << "***************************************************" << endl;

	while (!in.eof())
	{
		getline(in, file_name);
		cout << file_name << endl;

		//根据文件中的数据构造客户节点
		Data data(dir + file_name);
		//计算距离信息
		data.computeDis();

		Solve solve(&data);
		//out << "=====================================" << endl;
		//out << file_name << endl;

		start = clock();

		/*插入启发式算法*/
		solve.useInsertion();
		solve.simulatedAnnealing();

		finish = clock();
		double duration = (double)(finish - start) / CLOCKS_PER_SEC;
		out << duration << endl;
	}
	return 0;
}