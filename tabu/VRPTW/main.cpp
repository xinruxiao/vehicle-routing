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
	out.open("C:\\Users\\MiaoAYao\\Desktop\\tabu.csv", std::ios::app);

	string dir = "C:\\Users\\MiaoAYao\\Desktop\\tabu\\VRPTW\\Data\\";
	string file;
	cout << "输入文件名：" << endl;
	getline(cin, file);

	/*
	out << "**************************************************" << endl;
	out << "*************禁忌搜索算法**************************" << endl;
	out << "***************************************************" << endl; */

	string file_name;
	ifstream in(file);

	while (!in.eof())
	{
		getline(in, file_name);
		cout << file_name << endl;

		//根据文件中的数据构造客户节点
		Data data(dir + file_name);
		//计算距离信息
		data.computeDis();

		//data.print();

		Solve solve(&data);
		//out << "=====================================" << endl;
		//out << file_name << ",";

		start = clock();

		/*插入启发式算法*/
		solve.useInsertion();
		solve.tabuSearch();

		finish = clock();
		double duration = (double)(finish - start) / CLOCKS_PER_SEC;
		out  << duration << endl;
	}
	return 0;
}