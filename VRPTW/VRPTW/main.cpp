#include <iostream>
using namespace std;

#include "Data.h"
#include "Solve.h"
#include<string>
#include<direct.h>

#define DIV 6

int main()
{
	/*获取当前目录
	char* direct = new char(100);
	cout << _getcwd(direct, 100) << endl;*/
	ofstream out;
	out.open("C:\\Users\\MiaoAYao\\Desktop\\insertion.txt", std::ios::app);

	string dir = "C:\\Users\\MiaoAYao\\Desktop\\VRPTW\\VRPTW\\Data\\";
	string file;
	cout << "输入文件名：" << endl;
	getline(cin, file);

	string file_name;
	ifstream in(file);

	/*
	out << "*******************************************" << endl;
	out << "-------------节省启发式算法-----------------" << endl;
	out << "*******************************************" << endl;
	out << endl;
	out << endl;
	*/

	while (!in.eof())
	{
		getline(in, file_name);
		cout << file_name << endl;

		//根据文件中的数据构造客户节点
		Data data(dir + file_name);
		//对客户节点进行分格
		data.buildGrid(DIV);
		//计算距离信息
		data.computeDis();
		//输出距离信息
		//data.printDistance();

		Solve solve(&data);

		/*选择使用哪种路径构建算法
		int flag;
		cout << "-----------选择路径构建算法-------------" << endl;
		cout << "-----------如果选择节省启发式算法输入：1------------" << endl;
		cout << "-----------如果选择插入启发式算法输入：2------------" << endl;
		cin >> flag;

		if (flag == 1)
		{
			solve.buildSavings();     //根据距离信息计算节省
			solve.useSavings();
		}
		else if (flag == 2)
		{
			solve.useInsertion();
			//problem.localSearch1();
		}
		*/

		/*使用解决方案改进算法对解决方案进行改进
		cout << "-----------选择是否进行路径数目最小化---------------" << endl;
		cout << "-----------如果进行路径数目最小化输入：1------------" << endl;
		cout << "-----------否则输入：2-----------------------------" << endl;
		cin >> flag;
		if(flag == 1)
			solve.minimizeRoute(); */

		//solve.localSearch();
		
		/*节省启发式算法*/
		/*solve.buildSavings();     //根据距离信息计算节省
		solve.useSavings();*/


		out << "=====================================" << endl;
		out << file_name << endl;

		/*插入启发式算法*/
		solve.useInsertion();

		//传统的局部搜索算法
		//solve.localSearch();

		//solve.simulatedAnneal();
		//solve.tabuSearch();

	}

	/*使用R101.txt进行测试
	string file_name = "R101.txt";*/

	/* 自己选择文件
	cout << "Enter the name of the file" << endl;
	getline(cin, file_name);
	ifstream in(file_name); */

	
	return 0;
}