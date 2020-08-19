#include"Data.h"

Data::Data(string s)
{
	ifstream in(s);

	/* 根据文件中存储的数据构造一个图 */
	vector<double> *cus_data;	//每一个客户的数据
	Customer* tmp;                  //用客户的数据构造一个节点
	while (!in.eof())
	{
		cus_data = new vector<double>();
		double d;
		for (int i = 0; i < 7; ++i)
		{
			in >> d;
			if (i == 0)
				d++;
			cus_data->push_back(d);
		}
		tmp = new Customer(cus_data);
		data.push_back(tmp);
	}
	cout << "Finish reading the data from file..." << endl;
}

void Data::buildGrid(double DIV)
{
	int size;
	size = data.size();
	//max_X 所有节点的x坐标最大的那个
	//max_Y 所有节点的y坐标最大的那个
	double max_X = data[0]->x, max_Y = data[0]->y;
	for (int i = 1; i < size; ++i)
	{
		if (data[i]->x > max_X)
			max_X = data[i]->x;
		if (data[i]->y > max_Y)
			max_Y = data[i]->y;
	}

	//将所有节点包括在内的一个长方形，宽为WIDTH，高为HEIGHT
	const double WIDTH = (floor(max_X / 100) + 1) * 100;
	const double HEIGHT = (floor(max_Y / 100) + 1) * 100;
	cout << "width = " << WIDTH << endl;
	cout << "Height = " << HEIGHT << endl;

	//划分的小方格的宽step_X和高step_X
	const int step_X = WIDTH / DIV;
	const int step_Y = HEIGHT / DIV;

	// 仓库不在任一方格之内
	data[0]->box_x = 0;
	data[0]->box_y = 0;
	for (int i = 1; i < size; ++i)
	{
		int box_X = 0, box_Y = 0;
		while (data[i]->x > box_X)
		{
			box_X += step_X;
		}
		while (data[i]->y > box_Y)
		{
			box_Y += step_X;
		}
		data[i]->box_x = box_X;
		data[i]->box_y = box_Y;
	}
	cout << "Finish building the grid..." << endl;
}

void Data::computeDis()
{
	int size = data.size();
	vector<double>* tmp;
	for (int i = 1; i < size; ++i)
	{
		tmp = new vector<double>();
		//计算i和比i小的节点之间的距离
		for (int j = 0; j < i; ++j)
		{
			double distance = sqrt(pow(data[i]->x - data[j]->x, 2) + pow(data[i]->y - data[j]->y, 2));
			tmp->push_back(distance);
		}
		dis.push_back(*tmp);
	}
}

double Data::distance(double cus_i, double cus_j)
{
	if (cus_i > cus_j)
		return dis[cus_i - 2][cus_j - 1];
	else
		return dis[cus_j - 2][cus_i - 1];
}

void Data::printDistance()
{
	for (int i = 0; i < dis.size(); ++i)
	{
		for (int j = 0; j < dis[i].size(); ++j)
			cout << dis[i][j] << "   ";
		cout << endl;
	}
}