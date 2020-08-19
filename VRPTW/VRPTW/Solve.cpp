#include "Solve.h"

#include<math.h>

#define DEMAND 1000

//插入启发式算法需要的参数
/*1 1 1 0
  1 2 1 0
  1 1 0 1
  1 2 0 1*/
#define U1 1
#define U2 1      
#define M1 1
#define M2 0

  //Russell(1995)所需参数
#define V1 0.5
#define V2 0.5

//Russell(2004)所需参数
#define T 3000 //初始温度
#define EPS 1e-8 //终止温度
#define R 0.98 //温度降低速率

void Solve::record(Solution* temp_s)
{
	ofstream out;
	out.open("C:\\Users\\MiaoAYao\\Desktop\\insertion.txt", std::ios::app);

	/*
	out << "路径数目：";
	out << temp_s->routeSet.size() << endl;
	out << "路程持续时间：";
	out << temp_s->allTime << endl;
	out << "路程总长度：";
	out << temp_s->allLength << endl;
	if (temp_s->isRegular(data))
		out << "该解决方案合格..." << endl;
	else
		out << "该解决方案不合格..." << endl;
	out << "-------------------------------------" << endl;*/

	out << temp_s->routeSet.size() << "  " << temp_s->allLength * 0.7 + temp_s->allTime * 0.3;
	out << endl;

}

Solve::Solve(Data* d)
{
	cus = &(d->data);
	data = d;
}

/*------------------------------------------------------------------------------*/
/*-------------------------------solomon(1987)----------------------------------*/
/*--------------------------节省启发式路径构建算法-------------------------------*/
/*------------------------------------------------------------------------------*/
void Solve::buildSavings()
{
	vector<double> *saving;
	int size = cus->size();

	//计算节省的值
	for (int i = 1; i < size - 1; ++i)
	{
		for (int j = i + 1; j < size; ++j)
		{
			//只有在同一个方格时才计算距离
			if ((*cus)[i]->box_x == (*cus)[j]->box_x && (*cus)[i]->box_y == (*cus)[j]->box_y)
			{
				saving = new vector<double>();
				saving->push_back(data->distance(i + 1, 1) + data->distance(j + 1, 1) + data->distance(i + 1, j + 1));
				saving->push_back(i + 1);
				saving->push_back(j + 1);

				savings.push_back(saving);
				// 对savings进行调整
				int m = savings.size();
				if (m > 1)
				{
					int n = m - 1;
					while (n > 0)
					{
						int nn = (n - 1) / 2;
						if ((*savings[n])[0] > (*savings[nn])[0])
						{
							vector<double> *temp;
							temp = savings[n];
							savings[n] = savings[nn];
							savings[nn] = temp;
						}
						n = nn;
					}
				}
			}
		}
	}
	cout << "Finish building the savings..." << endl;
}

void Solve::printSaving()
{
	for (int i = 0; i < savings.size(); ++i)
	{
		for (int j = 0; j < savings[i]->size(); ++j)
			cout << (*savings[i])[j] << "   ";

		cout << endl;
	}
}

void Solve::deleteSavings(int cus_i, int cus_j)
{
	//i和j不可能等于0，因为0代表仓库。如果不带参数，说明只删除最上面的。如果带参数，则既删除更上面的，又删除其他的。
	vector<double> * tmp = savings.back();
	savings[0] = tmp;
	savings.pop_back();
	downAdjust(0);

	int size = savings.size();
	for (int i = size - 1; i >= 0; --i)
	{
		vector<double>* temp = savings[i];
		if ((*temp)[1] == cus_i || (*temp)[1] == cus_j || (*temp)[2] == cus_i || (*temp)[2] == cus_j)
		{
			//cout << "***************************************" << endl;
			tmp = savings.back();
			savings[i] = tmp;
			savings.pop_back();
			downAdjust(i);
		}
	}
}

void Solve::downAdjust(int i)
{
	int size = savings.size();
	int j = i;
	while(true)
	{
		int maxSav = j * 2 + 1;
		if (maxSav >= size)
			break;

		vector<double>* temp = savings[j];
		if (maxSav + 1 < size)
		{
			if ((*savings[maxSav])[0] < (*savings[maxSav + 1])[0])
				maxSav = maxSav + 1;
		}

		if ((*temp)[0] < (*savings[maxSav])[0])
		{
			savings[j] = savings[maxSav];
			savings[maxSav] = temp;

			j = maxSav;
		}
		else
			break;
	}
}

void Solve::useSavings()
{
	Route *route;

	//Build original solution
	for (int i = 1; i < (*cus).size(); ++i)
	{
		route = new Route(i + 1, data);
		s.routeSet.push_back(route);
	}

	//初始解决方案
	cout << "-------------------------初始解决方案--------------------------" << endl;
	s.computeTime();
	s.computeLength(data);
	s.printSolution();
	cout << "---------------------------------------------------------------" << endl;

	//Build better solution
	while (savings.size() != 0)
	{
		cout << "--------------------------------------------" << endl;
		cout << "Savings的条目数为： " << savings.size() << endl;
		//printSaving();

		/* 加入边cus_i -> cus_j后，能够节省最大距离的边的信息*/
		int cus_i = (*savings[0])[1];
		int cus_j = (*savings[0])[2];
		double sav = (*savings[0])[0];

		cout << "试探着连接节点：" << cus_i << "和" << cus_j << endl;

		/* 找到这条边上的两个节点分别在哪个路径上 */
		/* route_i:包含节点i的路径   route_j:包含节点j的路径
		   flag_i = 0:节点i在路径头  flag_i = 1:节点i在路径尾*/
		   /* del_i = 0:表示合并两条路径后，节点i仍在路径头或尾，反之del_i = 1*/
		int route_i = -1, route_j = -1;
		Node* node_i = NULL;
		Node* node_j = NULL;
		int flag_i = -1, flag_j = -1;
		int del_i = 0, del_j = 0;

		//找到路径集中包含客户cus_i和cus_j的两条路径
		s.search(cus_i, &route_i, &node_i);
		s.search(cus_j, &route_j, &node_j);

		if (node_i == (s.routeSet[route_i]->head)->succ)
			flag_i = 0;
		else if (node_i == (s.routeSet[route_i]->tail)->pred)
			flag_i = 1;

		if (node_j == (s.routeSet[route_j]->head)->succ)
			flag_j = 0;
		else if (node_j == (s.routeSet[route_j]->tail)->pred)
			flag_j = 1;

		if (s.routeSet[route_i]->size > 1)
		{
			del_i = 1;
		}
		if (s.routeSet[route_j]->size > 1)
		{
			del_j = 1;
		}

		//存在特殊情况，saving最大的边上的两个客户，已经在一条路径上了。
		if (route_i == route_j)
		{
			deleteSavings();
			continue;
		}


		/* 合并路径 */
		/* 是否真的要合并路径，要首先判断一下可行性 */
		/* 1. 路径上的节点的需求是否超过车辆的总容量*/
		/* 2. 路径上的节点是否违背时间窗约束*/
		/* 如果可行，那么进行路径的合并，否则，删除当前节省*/
		if (s.routeSet[route_i]->demand + s.routeSet[route_j]->demand > DEMAND)
		{
			//路径上的节点的需求超过车辆的总容量
			deleteSavings();
			continue;
		}

		if (flag_i == 0 && flag_j == 1)
		{
			/* 要连在一起的点为i和j，i在路径route_i的头，j在路径route_j的尾
			   两种连接方式：
			   1.将route_i上的节点加到route_j后
			   2.将route_i和route_j反转，并将route_j加到route_i后 */

			//1.将route_i上的节点加到route_j后，是否可行
			if (s.routeSet[route_i]->isFeasibleGC(node_j->num, node_j->d_t, node_i, data))
			{
				cout << "确定连接节点：" << cus_i << "和" << cus_j << endl;

				//如果可行，将route_i合并到route_j，并更新路径
				s.routeSet[route_j]->mergeRoute(s.routeSet[route_i], data);
				s.routeSet[route_j]->updateRoute(node_i, data);

				//删除路径route_i
				s.routeSet.erase(s.routeSet.begin() + route_i);
			}
			/*将route_i和route_j反转，并将route_j加到route_i后*/
			else
			{
				//反转路径i，j
				s.routeSet[route_i]->reverseRoute();
				s.routeSet[route_j]->reverseRoute();

				//将route_j上的节点加到route_i后，是否可行
				if (s.routeSet[route_i]->isFeasibleAddRoute(s.routeSet[route_j], data))
				{
					cout << "确定连接节点：" << cus_i << "和" << cus_j << endl;

					//如果可行
					s.routeSet[route_i]->mergeRoute(s.routeSet[route_j], data);
					s.routeSet[route_i]->updateRoute(s.routeSet[route_i]->head->succ, data);

					//删除路径route_j
					s.routeSet.erase(s.routeSet.begin() + route_j);
				}
				//如果不可行
				else
				{
					//如果不可行，还原路径i和j的反转
					s.routeSet[route_i]->reverseRoute();
					s.routeSet[route_j]->reverseRoute();

					deleteSavings();
					continue;
				}
			}
		}
		else if (flag_i == 1 && flag_j == 0)
		{
			/* 要连在一起的点为i和j，i在路径route_i的尾，j在路径route_j的头
			   两种连接方式：
			   1.将route_i和route_j反转，并将route_i上的节点加到route_j后
			   2.将route_j加到route_i后 */

			//反转路径i和j
			s.routeSet[route_i]->reverseRoute();
			s.routeSet[route_j]->reverseRoute();

			//将route_i上的节点加到route_j后，是否可行
			if (s.routeSet[route_j]->isFeasibleAddRoute(s.routeSet[route_i], data))
			{
				cout << "确定连接节点：" << cus_i << "和" << cus_j << endl;

				//如果可行，将route_i合并到route_j，并更新路径
				s.routeSet[route_j]->mergeRoute(s.routeSet[route_i], data);
				s.routeSet[route_j]->updateRoute(s.routeSet[route_j]->head->succ, data);

				//删除路径route_i
				s.routeSet.erase(s.routeSet.begin() + route_i);
			}
			else
			{
				//还原反转路径i，j
				s.routeSet[route_i]->reverseRoute();
				s.routeSet[route_j]->reverseRoute();

				//将route_j上的节点加到route_i后，是否可行
				if (s.routeSet[route_j]->isFeasibleGC(node_i->num, node_i->d_t, node_j, data))
				{
					cout << "确定连接节点：" << cus_i << "和" << cus_j << endl;

					//如果可行
					s.routeSet[route_i]->mergeRoute(s.routeSet[route_j], data);
					s.routeSet[route_i]->updateRoute(node_j, data);

					//删除路径route_j
					s.routeSet.erase(s.routeSet.begin() + route_j);
				}
				else
				{
					deleteSavings();
					continue;
				}
			}
		}
		else if(flag_i == 0 && flag_j == 0)
		{
		/* 要连在一起的点为i和j，i和j都在路径route_i的头
		   两种连接方式：
		   1.route_j反转，并将route_i上的节点加到route_j后
		   2.将route_i反转，并将route_j加到route_i后 */

			//反转路径j
			s.routeSet[route_j]->reverseRoute();
			//将route_i上的节点加到route_j后，是否可行
			if (s.routeSet[route_j]->isFeasibleAddRoute(s.routeSet[route_i], data))
			{
				cout << "确定连接节点：" << cus_i << "和" << cus_j << endl;

				//如果可行，将route_i合并到route_j，并更新路径
				s.routeSet[route_j]->mergeRoute(s.routeSet[route_i], data);
				s.routeSet[route_j]->updateRoute(s.routeSet[route_j]->head->succ, data);

				//删除路径route_i
				s.routeSet.erase(s.routeSet.begin() + route_i);
			}
			else
			{
				//还原反转j
				s.routeSet[route_j]->reverseRoute();

				//反转i
				s.routeSet[route_i]->reverseRoute();

				//将route_j上的节点加到route_i后，是否可行
				if (s.routeSet[route_i]->isFeasibleAddRoute(s.routeSet[route_j], data))
				{
					cout << "确定连接节点：" << cus_i << "和" << cus_j << endl;

					//如果可行
					s.routeSet[route_i]->mergeRoute(s.routeSet[route_j], data);
					s.routeSet[route_i]->updateRoute(s.routeSet[route_i]->head->succ, data);

					//删除路径route_j
					s.routeSet.erase(s.routeSet.begin() + route_j);
				}
				else
				{
					//还原i反转
					s.routeSet[route_i]->reverseRoute();

					deleteSavings();
					continue;
				}
			}
		}
		else if (flag_i == 1 && flag_j == 1)
		{

			//反转i
			s.routeSet[route_i]->reverseRoute();

			//将route_i上的节点加到route_j后，是否可行
			if (s.routeSet[route_i]->isFeasibleGC(node_j->num, node_j->d_t, node_i, data))
			{
				cout << "确定连接节点：" << cus_i << "和" << cus_j << endl;

				//如果可行，将route_i合并到route_j，并更新路径
				s.routeSet[route_j]->mergeRoute(s.routeSet[route_i], data);
				s.routeSet[route_j]->updateRoute(node_i, data);

				//删除路径route_i
				s.routeSet.erase(s.routeSet.begin() + route_i);
			}
			else
			{
				//还原i反转
				s.routeSet[route_i]->reverseRoute();

				//反转路径j
				s.routeSet[route_j]->reverseRoute();

				//将route_j上的节点加到route_i后，是否可行
				if (s.routeSet[route_j]->isFeasibleGC(node_i->num, node_i->d_t, node_j, data))
				{
					cout << "确定连接节点：" << cus_i << "和" << cus_j << endl;

					//如果可行
					s.routeSet[route_i]->mergeRoute(s.routeSet[route_j], data);
					s.routeSet[route_i]->updateRoute(node_j, data);

					//删除路径route_j
					s.routeSet.erase(s.routeSet.begin() + route_j);
				}
				else
				{
					//还原反转j
					s.routeSet[route_j]->reverseRoute();

					deleteSavings();
					continue;
				}

			}

		}


		/* 删除savings中已不在路径头或尾的节点所在的边，进行下一次加边*/
		if (del_i == 1 && del_j == 1)
		{
			cout << "删除顶点为" << cus_i << "或者" << cus_j  << "的saving" << endl;
			deleteSavings(cus_i, cus_j);
		}
		else if (del_i == 1 && del_j == 0)
		{
			cout << "删除顶点为" << cus_i << "的saving" << endl;
			deleteSavings(cus_i);
		}
		else if (del_i == 0 && del_j == 1)
		{
			cout << "删除顶点为" << cus_j << "的saving" << endl;
			deleteSavings(cus_j);
		}
		else
		{
			deleteSavings();
		}

		s.computeTime();
		s.computeLength(data);
		s.printSolution();
		s.isRegular(data);
	}
	cout << "------------------------------------------------------" << endl;
	cout << "---------由节省启发式算法得到的一个可行解-------------" << endl;
	cout << "------------------------------------------------------" << endl;
	s.computeTime();
	s.computeLength(data);
	s.printSolution();
	s.isRegular(data);
}

/*------------------------------------------------------------------------------*/
/*-------------------------------solomon(1987)----------------------------------*/
/*--------------------------插入启发式路径构建算法-------------------------------*/
/*------------------------------------------------------------------------------*/

void Solve::printInfor()
{
	for (int i = 0; i < infor.size(); ++i)
	{

		double u = infor[i].u_num;  //要插入的节点编号为u
		cout << u << "   ";
	}
	cout << endl;
}

void Solve::initInfor()
{
	Insertion* tmp;
	for (int i = 1; i < cus->size(); ++i)
	{
		tmp = new Insertion();
		tmp->u_num = (*cus)[i]->num;
		tmp->u_pred = NULL;
		tmp->u_succ = NULL;
		tmp->cost = -1;

		infor.push_back(*tmp);
	}
}

void Solve::updateInfor(Route* route)
{
	for (int i = 0; i < infor.size(); ++i)
	{

		double u = infor[i].u_num;  //要插入的节点编号为u
		
		Node* node_m = route->head;
		Node* node_n;
		double m, n;  //将节点u插入到节点编号为m和n的节点之间

		Node* insert_m = NULL;    //最好的插入位置
		Node* insert_n = NULL;

		double c_1 = -1;		//需要计算的值
		double c_11;
		double c_12;

		//当不违背容量约束时，才进行考虑插入位置的选择
		if ((*cus)[u - 1]->d + route->demand < DEMAND)
		{
			//m与当前节点u的距离；m和n的距离；u和n的距离
			double dis_mu, dis_mn, dis_un;

			/*at_u：到达节点u的时间*/
			double at_u;
			double wt_u;
			double dt_u;
			double at_n;

			while (true)
			{
				node_n = node_m->succ;

				m = node_m->num;
				n = node_n->num;

				dis_mu = data->distance(m, u);
				dis_un = data->distance(u, n);
				dis_mn = data->distance(m, n);

				at_u = node_m->d_t + dis_mu;
				/*判断是否违背了节点u的时间窗约束
				如果违背了，那么，既然在当前位置插入都不可行了，那么之后的位置也不用尝试了*/
				if (at_u > (*cus)[u - 1]->l_t)
				{
					break;
				}

				//如果当前节点没有违背时间窗约束
				/*如果插入位置不是路径末尾*/
				c_11 = dis_mu + dis_un - U1 * dis_mn;

				//at_n:插入节点u后到达n的时间；wt_u：在节点u的等待时间；dt_u：从节点u的出发时间
				wt_u = at_u > (*cus)[u - 1]->e_t ? 0 : (*cus)[u - 1]->e_t - at_u;
				dt_u = at_u + wt_u + (*cus)[u - 1]->s_t;
				at_n = dt_u + dis_un;
				double wt_n = at_n > (*cus)[n - 1]->e_t ? 0 : (*cus)[n - 1]->e_t - at_n;
				c_12 = at_n + wt_n - (node_n->a_t + node_n->w_t);


				//只有插入节点u后，可以使路径上在u之后的节点都可行，才能插入在这个位置
				if (route->isFeasible(u, dt_u, node_n, data))
				{
					double tmp_c_1 = M1 * c_11 + M2 * c_12;
					if (c_1 == -1 || tmp_c_1 < c_1)
					{
						c_1 = tmp_c_1;
						insert_m = node_m;
						insert_n = node_n;
					}
				}

				node_m = node_n;
				if (node_m == route->tail)
				{
					break;
				}
			}
		}

		infor[i].u_pred = insert_m;
		infor[i].u_succ = insert_n;
		infor[i].cost = c_1;
	}
}

void Solve::initRoute(int flag)
{
	//选择的用来初始化路径的节点编号
	double c_num;
	//infor中要删除的条目
	int del;

	/*两种选择：1，离仓库最远的节点；2，服务时间开始最早的节点*/
	if (flag == 1)
	{
		c_num = infor[0].u_num;
		double maxDis = data->distance(c_num, 1);
		del = 0;

		//找没被加入路径中离仓库最远的节点
		for (int i = 1; i < infor.size(); ++i)
		{
			if (data->distance(infor[i].u_num, 1) > maxDis)
			{
				c_num = infor[i].u_num;
				maxDis = data->distance(c_num, 1);
				del = i;
			}
		}
	}
	else if (flag == 2)
	{
		c_num = infor[0].u_num;
		double eTime = (*cus)[c_num - 1]->e_t;
		del = 0;

		//找没被加入路径中服务时间开始最早的节点
		for (int j = 1; j < infor.size(); ++j)
		{
			if ((*cus)[infor[j].u_num - 1]->e_t < eTime)
			{
				c_num = infor[j].u_num;
				eTime = (*cus)[c_num - 1]->e_t;
				del = j;
			}
		}
	}

	//删除infor中第del个条目
	infor.erase(infor.begin() + del);

	Route* route = new Route(c_num, data);
	s.routeSet.push_back(route);

}

void Solve::useInsertion()
{
	/*
	int flag;
	cout << "------- --选择初始化一条路径的方法----------------------" << endl;
	cout << "----------如果选择使用离仓库最远的节点初始化路径，输入：1---------------" << endl;
	cout << "----------如果选择使用服务时间开始最早的节点初始化路径，输入：2---------" << endl;
	cin >> flag;*/

	int flag = 1;

	Route* nRoute;
	initInfor();
	while (infor.size() != 0)   //一直产生新的路径，直到所有的节点都加入到一个路径中
	{
		cout << "产生一条新的路径..." << endl;
		//产生一条新的路径添加到路径集的末尾
		initRoute(flag);
		//nRoute为当前的路径
		nRoute = s.routeSet.back();


		/*如果能够在当前路径中可以找到可行的插入*/
		while (true)
		{

			cout << "-----------------------------------------" << endl;
			cout << "当前路径为：";
			nRoute->printRoute();

			//更新infor
			updateInfor(nRoute);

			//找到最好的那个节点插入到路径中
			double c_2;
			double num = -1; //要插入的节点的条目为infor[num]
			int i;
			for (i = 0; i < infor.size(); ++i)
			{
				if (infor[i].cost != -1) //infor[i][1]为-1时表示在当前路径中没有可行的插入选择
				{
					c_2 = U2 * data->distance(infor[i].u_num, 1) - infor[i].cost;
					num = i;
					break;
				}
			}
			for (i = i + 1; i < infor.size(); ++i)
			{
				if (infor[i].cost != -1)
				{
					double tmp_c_2 = U2 * data->distance(infor[i].u_num, 1) - infor[i].cost;
					if (tmp_c_2 > c_2)
					{
						c_2 = tmp_c_2;
						num = i;
					}
				}
			}

			if (num == -1)  //在当前路径中无法找到可行的插入
			{
				break;
			}
			else
			{
				double u;
				Node* insert_m;
				Node* insert_n;  //在编insert_m和insert_n节点中间插入该节点

				//插入信息在infor[num]中存储
				u = infor[num].u_num;
				insert_m = infor[num].u_pred;
				insert_n = infor[num].u_succ;

				//将条目infor[num]删除，表示节点u已经添加到路径中
				//cout << "删除与" << u << "相关的条目前" << endl;
				//printInfor();
				infor.erase(infor.begin() + num);
				//cout << "删除与" << u << "相关的条目后" << endl;
				//printInfor();

				/*将节点u插入到insert_m和insert_n之间*/
				cout << "在" << insert_m->num << "和" << insert_n->num << "之间插入节点：" << u << endl;
				Node* node_u = new Node();
				node_u->num = u;

				insert_m->succ = node_u;
				insert_n->pred = node_u;
				node_u->succ = insert_n;
				node_u->pred = insert_m;

				nRoute->size++;
				nRoute->demand += (*cus)[u - 1]->d;

				/*更新节点u,以及其之后节点的到达时间等信息*/
				nRoute->updateRoute(node_u, data);
			}
		}
	}


	cout << "------------------------------------------------------" << endl;
	cout << "---------由插入启发式算法得到的一个可行解-------------" << endl;
	cout << "------------------------------------------------------" << endl;
	s.computeLength(data);
	s.computeTime();
	s.printSolution();
	s.isRegular(data);

	record(&s);

	/*测试一下反转路径是否反转正确
	for (int i = 0; i < s.routeSet.size(); ++i)
	{
		cout << "第" << i + 1 << "条路径.............." << endl;
		s.routeSet[i]->reverseRoute();
		s.routeSet[i]->printRoute();
		s.routeSet[i]->reverseRoute();
		s.routeSet[i]->printRoute();
	}*/
}


/*------------------------------------------------------------------------------*/
/*-------------------------------Local Search----------------------------------*/
/*-------------------------------------------------------------------------------*/

void Solve::OrExchangeNeighbour(Node* node_u, int r_num)
{
	/*清空上一次产生的邻居的评价值等信息*/
	vector<vector<double>*>().swap(neighbour);
	vector<vector<double>>().swap(evaluation);
	vector<int>().swap(order);

	double i = node_u->num;
	Node* node_i = node_u;
	int r = r_num; //客户c所在的路径
	Node* node_i_succ = node_i->succ;
	Node* node_i_pred = node_i->pred;

	//将i_pred->i,i->i_succ,j->j_succ换成i_pred->i_succ,j->i,i->j_succ
	//i固定后，i_pred和i_succ都是固定的；改变的是j
	//j的变化,只要j不等于i_pred,i都可以
	//j可以是从仓库到回到仓库前的最后一个节点
	vector<double>* n;
	vector<double>* e;
	Node* node_j;
	Node* node_j_succ;

	/*j与i在同一条路径上*/
	//j在i前面
	Node* tmp = s.routeSet[r]->head;
	while (tmp != node_i_pred)
	{

		node_j = tmp;
		node_j_succ = tmp->succ;

		cout << "------------------------------------------------------------" << endl;
		cout << "对客户" << node_i->num << "和" << node_j->num << "进行OrExchange..." << endl;
		cout << "路径r：";
		s.routeSet[r]->printRoute();
		cout << "=====" << endl;

		s.doOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);

		cout << "路径r：";
		s.routeSet[r]->printRoute();

		//判断OrExchange邻居是否可行
		if (s.routeSet[r]->isFeasibleGC(node_j->num, node_j->d_t, node_i, data))
		{
			//如果可行
			//记录该邻居信息
			n = new vector<double>();
			n->push_back(2);
			n->push_back(node_i->num);
			n->push_back(node_j->num);
			n->push_back(r);
			n->push_back(1);

			//计算这种情况下的开销
			e = new vector<double>();
			e->push_back(s.routeSet.size());
			double tmp_length = s.allLength + data->distance(node_i_pred->num, node_i_succ->num)
				+ data->distance(node_j->num, node_i->num)
				+ data->distance(node_i->num, node_j_succ->num)
				- data->distance(node_i_pred->num, node_i->num)
				- data->distance(node_i->num, node_i_succ->num)
				- data->distance(node_j->num, node_j_succ->num);
			e->push_back(tmp_length);
			double tmp_time = s.routeSet[r]->computeNewAT(node_j->num, node_j->d_t, node_i, data);
			tmp_time -= (*cus)[0]->e_t;
			e->push_back(s.allTime + tmp_time - (s.routeSet[r]->tail->a_t - s.routeSet[r]->head->d_t));

			neighbour.push_back(n);
			evaluation.push_back(*e);

			s.restoreOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);
		}
		else
		{
			cout << "进行OrExchange不可行..." << endl;
			//如果不可行
			s.restoreOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);
		}

		tmp = tmp->succ;
	}

	//j在i后面
	tmp = node_i_succ;
	while (tmp != s.routeSet[r]->tail)
	{
		node_j = tmp;
		node_j_succ = node_j->succ;

		cout << "------------------------------------------------------------" << endl;
		cout << "对客户" << node_i->num << "和" << node_j->num << "进行OrExchange..." << endl;
		cout << "路径r：";
		s.routeSet[r]->printRoute();
		cout << "=====" << endl;

		s.doOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);

		cout << "路径r：";
		s.routeSet[r]->printRoute();

		//判断OrExchange邻居是否可行
		if (s.routeSet[r]->isFeasible(node_j->num, node_j->d_t, node_i, data))
		{
			//如果可行
			//记录该邻居信息
			n = new vector<double>();
			n->push_back(2);
			n->push_back(node_i->num);
			n->push_back(node_j->num);
			n->push_back(r);
			n->push_back(0);

			
			//计算这种情况下的开销
			e = new vector<double>();
			e->push_back(s.routeSet.size());
			double tmp_length = s.allLength + data->distance(node_i_pred->num, node_i_succ->num)
				+ data->distance(node_j->num, node_i->num)
				+ data->distance(node_i->num, node_j_succ->num)
				- data->distance(node_i_pred->num, node_i->num)
				- data->distance(node_i->num, node_i_succ->num)
				- data->distance(node_j->num, node_j_succ->num);
			e->push_back(tmp_length);
			double tmp_time = s.routeSet[r]->computeNewAT(node_j->num, node_j->d_t, node_i, data);
			tmp_time -= (*cus)[0]->e_t;
			e->push_back(s.allTime + tmp_time - (s.routeSet[r]->tail->a_t - s.routeSet[r]->head->d_t));

			neighbour.push_back(n);
			evaluation.push_back(*e);

			s.restoreOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);

		}
		else
		{
			//如果不可行
			cout << "进行OrExchange不可行..." << endl;
			s.restoreOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);
		}

		tmp = tmp->succ;
	}

	/*j与i不在同一条路径上*/
	for (int m = 0; m < s.routeSet.size(); ++m)
	{
		//不在i在的路径上
		//并且加上节点i之后，路径m不会超出容量
		if (m != r && s.routeSet[m]->demand + (*cus)[node_i->num - 1]->d <= DEMAND)
		{
			Node* tmp = s.routeSet[m]->head;
			while (tmp != s.routeSet[m]->tail)
			{
				node_j = tmp;
				node_j_succ = node_j->succ;

				cout << "------------------------------------------------------------" << endl;
				cout << "对客户" << node_i->num << "和" << node_j->num << "进行Relocation..." << endl;
				cout << "路径r：";
				s.routeSet[r]->printRoute();
				cout << "路径m：";
				s.routeSet[m]->printRoute();
				cout << "=====" << endl;

				s.doRelocation(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);

				cout << "路径r：";
				s.routeSet[r]->printRoute();
				cout << "路径m：";
				s.routeSet[m]->printRoute();

				//判断Relocation邻居是否可行
				if (s.routeSet[m]->isFeasibleGC(node_j->num, node_j->d_t, node_i, data))
				{
					cout << "可行..." << endl;
					//如果可行
					//记录该邻居信息
					n = new vector<double>();
					n->push_back(3);
					n->push_back(node_i->num);
					n->push_back(node_j->num);
					n->push_back(r);
					n->push_back(m);


					//计算这种情况下的开销
					e = new vector<double>();
					double s_size = s.routeSet.size();
					if (s.routeSet[r]->size == 1)
						--s_size;
					e->push_back(s_size);
					double tmp_length;
					double tmp_time_r, tmp_time_m;
					if (s.routeSet[r]->size == 1)
					{
						tmp_length = s.allLength
							+ data->distance(node_j->num, node_i->num)
							+ data->distance(node_i->num, node_j_succ->num)
							- data->distance(node_i_pred->num, node_i->num)
							- data->distance(node_i->num, node_i_succ->num)
							- data->distance(node_j->num, node_j_succ->num);
						tmp_time_r = 0;
					}
					else
					{
						tmp_length = s.allLength + data->distance(node_i_pred->num, node_i_succ->num)
							+ data->distance(node_j->num, node_i->num)
							+ data->distance(node_i->num, node_j_succ->num)
							- data->distance(node_i_pred->num, node_i->num)
							- data->distance(node_i->num, node_i_succ->num)
							- data->distance(node_j->num, node_j_succ->num);
						tmp_time_r = s.routeSet[r]->computeNewAT(node_i_pred->num, node_i_pred->d_t, node_i_succ, data);
						tmp_time_r -= (*cus)[0]->e_t;
					}
						
					tmp_time_m = s.routeSet[m]->computeNewAT(node_j->num, node_j->d_t, node_i, data);
					tmp_time_m -= (*cus)[0]->e_t;
					e->push_back(s.allTime + tmp_time_m + tmp_time_r - (s.routeSet[r]->tail->a_t - s.routeSet[r]->head->d_t) - (s.routeSet[m]->tail->a_t - s.routeSet[m]->head->d_t));

					neighbour.push_back(n);
					evaluation.push_back(*e);

					s.restoreRelocation(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);

				}
				else
				{
					cout << "进行Relocation不可行..." << endl;
					s.restoreRelocation(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);
				}

				tmp = tmp->succ;
			}
		}
	}
}

bool Solve::useOrOpt()
{
	
	for (int i = 0; i < s.routeSet.size(); )
	{
		Node* node_u = s.routeSet[i]->head->succ;
		while (node_u != s.routeSet[i]->tail)
		{
			OrExchangeNeighbour(node_u, i);
			computeOrder();

			double v_1 = s.routeSet.size();
			double v_2 = s.allLength;
			double v_3 = s.allTime;

			if (neighbour.size() == 0)
			{
				node_u = node_u->succ;
				continue;
			}

			if (evaluation[order[0]][0] < v_1 ||
				(evaluation[order[0]][0] == v_1 && evaluation[order[0]][1] < v_2) ||
				(evaluation[order[0]][0] == v_1 && evaluation[order[0]][1] == v_2 && evaluation[order[0]][2] < v_3))
			{
				changeToNeighbour(neighbour[order[0]], &s);

				cout << "---------------------------------------------------------" << endl;
				s.computeTime();
				s.computeLength(data);
				s.printSolution();
				s.isRegular(data);
				cout << "----------------------------------------------------------" << endl;

				return true;
			}
			else
			{
				node_u = node_u->succ;
				continue;
			}
			node_u = node_u->succ;
		}
		++i;
	}
	return false;
}

void Solve::localSearch()
{
	
	/*
	Route *route;

	//Build original solution
	cout << "------------------------初始解决方案------------------------" << endl;
	for (int i = 1; i < (*cus).size(); ++i)
	{
		route = new Route(i + 1, data);
		s.routeSet.push_back(route);
	}

	s.computeTime();
	s.computeLength(data);
	s.printSolution();
	s.isRegular(data);
	cout << "------------------------------------------------------------" << endl; 
	*/

	int i = 0;
	while (true)
	{
		if (i > 10)
			break;
		if (useOrOpt() == false)
			break;
		++i;
	}
}

/*------------------------------------------------------------------------------*/
/*-------------------------------Russell(1995)----------------------------------*/
/*-------------------------------------------------------------------------------*/
void Solve::printU()
{
	for (int i = 0; i < u.size(); ++i)
	{
		for (int j = 0; j < u[i].size(); ++j)
		{
			cout << u[i][j] << " ";
		}
		cout << endl;
	}
}

void Solve::computeU()
{
	//为了有序排列，初始化u，u[0]是编号为2的节点的U_i
	int size = (*cus).size();
	vector<double>* tmp;
	for (int i = 0; i < size - 1; ++i)
	{
		tmp = new vector<double>();
		//M1
		tmp->push_back(-1);
		tmp->push_back(-1);
		//M2
		tmp->push_back(-1);
		tmp->push_back(-1);
		u.push_back(*tmp);
	}

	//按路径计算u
	for (int i = 0; i < s.routeSet.size(); ++i)
	{
		Route* tmp = s.routeSet[i];
		Node* temp = tmp->head;
		double m = 1, n; //当前节点的前一个节点和后一个节点的编号
		double u_1, u_2;  //M1中的节点1和节点2
		double v_1 = -1, v_2 = -1;  //M2中的节点1和节点2
		while (temp)
		{
			//确定u_1和u_2
			u_1 = temp->num;
			if (temp->succ != NULL)
			{
				temp = temp->succ;
				u_2 = temp->num;
				if (temp->succ != NULL)
				{
					n = temp->succ->num;
				}
				else
				{
					n = 1;
				}
			}
			else
			{
				temp = temp->succ;
				//路径上只有一个节点
				u_2 = -1;
				n = 1;
			}

			//确定v_1和v_2
			double d_1 = -1, d_2 = -1;
			double dis_mv, dis_vn;
			for (int j = 0; j < s.routeSet.size(); ++j)
			{
				if (j != i)
				{
					Node* v = (s.routeSet[j])->head;
					for (; v; v = v->succ)
					{
						dis_mv = data->distance(v->num, m);
						dis_vn = data->distance(v->num, n);

						double temp_d = dis_mv + dis_vn;
						if (d_1 == -1 || temp_d <= d_1)
						{
							d_2 = d_1;
							v_2 = v_1;

							d_1 = temp_d;
							v_1 = v->num;
						}
						else if (d_2 == -1 || temp_d <= d_2)
						{
							d_2 = temp_d;
							v_2 = v->num;
						}
					}
				}
			}
			double num = u_1 - 2;
			u[num][0] = u_1;
			u[num][1] = u_2;
			u[num][2] = v_1;
			u[num][3] = v_2;
		}
	}
}

void Solve::localSearch1()
{
	//使用z来衡量是否新的解比原始解更优
	double z = V1 * s.allLength + V2 * s.allTime;
	//路径的条数
	int v = s.routeSet.size();

	//计算子集U_i
	computeU();
	/*打印子集U_i
	printU();*/

	int i = 0;

}


/*------------------------------------------------------------------------------*/
/*-------------------------------Russell(2004)----------------------------------*/
/*--------------------------------模拟退火算法-----------------------------------*/
/*-------------------------------------------------------------------------------*/

double Solve::computeLatestArriveTime(Node* node_j_succ)
{
	double t;
	double j = node_j_succ->num;

	if(node_j_succ->succ == NULL)
	{
		t = (*cus)[0]->l_t;
		return t;
	}
	else
	{
		double j_succ = node_j_succ->succ->num;
		double d = data->distance(j, j_succ);

		t = computeLatestArriveTime(node_j_succ) - d - (*cus)[j - 1]->s_t;
	}
	return (*cus)[j - 1]->l_t < t ? (*cus)[j - 1]->l_t : t;
}

void Solve::computeOrder()
{
	for (int i = 0; i < evaluation.size(); ++i)
	{
		order.push_back(i);
		for (int j = i - 1; j >= 0; --j)
		{
			if ((evaluation[order[j + 1]][0] < evaluation[order[j]][0]) ||
				(evaluation[order[j + 1]][0] == evaluation[order[j]][0] && evaluation[order[j + 1]][1] < evaluation[order[j]][1]) ||
				(evaluation[order[j + 1]][0] == evaluation[order[j]][0] && evaluation[order[j + 1]][1] < evaluation[order[j]][1] && evaluation[order[j + 1]][2] >= evaluation[order[j]][2]))
			{
				int temp = order[j + 1];
				order[j + 1] = order[j];
				order[j] = temp;
			}
			else
			{
				break;
			}
		}
	}
}

void Solve::generateNeighbour(int operate, double num, Solution* n_s)
{
	int o = operate;

	if (o == 1)  //进行2-exchange  在一条路径上
	{
		generate2Exchange(num, n_s);
	}
	else if (o == 2)  //进行Or-exchange  在一条路径上
	{
		generateOrExchange(num, n_s);
	}
	else if (o == 3) //进行Relocation  在两条路径间
	{
		generateRelocation(num, n_s);
	}
	else if (o == 4) //进行Exchange  在两条路径间
	{
		generateExchange(num, n_s);
	}
	else if (o == 5) //进行Crossover  在两条路径间
	{
		generateCrossover(num, n_s);
	}
}

void Solve::generate2Exchange(double num, Solution* n_s)
{
	double i = num;
	Node* node_i;
	int r; //客户c所在的路径
	n_s->search(i, &r, &node_i);
	Node* node_i_succ = node_i->succ;
	double i_succ = node_i_succ->num;  //当前客户节点i的后一个节点的编号

	//将边i->i_succ,j->j_succ换成i->j,i_succ->j_succ
	vector<double>* n;
	vector<double>* e;

	//当j在i后面时
	Node* node_j;
	Node* node_j_succ;

	Node* tmp = node_i_succ;  //i后面的节点i+1
	if (tmp != n_s->routeSet[r]->tail) //这个节点不为仓库节点时
	{
		tmp = tmp->succ; //节点i+2
		while (tmp != n_s->routeSet[r]->tail)
		{
			node_j = tmp;
			node_j_succ = node_j->succ;
			
			cout << "------------------------------------------------------------" << endl;
			cout << "对客户" << node_i->num << "和" << node_j->num << "进行2Exchange..." << endl;
			cout << "路径r：";
			n_s->routeSet[r]->printRoute();
			cout << "=====" << endl;

			n_s->do2Exchange(n_s->routeSet[r], node_i,  node_i_succ, node_j, node_j_succ);

			cout << "路径r：";
			n_s->routeSet[r]->printRoute();

			//判断进行2Exchange之后得到的邻居可不可行
			if (n_s->routeSet[r]->isFeasibleGC(node_i->num, node_i->d_t, node_j, data))
			{
				//如果可行
				//记录该邻居信息
				n = new vector<double>();
				n->push_back(1);
				n->push_back(node_i->num);
				n->push_back(node_j->num);
				n->push_back(r);

				n_s->restore2Exchange(n_s->routeSet[r], node_i, node_i_succ, node_j, node_j_succ);

				//计算这种情况下的开销
				e = new vector<double>();
				computeEvaluation(&e, n, n_s);

				/*
				e->push_back(computeFirstEvaluation(n, n_s));
				e->push_back(computeSecondEvaluation(n, n_s));
				e->push_back(computeThirdEvaluation(n, n_s));*/

				neighbour.push_back(n);
				evaluation.push_back(*e);
			}
			else
			{
				//如果不可行
				cout << "进行的2Exchange不可行..." << endl;
				n_s->restore2Exchange(n_s->routeSet[r], node_i, node_i_succ, node_j, node_j_succ);

			}

			tmp = tmp->succ;
		}
	}

	//当j在i前面时
	tmp = n_s->routeSet[r]->head;
	if (tmp != node_i->pred)
	{
		while (tmp->succ != node_i->pred)
		{
			node_j = tmp;
			node_j_succ = node_j->succ;

			cout << "------------------------------------------------------------" << endl;
			cout << "对客户" << node_i->num << "和" << node_j->num << "进行2Exchange..." << endl;
			cout << "路径r：";
			n_s->routeSet[r]->printRoute();
			cout << "=====" << endl;

			n_s->do2Exchange(n_s->routeSet[r], node_j, node_j_succ, node_i, node_i_succ);

			cout << "路径r：";
			n_s->routeSet[r]->printRoute();

			//判断进行2Exchange之后得到的邻居可不可行
			if (n_s->routeSet[r]->isFeasibleGC(node_j->num, node_j->d_t, node_i, data))
			{
				//如果可行
				//记录该邻居信息
				n = new vector<double>();
				n->push_back(1);
				n->push_back(node_j->num);
				n->push_back(node_i->num);
				n->push_back(r);

				n_s->restore2Exchange(n_s->routeSet[r], node_j, node_j_succ, node_i, node_i_succ);

				//计算这种情况下的开销
				e = new vector<double>();
				computeEvaluation(&e, n, n_s);

				/*
				e->push_back(computeFirstEvaluation(n, n_s));
				e->push_back(computeSecondEvaluation(n, n_s));
				e->push_back(computeThirdEvaluation(n, n_s));*/

				neighbour.push_back(n);
				evaluation.push_back(*e);
			}
			else
			{
				cout << "进行的2Exchange不可行..." << endl;
				//如果不可行
				n_s->restore2Exchange(n_s->routeSet[r], node_j, node_j_succ, node_i, node_i_succ);
			}

			tmp = tmp->succ;
		}
	}
}

void Solve::generateOrExchange(double num, Solution* n_s)
{
	double i = num;
	Node* node_i;
	int r; //客户c所在的路径
	n_s->search(i, &r, &node_i);
	Node* node_i_succ = node_i->succ;
	Node* node_i_pred = node_i->pred;

	//将i_pred->i,i->i_succ,j->j_succ换成i_pred->i_succ,j->i,i->j_succ
	//i固定后，i_pred和i_succ都是固定的；改变的是j
	//j的变化,只要j不等于i_pred,i都可以
	//j可以是从仓库到回到仓库前的最后一个节点
	vector<double>* n;
	vector<double>* e;
	Node* node_j;
	Node* node_j_succ;

	//j在i前面
	Node* tmp = n_s->routeSet[r]->head;
	while (tmp != node_i_pred)
	{
		
		node_j = tmp;
		node_j_succ = tmp->succ;

		cout << "------------------------------------------------------------" << endl;
		cout << "对客户" << node_i->num << "和" << node_j->num << "进行OrExchange..." << endl;
		cout << "路径r：";
		n_s->routeSet[r]->printRoute();
		cout << "=====" << endl;
		
		n_s->doOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);

		cout << "路径r：";
		n_s->routeSet[r]->printRoute();

		double at_i, wt_i, dt_i;
		at_i = node_j->d_t + data->distance(node_j->num, node_i->num);
		wt_i = at_i > (*cus)[node_i->num - 1]->e_t ? 0 : (*cus)[node_i->num - 1]->e_t - at_i;
		dt_i = at_i + wt_i + (*cus)[node_i->num - 1]->s_t;

		//判断OrExchange邻居是否可行
		if (n_s->routeSet[r]->isFeasible(node_i->num, dt_i, node_j_succ, data))
		{
			//如果可行
			//记录该邻居信息
			n = new vector<double>();
			n->push_back(2);
			n->push_back(node_i->num);
			n->push_back(node_j->num);
			n->push_back(r);
			n->push_back(1);  //为1表示j在i前面

			n_s->restoreOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);

			//计算这种情况下的开销
			e = new vector<double>();
			computeEvaluation(&e, n, n_s);

			/*
			e->push_back(computeFirstEvaluation(n, n_s));
			e->push_back(computeSecondEvaluation(n, n_s));
			e->push_back(computeThirdEvaluation(n, n_s));*/

			neighbour.push_back(n);
			evaluation.push_back(*e);

		}
		else
		{
			cout << "进行OrExchange不可行..." << endl;
			//如果不可行
			n_s->restoreOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);
		}

		tmp = tmp->succ;
	}

	//j在i后面
	tmp = node_i_succ;
	while (tmp != n_s->routeSet[r]->tail)
	{
		node_j = tmp;
		node_j_succ = node_j->succ;

		cout << "------------------------------------------------------------" << endl;
		cout << "对客户" << node_i->num << "和" << node_j->num << "进行OrExchange..." << endl;
		cout << "路径r：";
		n_s->routeSet[r]->printRoute();
		cout << "=====" << endl;

		n_s->doOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);

		cout << "路径r：";
		n_s->routeSet[r]->printRoute();

		double at_i, wt_i, dt_i;
		at_i = node_j->d_t + data->distance(node_j->num, node_i->num);
		//如果i和j交换，i都不可行，那么j后面的都不用尝试了
		if (at_i > (*cus)[node_i->num - 1]->l_t)
		{
			//还原交换
			n_s->restoreOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);
			break;
		}
		else
		{
			wt_i = at_i > (*cus)[node_i->num - 1]->e_t ? 0 : (*cus)[node_i->num - 1]->e_t - at_i;
			dt_i = at_i + wt_i + (*cus)[node_i->num - 1]->s_t;

			//判断OrExchange邻居是否可行
			if (n_s->routeSet[r]->isFeasible(node_i->num, dt_i, node_j_succ, data))
			{
				//如果可行
				//记录该邻居信息
				n = new vector<double>();
				n->push_back(2);
				n->push_back(node_i->num);
				n->push_back(node_j->num);
				n->push_back(r);
				n->push_back(0);     //为0表示j在i后面

				n_s->restoreOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);

				//计算这种情况下的开销
				e = new vector<double>();
				computeEvaluation(&e, n, n_s);

				/*
				e->push_back(computeFirstEvaluation(n, n_s));
				e->push_back(computeSecondEvaluation(n, n_s));
				e->push_back(computeThirdEvaluation(n, n_s));*/

				neighbour.push_back(n);
				evaluation.push_back(*e);

			}
			else
			{
				//如果不可行
				cout << "进行OrExchange不可行..." << endl;
				n_s->restoreOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);
			}
		}
		tmp = tmp->succ;
	}
}

void Solve::generateRelocation(double num, Solution* n_s)
{
	double i = num;
	Node* node_i;
	int r; //客户c所在的路径
	n_s->search(i, &r, &node_i);
	Node* node_i_succ = node_i->succ;
	Node* node_i_pred = node_i->pred;

	//将i_pred->i,i->i_succ,j->j_succ换成i_pred->i_succ,j->i,i->j_succ
	//i固定后，i_pred和i_succ都是固定的；改变的是j
	//j可以是其它路径上的节点，从仓库到仓库前的最后一个节点
	vector<double>* n;
	vector<double>* e;
	Node* node_j;
	Node* node_j_succ;
	for (int m = 0; m < n_s->routeSet.size(); ++m)
	{
		//不在i在的路径上
		//并且加上节点i之后，路径m不会超出容量
		if (m != r && n_s->routeSet[m]->demand + (*cus)[node_i->num - 1]->d <= DEMAND)
		{
			Node* tmp = n_s->routeSet[m]->head;
			while (tmp != n_s->routeSet[m]->tail)
			{
				node_j = tmp;
				node_j_succ = node_j->succ;

				cout << "------------------------------------------------------------" << endl;
				cout << "对客户" << node_i->num << "和" << node_j->num << "进行Relocation..." << endl;
				cout << "路径r：";
				n_s->routeSet[r]->printRoute();
				cout << "路径m：";
				n_s->routeSet[m]->printRoute();
				cout << "=====" << endl;

				n_s->doRelocation(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);

				cout << "路径r：";
				n_s->routeSet[r]->printRoute();
				cout << "路径m：";
				n_s->routeSet[m]->printRoute();


				double at_i, wt_i, dt_i;
				at_i = node_j->d_t + data->distance(node_j->num, node_i->num);
				//如果i和j交换，i都不可行，那么j后面的都不用尝试了
				if (at_i > (*cus)[node_i->num - 1]->l_t)
				{
					//还原交换
					n_s->restoreRelocation(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);
					break;
				}
				else
				{
					wt_i = at_i > (*cus)[node_i->num - 1]->e_t ? 0 : (*cus)[node_i->num - 1]->e_t - at_i;
					dt_i = at_i + wt_i + (*cus)[node_i->num - 1]->s_t;

					//判断Relocation邻居是否可行
					if (n_s->routeSet[m]->isFeasible(node_i->num, dt_i, node_j_succ, data))
					{
						cout << "可行..." << endl;
						//如果可行
						//记录该邻居信息
						n = new vector<double>();
						n->push_back(3);
						n->push_back(node_i->num);
						n->push_back(node_j->num);
						n->push_back(r);
						n->push_back(m);

						n_s->restoreRelocation(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);

						//计算这种情况下的开销
						e = new vector<double>();
						computeEvaluation(&e, n, n_s);

						/*
						e->push_back(computeFirstEvaluation(n, n_s));
						e->push_back(computeSecondEvaluation(n, n_s));
						e->push_back(computeThirdEvaluation(n, n_s));*/

						neighbour.push_back(n);
						evaluation.push_back(*e);

					}
					else
					{
						cout << "进行Relocation不可行..." << endl;
						n_s->restoreRelocation(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);
					}
				}
				tmp = tmp->succ;
			}
		}
	}
	cout << "here" << endl;
}

void Solve::generateExchange(double num, Solution* n_s)
{
	double i = num;
	Node* node_i;
	int r; //客户c所在的路径
	n_s->search(i, &r, &node_i);
	Node* node_i_succ = node_i->succ;
	Node* node_i_pred = node_i->pred;

	//将i_pred->i,i->i_succ,j->pred->j,j->j_succ换成i_pred->j,j->i_succ,j_pred->i,i->j_succ
	//i固定后，i_pred和i_succ都是固定的；改变的是j
	//j可以是其它路径上的节点，从仓库后第一个节点到仓库前的最后一个节点
	vector<double>* n;
	vector<double>* e;
	Node* node_j_pred;
	Node* node_j;
	Node* node_j_succ;
	for (int m = 0; m < n_s->routeSet.size(); ++m)
	{
		//不在i在的路径上
		if (m != r) 
		{
			Node* tmp = n_s->routeSet[m]->head->succ;
			while (tmp != n_s->routeSet[m]->tail)
			{
				node_j_pred = tmp->pred;
				node_j = tmp;
				node_j_succ = node_j->succ;

				//并且路径上的总需求不会超过车辆的容量
				if (n_s->routeSet[m]->demand + (*cus)[node_i->num - 1]->d - (*cus)[node_j->num - 1]->d <= DEMAND &&
					n_s->routeSet[r]->demand + (*cus)[node_j->num - 1]->d - (*cus)[node_i->num - 1]->d <= DEMAND)
				{

					
					cout << "------------------------------------------------------------" << endl;
					cout << "对客户" << node_i->num << "和" << node_j->num << "进行Exchange..." << endl;
					cout << "路径r：";
					n_s->routeSet[r]->printRoute();
					cout << "路径m：";
					n_s->routeSet[m]->printRoute();
					cout << "=====" << endl;

					n_s->doExchange(node_i_pred, node_i, node_i_succ, node_j_pred, node_j, node_j_succ);

					cout << "路径r：";
					n_s->routeSet[r]->printRoute();
					cout << "路径m：";
					n_s->routeSet[m]->printRoute();

					double at_i, wt_i, dt_i;
					double at_j, wt_j, dt_j;
					at_i = node_j_pred->d_t + data->distance(node_j_pred->num, node_i->num);
					//如果i和j交换，i都不可行，那么j后面的都不用尝试了
					if (at_i > (*cus)[node_i->num - 1]->l_t)
					{
						//还原交换
						n_s->restoreExchange(node_i_pred, node_i, node_i_succ, node_j_pred, node_j, node_j_succ);
						break;
					}
					else
					{
						at_j = node_i_pred->d_t + data->distance(node_i_pred->num, node_j->num);
						//如果i可行，j不可行
						if (at_j > (*cus)[node_j->num - 1]->l_t)
						{
							//还原交换
							n_s->restoreExchange(node_i_pred, node_i, node_i_succ, node_j_pred, node_j, node_j_succ);
							tmp = tmp->succ;
							continue;
						}
						//如果i和j都可行
						else
						{
							wt_i = at_i > (*cus)[node_i->num - 1]->e_t ? 0 : (*cus)[node_i->num - 1]->e_t - at_i;
							dt_i = at_i + wt_i + (*cus)[node_i->num - 1]->s_t;

							wt_j = at_j > (*cus)[node_j->num - 1]->e_t ? 0 : (*cus)[node_j->num - 1]->e_t - at_j;
							dt_j = at_j + wt_j + (*cus)[node_j->num - 1]->s_t;

							//判断Exchange邻居是否可行
							if (n_s->routeSet[m]->isFeasible(node_i->num, dt_i, node_j_succ, data) &&
								n_s->routeSet[r]->isFeasible(node_j->num, dt_j, node_i_succ, data))
							{

								//如果可行
								//记录该邻居信息
								n = new vector<double>();
								n->push_back(4);
								n->push_back(node_i->num);
								n->push_back(node_j->num);
								n->push_back(r);
								n->push_back(m);

								n_s->restoreExchange(node_i_pred, node_i, node_i_succ, node_j_pred, node_j, node_j_succ);

								//计算这种情况下的开销
								e = new vector<double>();
								computeEvaluation(&e, n, n_s);

								/*
								e->push_back(computeFirstEvaluation(n, n_s));
								e->push_back(computeSecondEvaluation(n, n_s));
								e->push_back(computeThirdEvaluation(n, n_s));*/

								neighbour.push_back(n);
								evaluation.push_back(*e);

							}
							else
							{
								cout << "进行Exchange不可行..." << endl;
								n_s->restoreExchange(node_i_pred, node_i, node_i_succ, node_j_pred, node_j, node_j_succ);
							}
						}
					}	
				}
				tmp = tmp->succ;
			}
		}
	}
}

void Solve::generateCrossover(double num, Solution* n_s)
{
	double i = num;
	Node* node_i;
	int r; //客户c所在的路径
	n_s->search(i, &r, &node_i);
	Node* node_i_succ = node_i->succ;

	double r_1_demand, r_2_demand = 0;
	for (Node* t = node_i_succ; t != n_s->routeSet[r]->tail; t = t->succ)
		r_2_demand += (*cus)[t->num - 1]->d;
	r_1_demand = n_s->routeSet[r]->demand - r_2_demand;

	double m_1_demand, m_2_demand = 0;

	//将i->i_succ,j->j_succ换成j->i_succ,i->j_succ
	//i固定后，i_succ都是固定的；改变的是j
	vector<double>* n;
	vector<double>* e;
	Node* node_j;
	Node* node_j_succ;
	for (int m = 0; m < n_s->routeSet.size(); ++m)
	{
		//不在i在的路径上
		if (m != r)
		{
			Node* tmp = n_s->routeSet[m]->head;
			m_2_demand = n_s->routeSet[m]->demand;
			while (tmp != n_s->routeSet[m]->tail)
			{
				node_j = tmp;
				node_j_succ = node_j->succ;

				//如果两个节点都分别是彼此路径的最后一个客户节点，那么不必尝试crossover，因为并没有任何改变
				if (node_i->succ == n_s->routeSet[r]->tail && node_j->succ == n_s->routeSet[m]->tail)
				{
					tmp = tmp->succ;
					continue;
				}

				m_2_demand -= (*cus)[node_j->num - 1]->d;
				m_1_demand = n_s->routeSet[m]->demand - m_2_demand;

				//不超过容量限制，才考虑其它
				if (r_1_demand + m_2_demand < DEMAND && m_1_demand + r_2_demand < DEMAND)
				{

					cout << "------------------------------------------------------------" << endl;
					cout << "对" << node_i->num << "和" << node_j->num << "做crossover..." << endl;
					cout << "路径r：";
					n_s->routeSet[r]->printRoute();
					cout << "路径m：";
					n_s->routeSet[m]->printRoute();
					cout << "=====" << endl;

					n_s->doCrossover(node_i, node_i_succ, node_j, node_j_succ);
					Node* ptail = n_s->routeSet[r]->tail;
					n_s->routeSet[r]->tail = n_s->routeSet[m]->tail;
					n_s->routeSet[m]->tail = ptail;

					cout << "路径r：";
					n_s->routeSet[r]->printRoute();
					cout << "路径m：";
					n_s->routeSet[m]->printRoute();

					//判断Crossover邻居是否可行
					if (n_s->routeSet[m]->isFeasible(node_j->num, node_j->d_t, node_i_succ, data) &&
						n_s->routeSet[r]->isFeasible(node_i->num, node_i->d_t, node_j_succ, data))
					{	
						//记录该邻居信息
						n = new vector<double>();
						n->push_back(5);
						n->push_back(node_i->num);
						n->push_back(node_j->num);
						n->push_back(r);
						n->push_back(m);

						cout << "可行,记录邻居信息..." << endl;
						n_s->restoreCrossover(node_i, node_i_succ, node_j, node_j_succ);
						ptail = n_s->routeSet[r]->tail;
						n_s->routeSet[r]->tail = n_s->routeSet[m]->tail;
						n_s->routeSet[m]->tail = ptail;

						

						//计算这种情况下的开销
						e = new vector<double>();
						computeEvaluation(&e, n, n_s);

						/*
						e->push_back(computeFirstEvaluation(n, n_s));
						e->push_back(computeSecondEvaluation(n, n_s));
						e->push_back(computeThirdEvaluation(n, n_s));*/

						neighbour.push_back(n);
						evaluation.push_back(*e);

					}
					else
					{
						cout << "此种操作不可行..." << endl;
						n_s->restoreCrossover(node_i, node_i_succ, node_j, node_j_succ);
						ptail = n_s->routeSet[r]->tail;
						n_s->routeSet[r]->tail = n_s->routeSet[m]->tail;
						n_s->routeSet[m]->tail = ptail;
					}

				}

				tmp = tmp->succ;
			}
		}
	}
}

double Solve::computeFirstEvaluation(vector<double>* n, Solution* n_s)
{
	double x;
	x = n_s->routeSet.size();

	//两种可能减少路径数目的操作
	if ((*n)[0] == 3) //relocation邻居
	{
		double r1;
		r1 = (*n)[3];
		if (n_s->routeSet[r1]->size == 1)
			x--;
	}
	else if ((*n)[0] == 5) //crossover邻居
	{
		double i = (*n)[1];
		double j = (*n)[2];
		double r1 = (*n)[3];
		double r2 = (*n)[4];
		Node* node_i = n_s->routeSet[r1]->searchNode(i);
		Node* node_j = n_s->routeSet[r2]->searchNode(j);
		if (node_i->succ == n_s->routeSet[r1]->tail && node_j == n_s->routeSet[r2]->head)
			x--;
	}

	return x;
}

double Solve::computeSecondEvaluation(vector<double>* n, Solution* n_s)
{
	double r1, r2;
	double sum = 0;
	if ((*n)[0] == 1 || (*n)[0] == 2 || (*n)[0] == 4)
	{
		//每条路径上的节点数目并没有发生改变	
		for (int m = 0; m < n_s->routeSet.size(); ++m)
		{
			sum += pow(n_s->routeSet[m]->size, 2);
		}
	}
	else if ((*n)[0] == 3) //relocation邻居
	{
		r1 = (*n)[3];
		r2 = (*n)[4];

		for (int n = 0; n < n_s->routeSet.size(); ++n)
		{
			if (n == r1)
				sum += pow(n_s->routeSet[n]->size - 1, 2);
			else if (n == r2)
				sum += pow(n_s->routeSet[n]->size + 1, 2);
			else
				sum += pow(n_s->routeSet[n]->size, 2);
		}
	}
	else if ((*n)[0] == 5) //crossover邻居
	{
		double i = (*n)[1];
		double j = (*n)[2];

		r1 = (*n)[3];
		r2 = (*n)[4];

		double r1_1, r1_2 = 0;
		double r2_1, r2_2 = 0;

		Node* node_i = n_s->routeSet[r1]->searchNode(i);
		Node* node_j;
		if (j == 1)
			node_j = n_s->routeSet[r2]->head;
		else
			node_j = n_s->routeSet[r2]->searchNode(j);

		for (Node* tmp = node_i->succ; tmp != n_s->routeSet[r1]->tail; tmp = tmp->succ)
		{
			r1_2++;
		}
		r1_1 = n_s->routeSet[r1]->size - r1_2;

		for (Node* tmp = node_j->succ; tmp != n_s->routeSet[r2]->tail; tmp = tmp->succ)
		{
			r2_2++;
		}
		r2_1 = n_s->routeSet[r2]->size - r2_2;

		for (int n = 0; n < n_s->routeSet.size(); ++n)
		{
			if (n == r1)
				sum += pow(r1_1 + r2_2, 2);
			else if (n == r2)
				sum += pow(r2_1 + r1_2, 2);
			else
				sum += pow(n_s->routeSet[n]->size, 2);
		}
	}

	return -sum;
}

double Solve::computeMinimalDelay(Solution* n)
{
	double num = 0;

	//r表示节点数最少的路径，首先找到路径r
	int r = 0;
	for (int i = 1; i < n->routeSet.size(); ++i)
	{
		if (n->routeSet[i]->size < n->routeSet[r]->size)
		{
			r = i;
		}
	}

	double t_num;
	Node* node_u = n->routeSet[r]->head->succ;
	while (node_u != n->routeSet[r]->tail)
	{
		t_num = -1;

		Node* node_u_pred = node_u->pred;
		Node* node_u_succ = node_u->succ;

		double u_num = node_u->num;
		for (int i = 0; i < n->routeSet.size(); ++i)
		{

			//对于不是当前路径的路径
			if (i != r)
			{
				//无法加入当前路径
				if ((*cus)[u_num - 1]->d + n->routeSet[i]->demand > DEMAND)
				{
					continue;
				}
				else
				{
					//尝试将节点node_u在路径routeSet[i]上重定位
					Node* node_j = n->routeSet[i]->head;
					Node* node_j_succ;
					while (node_j != n->routeSet[i]->tail)
					{

						node_j_succ = node_j->succ;

						//首先判断在node_j后面插入会不会造成node_u以及node_u后面的节点不可行
						n->doRelocation(node_u_pred, node_u, node_u_succ, node_j, node_j_succ);
						if (n->routeSet[i]->isFeasible(node_j->num, node_j->d_t, node_u, data))
						{
							//如果插入位置可行
							t_num = 0;
							n->restoreRelocation(node_u_pred, node_u, node_u_succ, node_j, node_j_succ);
							break;
						}
						else
						{
							//如果插入位置不可行
							double x = node_j->d_t + data->distance(node_j->num, node_u->num) - (*cus)[node_u->num - 1]->l_t;
							double n1 = x > 0 ? x : 0;

							x = node_u->d_t + data->distance(node_j_succ->num, node_u->num) - computeLatestArriveTime(node_j_succ);
							double n2 = x > 0 ? x : 0;

							if (t_num == -1 || n1 + n2 < t_num)
								t_num = n1 + n2;

							n->restoreRelocation(node_u_pred, node_u, node_u_succ, node_j, node_j_succ);
						}

						node_j = node_j->succ;
					}
				}
			}
			if (t_num == 0)
				break;
		}

		if (t_num = -1)
			num = -1;
		else
			num += t_num;

		node_u = node_u->succ;
	}

	return num;
}

double Solve::computeThirdEvaluation(vector<double>* n, Solution* n_s)
{
	Solution* temp_s = new Solution();
	temp_s->doCopy(n_s);

	changeToNeighbour(n, temp_s);
	double x = computeMinimalDelay(temp_s);

	delete temp_s;
	
	return x;
}

void Solve::computeEvaluation(vector<double>** e, vector<double>* n, Solution* n_s)
{
	Solution* temp_s = new Solution();
	temp_s->doCopy(n_s);

	changeToNeighbour(n, temp_s);
	(*e)->push_back(temp_s->routeSet.size());
	(*e)->push_back(temp_s->allLength);
	(*e)->push_back(temp_s->allTime);

	delete temp_s;
}

void Solve::changeToNeighbour(vector<double>* n, Solution* temp_s)
{
	if ((*n)[0] == 0)
		return;

	double r, m;
	double i, j;
	double x;
	Node* node_i, *node_j;
	Node* node_i_pred, *node_i_succ;
	Node* node_j_pred, *node_j_succ;

	i = (*n)[1];
	j = (*n)[2];
	r = (*n)[3];

	if ((*n)[0] == 1) //2-exchange邻居
	{
		if(i == 1)
			node_i = temp_s->routeSet[r]->head;
		else
			node_i = temp_s->routeSet[r]->searchNode(i);
		node_i_succ = node_i->succ;

		node_j = temp_s->routeSet[r]->searchNode(j);
		node_j_succ = node_j->succ;

		temp_s->do2Exchange(temp_s->routeSet[r], node_i, node_i_succ, node_j, node_j_succ);

		temp_s->routeSet[r]->updateRoute(node_j, data);
	}
	else if ((*n)[0] == 2) //or-exchange邻居
	{
		double flag = (*n)[4];

		node_i = temp_s->routeSet[r]->searchNode(i);
		node_i_succ = node_i->succ;

		if (j == 1)
			node_j = temp_s->routeSet[r]->head;
		else
			node_j = temp_s->routeSet[r]->searchNode(j);
		node_j_succ = node_j->succ;

		node_i_pred = node_i->pred;

		temp_s->doOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);

		if(flag == 1)   //j在i前面
			temp_s->routeSet[r]->updateRoute(node_i, data);
		else if(flag == 0)  //j在i后面
			temp_s->routeSet[r]->updateRoute(node_i_succ, data);

	}
	else if ((*n)[0] == 3) //relocation邻居
	{
		m = (*n)[4];

		/*
		cout << "路径：";
		temp_s->routeSet[r]->printRoute();
		cout << "寻找：" << i << endl;
		cout << "另一条路径：";
		temp_s->routeSet[m]->printRoute();
		cout << "寻找：" << j << endl;
		*/

		node_i = temp_s->routeSet[r]->searchNode(i);
		node_i_succ = node_i->succ;

 		
		if (j == 1)
			node_j = temp_s->routeSet[m]->head;
		else
			node_j = temp_s->routeSet[m]->searchNode(j);
		node_j_succ = node_j->succ;

		node_i_pred = node_i->pred;

		temp_s->doRelocation(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);
	
		temp_s->routeSet[r]->size--;
		temp_s->routeSet[m]->size++;

		temp_s->routeSet[r]->demand -= (*cus)[i - 1]->d;
		temp_s->routeSet[m]->demand += (*cus)[i - 1]->d;

		temp_s->routeSet[m]->updateRoute(node_i, data);
		//r1可能会变空
		if (temp_s->routeSet[r]->size == 0)
		{
			temp_s->routeSet.erase(temp_s->routeSet.begin() + r);
		}
		else
			temp_s->routeSet[r]->updateRoute(node_i_succ, data);
	}
	else if ((*n)[0] == 4)  //exchange邻居
	{
		m = (*n)[4];

		node_i = temp_s->routeSet[r]->searchNode(i);
		node_i_succ = node_i->succ;

		node_j = temp_s->routeSet[m]->searchNode(j);
		node_j_succ = node_j->succ;

		node_i_pred = node_i->pred;
		node_j_pred = node_j->pred;

		temp_s->doExchange(node_i_pred, node_i, node_i_succ, node_j_pred, node_j, node_j_succ);

		temp_s->routeSet[r]->updateRoute(node_j, data);
		temp_s->routeSet[m]->updateRoute(node_i, data);

		temp_s->routeSet[r]->demand += (*cus)[j - 1]->d - (*cus)[i - 1]->d;
		temp_s->routeSet[m]->demand += (*cus)[i - 1]->d - (*cus)[j - 1]->d;

	}
	else if ((*n)[0] == 5) //crossover邻居
	{
		m = (*n)[4];	

		node_i = temp_s->routeSet[r]->searchNode(i);
		node_i_succ = node_i->succ;

		if (j == 1)
			node_j = temp_s->routeSet[m]->head;
		else
			node_j = temp_s->routeSet[m]->searchNode(j);
		node_j_succ = node_j->succ;

		double r_1_demand, r_2_demand = 0;
		double r_1, r_2 = 0;
		for (Node* t = node_i_succ; t != temp_s->routeSet[r]->tail; t = t->succ)
		{
			++r_2;
			r_2_demand += (*cus)[t->num - 1]->d;
		}
		r_1 = temp_s->routeSet[r]->size - r_2;
		r_1_demand = temp_s->routeSet[r]->demand - r_2_demand;

		double m_1_demand, m_2_demand = 0;
		double m_1, m_2 = 0;

		for (Node* t = node_j_succ; t != temp_s->routeSet[m]->tail; t = t->succ)
		{
			++m_2;
			m_2_demand += (*cus)[t->num - 1]->d;
		}
		m_1 = temp_s->routeSet[m]->size - m_2;
		m_1_demand = temp_s->routeSet[m]->demand - m_2_demand;

		temp_s->doCrossover(node_i, node_i_succ, node_j, node_j_succ);
		Node* ptail = temp_s->routeSet[r]->tail;
		temp_s->routeSet[r]->tail = temp_s->routeSet[m]->tail;
		temp_s->routeSet[m]->tail = ptail;

		//更新路径的容量
		temp_s->routeSet[r]->demand = r_1_demand + m_2_demand;
		temp_s->routeSet[m]->demand = m_1_demand + r_2_demand;
		//更新路径的节点数目
		temp_s->routeSet[r]->size = r_1 + m_2;
		temp_s->routeSet[m]->size = m_1 + r_2;

		//路径m可能变空，路径r不会出现变空的情况
		temp_s->routeSet[r]->updateRoute(node_j_succ, data);
		if (temp_s->routeSet[m]->size != 0)
			temp_s->routeSet[m]->updateRoute(node_i_succ, data);
		else
		{
			//将m这条路径删掉
			delete temp_s->routeSet[m];
			(temp_s->routeSet).erase(temp_s->routeSet.begin() + m);
		}
	}
	temp_s->computeLength(data);
	temp_s->computeTime();
}

void Solve::minimizeRoute()
{
	int maxIteration = 20;
	/*
	Route *route;

	Build original solution
	cout << "------------------------初始解决方案------------------------" << endl;
	for (int i = 1; i < (*cus).size(); ++i)
	{
		route = new Route(i + 1, data);
		s.routeSet.push_back(route);
	}
	s.printSolution();
	cout << "------------------------------------------------------------" << endl; */

	
	double t;
	int o;  //操作：1.Two-exchange 2.Or-exchange 3.Relocation 4.Exchange 5.Crossover
	double c; //随机产生的一个客户编号

	//第一个数为0时表示解决方案tmp_s等于当前解决方案，不做任何变化
	//1：做一次2-exchange 2：做一次or-exchange 3：做一次relocation 4：做一次exchange 5：做一次crossover
	Solution* n_s = new Solution();
	n_s->doCopy(&s);
	vector<double>* flag = new vector<double>();
	flag->push_back(0);
	
	/*
	n_s->routeSet[1]->mergeRoute(n_s->routeSet[2], data);
	cout << "-----------------n_s--------------" << endl;
	n_s->printSolution();

	cout << "-------------------s--------------" << endl;
	s.printSolution();
	*/

	//获得当前时间
	time_t iTime = time(0);
	time_t nTime = time(0);
	while (nTime - iTime < 10)
	{
		t = T;
		while (nTime - iTime < 10 && t > EPS)
		{
			for (int i = 1; i < maxIteration; i++)
			{
				o = (rand() % 5) + 1;
				c = (double)(rand() % 99) + 2;

				cout << "*******************************第" << i << "次操作**********************" << endl;
				cout << "对客户" << c << "进行" << o << "操作" << endl;
				//产生n_s的邻居并计算邻居的评价值
				generateNeighbour(o, c, n_s);
				if (neighbour.size() == 0)
				{
					if(o == 1)
						cout << "对客户" << c << "没有可行的" << "2-exchange操作" << endl;
					else if(o == 2)
						cout << "对客户" << c << "没有可行的" << "or-exchange操作" << endl;
					else if(o == 3)
						cout << "对客户" << c << "没有可行的" << "relocation操作" << endl;
					else if(o == 4)
						cout << "对客户" << c << "没有可行的" << "exchange操作" << endl;
					else if(o == 5)
						cout << "对客户" << c << "没有可行的" << "crossover操作" << endl;
					continue;
				}

				//产生邻居的排序
				computeOrder();

				double v1 = computeFirstEvaluation(flag, &s);
				double v2 = computeSecondEvaluation(flag, &s);
				double v3 = computeThirdEvaluation(flag, &s);
				double v_1 = computeFirstEvaluation(flag, n_s);
				double v_2 = computeSecondEvaluation(flag, n_s);
				double v_3 = computeThirdEvaluation(flag, n_s);
				if (evaluation[order[0]][0] > v1 && evaluation[order[0]][1] > v2 && evaluation[order[0]][2] > v3)
				{
					int r = ceil((rand() / double(RAND_MAX)) * order.size());
					double diff = 0.5 * (evaluation[order[r]][0] - v_1) + 0.3 * (evaluation[order[r]][1] - v_2) + 0.2 * (evaluation[order[r]][2] - v_3);
					if (evaluation[order[r]][0] > v_1 && evaluation[order[r]][1] > v_2 && evaluation[order[r]][2] > v_3)
					{
						if (rand() % 100 / (double)100 <= exp(diff / t))
						{
							changeToNeighbour(neighbour[order[r]], n_s);
						}
					}
					else
					{
						changeToNeighbour(neighbour[order[r]], n_s);
					}
				}
				else
				{
					changeToNeighbour(neighbour[order[0]], &s);
					changeToNeighbour(neighbour[order[0]], n_s);
				}

				s.computeLength(data);
				s.computeTime();
				s.printSolution();
				s.isRegular(data);
			}
			t = R * t;
			nTime = time(0);
		}
	}

	cout << "------------------------------------------------------" << endl;
	cout << "---------最小化路径数目后得到的一个可行解-------------" << endl;
	cout << "------------------------------------------------------" << endl;
	s.computeLength(data);
	s.computeTime();
	s.printSolution();
	s.isRegular(data);
}

bool Solve::compareE(vector<double>* e1, vector<double>* e2)
{
	//如果e1<e2,则返回true
	if ((*e1)[0] < (*e2)[0] ||
		((*e1)[0] == (*e2)[0] && (*e1)[1] < (*e2)[1]) ||
		((*e1)[0] == (*e2)[0] && (*e1)[1] == (*e2)[1] && (*e1)[2] < (*e2)[2]))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Solve::simulatedAnneal()
{
	int maxIteration = 100;

	/*Route *route;

	Build original solution
	cout << "------------------------初始解决方案------------------------" << endl;
	for (int i = 1; i < (*cus).size(); ++i)
	{
		route = new Route(i + 1, data);
		s.routeSet.push_back(route);
	}
	s.printSolution();
	cout << "------------------------------------------------------------" << endl; */

	double t;
	int o;  //操作：1.Two-exchange 2.Or-exchange 3.Relocation 4.Exchange 5.Crossover
	double c; //随机产生的一个客户编号

	//第一个数为0时表示解决方案tmp_s等于当前解决方案，不做任何变化
	//1：做一次2-exchange 2：做一次or-exchange 3：做一次relocation 4：做一次exchange 5：做一次crossover
	Solution* n_s = new Solution();
	n_s->doCopy(&s);
	vector<double>* flag = new vector<double>();
	flag->push_back(0);

	Solution* best_s = new Solution();
	best_s->doCopy(&s);

	/*
	n_s->routeSet[1]->mergeRoute(n_s->routeSet[2], data);
	cout << "-----------------n_s--------------" << endl;
	n_s->printSolution();

	cout << "-------------------s--------------" << endl;
	s.printSolution();
	*/

	//获得当前时间
	time_t iTime = time(0);
	time_t nTime = time(0);
	while (nTime - iTime < 10)
	{
		t = T;
		while (nTime - iTime < 10 && t > EPS)
		{
			for (int i = 1; i < maxIteration; i++)
			{
				//产生n_s的邻居并计算邻居的评价值
				/*清空上一次产生的邻居的评价值等信息*/
				vector<vector<double>*>().swap(neighbour);
				vector<vector<double>>().swap(evaluation);
				vector<int>().swap(order);
				for (int m = 0; m < 5; ++m)
				{
					o = (rand() % 5) + 1;
					c = (double)(rand() % 99) + 2;

					cout << "*******************************第" << i << "次操作**********************" << endl;
					cout << "对客户" << c << "进行" << o << "操作" << endl;

					generateNeighbour(o, c, n_s);
				}
					

				if (neighbour.size() == 0)
				{
					if (o == 1)
						cout << "对客户" << c << "没有可行的" << "2-exchange操作" << endl;
					else if (o == 2)
						cout << "对客户" << c << "没有可行的" << "or-exchange操作" << endl;
					else if (o == 3)
						cout << "对客户" << c << "没有可行的" << "relocation操作" << endl;
					else if (o == 4)
						cout << "对客户" << c << "没有可行的" << "exchange操作" << endl;
					else if (o == 5)
						cout << "对客户" << c << "没有可行的" << "crossover操作" << endl;
					continue;
				}

				//产生邻居的排序
				computeOrder();

				vector<double>* e;
				vector<double>* n_e;
				e = new vector<double>();
				n_e = new vector<double>();
				computeEvaluation(&e, flag, best_s);
				computeEvaluation(&n_e, flag, n_s);
				if (compareE(e, &evaluation[order[0]]))
				{
					int r = rand() % order.size();
					cout << order.size() << endl;
					double diff = 0.5 * (evaluation[order[r]][0] - (*n_e)[0]) + 0.3 * (evaluation[order[r]][1] - (*n_e)[1]) + 0.2 * (evaluation[order[r]][2] - (*n_e)[2]);
					if (compareE(n_e, &evaluation[order[0]]))
					{
						if (rand() % 100 / (double)100 <= exp(diff / t))
						{
							changeToNeighbour(neighbour[order[r]], n_s);
						}
					}
					else
					{
						changeToNeighbour(neighbour[order[r]], n_s);
					}
				}
				else
				{
					changeToNeighbour(neighbour[order[0]], n_s);
					delete best_s;
					best_s = new Solution();
					best_s->doCopy(n_s);

					best_s->printSolution();
					best_s->isRegular(data);
				}

				/*s.computeLength(data);
				s.computeTime();*/

			}
			t = R * t;
			nTime = time(0);
		}
	}

	/*cout << "------------------------------------------------------" << endl;
	cout << "---------最小化路径数目后得到的一个可行解-------------" << endl;
	cout << "------------------------------------------------------" << endl;
	s.computeLength(data);
	s.computeTime();
	s.printSolution();
	s.isRegular(data);

	best_s->computeLength(data);
	best_s->computeTime();*/

	best_s->printSolution();
	best_s->isRegular(data);

	record(best_s);
}

/*
void Solve::orderRelatedness(vector<vector<double>>* relatedness)
{

}

void Solve::selectCustomers(vector<double>* cus_set, int n)
{
	vector<double>().swap(*cus_set);

	vector<vector<double>>* relatedness;

	//c[num - 2] == 1表示没有加入cus_set里
	vector<double> c;
	for (int i = 1; i < cus->size(); ++i)
	{
		c.push_back(1);
	}

	//随机选择一个客户
	cus_set->push_back((double)(rand() % 99) + 2);
	c[(*cus_set)[0]] = 0;

	for (int i = 2; i <= n; ++i)
	{
		double c = (double)(rand() % cus_set->size());
		vector<vector<double>>().swap(*relatedness);
		for (int num = 2; num < 102; ++num)
		{

		}
	}
}

void Solve::minimizeTravelCost()
{
	vector<double>* cus_set;
	int maxSearches = 5;
	int maxIterations = 5;
	int p = 10;
	for (int l = 1; l <= maxSearches; ++l)
	{
		for (int n = 1; n <= p; ++n)
		{
			for (int i = 1; i < maxIterations; ++i)
			{
				selectCustomers(cus_set, n);
				//选择最小开销的邻居
				//判断邻居是否比当前方案好，如果好，则将当前方案更新为邻居方案
			}
		}
	}
}

*/

/*------------------------------------------------------------------------------*/
/*--------------------------------禁忌搜索算法-----------------------------------*/
/*-------------------------------------------------------------------------------*/
void Solve::generateCross(Solution* n_s)
{
	/*清空上一次产生的邻居的评价值等信息*/
	vector<vector<double>*>().swap(neighbour);
	vector<vector<double>>().swap(evaluation);
	vector<int>().swap(order);


}

bool Solve::isTabu(vector<vector<double>*>* tabu_list, vector<double>* n)
{

	for (int i = 0; i < tabu_list->size(); ++i)
	{
		int j;
		for (j = 0; j < n->size();)
		{
			if ((*n)[j] == (*((*tabu_list)[i]))[j + 1])
				++j;
		}
		if (j == n->size())
			return true;
	}

	return false;
}

void Solve::updateTabuList(vector<vector<double>*>* tabu_list, int iteration, int tenure)
{
	for (int i = 0; i < tabu_list->size(); ++i)
	{
		if ((*((*tabu_list)[i]))[0] + tenure >= iteration)
		{
			tabu_list->erase(tabu_list->begin() + i);
		}
	}
}

void Solve::addToTabuList(vector<vector<double>*>* tabu_list, int iteration, vector<double>* n)
{
	vector<double>* tabu_entry = new vector<double>();
	tabu_entry->push_back(iteration);
	for (int j = 0; j < n->size(); ++j)
	{
		tabu_entry->push_back((*n)[j]);
	}
	tabu_list->push_back(tabu_entry);
}

void Solve::reverseOperate(vector<double>** r_n, vector<double>* n, Solution* n_s)
{
	double o = (*n)[0];
	double i = (*n)[1];
	double j = (*n)[2];
	double r = (*n)[3];
	double m;

	if (o == 1)
	{
		Node* node_i;
		if (i == 1)
			node_i = n_s->routeSet[r]->head;
		else
			node_i = n_s->routeSet[r]->searchNode(i);
		(*r_n)->push_back(1);
		(*r_n)->push_back(i);
		(*r_n)->push_back(node_i->succ->num);
		(*r_n)->push_back(r);
	}
	else if (o == 2)
	{
		Node* node_i;
		node_i = n_s->routeSet[r]->searchNode(i);
		(*r_n)->push_back(2);
		(*r_n)->push_back(i);
		(*r_n)->push_back(node_i->pred->num);
		(*r_n)->push_back(r);
	}
	else if (o == 3)
	{
		m = (*n)[4];

		Node* node_i;
		node_i = n_s->routeSet[r]->searchNode(i);
		(*r_n)->push_back(3);
		(*r_n)->push_back(i);
		(*r_n)->push_back(node_i->pred->num);
		(*r_n)->push_back(m);
		(*r_n)->push_back(r);
	}
	else if (o == 4)
	{
		m = (*n)[4];

		Node* node_i;
		(*r_n)->push_back(4);
		(*r_n)->push_back(i);
		(*r_n)->push_back(j);
		(*r_n)->push_back(m);
		(*r_n)->push_back(r);
	}
	else if (o == 5)
	{
		m = (*n)[4];

		Node* node_i;
		(*r_n)->push_back(5);
		(*r_n)->push_back(i);
		(*r_n)->push_back(j);
		(*r_n)->push_back(r);
		(*r_n)->push_back(m);
	}
}

void Solve::tabuSearch()
{
	Solution* best_s = new Solution();
	best_s->doCopy(&s);

	Solution* n_s = new Solution();
	n_s->doCopy(&s);
	vector<double>* flag = new vector<double>();
	flag->push_back(0);


	double t;
	int o;  //操作：1.Two-exchange 2.Or-exchange 3.Relocation 4.Exchange 5.Crossover
	double c; //随机产生的一个客户编号

	vector<vector<double>*> tabu_list;
	/*加入时的迭代次数+邻居操作*/
	vector<double>* tabu_entry;
	int tenure;
	int maxIteration = 500;
	tenure = maxIteration / 2;
	for (int i = 0; i < maxIteration; ++i)
	{
		updateTabuList(&tabu_list, i, tenure);

		o = (rand() % 5) + 1;
		c = (double)(rand() % 99) + 2;

		cout << "*******************************第" << i << "次操作**********************" << endl;
		cout << "对客户" << c << "进行" << o << "操作" << endl;

		//产生n_s的邻居并计算邻居的评价值
		/*清空上一次产生的邻居的评价值等信息*/
		vector<vector<double>*>().swap(neighbour);
		vector<vector<double>>().swap(evaluation);
		vector<int>().swap(order);
		for(int m = 0; m < 10;++m)
			generateNeighbour(o, c, n_s);

		if (neighbour.size() == 0)
		{
			if (o == 1)
				cout << "对客户" << c << "没有可行的" << "2-exchange操作" << endl;
			else if (o == 2)
				cout << "对客户" << c << "没有可行的" << "or-exchange操作" << endl;
			else if (o == 3)
				cout << "对客户" << c << "没有可行的" << "relocation操作" << endl;
			else if (o == 4)
				cout << "对客户" << c << "没有可行的" << "exchange操作" << endl;
			else if (o == 5)
				cout << "对客户" << c << "没有可行的" << "crossover操作" << endl;
			continue;
		}

		//产生邻居的排序
		computeOrder();

		int m;
		for (m = 0; m < neighbour.size(); ++m)
		{
			if (!isTabu(&tabu_list, neighbour[order[m]]))
				break;
		}

		//将neighbour[order[m]]的反向操作加入到禁忌表中
		vector<double>* r_n = new vector<double>();
		reverseOperate(&r_n, neighbour[order[m]], n_s);
		addToTabuList(&tabu_list, i, r_n);

		changeToNeighbour(neighbour[order[m]], n_s);

		vector<double>* e;
		vector<double>* n_e;
		e = new vector<double>();
		n_e = new vector<double>();
		computeEvaluation(&e, flag, best_s);
		computeEvaluation(&n_e, flag, n_s);
		//如果解决方案比遇到的最好的解决方案还好,改变最好的解决方案
		if (compareE(n_e, e))
		{
			delete best_s;
			best_s = new Solution();
			best_s->doCopy(n_s);

			best_s->printSolution();
			best_s->isRegular(data);
		}	
	}

	best_s->printSolution();
	best_s->isRegular(data);

	record(best_s);
}