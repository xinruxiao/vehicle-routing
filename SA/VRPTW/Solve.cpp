#include "Solve.h"

#include<math.h>

#define DEMAND 200

//插入启发式算法需要的参数
/*1 1 1 0
  1 2 1 0
  1 1 0 1
  1 2 0 1*/
#define U1 1
#define U2 1      
#define M1 1
#define M2 0

//目标函数参数
#define G1 0.7
#define G2 0.3

//模拟退火算法需要的参数
#define T 3000 //初始温度
#define EPS 1e-8 //终止温度
#define R 0.98 //温度降低速率


void Solve::record(Solution* temp_s)
{
	ofstream out;
	out.open("C:\\Users\\MiaoAYao\\Desktop\\SA.csv", std::ios::app);

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

	out << temp_s->routeSet.size() << " ,";
	out << temp_s->allLength << ",";
	out << temp_s->allTime << ",";
	out << 0.7 * temp_s->allLength + 0.3 * temp_s->allTime << ",";

}

Solve::Solve(Data* d)
{
	cus = &(d->data);
	data = d;
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
		//cout << "产生一条新的路径..." << endl;
		//产生一条新的路径添加到路径集的末尾
		initRoute(flag);
		//nRoute为当前的路径
		nRoute = s.routeSet.back();


		/*如果能够在当前路径中可以找到可行的插入*/
		while (true)
		{

			//cout << "-----------------------------------------" << endl;
			//cout << "当前路径为：";
			//nRoute->printRoute();

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
				//cout << "在" << insert_m->num << "和" << insert_n->num << "之间插入节点：" << u << endl;
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


	//cout << "------------------------------------------------------" << endl;
	//cout << "---------由插入启发式算法得到的一个可行解-------------" << endl;
	//cout << "------------------------------------------------------" << endl;
	s.computeLength(data);
	s.computeTime();
	//s.printSolution();
	//s.isRegular(data);

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
/*------------------------- ------模拟退火算法----------------------------------*/
/*-----------------------------------------------------------------------------*/
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
			
			//cout << "------------------------------------------------------------" << endl;
			//cout << "对客户" << node_i->num << "和" << node_j->num << "进行2Exchange..." << endl;
			//cout << "路径r：";
			//n_s->routeSet[r]->printRoute();
			//cout << "=====" << endl;

			n_s->do2Exchange(n_s->routeSet[r], node_i,  node_i_succ, node_j, node_j_succ);

			//cout << "路径r：";
			//
			//n_s->routeSet[r]->printRoute();

			//判断进行2Exchange之后得到的邻居可不可行
			if (n_s->routeSet[r]->isFeasibleGC(node_i->num, node_i->d_t, node_j, data))
			{
				//如果可行
				//计算这种情况下的开销
				double tmp_g;
				double tmp_allLength, tmp_allTime;
				double r_time;
				tmp_allLength = n_s->allLength -
					data->distance(node_j->num, node_j_succ->num) -
					data->distance(node_i->num, node_i_succ->num) +
					data->distance(node_j->num, node_i->num) +
					data->distance(node_j_succ->num, node_i_succ->num);
				r_time = n_s->routeSet[r]->computeNewAT(node_i->num, node_i->d_t, node_j, data);
				tmp_allTime = n_s->allTime + r_time - n_s->routeSet[r]->tail->a_t;
				tmp_g = G1 * tmp_allLength + G2 * tmp_allTime;

				if (g_best_neighbour == -1 || tmp_g < g_best_neighbour)
				{
					//该邻居为当前探索到的最好的邻居
					vector<double>().swap(best_neighbour);
					best_neighbour.push_back(1);
					best_neighbour.push_back(node_i->num);
					best_neighbour.push_back(node_j->num);
					best_neighbour.push_back(r);

					g_best_neighbour = tmp_g;
				}
			}

			n_s->restore2Exchange(n_s->routeSet[r], node_i, node_i_succ, node_j, node_j_succ);
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

			//cout << "------------------------------------------------------------" << endl;
			//cout << "对客户" << node_i->num << "和" << node_j->num << "进行2Exchange..." << endl;
			//cout << "路径r：";
			//n_s->routeSet[r]->printRoute();
			//cout << "=====" << endl;

			n_s->do2Exchange(n_s->routeSet[r], node_j, node_j_succ, node_i, node_i_succ);

			//cout << "路径r：";
			//n_s->routeSet[r]->printRoute();

			//判断进行2Exchange之后得到的邻居可不可行
			if (n_s->routeSet[r]->isFeasibleGC(node_j->num, node_j->d_t, node_i, data))
			{
				//如果可行
				//计算这种情况下的开销
				double tmp_g;
				double tmp_allLength, tmp_allTime;
				double r_time;
				tmp_allLength = n_s->allLength -
					data->distance(node_j->num, node_j_succ->num) -
					data->distance(node_i->num, node_i_succ->num) +
					data->distance(node_j->num, node_i->num) +
					data->distance(node_j_succ->num, node_i_succ->num);
				r_time = n_s->routeSet[r]->computeNewAT(node_j->num, node_j->d_t, node_i, data);
				tmp_allTime = n_s->allTime + r_time - n_s->routeSet[r]->tail->a_t;
				tmp_g = G1 * tmp_allLength + G2 * tmp_allTime;

				if (g_best_neighbour == -1 || tmp_g < g_best_neighbour)
				{
					//该邻居为当前探索到的最好的邻居
					vector<double>().swap(best_neighbour);
					best_neighbour.push_back(1);
					best_neighbour.push_back(node_j->num);
					best_neighbour.push_back(node_i->num);
					best_neighbour.push_back(r);

					g_best_neighbour = tmp_g;
				}
				
			}
			
			n_s->restore2Exchange(n_s->routeSet[r], node_j, node_j_succ, node_i, node_i_succ);
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

		//cout << "------------------------------------------------------------" << endl;
		//cout << "对客户" << node_i->num << "和" << node_j->num << "进行OrExchange..." << endl;
		//cout << "路径r：";
		//n_s->routeSet[r]->printRoute();
		//cout << "=====" << endl;
		
		n_s->doOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);

		//cout << "路径r：";
		//n_s->routeSet[r]->printRoute();

		double at_i, wt_i, dt_i;
		at_i = node_j->d_t + data->distance(node_j->num, node_i->num);
		wt_i = at_i > (*cus)[node_i->num - 1]->e_t ? 0 : (*cus)[node_i->num - 1]->e_t - at_i;
		dt_i = at_i + wt_i + (*cus)[node_i->num - 1]->s_t;

		//判断OrExchange邻居是否可行
		if (n_s->routeSet[r]->isFeasible(node_i->num, dt_i, node_j_succ, data))
		{
			//如果可行
			//计算这种情况下的开销
			double tmp_g;
			double tmp_allLength, tmp_allTime;
			double  r_time;
			tmp_allLength = n_s->allLength
				+ data->distance(node_i_pred->num, node_i_succ->num)
				+ data->distance(node_j->num, node_i->num)
				+ data->distance(node_i->num, node_j_succ->num)
				- data->distance(node_i_pred->num, node_i->num)
				- data->distance(node_i->num, node_i_succ->num)
				- data->distance(node_j->num, node_j_succ->num);
			r_time = n_s->routeSet[r]->computeNewAT(node_j->num, node_j->d_t, node_i, data);
			tmp_allTime = n_s->allTime + r_time - n_s->routeSet[r]->tail->a_t;
			tmp_g = G1 * tmp_allLength + G2 * tmp_allTime;

			if (g_best_neighbour == -1 || tmp_g < g_best_neighbour)
			{
				//该邻居为当前探索到的最好的邻居
				vector<double>().swap(best_neighbour);
				best_neighbour.push_back(2);
				best_neighbour.push_back(node_i->num);
				best_neighbour.push_back(node_j->num);
				best_neighbour.push_back(r);
				best_neighbour.push_back(1);   //为1表示j在i前面

				g_best_neighbour = tmp_g;
			}
			
		}

		n_s->restoreOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);
		tmp = tmp->succ;
	}

	//j在i后面
	tmp = node_i_succ;
	while (tmp != n_s->routeSet[r]->tail)
	{
		node_j = tmp;
		node_j_succ = node_j->succ;

		//cout << "------------------------------------------------------------" << endl;
		//cout << "对客户" << node_i->num << "和" << node_j->num << "进行OrExchange..." << endl;
		//cout << "路径r：";
		//n_s->routeSet[r]->printRoute();
		//cout << "=====" << endl;

		n_s->doOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);

		//cout << "路径r：";
		//n_s->routeSet[r]->printRoute();

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
				//计算这种情况下的开销
				double tmp_g;
				double tmp_allLength, tmp_allTime;
				double  r_time;
				tmp_allLength = n_s->allLength
					+ data->distance(node_i_pred->num, node_i_succ->num)
					+ data->distance(node_j->num, node_i->num)
					+ data->distance(node_i->num, node_j_succ->num)
					- data->distance(node_i_pred->num, node_i->num)
					- data->distance(node_i->num, node_i_succ->num)
					- data->distance(node_j->num, node_j_succ->num);
				r_time = n_s->routeSet[r]->computeNewAT(node_i_pred->num, node_i_pred->d_t, node_i_succ, data);
				tmp_allTime = n_s->allTime + r_time - n_s->routeSet[r]->tail->a_t;
				tmp_g = G1 * tmp_allLength + G2 * tmp_allTime;

				if (g_best_neighbour == -1 || tmp_g < g_best_neighbour)
				{
					//该邻居为当前探索到的最好的邻居
					vector<double>().swap(best_neighbour);
					best_neighbour.push_back(2);
					best_neighbour.push_back(node_i->num);
					best_neighbour.push_back(node_j->num);
					best_neighbour.push_back(r);
					best_neighbour.push_back(0);   //为1表示j在i前面

					g_best_neighbour = tmp_g;
				}

			}
		}

		n_s->restoreOrExchange(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);
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

				//cout << "------------------------------------------------------------" << endl;
				//cout << "对客户" << node_i->num << "和" << node_j->num << "进行Relocation..." << endl;
				//cout << "路径r：";
				//n_s->routeSet[r]->printRoute();
				//cout << "路径m：";
				//n_s->routeSet[m]->printRoute();
				//cout << "=====" << endl;

				n_s->doRelocation(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);

				//cout << "路径r：";
				//n_s->routeSet[r]->printRoute();
				//cout << "路径m：";
				//n_s->routeSet[m]->printRoute();


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
						
						//如果可行
						//计算这种情况下的开销
						double tmp_g;
						double tmp_allLength, tmp_allTime;
						double  r_time, m_time;
						if (n_s->routeSet[r]->size == 1)
						{
							tmp_allLength = n_s->allLength
								+ data->distance(node_j->num, node_i->num)
								+ data->distance(node_i->num, node_j_succ->num)
								- data->distance(node_i_pred->num, node_i->num)
								- data->distance(node_i->num, node_i_succ->num)
								- data->distance(node_j->num, node_j_succ->num);
							r_time = n_s->routeSet[r]->head->d_t;
						}
						else
						{
							tmp_allLength = n_s->allLength
								+ data->distance(node_i_pred->num, node_i_succ->num)
								+ data->distance(node_j->num, node_i->num)
								+ data->distance(node_i->num, node_j_succ->num)
								- data->distance(node_i_pred->num, node_i->num)
								- data->distance(node_i->num, node_i_succ->num)
								- data->distance(node_j->num, node_j_succ->num);
							r_time = n_s->routeSet[r]->computeNewAT(node_i_pred->num, node_i_pred->d_t, node_i_succ, data);
						}
						m_time = n_s->routeSet[m]->computeNewAT(node_j->num, node_j->d_t, node_i, data);
						tmp_allTime = n_s->allTime + r_time + m_time - n_s->routeSet[r]->tail->a_t - n_s->routeSet[m]->tail->a_t;
						tmp_g = G1 * tmp_allLength + G2 * tmp_allTime;

						if (g_best_neighbour == -1 || tmp_g < g_best_neighbour)
						{
							//该邻居为当前探索到的最好的邻居
							vector<double>().swap(best_neighbour);
							best_neighbour.push_back(3);
							best_neighbour.push_back(node_i->num);
							best_neighbour.push_back(node_j->num);
							best_neighbour.push_back(r);
							best_neighbour.push_back(m);

							g_best_neighbour = tmp_g;
						}

						
					}
				
				}
				n_s->restoreRelocation(node_i_pred, node_i, node_i_succ, node_j, node_j_succ);
				tmp = tmp->succ;
			}
		}
	}
	//cout << "here" << endl;
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

					
					//cout << "------------------------------------------------------------" << endl;
					//cout << "对客户" << node_i->num << "和" << node_j->num << "进行Exchange..." << endl;
					//cout << "路径r：";
					//n_s->routeSet[r]->printRoute();
					//cout << "路径m：";
					//n_s->routeSet[m]->printRoute();
					//cout << "=====" << endl;

					n_s->doExchange(node_i_pred, node_i, node_i_succ, node_j_pred, node_j, node_j_succ);

					//cout << "路径r：";
					//n_s->routeSet[r]->printRoute();
					//cout << "路径m：";
					//n_s->routeSet[m]->printRoute();

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
								//计算这种情况下的开销
								double tmp_g;
								double tmp_allLength, tmp_allTime;
								double  r_time, m_time;

								tmp_allLength = n_s->allLength
									+ data->distance(node_i_pred->num, node_j->num)
									+ data->distance(node_j->num, node_i_succ->num)
									+ data->distance(node_j_pred->num, node_i->num)
									+ data->distance(node_i->num, node_j_succ->num)
									- data->distance(node_i_pred->num, node_i->num)
									- data->distance(node_i->num, node_i_succ->num)
									- data->distance(node_j_pred->num, node_j->num)
									- data->distance(node_j->num, node_j_succ->num);
								r_time = n_s->routeSet[r]->computeNewAT(node_i_pred->num, node_i_pred->d_t, node_j, data);
								m_time = n_s->routeSet[m]->computeNewAT(node_j_pred->num, node_j_pred->d_t, node_i, data);
								tmp_allTime = n_s->allTime + r_time + m_time - n_s->routeSet[r]->tail->a_t - n_s->routeSet[m]->tail->a_t;
								tmp_g = G1 * tmp_allLength + G2 * tmp_allTime;

								if (g_best_neighbour == -1 || tmp_g < g_best_neighbour)
								{
									//该邻居为当前探索到的最好的邻居
									vector<double>().swap(best_neighbour);
									best_neighbour.push_back(4);
									best_neighbour.push_back(node_i->num);
									best_neighbour.push_back(node_j->num);
									best_neighbour.push_back(r);
									best_neighbour.push_back(m);

									g_best_neighbour = tmp_g;
								}
								
							}
						}
					}	
				}
				n_s->restoreExchange(node_i_pred, node_i, node_i_succ, node_j_pred, node_j, node_j_succ);
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

					//cout << "------------------------------------------------------------" << endl;
					//cout << "对" << node_i->num << "和" << node_j->num << "做crossover..." << endl;
					//cout << "路径r：";
					//n_s->routeSet[r]->printRoute();
					//cout << "路径m：";
					//n_s->routeSet[m]->printRoute();
					//cout << "=====" << endl;

					n_s->doCrossover(node_i, node_i_succ, node_j, node_j_succ);
					Node* ptail = n_s->routeSet[r]->tail;
					n_s->routeSet[r]->tail = n_s->routeSet[m]->tail;
					n_s->routeSet[m]->tail = ptail;

					//cout << "路径r：";
					//n_s->routeSet[r]->printRoute();
					//cout << "路径m：";
					//n_s->routeSet[m]->printRoute();

					//判断Crossover邻居是否可行
					if (n_s->routeSet[m]->isFeasible(node_j->num, node_j->d_t, node_i_succ, data) &&
						n_s->routeSet[r]->isFeasible(node_i->num, node_i->d_t, node_j_succ, data))
					{	
						//如果可行
						//计算这种情况下的开销
						double tmp_g;
						double tmp_allLength, tmp_allTime;
						double  r_time, m_time;

						if (node_j->num == 1 && node_i_succ->num == 1)
						{
							tmp_allLength = n_s->allLength
								+ data->distance(node_i->num, node_j_succ->num)
								- data->distance(node_i->num, node_i_succ->num)
								- data->distance(node_j->num, node_j_succ->num);
							m_time = n_s->routeSet[m]->head->d_t;
						}
						else
						{
							tmp_allLength = n_s->allLength
								+ data->distance(node_j->num, node_i_succ->num)
								+ data->distance(node_i->num, node_j_succ->num)
								- data->distance(node_i->num, node_i_succ->num)
								- data->distance(node_j->num, node_j_succ->num);
							m_time = n_s->routeSet[m]->computeNewAT(node_j->num, node_j->d_t, node_i_succ, data);
						}
						r_time = n_s->routeSet[r]->computeNewAT(node_i->num, node_i->d_t, node_j_succ, data);

						tmp_allTime = n_s->allTime + r_time + m_time - n_s->routeSet[r]->tail->a_t - n_s->routeSet[m]->tail->a_t;
						tmp_g = G1 * tmp_allLength + G2 * tmp_allTime;

						if (g_best_neighbour == -1 || tmp_g < g_best_neighbour)
						{
							//该邻居为当前探索到的最好的邻居
							vector<double>().swap(best_neighbour);
							best_neighbour.push_back(5);
							best_neighbour.push_back(node_i->num);
							best_neighbour.push_back(node_j->num);
							best_neighbour.push_back(r);
							best_neighbour.push_back(m);

							g_best_neighbour = tmp_g;
						}
								
					}
					n_s->restoreCrossover(node_i, node_i_succ, node_j, node_j_succ);
					ptail = n_s->routeSet[r]->tail;
					n_s->routeSet[r]->tail = n_s->routeSet[m]->tail;
					n_s->routeSet[m]->tail = ptail;
				}
				tmp = tmp->succ;
			}
		}
	}
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

/*模拟退火算法*/
void Solve::simulatedAnnealing()
{
	Solution* best_s = new Solution();
	best_s->doCopy(&s);

	Solution* n_s = new Solution();
	n_s->doCopy(&s);

	//第一个数为0时表示解决方案tmp_s等于当前解决方案，不做任何变化
    //1：做一次2-exchange 2：做一次or-exchange 3：做一次relocation 4：做一次exchange 5：做一次crossover		 
	vector<double>* flag = new vector<double>();
	flag->push_back(0);

	int maxIteration = 2500;
	double t;
	int o;  //操作：1.Two-exchange 2.Or-exchange 3.Relocation 4.Exchange 5.Crossover
	double c; //随机产生的一个客户编号

	//获得当前时间
	time_t iTime = time(0);
	time_t nTime = time(0);
	while (nTime - iTime < 0.1)
	{
		t = T;
		while (nTime - iTime < 0.1 && t > EPS)
		{
			for (int i = 1; i < maxIteration; i++)
			{
				//产生n_s的邻居并计算邻居的评价值
				/*清空上一次产生的邻居的评价值等信息*/
				vector<double>().swap(best_neighbour);
				//0表示没有找到可行的邻居
				best_neighbour.push_back(0);
				g_best_neighbour = -1;

				/*
				for (int l = 0; l < 50; ++l)
				{
					o = (rand() % 5) + 1;
					c = (double)(rand() % 99) + 2;
					//cout << "*****************************************************" << endl;
					//cout << "对客户" << c << "进行" << o << "操作" << endl;
					generateNeighbour(o, c, n_s);
				}
				*/

				for (c = 2; c <= 101; ++c)
				{
					for(int k = 1; k <=5;++k)
						generateNeighbour(k, c, n_s);
				}

				if (g_best_neighbour != -1)
				{
					double e = G1 * best_s->allLength + G2 * best_s->allTime;
					double n_e = G1 * n_s->allLength + G2 * n_s->allTime;

					if (e < g_best_neighbour)
					{
						double diff = n_e - g_best_neighbour;
						if (diff < 0)
						{
							if (rand() % 100 / (double)100 <= exp(diff / t))
							{
								changeToNeighbour(&best_neighbour, n_s);
							}
						}
						else
						{
							changeToNeighbour(&best_neighbour, n_s);
						}
					}
					else
					{
						changeToNeighbour(&best_neighbour, n_s);
						delete best_s;
						best_s = new Solution();
						best_s->doCopy(n_s);

						//best_s->printSolution();
						//best_s->isRegular(data);
					}
				}

			}
			t = R * t;
			nTime = time(0);
		}
	}
	//best_s->printSolution();
	//best_s->isRegular(data);

	record(best_s);
}