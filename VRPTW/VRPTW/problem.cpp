#include "Problem.h"

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

//Russell(1995)所需参数
#define V1 0.5
#define V2 0.5

//Russell(2004)所需参数
#define T 3000 //初始温度
#define EPS 1e-8 //终止温度
#define R 0.98 //温度降低速率

#define maxIt 10 //最大迭代次数

/*------------------------------------------------------------------------------*/
/*-------------------------------solomon(1987)----------------------------------*/
/*--------------------------节省启发式路径构建算法-------------------------------*/
/*------------------------------------------------------------------------------*/
void Problem::buildSavings()
{
	allLength = 0;   //初始解的总长度
	vector<double> *saving;
	int size = data.size();
	//计算节省的值
	for (int i = 1; i < size - 1; ++i)  
	{
		for (int j = i + 1; j < size; ++j)
		{
			//只有在同一个方格时才计算距离
			if (data[i]->box_x == data[j]->box_x && data[i]->box_y == data[j]->box_y)
			{
				saving = new vector<double>();
				saving->push_back(dis[i - 1][0] + dis[j - 1][0] - dis[j - 1][i]);
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

void Problem::deleteSavings(int cus_i, int cus_j)
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

void Problem::downAdjust(int i)
{
	int size = savings.size();
	for (int j = 2 * i + 1; j < size;)
	{
		vector<double>* temp = savings[i];
		int maxSav = j;
		if (j + 1 < size)
		{
			if ((*savings[j])[0] < (*savings[j + 1])[0])
				maxSav = j + 1;
		}
		savings[i] = savings[maxSav];
		savings[maxSav] = temp;
		j = 2 * maxSav + 1;
	}
}

//前一个节点的编号;前一个节点的离开时间;加入的路径
bool Problem::judge(double pre_num, double pre_dTime, Route* r)
{
	double p_num = pre_num;
	double p_dTime= pre_dTime;
	double aTime;              //到达当前节点的时间
	double wTime;				//在当前节点等待的时间
	Node* tmp = r->head;        //当前节点
	while (tmp)
	{
		double distance;
		if (p_num > tmp->num)
			distance = dis[p_num - 2][tmp->num - 1];
		else
			distance = dis[tmp->num - 2][p_num - 1];
		aTime = p_dTime + distance;
		if (aTime > tmp->l_t)
		{
			return false;
		}
		else
		{
			wTime = tmp->e_t > aTime ? tmp->e_t - aTime : 0;
			p_dTime = aTime + wTime + tmp->s_t;
			p_num = tmp->num;
			tmp = tmp->next;
		}
	}
	//判断回到仓库的时间有没有违背时间窗
	if (p_dTime + dis[p_num - 2][0] > data[0]->l_t)
		return false;

	return true;
}

void Problem::updateRoute(Route* route, double num)
{
	
	double i = num;		//前一个节点编号
	double dt_i = data[i - 1]->d_t;   //从前一个节点离开的时间
	double dis_ij;    //两个节点之间的距离
	double j;		//当前节点编号
	double at_j;
	double wt_j;
	double dt_j;

	Node* tmp = data[num - 1]->next;
	while (tmp)
	{
		j = tmp->num;

		if (i > j)
			dis_ij = dis[i - 2][j - 1];
		else
			dis_ij = dis[j - 2][i - 1];

		at_j = dt_i + dis_ij;
		wt_j = at_j > data[j - 1]->e_t ? 0 : data[j - 1]->e_t - at_j;
		dt_j = at_j + wt_j + data[j - 1]->s_t;

		data[j - 1]->a_t = at_j;
		data[j - 1]->w_t = wt_j;
		data[j - 1]->d_t = dt_j;

		tmp = tmp->next;
		i = j;
		dt_i = dt_j;
	}

	route->at_depot = dt_i + dis[i - 2][0];
}

void Problem::addRoute(Route* r1, Route* r2)
{
	r1->tail->next = r2->head;
	r1->tail = r2->tail;
	r1->size += r2->size;
	r1->demand += r2->demand;
	
	updateRoute(r1, r1->tail->num);
}

void Problem::useSavings()
{
	Route *route;

	//Build original solution
	for (int i = 1; i < data.size(); ++i)
	{

		//从仓库节点data[0]的出发时间为data[0]->e_t
		data[i]->a_t = data[0]->e_t + dis[i - 1][0];
		data[i]->w_t = data[i]->e_t > data[i]->a_t ? data[i]->e_t - data[i]->a_t : 0;
		data[i]->d_t = data[i]->a_t + data[i]->w_t + data[i]->s_t;

		route = new Route(data[i], data[0]->e_t, data[i]->d_t + dis[data[i]->num - 2][0]);
		routeSet.push_back(route);
	}
	//Compute original length
	allLength = 0;
	for (int i = 0; i < dis.size(); ++i)
	{
		allLength += dis[i][0] * 2;
	}

	//Build better solution
	while (savings.size() != 0)
	{
		cout << "--------------------------------------------" << endl;
		cout <<"Savings的条目数为： " << savings.size() << endl;
		/* 加入边cus_i -> cus_j后，能够节省最大距离的边的信息*/
		int cus_i = (*savings[0])[1];
		int cus_j = (*savings[0])[2];
		double sav = (*savings[0])[0];

		cout << "连接节点：" << cus_i << "和" << cus_j << endl;

		/* 找到这条边上的两个节点分别在哪个路径上 */
		/* route_i:包含节点i的路径   route_j:包含节点j的路径
		   flag_i = 0:节点i在路径头  flag_i = 1:节点i在路径尾*/
		   /* del_i = 0:表示合并两条路径后，节点i仍在路径头或尾，反之del_i = 1*/
		int route_i, route_j;
		int flag_i = 0, flag_j = 0;
		int del_i = 0, del_j = 0;

		//找到路径集中包含客户cus_i和cus_j的两条路径
		int flag = 0;  //如果flag==2，表示已经找到这两条路径，跳出循环
		for (int i = 0; i < routeSet.size(); ++i)
		{
			Route* temp = routeSet[i];
			if ((temp->head)->num == cus_i)
			{
				route_i = i;
				++flag;
			}
			else if ((temp->tail)->num == cus_i)
			{
				route_i = i;
				flag_i = 1;
				++flag;
			}

			if ((temp->head)->num == cus_j)
			{
				route_j = i;
				++flag;
			}
			else if ((temp->tail)->num == cus_j)
			{
				route_j = i;
				flag_j = 1;
				++flag;
			}

			if (flag == 2)
				break;
		}
		
		if (routeSet[route_i]->size > 1)
		{
			del_i = 1;
			cout << "删除与" << cus_i << "相关的saving" << endl;
		}
		if (routeSet[route_j]->size > 1)
		{
			del_j = 1;
			cout << "删除与" << cus_j << "相关的saving" << endl;
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
		if (routeSet[route_i]->demand + routeSet[route_j]->demand > DEMAND)
		{
			//路径上的节点的需求超过车辆的总容量
			deleteSavings();
			continue;
		}


		if (flag_i == 0 && flag_j == 1)
		{
			//如果不违背时间窗的话
			if (judge(routeSet[route_j]->tail->num, routeSet[route_j]->tail->d_t, routeSet[route_i]))
			{
				//将route_i上的节点添加到route_j上
				addRoute(routeSet[route_j], routeSet[route_i]);

				//删除路径route_i
				routeSet.erase(routeSet.begin() + route_i);
			}
			else
			{
				deleteSavings();
				continue;
			}
		}
		else if (flag_i == 1 && flag_j == 0)
		{
			//如果不违背时间窗的话
			if (judge(routeSet[route_i]->tail->num, routeSet[route_i]->tail->d_t, routeSet[route_j]))
			{
				//将route_j上的节点添加到route_i上
				addRoute(routeSet[route_i], routeSet[route_j]);

				//删除路径route_i
				routeSet.erase(routeSet.begin() + route_j);
			}
			else
			{
				deleteSavings();
				continue;
			}
		}
		else
		{
			if (flag_i == 0 && flag_j == 0)
			{
				//将route_i上的节点反转，并把route_j添加到route_i之后
				routeSet[route_i]->reverseRoute();
			}
			else if (flag_i == 1 && flag_j == 1)
			{
				//将route_j上的节点反转,并且添加到route_i之后
				routeSet[route_j]->reverseRoute();
			}

			//如果不违背时间窗的话
			if (judge(routeSet[route_i]->tail->num, routeSet[route_i]->tail->d_t, routeSet[route_j]))
			{
				//将route_j上的节点添加到route_i上
				addRoute(routeSet[route_i], routeSet[route_j]);

				//删除路径route_i
				routeSet.erase(routeSet.begin() + route_j);
			}
			else
			{
				//将之前做出的改变还原
				if (flag_i == 0 && flag_j == 0)
				{
					//将route_i上的节点反转，并把route_j添加到route_i之后
					routeSet[route_i]->reverseRoute();
				}
				else if (flag_i == 1 && flag_j == 1)
				{
					//将route_j上的节点反转,并且添加到route_i之后
					routeSet[route_j]->reverseRoute();
				}

				deleteSavings();
				continue;
			}
		}

		/* 计算当前所有路径总长度 */
		allLength -= sav;
		cout << "allLength = " << allLength << endl;

		/* 删除savings中已不在路径头或尾的节点所在的边，进行下一次加边*/
		if (del_i == 1 && del_j == 1)
		{
			deleteSavings(cus_i, cus_j);
		}
		else if (del_i == 1 && del_j == 0)
		{
			deleteSavings(cus_i);
		}
		else if (del_i == 0 && del_j == 1)
		{
			deleteSavings(cus_j);
		}
		else
		{
			deleteSavings();
		}
		
	}
	cout << "------------------------------------------------------" << endl;
	cout << "---------由节省启发式算法得到的一个可行解-------------" << endl;
	cout << "------------------------------------------------------" << endl;
	computeTime();
	printSolution();
}

/*------------------------------------------------------------------------------*/
/*-------------------------------solomon(1987)----------------------------------*/
/*--------------------------插入启发式路径构建算法-------------------------------*/
/*------------------------------------------------------------------------------*/
void Problem::initInfor()
{
	vector<double>* tmp;
	for (int i = 1; i < data.size(); ++i)
	{
		tmp = new vector<double>();
		tmp->push_back(data[i]->num);
		tmp->push_back(-1);
		tmp->push_back(-1);    
		tmp->push_back(-1);
		infor.push_back(*tmp);
	}
}

bool Problem::judgeInsert(double num, double d_t, Node* node)
{
	
	Node* tmp = node;
	double m = num;  //前一个节点的编号
	double n;              //当前节点的编号
	double m_dt = d_t;   //离开前一个节点的时间
	double n_at;           //到达当前节点的时间
	double n_wt;		//在当前节点等待的时间
	double n_dt;		//离开当前节点的时间
	double dis_mn;  //前一节点到当前节点的距离
	double pf;
	while (tmp)
	{	
		n = tmp->num;
		if (m > n)
			dis_mn = dis[m - 2][n - 1];
		else
			dis_mn = dis[n - 2][m - 1];

		n_at = m_dt + dis_mn;
		n_wt = n_at > data[n - 1]->e_t ? 0 : data[n - 1]->e_t - n_at;
		n_dt = n_at + n_wt + data[n - 1]->s_t;

		pf = n_at - data[n - 1]->a_t;
		if (n_at > data[n - 1]->l_t)
		{
			return false;
		}
		else if (pf <= data[n - 1]->w_t)   //之后的节点都是可行的
		{
			return true;
		}
		else
		{
			m_dt = n_dt;
			m = n;
			tmp = tmp->next;
		}
	}
	//判断回到仓库的时间有没有超过最晚服务时间
	if (m_dt + dis[m - 2][0] > data[0]->l_t)
		return false;

	return true;
}

void Problem::updateInfor(Route* route)
{
	for (int i = 0; i < infor.size(); ++i)
	{

		double u = infor[i][0];  //要插入的节点u
		Node* tmp = route->head;
		int m = 1, n = tmp->num; //将节点u插入到节点m和n之间

		double insert_m = -1;    //最好的插入位置
		double insert_n = -1;

		double c_1 = -1;		//需要计算的值
		double c_11;
		double c_12;
		
		//当不违背容量约束时，才进行考虑插入位置的选择
		if (data[u - 1]->d + route->demand < DEMAND)
		{
			//m与当前节点u的距离；m和n的距离；u和n的距离
			double dis_mu, dis_mn, dis_un;

			/*at_u：到达节点u的时间*/
			double at_u;
			double wt_u;
			double dt_u;
			double at_n;

			while (tmp || n == 1)
			{
				if (m > u)
					dis_mu = dis[m - 2][u - 1];
				else
					dis_mu = dis[u - 2][m - 1];

				
				if (m > n)
					dis_mn = dis[m - 2][n - 1];
				else
					dis_mn = dis[n - 2][m - 1];

				if (n > u)
					dis_un = dis[n - 2][u - 1];
				else
					dis_un = dis[u - 2][n - 1];

				at_u = data[m - 1]->d_t + dis_mu;
				/*判断是否违背了节点u的时间窗约束
				如果违背了，那么，既然在当前位置插入都不可行了，那么之后的位置也不用尝试了*/
				if (at_u + data[u - 1]->s_t > data[u - 1]->l_t)
				{
					break;
				}

				//如果当前节点没有违背时间窗约束
				/*如果插入位置不是路径末尾*/
				c_11 = dis_mu + dis_un - U1 * dis_mn;

				//at_n:插入节点u后到达n的时间；wt_u：在节点u的等待时间；dt_u：从节点u的出发时间
				wt_u = at_u > data[u - 1]->e_t ? 0 : data[u - 1]->e_t - at_u;
				dt_u = at_u + wt_u + data[u - 1]->s_t;
				at_n = dt_u + dis_un;
				double wt_n = at_n > data[n - 1]->e_t ? 0 : data[n - 1]->e_t - at_n;
				c_12 = at_n + wt_n - (data[n - 1]->a_t + data[n - 1]->w_t);

				
				//只有插入节点u后，可以使路径上在u之后的节点都可行，才能插入在这个位置
				if (judgeInsert(u, dt_u, tmp))
				{
					double tmp_c_1 = M1 * c_11 + M2 * c_12;
					if (c_1 == -1 || tmp_c_1 < c_1)
					{
						c_1 = tmp_c_1;
						insert_m = m;
						insert_n = n;
					}
				}
				
				if (n == 1)
					break;

				tmp = tmp->next;
				if (tmp)     
				{
					m = n;
					n = tmp->num;
				}
				else     //如果在仓库之前插入节点u
				{
					m = n;
					n = 1;
				}
			}	
		}
		
		infor[i][1] = insert_m;
		infor[i][2] = insert_n;
		infor[i][3] = c_1;
	}
}

void Problem::initRoute(int flag)
{
	//选择的用来初始化路径的节点编号
	double c_num;
	//infor中要删除的条目
	int del;

	/*两种选择：1，离仓库最远的节点；2，服务时间开始最早的节点*/
	if (flag == 1)
	{
		c_num = infor[0][0];
		double maxDis = dis[c_num - 2][0];
		del = 0;

		//找没被加入路径中离仓库最远的节点
		for (int i = 1;i < infor.size(); ++i)
		{
			if (dis[infor[i][0] - 2][0] > maxDis)
			{
				c_num = infor[i][0];
				maxDis = dis[c_num - 2][0];
				del = i;
			}
		}
	}
	else if (flag == 2)
	{	
		c_num = infor[0][0];
		double eTime = data[c_num]->e_t;
		del = 0;
		
		
		//找没被加入路径中服务时间开始最早的节点
		for (int j = 1 ; j < infor.size(); ++j)
		{
			if (data[infor[j][0]]->e_t < eTime)
			{
				c_num = infor[j][0];
				eTime = data[c_num]->e_t;
				del = j;
			}
		}
	}

	//删除infor中第del个条目
	infor.erase(infor.begin() + del);

	//从仓库节点data[0]的出发时间为data[0]->e_t
	data[c_num - 1]->a_t = data[0]->e_t + dis[c_num - 2][0];
	data[c_num - 1]->w_t = data[c_num - 1]->e_t > data[c_num - 1]->a_t ? data[c_num - 1]->e_t - data[c_num - 1]->a_t : 0;
	data[c_num - 1]->d_t = data[c_num - 1]->a_t + data[c_num - 1]->w_t + data[c_num - 1]->s_t;
	
	Route* route = new Route(data[c_num - 1], data[0]->e_t, data[c_num]->d_t + dis[c_num - 1][0]);
	routeSet.push_back(route);

}

void Problem::useInsertion()
{
	int flag;
	cout << "------- --选择初始化一条路径的方法----------------------" << endl;
	cout << "----------如果选择使用离仓库最远的节点初始化路径，输入：1---------------" << endl;
	cout << "----------如果选择使用服务时间开始最早的节点初始化路径，输入：2---------" << endl;
	cin >> flag;

	Route* nRoute;
	initInfor();
	while (infor.size() != 0)   //一直产生新的路径，直到所有的节点都加入到一个路径中
	{
		cout << "产生一条新的路径..." << endl;
		//产生一条新的路径添加到路径集的末尾
		initRoute(flag);         
		//nRoute为当前的路径
		nRoute = routeSet.back();
		

		/*如果能够在当前路径中可以找到可行的插入*/
		while (true)
		{

			cout << "当前路径为：";
			nRoute->print();

			//更新infor
			updateInfor(nRoute);

			//找到最好的那个节点插入到路径中
			double c_2;
			double num = -1; //要插入的节点的条目为infor[num]
			int i;
			for (i = 0; i < infor.size(); ++i)
			{
				if (infor[i][1] != -1) //infor[i][1]为-1时表示在当前路径中没有可行的插入选择
				{
					c_2 = U2 * dis[infor[i][0] - 2][0] - infor[i][2];
					num = i;
					break;
				}
			}
			for (; i < infor.size(); ++i)
			{
				if (infor[i][1] != -1)
				{
					double tmp_c_2 = U2 * dis[infor[i][0] - 2][0] - infor[i][3];
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
				double insert_m, insert_n;  //在编号为insert_m和insert_n的节点中间插入该节点
				//插入信息在infor[num]中存储
				u = infor[num][0];
				insert_m = infor[num][1];
				insert_n = infor[num][2];
				//将条目infor[num]删除，表示节点u已经添加到路径中
				infor.erase(infor.begin() + num);

				/*将节点u插入到insert_m和insert_n之间*/
				cout << "在" << insert_m << "和" << insert_n << "之间插入节点：" << u << endl;
				if (insert_m == 1)
				{
					data[u - 1]->next = data[insert_n - 1];
					nRoute->head = data[u - 1];
				}
				else if (insert_n == 1)
				{
					data[insert_m - 1]->next = data[u - 1];
					nRoute->tail = data[insert_m - 1];
				}
				else
				{
					data[insert_m - 1]->next = data[u - 1];
					data[u - 1]->next = data[insert_n - 1];
				}
					

				/*更新节点u的到达时间等信息*/
				double dis_mu;
				if (insert_m > u)
					dis_mu = dis[insert_m - 2][u - 1];
				else
					dis_mu = dis[u - 2][insert_m - 1];

				data[u - 1]->a_t = data[insert_m - 1]->d_t + dis_mu;
				data[u - 1]->w_t = data[u - 1]->a_t > data[u - 1]->e_t ? 0 : data[u - 1]->e_t - data[u - 1]->a_t;
				data[u - 1]->d_t = data[u - 1]->a_t + data[u - 1]->w_t + data[u - 1]->s_t;


				/*更新节点u之后节点的到达时间等*/
				updateRoute(nRoute, u);
			}
		}
	}
	

	cout << "------------------------------------------------------" << endl;
	cout << "---------由插入启发式算法得到的一个可行解-------------" << endl;
	cout << "------------------------------------------------------" << endl;
	computeLength();
	computeTime();
	printSolution();
}

/*------------------------------------------------------------------------------*/
/*-------------------------------Russell(1995)----------------------------------*/
/*-------------------------------------------------------------------------------*/
void Problem::printU()
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

void Problem::computeU()
{
	//为了有序排列，初始化u，u[0]是编号为2的节点的U_i
	int size = data.size();
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
	for (int i = 0; i < routeSet.size(); ++i)
	{
		Route* tmp = routeSet[i];
		Node* temp = tmp->head;
		double m = 1, n; //当前节点的前一个节点和后一个节点的编号
		double u_1, u_2;  //M1中的节点1和节点2
		double v_1 = -1, v_2 = -1;  //M2中的节点1和节点2
		while (temp)
		{
			//确定u_1和u_2
			u_1 = temp->num;
			if (temp->next != NULL)
			{
				temp = temp->next;
				u_2 = temp->num;
				if (temp->next != NULL)
				{
					n = temp->next->num;
				}
				else
				{
					n = 1;
				}
			}
			else
			{
				temp = temp->next;
				//路径上只有一个节点
				u_2 = -1;
				n = 1;
			}

			//确定v_1和v_2
			double d_1 = -1, d_2 = -1;
			double dis_mv, dis_vn;
			for (int j = 0; j < routeSet.size(); ++j)
			{
				if (j != i)
				{
					Node* v = (routeSet[j])->head;
					for (; v; v = v->next)
					{
						if (v->num > m)
							dis_mv = dis[v->num - 2][m - 1];
						else
							dis_mv = dis[m - 2][v->num - 1];

						if (v->num > n)
							dis_vn = dis[v->num - 2][n - 1];
						else
							dis_vn = dis[n - 2][v->num - 1];

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

void Problem::localSearch1()
{
	//使用z来衡量是否新的解比原始解更优
	double z = V1 * allLength + V2 * allTime;
	//路径的条数
	int v = routeSet.size();

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
int Problem::searchRoute(double num)
{
	Node* tmp = data[num - 1];
	Node* tail = tmp;
	while (tmp->next)
	{
		tail = tmp->next;
		tmp = tmp->next;
	}
	int i;
	for (i = 0; i < routeSet.size(); ++i)
	{
		if (routeSet[i]->tail == tail)
			break;
	}
	return i;
}

double Problem::computeLatestArriveTime(double num)
{
	double t;
	if (num == 1)
	{
		t = data[0]->l_t;
		return t;
	}

	if (data[num - 1]->next == NULL)
	{
		t = data[0]->l_t - dis[num - 2][0] - data[num - 1]->s_t;
	}
	else
	{
		double d;
		double n_num = data[num - 1]->next->num;
		if (num > n_num)
			d = dis[num - 2][n_num - 1];
		else
			d = dis[n_num - 2][num - 1];

		t = computeLatestArriveTime(n_num) - d - data[num - 1]->s_t;
	}
	return data[num - 1]->l_t < t ? data[num - 1]->l_t : t;
}

void Problem::computeOrder()
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
				int temp = order[j - 1];
				order[j - 1] = order[j];
				order[j] = temp;
			}
			else
			{
				break;
			}
		}
	}
}

double Problem::computeMinimalDelay()
{
	double num = 0;

	//r表示节点数最少的路径，首先找到路径r
	int r = 0;
	for (int i = 1; i < routeSet.size(); ++i)
	{
		if (routeSet[i]->size < routeSet[r]->size)
		{
			r = i;
		}
	}

	Node* tmp = routeSet[r]->head;
	while (tmp)
	{
		double t_num = -1;
		for (int i = 0; i < routeSet.size(); ++i)
		{
			//对于不是当前路径的路径
			if (i != r)
			{
				//无法加入当前路径
				if (tmp->d + routeSet[i]->demand > DEMAND)
				{
					continue;
				}
				else
				{
					//尝试将节点tmp在路径routeSet[i]上重定位
					double u = tmp->num;
					int m = 1, n;
					Node* p = routeSet[i]->head;
					while (p)
					{
						if (p)
							n = p->num;
						else
							n = 1;

						//首先判断在n前面插入会不会造成u以及u后面的节点不可行
						double dis_mu;
						if (m > u)
							dis_mu = dis[m - 2][u - 1];
						else
							dis_mu = dis[u - 2][m - 1];

						double dis_un;
						if (n > u)
							dis_un = dis[n - 2][u - 1];
						else
							dis_un = dis[u - 2][n - 1];

						double dt_m;
						if (m == 1)
							dt_m = data[0]->e_t;
						else
							dt_m = data[m - 1]->d_t;

						double at_u = dt_m + dis_mu;
						double wt_u = at_u > data[u - 1]->e_t ? 0 : data[u - 1]->e_t - at_u;
						double dt_u = at_u + wt_u + data[u - 1]->s_t;


						//如果插入位置可行
						if (at_u <= data[u - 1]->l_t && judgeInsert(u, dt_u, p))
						{
							t_num = 0;
							break;
						}
						//如果插入位置不可行
						else
						{
							double x = dt_m + dis_mu - data[u - 1]->l_t;
							double n1 = x > 0 ? x : 0;

							x = dt_u + dis_un - computeLatestArriveTime(n);
							double n2 = x > 0 ? x : 0;

							if (t_num == -1 || n1 + n2 < t_num)
								t_num = n1 + n2;
						}

						m = n;
						p = p->next;
					}
				}
			}

		}

		if (t_num = -1)
			num = -1;
		else
			num += t_num;
		tmp = tmp->next;
	}

	return num;
}

void Problem::reversePartialRoute(double i, double j)
{
	Node* pre = data[i - 1];
	Node* tmp = pre->next;
	while (tmp != data[j - 1])
	{
		Node* suc = tmp->next;
		tmp->next = pre;

		pre = tmp;
		tmp = suc;
	}
	tmp->next = pre;
}

void Problem::generate2Exchange(double num)
{
	double i = num;
	int r = searchRoute(i); //客户c所在的路径

	//将边i->i_succ,j->j_succ换成i->j,i_succ->j_succ
	vector<double>* n;
	vector<double>* e;
	double i_succ;  //当前客户节点i的后一个节点
	if (data[i - 1]->next)
		i_succ = data[i - 1]->next->num;
	else
		i_succ = 1;

	//当j在i后面时
	double j, j_succ;   //j和j的后一个节点
	Node* tmp = data[i - 1];
	tmp = tmp->next;  //i后面的节点i+1
	if (tmp) //这个节点不为空时，才能找到合适的j
	{
		tmp = tmp->next; //节点i+2
		while (tmp)
		{
			j = tmp->num;
			if (tmp->next)
				j_succ = tmp->next->num;
			else
				j_succ = 1;

			n = new vector<double>();
			n->push_back(1);
			n->push_back(i);
			n->push_back(i_succ);
			n->push_back(j);
			n->push_back(j_succ);

			n->push_back(r);

			//计算这种情况下的开销
			e = new vector<double>();
			e->push_back(computeFirstEvaluation(n));
			e->push_back(computeSecondEvaluation(n));
			e->push_back(computeThirdEvaluation(n));

			neighbour.push_back(*n);
			evaluation.push_back(*e);

			tmp = tmp->next;
		}
	}

	//当j在i前面时
	j = 1;
	tmp = routeSet[r]->head;
	while (tmp->num != i)
	{
		j_succ = tmp->num;
		n = new vector<double>();
		n->push_back(1);
		n->push_back(j);
		n->push_back(j_succ);
		n->push_back(i);
		n->push_back(i_succ);

		n->push_back(r);

		//计算这种情况下的开销
		e = new vector<double>();
		e->push_back(computeFirstEvaluation(n));
		e->push_back(computeSecondEvaluation(n));
		e->push_back(computeThirdEvaluation(n));

		neighbour.push_back(*n);
		evaluation.push_back(*e);

		j = j_succ;
		tmp = tmp->next;
	}
}

void Problem::generateOrExchange(double num)
{
	double i = num;
	int r = searchRoute(i); //客户c所在的路径

	//将i_pred->i,i->i_succ,j->j_succ换成i_pred->i_succ,j->i,i->j_succ
	//i固定后，i_pred和i_succ都是固定的；改变的是j
	double i_pred, i_succ;
	if (data[i - 1]->next)
		i_succ = data[i - 1]->next->num;
	else
		i_succ = 1;
	Node* pn = routeSet[r]->head;
	if (pn->num == i)
	{
		i_pred = 1;
	}
	else
	{
		while (pn)
		{
			i_pred = pn->num;
			pn = pn->next;
			if (pn->num == i)
				break;
		}
	}

	//j的变化,只要j不等于i_pred,i都可以
	//j可以是从仓库到回到仓库前的最后一个节点
	vector<double>* n;
	vector<double>* e;
	double j, j_succ;
	j = 1;
	Node* tmp = routeSet[r]->head;
	while (true)
	{
		if (tmp)
			j_succ = tmp->num;
		else
			j_succ = 1;

		if (j == i_pred)
		{
			if (i_succ == 1)
				break;
			j = i_succ;
			tmp = data[j - 1]->next;
			continue;
		}

		n = new vector<double>();
		n->push_back(2);
		n->push_back(i_pred);
		n->push_back(i);
		n->push_back(i_succ);
		n->push_back(j);
		n->push_back(j_succ);

		n->push_back(r);

		//计算这种情况下的开销
		e = new vector<double>();
		e->push_back(computeFirstEvaluation(n));
		e->push_back(computeSecondEvaluation(n));
		e->push_back(computeThirdEvaluation(n));
	
		neighbour.push_back(*n);
		evaluation.push_back(*e);

		if (j_succ == 1)
			break;
		j = j_succ;
		tmp = tmp->next;
	}
}

void Problem::generateRelocation(double num)
{
	double i = num;
	int r = searchRoute(i); //客户c所在的路径

	//将i_pred->i,i->i_succ,j->j_succ换成i_pred->i_succ,j->i,i->j_succ
	//i固定后，i_pred和i_succ都是固定的；改变的是j
	double i_pred, i_succ;
	if (data[i - 1]->next)
		i_succ = data[i - 1]->next->num;
	else
		i_succ = 1;
	Node* pn = routeSet[r]->head;
	if (pn->num == i)
	{
		i_pred = 1;
	}
	else
	{
		while (pn)
		{
			i_pred = pn->num;
			pn = pn->next;
			if (pn->num == i)
				break;
		}
	}

	//j可以是其它路径上的节点，从仓库到仓库前的最后一个节点
	vector<double>* n;
	vector<double>* e;
	double j, j_succ;
	for (int m = 0; m < routeSet.size(); ++m)
	{
		//不在i在的路径上
		if (m != r)
		{
			j = 1;
			Node* tmp = routeSet[m]->head;
			while(true)
			{
				if (tmp)
					j_succ = tmp->num;
				else
					j_succ = 1;

				n = new vector<double>();
				n->push_back(3);
				n->push_back(i_pred);
				n->push_back(i);
				n->push_back(i_succ);
				n->push_back(j);
				n->push_back(j_succ);

				n->push_back(r);
				n->push_back(m);

				//计算这种情况下的开销
				e = new vector<double>();
				e->push_back(computeFirstEvaluation(n));
				e->push_back(computeSecondEvaluation(n));
				e->push_back(computeThirdEvaluation(n));

				neighbour.push_back(*n);
				evaluation.push_back(*e);

				if (j_succ == 1)
					break;
				j = j_succ;
				tmp = tmp->next;
			}
		}
	}
}

void Problem::generateExchange(double num)
{
	double i = num;
	int r = searchRoute(i); //客户c所在的路径

	//将i_pred->i,i->i_succ,j->pred->j,j->j_succ换成i_pred->j,j->i_succ,j_pred->i,i->j_succ
	//i固定后，i_pred和i_succ都是固定的；改变的是j
	double i_pred, i_succ;
	if (data[i - 1]->next)
		i_succ = data[i - 1]->next->num;
	else
		i_succ = 1;
	Node* pn = routeSet[r]->head;
	if (pn->num == i)
	{
		i_pred = 1;
	}
	else
	{
		while (pn)
		{
			i_pred = pn->num;
			pn = pn->next;
			if (pn->num == i)
				break;
		}
	}

	//j可以是其它路径上的节点，从仓库后第一个节点到仓库前的最后一个节点
	vector<double>* n;
	vector<double>* e;
	double j_pred, j, j_succ;
	for (int m = 0; m < routeSet.size(); ++m)
	{
		//不在i在的路径上
		if (m != r)
		{

			j_pred = 1;
			Node* tmp = routeSet[m]->head;
			while (tmp)
			{
				j = tmp->num;
				if (tmp->next)
					j_succ = tmp->next->num;
				else
					j_succ = 1;

				n = new vector<double>();
				n->push_back(4);
				n->push_back(i_pred);
				n->push_back(i);
				n->push_back(i_succ);
				n->push_back(j_pred);
				n->push_back(j);
				n->push_back(j_succ);

				n->push_back(r);
				n->push_back(m);

				//计算这种情况下的开销
				e = new vector<double>();
				e->push_back(computeFirstEvaluation(n));
				e->push_back(computeSecondEvaluation(n));
				e->push_back(computeThirdEvaluation(n));

				neighbour.push_back(*n);
				evaluation.push_back(*e);

				j_pred = j;
				tmp = tmp->next;
			}
		}
	}
}

void Problem::generateCrossover(double num)
{
	double i = num;
	int r = searchRoute(i); //客户c所在的路径

	//将i->i_succ,j->j_succ换成j->i_succ,i->j_succ
	//i固定后，i_succ都是固定的；改变的是j
	double i_succ;
	if (data[i - 1]->next)
		i_succ = data[i - 1]->next->num;
	else
		i_succ = 1;

	//j可以是其它路径上的节点，从仓库后第一个节点到仓库前的最后一个节点
	vector<double>* n;
	vector<double>* e;
	double j, j_succ;
	for (int m = 0; m < routeSet.size(); ++m)
	{
		//不在i在的路径上
		if (m != r)
		{
			j = 1;
			Node* tmp = routeSet[m]->head;
			while (tmp)
			{
				if (tmp)
					j_succ = tmp->num;
				else
					j_succ = 1;

				//如果i和j都是最后一个节点，那么这种情况下的邻居与当前情况相同
				//这种邻居不被考虑在内
				if (i_succ != 1 || j_succ != 1)
				{
					n = new vector<double>();
					n->push_back(5);
					n->push_back(i);
					n->push_back(i_succ);
					n->push_back(j);
					n->push_back(j_succ);

					n->push_back(r);
					n->push_back(m);

					//计算这种情况下的开销
					e = new vector<double>();
					e->push_back(computeFirstEvaluation(n));
					e->push_back(computeSecondEvaluation(n));
					e->push_back(computeThirdEvaluation(n));

					neighbour.push_back(*n);
					evaluation.push_back(*e);
				}
				
				if (j_succ == 1)
					break;
				j = j_succ;
				tmp = tmp->next;
			}
		}
	}
}

void Problem::generateNeighbour(int operate, double num)
{
	int o = operate;

	/*清空上一次产生的邻居等信息*/
	vector<vector<double>>().swap(neighbour);
	vector<vector<double>>().swap(evaluation);
	vector<int>().swap(order);
	
	if (o == 1)  //进行2-exchange  在一条路径上
	{
		generate2Exchange(num);
	}
	else if (o == 2)  //进行Or-exchange  在一条路径上
	{
		generateOrExchange(num);
	}
	else if (o == 3) //进行Relocation  在两条路径间
	{
		generateRelocation(num);
	}
	else if (o == 4) //进行Exchange  在两条路径间
	{
		generateExchange(num);
	}
	else if (o == 5) //进行Crossover  在两条路径间
	{
		generateCrossover(num);
	}
}

double Problem::computeFirstEvaluation(vector<double>* n)
{
	
	double x;
	x = routeSet.size();

	//两种可能减少路径数目的操作
	if ((*n)[0] == 3) //relocation邻居
	{
		double r1;
		r1 = (*n)[6];
		if (routeSet[r1]->size == 1)
			x = x - 1;
	}
	else if ((*n)[0] == 5) //crossover邻居
	{
		double i_succ = (*n)[2];
		double j = (*n)[3];
		if (j == 1 && i_succ == 1)
			x = x - 1;
	}

	return x;
}

double Problem::computeSecondEvaluation(vector<double>* n)
{
	double r1, r2;
	double sum = 0;
	if ((*n)[0] == 0 || (*n)[0] == 1 || (*n)[0] == 2 || (*n)[0] == 4)
	{
		//每条路径上的节点数目并没有发生改变	
		for (int m = 0; m < routeSet.size(); ++m)
		{
			sum += pow(routeSet[m]->size, 2);
		}
	}
	else if ((*n)[0] == 3) //relocation邻居
	{
		r1 = (*n)[6];
		r2 = (*n)[7];

		for (int n = 0; n < routeSet.size(); ++n)
		{
			if (n == r1)
				sum += pow(routeSet[n]->size - 1, 2);
			else if (n == r2)
				sum += pow(routeSet[n]->size + 1, 2);
			else
				sum += pow(routeSet[n]->size, 2);
		}
	}
	else if ((*n)[0] == 5) //crossover邻居
	{
		double i = (*n)[1];
		double i_succ = (*n)[2];
		double j = (*n)[3];
		double j_succ = (*n)[4];

		r1 = (*n)[5];
		r2 = (*n)[6];

		double r1_1, r1_2 = 0;
		double r2_1, r2_2 = 0;
		for (Node* tmp = data[i - 1]->next; tmp != NULL; tmp = tmp->next)
		{
			r1_2++;
		}
		r1_1 = routeSet[r1]->size - r1_2;

		if (j == 1)
		{
			r2_2 = routeSet[r2]->size;
		}
		else
		{
			for (Node* tmp = data[j - 1]->next; tmp != NULL; tmp = tmp->next)
			{
				r2_2++;
			}
		}
		r2_1 = routeSet[r2]->size - r2_2;

		for (int n = 0; n < routeSet.size(); ++n)
		{
			if (n == r1)
				sum += pow(r1_1 + r2_2, 2);
			else if (n == r2)
				sum += pow(r2_1 + r1_2, 2);
			else
				sum += pow(routeSet[n]->size, 2);
		}
	}

	return -sum;
}

double Problem::computeThirdEvaluation(vector<double>* n)
{
	double r, r1, r2;
	double i_pred, i, i_succ;
	double j_pred, j, j_succ;
	double x;
	if((*n)[0] == 0)
	{
		x = computeMinimalDelay();
	}
	else if ((*n)[0] == 1) //2-exchange邻居
	{
		//i在节点j前面
		i = (*n)[1];
		i_succ = (*n)[2];
		j = (*n)[3];
		j_succ = (*n)[4];

		r = (*n)[5];

		//进行交换
		if (i == 1)
			routeSet[r]->head = data[j - 1];
		else
			data[i - 1]->next = data[j - 1];

		if (j_succ == 1)
			data[i_succ - 1]->next = NULL;
		else
			data[i_succ - 1]->next = data[j_succ - 1];
		//将i_succ->...->j变为j->...->i_succ
		reversePartialRoute(i_succ, j);

		//计算第三个评估标准
		x = computeMinimalDelay();

		//还原交换
		if (i == 1)
			routeSet[r]->head = data[i_succ - 1];
		else
			data[i - 1]->next = data[i_succ - 1];

		if (j_succ == 1)
			data[j - 1]->next = NULL;
		else
			data[j - 1]->next = data[j_succ - 1];
		//将j->...->i_succ变为i_succ->...->j
		reversePartialRoute(j, i_succ);
	}
	else if((*n)[0] == 2) //or-exchange邻居
	{
		i_pred = (*n)[1];
		i = (*n)[2];
		i_succ = (*n)[3];
		j = (*n)[4];
		j_succ = (*n)[5];

		r = (*n)[6];

		//进行交换
		//i_pred==1和i_succ不可能同时出现，因为那样的话就没有j了
		if (i_pred == 1)  //如果i是仓库后第一个节点
			routeSet[r]->head = data[i_succ - 1];
		else if (i_succ == 1) //如果i是仓库前的最后一个节点
			data[i_pred - 1]->next = NULL;
		else
			data[i_pred - 1]->next = data[i_succ - 1];

		if (j == 1)  //如果j是仓库
			routeSet[r]->head = data[i - 1];
		else
			data[j - 1]->next = data[i - 1];

		if (j_succ == 1)  //如果j_succ是仓库
			data[i - 1]->next = NULL;
		else
			data[i - 1]->next = data[j_succ - 1];

		x = computeMinimalDelay();

		//还原交换
		if (i_pred == 1)
			routeSet[r]->head = data[i - 1];
		else
			data[i_pred - 1]->next = data[i - 1];

		if (i_succ == 1)
			data[i - 1]->next = NULL;
		else
			data[i - 1]->next = data[i_succ - 1];

		if (j == 1)
			routeSet[r]->head = data[j_succ - 1];
		else if (j_succ == 1)
			data[j - 1]->next = NULL;
		else
			data[j - 1]->next = data[j_succ - 1];

	}
	else if ((*n)[0] == 3) //relocation邻居
	{
		i_pred = (*n)[1];
		i = (*n)[2];
		i_succ = (*n)[3];
		j = (*n)[4];
		j_succ = (*n)[5];

		r1 = (*n)[6];
		r2 = (*n)[7];

		//进行交换
		if (i_pred == 1 && i_succ == 1)
			routeSet[r1]->head = NULL;
		else if (i_pred == 1 && i_succ != 1)
			routeSet[r1]->head = data[i_succ - 1];
		else if (i_succ == 1 && i_pred != 1)
			data[i_pred - 1]->next = NULL;
		else
			data[i_pred - 1]->next = data[i_succ - 1];

		if (j == 1)
			routeSet[r2]->head = data[i - 1];
		else
			data[j - 1]->next = data[i - 1];

		if (j_succ == 1)
			data[i - 1]->next = NULL;
		else
			data[i - 1]->next = data[j_succ - 1];

		routeSet[r1]->size--;
		routeSet[r2]->size++;

		x = computeMinimalDelay();

		//还原交换
		if (i_pred == 1)
			routeSet[r1]->head = data[i - 1];
		else
			data[i_pred - 1]->next = data[i - 1];

		if (i_succ == 1)
			data[i - 1]->next = NULL;
		else
			data[i - 1]->next = data[i_succ - 1];

		//j==1和j_succ==1不可能同时出现，因为路径r2不可能为空
		if (j == 1)
			routeSet[r2]->head = data[j_succ - 1];
		else if (j_succ == 1)
			data[j - 1]->next = NULL;
		else
			data[j - 1]->next = data[j_succ - 1];

		routeSet[r2]->size--;
		routeSet[r1]->size++;
	}
	else if ((*n)[0] == 4)  //exchange邻居
	{
		i_pred = (*n)[1];
		i = (*n)[2];
		i_succ = (*n)[3];
		j_pred = (*n)[4];
		j = (*n)[5];
		j_succ = (*n)[6];

		r1 = (*n)[7];
		r2 = (*n)[8];

		//进行交换
		if (i_pred == 1)   //i是第一个客户节点
			routeSet[r1]->head = data[j - 1];
		else
			data[i_pred - 1]->next = data[j - 1];

		if (i_succ == 1)  //i是最后一个客户节点
			data[j - 1]->next = NULL;
		else
			data[j - 1]->next = data[i_succ - 1];

		if (j_pred == 1)  //j是第一个客户节点
			routeSet[r2]->head = data[i - 1];
		else
			data[j_pred - 1]->next = data[i - 1];

		if (j_succ == 1)   //j是最后一个客户节点
			data[i - 1]->next = NULL;
		else
			data[i - 1]->next = data[j_succ - 1];

		x = computeMinimalDelay();

		//还原交换
		if (i_pred == 1)   //i是第一个客户节点
			routeSet[r1]->head = data[i - 1];
		else
			data[i_pred - 1]->next = data[i - 1];

		if (i_succ == 1)  //i是最后一个客户节点
			data[i - 1]->next = NULL;
		else
			data[i - 1]->next = data[i_succ - 1];

		if (j_pred == 1) //j是第一个客户节点
			routeSet[r2]->head = data[j - 1];
		else
			data[j_pred - 1]->next = data[j - 1];

		if (j_succ == 1)  //j是最后一个客户节点
			data[j - 1]->next = NULL;
		else
			data[j - 1]->next = data[j_succ - 1];

	}
	else if ((*n)[0] == 5) //crossover邻居
	{
		i = (*n)[1];
		i_succ = (*n)[2];
		j = (*n)[3];
		j_succ = (*n)[4];

		r1 = (*n)[5];
		r2 = (*n)[6];

		//进行交换
		//i_succ和j_succ不能同时是仓库，因为这样的话，邻居跟自己是一样的
		if (j_succ == 1)  //j是最后一个客户节点
			data[i - 1]->next = NULL;
		else
			data[i - 1]->next = data[j_succ - 1];

		if (i_succ == 1 && j == 1)
			routeSet[r2]->head = NULL;
		else if (i_succ == 1 && j != 1)  //i是最后一个客户节点
			data[j - 1]->next = NULL;
		else if (j == 1 && i_succ != 1)
			routeSet[r2]->head = data[i_succ - 1];
		else
			data[j - 1]->next = data[i_succ - 1];

		int r1_1, r1_2 = 0;
		int r2_1, r2_2 = 0;
		for (Node* tmp = data[i - 1]->next; tmp != NULL; tmp = tmp->next)
		{
			r1_2++;
		}
		r1_1 = routeSet[r1]->size - r1_2;

		if (j == 1)
		{
			r2_2 = routeSet[r2]->size;
		}
		else
		{
			for (Node* tmp = data[j - 1]->next; tmp != NULL; tmp = tmp->next)
			{
				r2_2++;
			}
		}
		r2_1 = routeSet[r2]->size - r2_2;

		int r1_size = routeSet[r1]->size;
		int r2_size = routeSet[r2]->size;
		routeSet[r1]->size = r1_1 + r2_2;
		routeSet[r2]->size = r2_1 + r1_2;

		x = computeMinimalDelay();

		//还原交换
		//j==1和j_succ==1不可能同时出现，因为r2不可能为空
		if (j == 1)
			routeSet[r2]->head = data[j_succ - 1];
		else if(j_succ == 1)
			data[j - 1]->next = NULL;
		else
			data[j - 1]->next = data[j_succ - 1];

		if (i_succ == 1)   //i是最后一个客户节点
			data[i - 1]->next = NULL;
		else
			data[i - 1]->next = data[i_succ - 1];

		routeSet[r1]->size = r1_size;
		routeSet[r2]->size = r2_size;
	}

	return x;
}

void Problem::changeToNeighbour(int num)
{
	double r, r1, r2;
	double i_pred, i, i_succ;
	double j_pred, j, j_succ;
	if (neighbour[num][0] == 1)
	{
		i = neighbour[num][1];
		i_succ = neighbour[num][2];
		j = neighbour[num][3];
		j_succ = neighbour[num][4];

		r = neighbour[num][5];

		//进行交换
		if (j == 1)
			routeSet[r]->head = data[i - 1];
		else
			data[j - 1]->next = data[i - 1];

		if (i_succ == 1)
			data[j_succ - 1]->next = NULL;
		else
			data[j_succ - 1]->next = data[i_succ - 1];

		reversePartialRoute(j_succ, i);
	}
	else if(neighbour[num][0] == 2)
	{
		i_pred = neighbour[num][1];
		i = neighbour[num][2];
		i_succ = neighbour[num][3];
		j = neighbour[num][4];
		j_succ = neighbour[num][5];

		r = neighbour[num][6];

		//进行交换
		if (i_pred == 1)  //如果i是仓库后第一个节点
			routeSet[r]->head = data[i_succ - 1];
		else if (i_succ == 1) //如果i是仓库前的最后一个节点
			data[i_pred - 1]->next = NULL;
		else
			data[i_pred - 1]->next = data[i_succ - 1];

		if (j == 1)  //如果j是仓库
			routeSet[r]->head = data[i - 1];
		else
			data[j - 1]->next = data[i - 1];

		if (j_succ == 1)  //如果j_succ是仓库
			data[i - 1]->next = NULL;
		else
			data[i - 1]->next = data[j_succ - 1];
	}
	else if (neighbour[num][0] == 3)
	{
		i_pred = neighbour[num][1];
		i = neighbour[num][2];
		i_succ = neighbour[num][3];
		j = neighbour[num][4];
		j_succ = neighbour[num][5];

		r1 = neighbour[num][6];
		r2 = neighbour[num][7];

		//进行交换
		if (i_pred == 1 && i_succ == 1)
			routeSet[r1]->head = NULL;
		else if (i_pred == 1)
			routeSet[r1]->head = data[i_succ - 1];
		else if (i_succ == 1)
			data[i_pred - 1]->next = NULL;
		else
			data[i_pred - 1]->next = data[i_succ - 1];

		data[j - 1]->next = data[i - 1];

		if (j_succ == 1)
			data[i - 1]->next = NULL;
		else
			data[i - 1]->next = data[j_succ - 1];
	}
	else if (neighbour[num][0] == 4)
	{
		i_pred = neighbour[num][1];
		i = neighbour[num][2];
		i_succ = neighbour[num][3];
		j_pred = neighbour[num][4];
		j = neighbour[num][5];
		j_succ = neighbour[num][6];

		r1 = neighbour[num][7];
		r2 = neighbour[num][8];

		//进行交换
		if (i_pred == 1)   //i是第一个客户节点
			routeSet[r1]->head = data[j - 1];
		else
			data[i_pred - 1]->next = data[j - 1];

		if (i_succ == 1)  //i是最后一个客户节点
			data[j - 1]->next = NULL;
		else
			data[j - 1]->next = data[i_succ - 1];

		if (j_pred == 1)  //j是第一个客户节点
			routeSet[r2]->head = data[i - 1];
		else
			data[j_pred - 1]->next = data[i - 1];

		if (j_succ == 1)   //j是最后一个客户节点
			data[i - 1]->next = NULL;
		else
			data[i - 1]->next = data[j_succ - 1];
	}
	else if (neighbour[num][0] == 5)
	{
		i = neighbour[num][1];
		i_succ = neighbour[num][2];
		j = neighbour[num][3];
		j_succ = neighbour[num][4];

		//进行交换
		if (j_succ == 1)  //j是最后一个客户节点
			data[i - 1]->next = NULL;
		else
			data[i - 1]->next = data[j_succ - 1];

		if (i_succ == 1)  //i是最后一个客户节点
			data[j - 1]->next = NULL;
		else
			data[j - 1]->next = data[i_succ - 1];
	}
}

void Problem::minimizeRoute()
{
	double t;
	int o;  //操作：1.Two-exchange 2.Or-exchange 3.Relocation 4.Exchange 5.Crossover
	double c; //随机产生的一个客户编号

	//第一个数为0时表示解决方案tmp_s等于当前解决方案，不做任何变化
	//1：做一次2-exchange 2：做一次or-exchange 3：做一次relocation 4：做一次exchange 5：做一次crossover
	vector<double>* tmp_s = new vector<double>();
	tmp_s->push_back(0);

	//获得当前时间
	time_t iTime = time(0);

	while (difftime(time(0), iTime) < 10)
	{
		t = T;
		while (difftime(time(0), iTime) < 10 && t > EPS)
		{
			for (int i = 1; i < maxIt; i++)
			{
				o = rand() % 6;
				c = (double)(rand() % 100) + 1;
				//产生邻居并计算邻居的评价值
				generateNeighbour(o, c);
				//产生邻居的排序
				computeOrder();

				double v_1 = computeFirstEvaluation(tmp_s);
				double v_2 = computeSecondEvaluation(tmp_s);
				double v_3 = computeThirdEvaluation(tmp_s);
				if (evaluation[order[0]][0] > v_1 && evaluation[order[0]][1] > v_2 && evaluation[order[0]][2] > v_3)
				{
					int r = ceil((rand() / double(RAND_MAX)) * order.size());
					double diff = 0.5 * (evaluation[order[r]][0] - v_1) + 0.3 * (evaluation[order[r]][1] - v_2) + 0.2 * (evaluation[order[r]][2] - v_3);
					if (evaluation[order[r]][0] > v_1 && evaluation[order[r]][1] > v_2 && evaluation[order[r]][2] > v_3)
					{
						if (rand() % 100 / (double)100 <= exp(diff / t))
						{
							vector<double>().swap(*tmp_s);
							for (int m = 0; m < neighbour[order[r]].size(); ++m)
							{
								tmp_s->push_back(neighbour[order[r]][m]);
							}
						}
					}
					else
					{
						vector<double>().swap(*tmp_s);
						for (int m = 0; m < neighbour[order[r]].size(); ++m)
						{
							tmp_s->push_back(neighbour[order[r]][m]);
						}
					}
				}
				else
				{
					changeToNeighbour(order[0]);

					vector<double>().swap(*tmp_s);
					for (int m = 0; m < neighbour[order[0]].size(); ++m)
					{
						tmp_s->push_back(neighbour[order[0]][m]);
					}
				}
			}
			t = R * t;
		}
	}
}

