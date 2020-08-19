#include"Route.h"


Route::Route(double u, Data* d)
{
	vector<Customer*>* cus = &(d->data);

	double d_1u = d->distance(1, u);

	/*出发的仓库节点*/
	Node* h = new Node();
	h->num = 1;
	h->d_t = (*cus)[0]->e_t;

	/*经过的节点num*/
	Node* tmp = new Node();
	tmp->num = u;

	tmp->a_t = h->d_t + d_1u;
	tmp->w_t = tmp->a_t > (*cus)[u - 1]->e_t ? 0 : (*cus)[u - 1]->e_t - tmp->a_t;
	tmp->b_t = tmp->a_t + tmp->w_t;
	tmp->d_t = tmp->b_t + (*cus)[u - 1]->s_t;
	
	/*回到的仓库节点*/
	Node* t = new Node();
	t->num = 1;
	t->a_t = tmp->d_t + d_1u;

	h->pred = NULL;
	h->succ = tmp;

	tmp->pred = h;
	tmp->succ = t;

	t->pred = tmp;
	t->succ = NULL;

	//除仓库节点外的节点数
	size = 1;
	this->head = h;
	this->tail = t;
	this->demand = (*cus)[u - 1]->d;
}

//将路径r添加到当前路径后面是否可行
bool Route::isFeasibleAddRoute(Route* r, Data* d)
{
	/*判断前面一条路径是否可行*/
	vector<Customer*>* cus = &(d->data);

	double cus_i = this->head->num;
	double dt_i = this->head->d_t;

	double cus_j;
	double dis_ij;
	double at_j, wt_j, bt_j, dt_j;

	Node* tmp = this->head->succ;

	//判断从tmp到tail前的节点是否可行
	while (tmp != this->tail)
	{
		cus_j = tmp->num;
		dis_ij = d->distance(cus_i, cus_j);

		at_j = dt_i + dis_ij;
		if (at_j > (*cus)[cus_j - 1]->l_t)
			return false;

		wt_j = at_j > (*cus)[cus_j - 1]->e_t ? 0 : (*cus)[cus_j - 1]->e_t - at_j;
		bt_j = at_j + wt_j;
		dt_j = bt_j + (*cus)[cus_j - 1]->s_t;

		cus_i = cus_j;
		dt_i = dt_j;
		tmp = tmp->succ;
	}

	if (!r->isFeasibleGC(cus_i, dt_i, r->head->succ, d))
		return false;

	return true;
}

bool Route::isFeasible(double u, double dt_u, Node* p_j, Data* d)
{
	vector<Customer*>* cus = &(d->data);

	double cus_i = u;
	double dt_i = dt_u;

	double cus_j;
	double dis_ij;
	double at_j, wt_j, bt_j, dt_j;

	Node* tmp = p_j;

	if (tmp == this->tail && cus_i == 1)
		return true;

	//判断从p_j到tail前的节点是否可行
	while (tmp != this->tail)
	{
		cus_j = tmp->num;
		dis_ij = d->distance(cus_i, cus_j);

		at_j = dt_i + dis_ij;
		if (at_j > (*cus)[cus_j - 1]->l_t)
			return false;
		else if (at_j - tmp->a_t <= tmp->w_t)
			return true;

		wt_j = at_j > (*cus)[cus_j - 1]->e_t ? 0 : (*cus)[cus_j - 1]->e_t - at_j;
		bt_j = at_j + wt_j;
		dt_j = bt_j + (*cus)[cus_j - 1]->s_t;

		cus_i = cus_j;
		dt_i = dt_j;
		tmp = tmp->succ;
	}

	//判断tail是否违背时间窗
	at_j = dt_i + d->distance(cus_i, 1);
	if (at_j > (*cus)[0]->l_t)
		return false;

	return true;
}

double Route::computeNewAT(double u, double dt_u, Node* p_j, Data* d)
{
	vector<Customer*>* cus = &(d->data);

	double cus_i = u;
	double dt_i = dt_u;

	double cus_j;
	double dis_ij;
	double at_j, wt_j, bt_j, dt_j;

	Node* tmp = p_j;

	//判断从p_j到tail前的节点是否可行
	while (tmp != this->tail)
	{
		cus_j = tmp->num;
		dis_ij = d->distance(cus_i, cus_j);

		at_j = dt_i + dis_ij;
		wt_j = at_j > (*cus)[cus_j - 1]->e_t ? 0 : (*cus)[cus_j - 1]->e_t - at_j;
		bt_j = at_j + wt_j;
		dt_j = bt_j + (*cus)[cus_j - 1]->s_t;

		cus_i = cus_j;
		dt_i = dt_j;
		tmp = tmp->succ;
	}

	double at_depot = dt_i + d->distance(cus_i, 1);

	return at_depot;
}

bool Route::isFeasibleGC(double u, double dt_u, Node* p_j, Data* d)
{
	vector<Customer*>* cus = &(d->data);

	double cus_i = u;
	double dt_i = dt_u;

	double cus_j;
	double dis_ij;
	double at_j, wt_j, bt_j, dt_j;

	Node* tmp = p_j;

	//判断从p_j到tail前的节点是否可行
	while (tmp != this->tail)
	{
		cus_j = tmp->num;
		dis_ij = d->distance(cus_i, cus_j);

		at_j = dt_i + dis_ij;
		if (at_j > (*cus)[cus_j - 1]->l_t)
			return false;

		wt_j = at_j > (*cus)[cus_j - 1]->e_t ? 0 : (*cus)[cus_j - 1]->e_t - at_j;
		bt_j = at_j + wt_j;
		dt_j = bt_j + (*cus)[cus_j - 1]->s_t;

		cus_i = cus_j;
		dt_i = dt_j;
		tmp = tmp->succ;
	}

	//判断tail是否违背时间窗
	at_j = dt_i + d->distance(cus_i, 1);
	if (at_j > (*cus)[0]->l_t)
		return false;

	return true;
}

void Route::reverseRoute()
{
	if (this->size == 1)
		return;

	Node* tmp_m = this->head->succ;
	Node* tmp_n = tmp_m->succ;
	while (tmp_n != this->tail)
	{
		Node* tmp = tmp_n->succ;

		tmp_n->succ = tmp_m;
		tmp_m->pred = tmp_n;

		tmp_m = tmp_n;
		tmp_n = tmp;
	}

	Node* head_succ = this->head->succ;
	Node* tail_pred = tmp_m;

	this->head->succ = tail_pred;
	tail_pred->pred = this->head;

	this->tail->pred = head_succ;
	head_succ->succ = this->tail;
}

void Route::reversePartialRoute(Node* pi, Node* pj)
{
	Node* tmp_m = pi;
	Node* tmp_n = tmp_m->succ;
	Node* tmp;
	while (true)
	{
		tmp = tmp_n->succ;

		tmp_n->succ = tmp_m;
		tmp_m->pred = tmp_n;

		if (tmp_n == pj)
			break;

		tmp_m = tmp_n;
		tmp_n = tmp;
	}
}

void Route::updateRoute(Node* p_j, Data* d)
{
	vector<Customer*>* cus = &(d->data);
	double cus_i = p_j->pred->num;
	double dt_i = p_j->pred->d_t;

	double cus_j;
	double dis_ij;
	double at_j, wt_j, bt_j, dt_j;

	Node* tmp = p_j;

	//更新从p_j到tail前的节点的信息
	while (tmp != this->tail)
	{
		cus_j = tmp->num;
		dis_ij = d->distance(cus_i, cus_j);

		at_j = dt_i + dis_ij;
		wt_j = at_j > (*cus)[cus_j - 1]->e_t ? 0 : (*cus)[cus_j - 1]->e_t - at_j;
		bt_j = at_j + wt_j;
		dt_j = bt_j + (*cus)[cus_j - 1]->s_t;

		tmp->a_t = at_j;
		tmp->w_t = wt_j;
		tmp->b_t = bt_j;
		tmp->d_t = dt_j;

		cus_i = cus_j;
		dt_i = dt_j;
		tmp = tmp->succ;
	}

	//更新tail的信息
	this->tail->a_t = dt_i + d->distance(cus_i, 1);
}

void Route::mergeRoute(Route* p_j, Data* d)
{
	this->demand += p_j->demand;
	this->size += p_j->size;

	Node* tmp_i = this->tail->pred;
	Node* tmp_j = p_j->head->succ;

	tmp_i->succ = tmp_j;
	tmp_j->pred = tmp_i;

	this->tail = p_j->tail;
}

void Route::printRoute()
{
	Node* tmp = head;
	while (tmp != NULL)
	{
		cout << tmp->num;
		tmp = tmp->succ;
		if (tmp)
			cout << "->";
	}
	cout << endl;
}

Node* Route::searchNode(double num)
{
	Node* tmp = this->head->succ;
	while (true)
	{
		if (tmp->num == num)
			return tmp;
		tmp = tmp->succ;
	}
}