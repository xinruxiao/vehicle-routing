#include"Solution.h"

void Solution::doCopy(Solution* s)
{
	this->allLength = s->allLength;
	this->allTime = s->allTime;

	Route* tmp_r;

	Node* h;
	Node* t;
	Node* n;
	for (int i = 0; i < s->routeSet.size(); ++i)
	{
		tmp_r = new Route();

		h = new Node();
		*h = *(s->routeSet[i]->head);
		h->pred = NULL;
		Node* n_pred = h;

		Node* temp;
		for (temp = s->routeSet[i]->head->succ; temp != s->routeSet[i]->tail; temp = temp->succ)
		{
			n = new Node();
			*n = *temp;

			n_pred->succ = n;
			n->pred = n_pred;

			n_pred = n;
		}
		
		t = new Node();
		*t = *temp;
		t->succ = NULL;

		n_pred->succ = t;
		t->pred = n_pred;

		tmp_r->head = h;
		tmp_r->tail = t;
		tmp_r->demand = s->routeSet[i]->demand;
		tmp_r->size = s->routeSet[i]->size;

		routeSet.push_back(tmp_r);
	}
}

bool Solution::isRegular(Data* d)
{
	for (int i = 0; i < routeSet.size(); i++)
	{
		if (!routeSet[i]->isFeasibleGC(routeSet[i]->head->num, routeSet[i]->head->d_t, routeSet[i]->head->succ, d))
		{
			cout << "该解决方案不合格................" << endl;
			cout << "路径" << i + 1 << "不合格............" << endl;
			return false;
		}
	}

	cout << "该解决方案合格................" << endl;
	return true;
}

void Solution::search(double u, int* route_u, Node** node_u)
{
	Route* tmp_r;
	Node* tmp_n;
	for (int i = 0; i < routeSet.size(); ++i)
	{
		tmp_r = routeSet[i];
		tmp_n = tmp_r->head->succ;
		while (tmp_n != tmp_r->tail)
		{
			if (tmp_n->num == u)
			{
				*route_u = i;
				*node_u = tmp_n;
				break;
			}
			tmp_n = tmp_n->succ;
		}
	}
}

void Solution::printSolution()
{
	cout << "allLength = " << allLength << endl;
	cout << "allTime = " << allTime << endl;
	int size = routeSet.size();
	for (int i = 0; i < size; ++i)
	{
		cout << "路径" << i + 1 << "：";
		routeSet[i]->printRoute();
	}
}

void Solution::computeLength(Data* data)
{
	allLength = 0;
	for (int i = 0; i < routeSet.size(); ++i)
	{
		Node* tmp = routeSet[i]->head;
		double m;
		double n;
		double dis_mn;
		while (tmp != routeSet[i]->tail)
		{
			m = tmp->num;
			n = tmp->succ->num;
			dis_mn = data->distance(m, n);
			allLength += dis_mn;

			tmp = tmp->succ;
		}
	}
}

void Solution::computeTime()
{
	allTime = 0;
	for (int i = 0; i < routeSet.size(); ++i)
	{
		allTime += routeSet[i]->tail->a_t - routeSet[i]->head->d_t;
	}
}

void Solution::do2Exchange(Route* r, Node* i, Node* i_succ, Node* j, Node* j_succ)
{
	r->reversePartialRoute(i_succ, j);

	i->succ = j;
	j->pred = i;

	i_succ->succ = j_succ;
	j_succ->pred = i_succ;

}

void Solution::restore2Exchange(Route* r, Node* i, Node* i_succ, Node* j, Node* j_succ)
{
	//因为已经反转了，所以这里j在i_succ前面
	r->reversePartialRoute(j, i_succ);

	i->succ = i_succ;
	i_succ->pred = i;

	j->succ = j_succ;
	j_succ->pred = j;

}

void Solution::doOrExchange(Node* i_pred, Node* i, Node* i_succ, Node* j, Node* j_succ)
{
	i_pred->succ = i_succ;
	i_succ->pred = i_pred;

	j->succ = i;
	i->pred = j;

	i->succ = j_succ;
	j_succ->pred = i;
}

void Solution::restoreOrExchange(Node* i_pred, Node* i, Node* i_succ, Node* j, Node* j_succ)
{
	i_pred->succ = i;
	i->pred = i_pred;

	i->succ = i_succ;
	i_succ->pred = i;

	j->succ = j_succ;
	j_succ->pred = j;
}

void Solution::doRelocation(Node* i_pred, Node* i, Node* i_succ, Node* j, Node* j_succ)
{
	i_pred->succ = i_succ;
	i_succ->pred = i_pred;

	j->succ = i;
	i->pred = j;

	i->succ = j_succ;
	j_succ->pred = i;
}

void Solution::restoreRelocation(Node* i_pred, Node* i, Node* i_succ, Node* j, Node* j_succ)
{
	i_pred->succ = i;
	i->pred = i_pred;

	i->succ = i_succ;
	i_succ->pred = i;

	j->succ = j_succ;
	j_succ->pred = j;
}

void Solution::doExchange(Node* i_pred, Node* i, Node* i_succ, Node* j_pred, Node* j, Node* j_succ)
{
	i_pred->succ = j;
	j->pred = i_pred;

	j->succ = i_succ;
	i_succ->pred = j;

	j_pred->succ = i;
	i->pred = j_pred;

	i->succ = j_succ;
	j_succ->pred = i;
}

void Solution::restoreExchange(Node* i_pred, Node* i, Node* i_succ, Node* j_pred, Node* j, Node* j_succ)
{
	i_pred->succ = i;
	i->pred = i_pred;

	i->succ = i_succ;
	i_succ->pred = i;

	j_pred->succ = j;
	j->pred = j_pred;

	j->succ = j_succ;
	j_succ->pred = j;
}

void Solution::doCrossover(Node* i, Node* i_succ, Node* j, Node* j_succ)
{
	i->succ = j_succ;
	j_succ->pred = i;

	j->succ = i_succ;
	i_succ->pred = j;
}

void Solution::restoreCrossover(Node* i, Node* i_succ, Node* j, Node* j_succ)
{
	i->succ = i_succ;
	i_succ->pred = i;

	j->succ = j_succ;
	j_succ->pred = j;
}