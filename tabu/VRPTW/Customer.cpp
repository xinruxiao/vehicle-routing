#include "Customer.h"

Customer::Customer(vector<double>* data)
{
	auto it = data->begin();
	num = *it;
	it++;

	x = *it;
	it++;

	y = *it;
	it++;

	d = *it;
	it++;

	e_t = *it;
	it++;

	l_t = *it;
	it++;

	s_t = *it;

	box_x = 0;
	box_y = 0;
}

void Customer::print()
{
	cout << this->num << "  " << this->x << "  " << this->y << "  " << this->d << "  " << this->e_t
		<< "  " << this->l_t << "  " << this->s_t << endl;
}