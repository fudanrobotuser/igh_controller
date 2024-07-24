#include <iostream>
#include <iostream>
#include <algorithm>
#include <streambuf>
#include <sstream>
#include <stdlib.h>
#include <fstream>
#include <iostream>

#define REAL double
#define Vel 1
#define LSPB_V 1.5
REAL lspb_plan(REAL V, REAL dl) // V:(1,2)   dl:[0,1]
{

	REAL L = -1.0f;
	REAL tb = 1.0f;
	REAL a = 0.0f;

	if (V >= 2.0f || V <= 1.0f)
	{
		V = 1.5f;
	}

	if (dl <= 1.0f && dl >= 0.0f)
	{
		tb = (-1 + V) / V;
		a = V / tb;

		if (dl <= tb)
		{
			// initial blend
			L = a * 0.5f * dl * dl;
		}
		else if (dl <= (1 - tb))
		{
			// linear motion
			L = (1 - V) * 0.5f + V * dl;
		}
		else
		{
			// final blend
			L = 1 - a / 2 + a * dl - 0.5f * a * dl * dl;
		}
	}
	/*REAL L =0.f;
	L = 6 * dl * dl * dl * dl * dl - 15 * dl * dl * dl * dl+10 *dl * dl * dl;*/

	return L;
}

#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

// 计算五次多项式的系数

REAL norm_array(REAL *C, int n)
{
	REAL sum = 0.f;
	int i = 0;
	for (i = 0; i < n; i++)
	{
		sum += C[i] * C[i];
	}
	return sqrtf(sum);
}
int Trajectory(REAL Target[7], REAL Current[7], REAL V)
{

	/*REAL norm = 0;
	REAL initial_joint[7] = { 0 };
	REAL dl = 0;
	REAL current_joint[7] = { 0 };
	for (int i = 0; i < 7; i++)
	{
		initial_joint[i] = Current[i];
		current_joint[i] = Current[i];
	}
	REAL current_norm = 0;
	REAL Difference_ini[7] = { 0 };
	REAL Difference[7] = { 0 };
	REAL L_index = 0;
	REAL Current_L = 0;
	for (int i = 0; i < 7; i++)
	{
		Difference_ini[i] = Target[i] - Current[i];
	}
	REAL Value[2] = { 0 };
	Value[0]=*std::min_element(Difference_ini, Difference_ini + 7);
	Value[1] = *std::max_element(Difference_ini, Difference_ini + 7);
	REAL  Sum_time = 0;
	if (V != 0)
	{
	  Sum_time = Value[1] / V*1000;
	}
	else
	{
		std::cout << " The speed is zero.  " << std::endl;
	}
	int count = 0;

	while (1)
	{



		if (dl > 1.0f)
		{
			dl = 1.0f;
		}
		else if (dl == 1.0f)
		{
			std::cout << count << std::endl;
			break;
		}
		else
		{
			Current_L = lspb_plan(LSPB_V, dl);

			for (int i = 0; i < 7; i++)
			{
				current_joint[i] = initial_joint[i] + Current_L * Difference_ini[i];
				std::cout << "  joint:  " << i << "  === " << current_joint[i] << "      ";
			}
			std::cout << std::endl;
			dl = dl + 1 / Sum_time;
			count++;
		}
	}*/

	return 0;
}

int main()
{

	REAL Current[7] = {0};
	REAL V = 2;
	REAL Target[7] = {1, 2, 3, 4, 5, 6, 9};
	REAL norm = 0;
	REAL initial_joint[7] = {0};
	REAL dl = 0;
	REAL current_joint[7] = {0};
	for (int i = 0; i < 7; i++)
	{
		initial_joint[i] = Current[i];
		current_joint[i] = Current[i];
	}
	REAL current_norm = 0;
	REAL Difference_ini[7] = {0};
	REAL Difference[7] = {0};
	REAL L_index = 0;
	REAL Current_L = 0;
	for (int i = 0; i < 7; i++)
	{
		Difference_ini[i] = Target[i] - Current[i];
	}
	REAL Value[2] = {0};
	Value[0] = *std::min_element(Difference_ini, Difference_ini + 7);
	Value[1] = *std::max_element(Difference_ini, Difference_ini + 7);
	REAL Sum_time = 0;
	if (V != 0)
	{
		Sum_time = Value[1] / V * 1000;
	}
	else
	{
		std::cout << " The speed is zero.  " << std::endl;
	}
	int count = 0;

	while (1)
	{

		if (dl > 1.0f)
		{
			dl = 1.0f;
		}
		else if (dl == 1.0f)
		{
			std::cout << count << std::endl;
			break;
		}
		else
		{
			Current_L = lspb_plan(LSPB_V, dl);

			for (int i = 0; i < 7; i++)
			{
				current_joint[i] = initial_joint[i] + Current_L * Difference_ini[i];
				/*std::cout << "  joint:  " << i << "  === " << current_joint[i] << "      ";*/
			}

			std::cout << std::endl;
			dl = dl + 1 / Sum_time;
			count++;
		}
	}

	return 0;
}