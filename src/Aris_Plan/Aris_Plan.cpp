#include"Aris_Plan.h"


namespace Aris
{
	namespace Plan
	{
		double acc_up(unsigned int n, unsigned int i)
		{
			return (-1.0 / 2 / n / n / n * i*i*i + 3.0 / 2 / n / n * i*i);
		}
		double acc_down(unsigned int n, unsigned int i)
		{
			return (-1.0*i*i*i / 2.0 / n/n/n + 3.0 * i*i / 2 / n /n);
		}
		double dec_up(unsigned int n, unsigned int i)
		{
			return 1 - (-1.0 / 2 / n / n / n * (n - i)*(n - i)*(n - i) + 3.0 / 2 / n / n * (n - i)*(n - i));
		}
		double dec_down(unsigned int n, unsigned int i)
		{
			return 1 - (-1.0*(n - i)*(n - i)*(n - i) / 2.0 / n / n / n + 3.0 * (n - i)*(n - i) / 2 / n / n);
		}
		
		
		double acc_even(unsigned int n, unsigned int i)
		{
			return 1.0 / n / n  * i * i;
		}
		double dec_even(unsigned int n, unsigned int i)
		{
			return 1.0 - 1.0 / n / n * (n - i)*(n - i);
		}
		double even(unsigned int n, unsigned int i)
		{
			return 1.0 / n*i;
		}
	}
}