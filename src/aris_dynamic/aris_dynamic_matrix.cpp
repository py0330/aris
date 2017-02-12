#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <cstddef>
#include <array>
#include <list>

#include "aris_dynamic_matrix.h"

namespace aris
{
	namespace dynamic
	{
		auto dlmwrite(const char *FileName, const double *pMatrix, const Size m, const Size n)->void
		{
			std::ofstream file;

			file.open(FileName);

			file << std::setprecision(15);

			for (Size i = 0; i < m; i++)
			{
				for (Size j = 0; j < n; j++)
				{
					file << pMatrix[n*i + j] << "   ";
				}
				file << std::endl;
			}
		}
		auto dlmread(const char *FileName, double *pMatrix)->void
		{
			std::ifstream file;

			file.open(FileName);

			if (!file) throw std::logic_error("file not exist");


			Size i = 0;
			while (!file.eof())
			{
				file >> *(pMatrix + i);
				++i;
			}
		}
	}
}
