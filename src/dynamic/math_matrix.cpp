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

#include "aris/core/log.hpp"
#include "aris/dynamic/math_matrix.hpp"

namespace aris::dynamic {
	auto dlmread(const char* FileName, double* pMatrix)->void {
		std::ifstream file;

		file.open(FileName);

		if (!file) THROW_FILE_LINE("file not exist");


		Size i = 0;
		while (!file.eof()) {
			file >> *(pMatrix + i);
			++i;
		}
	}

	auto dlmread(const char *filename)->std::vector<double>	{
		std::vector<double> mtx;

		std::fstream file;

		file.open(filename);
		if (!file) THROW_FILE_LINE("file not exist");

		Size i = 0;
		while (!file.eof())	{
			double data;
			file >> data;

			if (file.fail()){
				file.clear();
				char c;
				file >> c;
				continue;
			}

			++i;
		}

		mtx.resize(i);

		file.close();
		file.open(filename);
		i = 0;
		while (!file.eof())	{
			double data;
			file >> data;

			if (file.fail()){
				file.clear();
				char c;
				file >> c;
				continue;
			}

			mtx[i] = data;

			++i;
		}


		return mtx;
	}
}
