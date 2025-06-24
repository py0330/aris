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

	auto s_quadprog(Size nG, Size nCE, Size nCI,
		const double* G, const double* g0,
		const double* CE, const double* ce0,
		const double* CI, const double* ci0,
		double* x, double* mem)->double
	{
		unsigned int l;
		
		const double* np = nullptr;

		aris::Size G_t = nG,
				   g0_t = 1,
				   CE_t = nG,
				   ce0_t = 1,
				   CI_t = nG,
				   ci0_t = 1,
				   x_t = 1,
				   R_t = nG,  // for R
				   J_t = nG,  // for J
				   z_t = 1,   // for z
				   r_t = 1;
		double* R = mem;
		double* J = mem + nG * nG;
		double* z = J + nG * nG;
		double* d = z + nG;
		double* x_old = d + nG;

		double* s = x_old + nG;
		double* r = s + nCE + nCI;
		double* u = r + nCE + nCI;
		double* u_old = u + nCE + nCI;

		

		int* A = reinterpret_cast<int*>(u_old + nCE + nCI);
		int* A_old = A + nCE + nCI;
		int* iai = A_old + nCE + nCI;
		bool* iaexcl = reinterpret_cast<bool*>(iai + nCE + nCI);

		double f_value, psi, c1, c2, sum, ss, R_norm;
		const double inf = std::numeric_limits<double>::has_infinity ? std::numeric_limits<double>::infinity() : 1.0E300;
		double t, t1, t2;

		unsigned int iq = 0, iter = 0;
		/////////////////////////////// PART 1, compute unconstrained solution ////////////////
		// compute trace of G //
		c1 = 0.0;
		for (int i = 0; i < nG; i++)
			c1 += G[at(i, i, G_t)];

		// decompose the matrix G in the form L^T L and init J //
		s_llt(nG, G, G_t, J, J_t);

		// Find the unconstrained minimizer of the quadratic form 0.5 * x G x + g0 x
		// this is a feasible point in the dual space
		// x = G^-1 * g0
		s_sov_lm(nG, 1, J, J_t, g0, g0_t, x, x_t);
		s_sov_um(nG, 1, J, J_t, x, x_t, x, x_t);
		s_iv(nG, x, x_t);
		f_value = 0.5 * s_vv(nG, g0, g0_t, x, x_t);

		// compute G^-1, which is init value for H // 
		s_inv_um(nG, J, J_t, J, J_t);

		// c1 * c2 is an estimate for cond(G) //
		c2 = 0.0;
		for (int i = 0; i < nG; i++)
			c2 += J[at(i, i, J_t)];

		// init R and R_norm //
		s_fill(nG, nG, 0.0, R, R_t);
		R_norm = 1.0;

#ifdef ARIS_DEBUG_QP
		std::cout << "G:" << std::endl;
		dsp(nG, nG, G);
		std::cout << "J:" << std::endl;
		dsp(nG, nG, J);
		std::cout << "c1:  " << c1 << std::endl;
		std::cout << "c2:  " << c2 << std::endl;
		std::cout << "Unconstrained solution: " << f_value << std::endl;
		std::cout << "x:" << std::endl;
		dsp(1, nG, x);
#endif

		auto add_constraint = [](int nG, unsigned int &iq, double &R_norm, double *R, double* J, double* d)->bool {
#ifdef ARIS_DEBUG_QP
			std::cout << "Add constraint " << iq << '/';
#endif
			// we have to find the Givens rotation which will reduce the element
			// d[j] to zero.
			// if it is already zero we don't have to do anything, except of
			// decreasing j
			for (int j = nG - 1; j >= iq + 1; j--){
				// The Givens rotation is done with the matrix (cc cs, cs -cc).
				// If cc is one, then element (j) of d is zero compared with element
				// (j - 1). Hence we don't have to do anything.
				// If cc is zero, then we just have to switch column (j) and column (j - 1)
				// of J. Since we only switch columns in J, we have to be careful how we
				// update d depending on the sign of gs.
				// Otherwise we have to apply the Givens rotation to these columns.
				// The i - 1 element of d has to be updated to h. 
				double cc = d[j - 1];
				double ss = d[j];
				double h = std::sqrt(cc * cc + ss * ss);

				if (h < std::numeric_limits<double>::epsilon()) // h == 0
					continue;

				d[j] = 0.0;
				ss = ss / h;
				cc = cc / h;
				if (cc < 0.0) {
					cc = -cc;
					ss = -ss;
					d[j - 1] = -h;
				}
				else {
					d[j - 1] = h;
				}

				double xny = ss / (1.0 + cc);
				for (int k = 0; k < nG; k++){
					double t1 = J[at(k, j - 1, nG)];
					double t2 = J[at(k, j, nG)];
					J[at(k, j - 1, nG)] = t1 * cc + t2 * ss;
					J[at(k, j, nG)] = xny * (t1 + J[at(k, j - 1, nG)]) - t2;
				}
			}	
			
			// update the number of constraints added //
			iq++;
			// To update R we have to put the iq components of the d vector
			// into column iq - 1 of R
			//
			for (int i = 0; i < iq; i++)
				R[at(i, iq - 1, nG)] = d[i];
#ifdef ARIS_DEBUG_QP
			std::cout << iq << std::endl;
			std::cout << "R:" << std::endl;
			dsp(iq, iq, R, nG);
			std::cout << "J:" << std::endl;
			dsp(nG, nG, J);
			std::cout << "d:" << std::endl;
			dsp(1, iq, d);
#endif

			if (std::abs(d[iq - 1]) <= std::numeric_limits<double>::epsilon() * R_norm){
				// problem degenerate
				return false;
			}
			R_norm = std::max<double>(R_norm, std::abs(d[iq - 1]));
			return true;
		};
		auto delete_constraint = [](int nG, int nCE, unsigned int &iq, int l, double *J, double *u, double *R, int* A)->void{
#ifdef ARIS_DEBUG_QP
			std::cout << "Delete constraint " << l << ' ' << iq;
#endif
			double cc, ss, h, xny, t1, t2;

			// Find the index qq for active constraint l to be removed //
			int qq = std::find(A + nCE, A + iq, l) - A;
			if (qq == iq)
			{
				std::ostringstream os;
				os << "Attempt to delete non existing constraint, constraint: " << l;
				throw std::invalid_argument(os.str());
			}

			// remove the constraint from the active set and the duals //
			for (int i = qq; i < iq - 1; i++)
			{
				A[i] = A[i + 1];
				u[i] = u[i + 1];
				for (int j = 0; j < nG; j++)
					R[at(j, i, nG)] = R[at(j, i + 1, nG)];
			}

			A[iq - 1] = A[iq];
			u[iq - 1] = u[iq];
			A[iq] = 0;
			u[iq] = 0.0;
			for (int j = 0; j < iq; j++)
				R[at(j, iq - 1, nG)] = 0.0;

			// constraint has been fully removed //
			iq--;
#ifdef ARIS_DEBUG_QP
			std::cout << '/' << iq << std::endl;
#endif 

			if (iq == 0)
				return;

			for (int j = qq; j < iq; j++){
				cc = R[at(j, j, nG)];
				ss = R[at(j + 1, j, nG)];
				h = std::sqrt(cc * cc + ss * ss);
				if (std::abs(h) < std::numeric_limits<double>::epsilon()) // h == 0
					continue;
				cc = cc / h;
				ss = ss / h;
				R[at(j + 1, j, nG)] = 0.0;
				if (cc >= 0.0)
				{
					R[at(j, j, nG)] = -h;
					cc = -cc;
					ss = -ss;
				}
				else
					R[at(j, j, nG)] = h;

				xny = ss / (1.0 + cc);
				for (int k = j + 1; k < iq; k++)
				{
					t1 = R[at(j, k, nG)];
					t2 = R[at(j + 1, k, nG)];
					R[at(j, k, nG)] = t1 * cc + t2 * ss;
					R[at(j + 1, k, nG)] = xny * (t1 + R[at(j, k, nG)]) - t2;
				}
				for (int k = 0; k < nG; k++)
				{
					t1 = J[at(k, j, nG)];
					t2 = J[at(k, j + 1, nG)];
					J[at(k, j, nG)] = t1 * cc + t2 * ss;
					J[at(k, j + 1, nG)] = xny * (J[at(k, j, nG)] + t1) - t2;
				}
			}
		};

		/////////////////////////////// PART 2, add equality constraints to the working set A ////////////////
		for (int i = 0; i < nCE; i++){
			auto np = CE + at(i, 0, nG);
			auto np_t = CE_t;

			// d = H^T * np //
			s_mm(nG, 1, nG, J, T(J_t), np, 1, d, 1);
			// z = H * d
			s_mm(nG, 1, nG - iq, J + at(0, iq, J_t), J_t, d + iq, 1, z, z_t);
			// r = R^-1 * d //
			s_sov_um(iq, 1, R, R_t, d, 1, r, r_t);

#ifdef ARIS_DEBUG_QP
			std::cout << "R:" << std::endl;
			dsp(iq, iq, R);
			std::cout << "z:" << std::endl;
			dsp(1, nG, z);
			std::cout << "r:" << std::endl;
			dsp(1, iq, r);
			std::cout << "d:" << std::endl;
			dsp(1, nG, d);
			std::cout << "x:" << std::endl;
			dsp(1, nG, x);
#endif

			// compute full step length t2: i.e., the minimum step in primal space s.t. the contraint
	        // becomes feasible 
			double t2 = 0.0;
			if (s_vv(nG, z, z) > std::numeric_limits<double>::epsilon()) // i.e. z != 0
				t2 = (-s_vv(nG, np, x) + ce0[i]) / s_vv(nG, z, np);
#ifdef ARIS_DEBUG_QP
			std::cout << "t2 : " << t2 << std::endl;
#endif
			// set x = x + t2 * z 
			s_va(nG, t2, z, x);

			// set u = u+
			u[iq] = t2;
			s_va(iq, -t2, r, u);

			// compute the new solution value //
			f_value += 0.5 * (t2 * t2) * s_vv(nG, z, np);
			A[i] = -i - 1;

			if (!add_constraint(nG, iq, R_norm, R, J, d)){
				// Equality constraints are linearly dependent
				throw std::runtime_error("Constraints are linearly dependent");
				return f_value;
			}
		}

		/////////////////////////////// PART 3, add inequality constraints to the working set A ////////////////
		// set iai = K \ A //
		for (int i = 0; i < nCI; i++)
			iai[i] = i;

#ifdef ARIS_DEBUG_QP
		std::cout << "Part 3:" << std::endl;
#endif

	l1:	iter++;
#ifdef ARIS_DEBUG_QP
		std::cout << "x:" << std::endl;
		dsp(1, nG, x);
#endif
		// step 1: choose a violated constraint //
		for (int i = nCE; i < iq; i++){
			iai[A[i]] = -1;
		}

		// compute s[x] = ci^T * x + ci0 for all elements of K \ A //
		ss = 0.0;
		psi = 0.0; // this value will contain the sum of all infeasibilities 
		int ip = 0;    // ip will be the index of the chosen violated constraint 
		
		s_vc(nCI, ci0, s);
		s_mms(nCI, 1, nG, CI, x, s);
		
		for (int i = 0; i < nCI; i++){
			iaexcl[i] = true;
			psi += std::min(s[i], sum);
		}
#ifdef ARIS_DEBUG_QP
		std::cout << "s:" << std::endl;
		dsp(1, nCI, s);
#endif

		if (std::abs(psi) <= nCI * std::numeric_limits<double>::epsilon() * c1 * c2 * 100.0){
			// numerically there are not infeasibilities anymore //
			return f_value;
		}

		// save old values for u and A, and x //
		std::copy_n(u, iq, u_old);
		std::copy_n(A, iq, A_old);
		std::copy_n(x, nG, x_old);

	l2: // Step 2: check for feasibility and determine a new S-pair //
		for (int i = 0; i < nCI; i++){
			if (s[i] < ss && iai[i] != -1 && iaexcl[i]){
				ss = s[i];
				ip = i;
			}
		}
		if (ss >= 0.0){
			return f_value;
		}

		// set np = n[ip] //
		np = CI + at(ip, 0, nG);
		
		// set u = [u 0]^T //
		u[iq] = 0.0;
		// add ip to the active set A //
		A[iq] = ip;


#ifdef ARIS_DEBUG_QP
		std::cout << "np:" << std::endl;
		dsp(1, nG, np);
#endif

	l2a:// Step 2a: determine step direction //
		// compute z = H np: the step direction in the primal space (through J, see the paper) //
		// compute_d(d, J, np);
		// update_z(z, J, d, iq);
		// compute N* np (if q > 0): the negative of the step direction in the dual space //
		// update_r(R, r, d, iq);

		// d = H^T * np //
		s_mmi(nG, 1, nG, J, T(nG), np, 1, d, 1);
		// z = H * d
		s_mm(nG, 1, nG - iq, J + at(0, iq, nG), nG, d + iq, 1, z, 1);
		// r = R^-1 * d //
		s_sov_um(iq, 1, R, nG, d, 1, r, 1);

#ifdef ARIS_DEBUG_QP
		std::cout << "Step direction z" << std::endl;
		std::cout << "z:" << std::endl;
		dsp(1, nG, z);
		std::cout << "r:" << std::endl;
		dsp(1, iq + 1, r);
		std::cout << "u:" << std::endl;
		dsp(1, iq + 1, u);
		std::cout << "d:" << std::endl;
		dsp(1, nG, d);
		std::cout << "A:" << std::endl;
		dsp(1, iq + 1, A);
#endif

		// Step 2b: compute step length //
		// Compute t1: partial step length (maximum step in dual space without violating dual feasibility //
		t1 = inf; // +inf //
		// find the index l s.t. it reaches the minimum of u+[x] / r //
		for (int k = nCE; k < iq; k++){
			if (r[k] > 0.0){
				if (u[k] / r[k] < t1){
					t1 = u[k] / r[k];
					l = A[k];
				}
			}
		}
		// Compute t2: full step length (minimum step in primal space such that the constraint ip becomes feasible */
		if (std::abs(s_vv(nG, z, z)) > std::numeric_limits<double>::epsilon()) // i.e. z != 0
		{
			t2 = s[ip] / s_vv(nG, z, np);
			if (t2 < 0) // patch suggested by Takano Akio for handling numerical inconsistencies
				t2 = inf;
		}
		else
			t2 = inf; // +inf //

		// the step is chosen as the minimum of t1 and t2 //
		t = std::min(t1, t2);
#ifdef ARIS_DEBUG_QP
		std::cout << "Step sizes: " << t << " (t1 = " << t1 << ", t2 = " << t2 << ") ";
#endif

		// Step 2c: determine new S-pair and take step: //
		// case (i): no step in primal or dual space //
		if (t >= inf){
			// QPP is infeasible //
			// FIXME: unbounded to raise
			return inf;
		}
		// case (ii): step in dual space //
		if (t2 >= inf){
			// set u = u +  t * [-r 1] and drop constraint l from the active set A //
			for (int k = 0; k < iq; k++)
				u[k] -= t * r[k];
			u[iq] += t;
			iai[l] = l;
			delete_constraint(nG, nCE, iq, l, J, u, R, A);
#ifdef ARIS_DEBUG_QP
			std::cout << " in dual space: "	<< f_value << std::endl;

			std::cout << "x:" << std::endl;
			dsp(1, nG, x);
			std::cout << "z:" << std::endl;
			dsp(1, nG, z);
			std::cout << "A:" << std::endl;
			dsp(1, iq + 1, A);
#endif
			goto l2a;
		}

		// case (iii): step in primal and dual space //
		// set x = x + t * z //
		s_va(nG, t, z, x);
		// update the solution value //
		f_value -= t * s_vv(nG, z, np) * (0.5 * t + u[iq]);
		// u = u + t * [-r 1] //
		s_va(iq, -t, r, u);
		u[iq] += t;
#ifdef ARIS_DEBUG_QP
		std::cout << " in both spaces: " << f_value << std::endl;

		std::cout << "x:" << std::endl;
		dsp(1, nG, x);
		std::cout << "u:" << std::endl;
		dsp(1, iq + 1, u);
		std::cout << "r:" << std::endl;
		dsp(1, iq + 1, r);
		std::cout << "A:" << std::endl;
		dsp(1, iq + 1, A);
#endif

		if (std::abs(t - t2) < std::numeric_limits<double>::epsilon())
		{
#ifdef ARIS_DEBUG_QP
			std::cout << "Full step has taken " << t << std::endl;
			std::cout << "x:" << std::endl;
			dsp(1, nG, x);
#endif
			// full step has taken //
			// add constraint ip to the active set //
			if (!add_constraint(nG, iq, R_norm, R, J, d)){
				iaexcl[ip] = false;
				delete_constraint(nG, nCE, iq, l, J, u, R, A);
#ifdef ARIS_DEBUG_QP
				std::cout << "R:" << std::endl;
				dsp(iq, iq, R, nG);
				std::cout << "A:" << std::endl;
				dsp(1, iq + 1, A);
				std::cout << "iai:" << std::endl;
				dsp(1, nCI+ nCE, iai);
#endif
				for (int i = 0; i < nCI; i++)
					iai[i] = i;
				for (int i = nCE; i < iq; i++){
					A[i] = A_old[i];
					u[i] = u_old[i];
					iai[A[i]] = -1;
				}
				for (int i = 0; i < nG; i++)
					x[i] = x_old[i];
				goto l2; // go to step 2 //
			}
			else
				iai[ip] = -1;
#ifdef ARIS_DEBUG_QP
			std::cout << "R:" << std::endl;
			dsp(iq, iq, R, nG);
			std::cout << "A:" << std::endl;
			dsp(1, iq + 1, A);
			std::cout << "iai:" << std::endl;
			dsp(1, nCI + nCE, iai);
#endif
			goto l1;
		}

		// a patial step has taken //
#ifdef ARIS_DEBUG_QP
		std::cout << "Partial step has taken " << t << std::endl;
		std::cout << "x:" << std::endl;
		dsp(1, nG, x);
#endif
		/* drop constraint l */
		iai[l] = l;
		delete_constraint(nG, nCE, iq, l, J, u, R, A);
#ifdef ARIS_DEBUG_QP
		std::cout << "R:" << std::endl;
		dsp(iq, iq, R, nG);
		std::cout << "A:" << std::endl;
		dsp(1, iq + 1, A);
#endif

		// update s[ip] = CI * x + ci0 //
		s[ip] = -s_vv(nG, CI + at(ip, 0, nG), x) + ci0[ip];

#ifdef ARIS_DEBUG_QP
		std::cout << "s:" << std::endl;
		dsp(1, nCI, s);
#endif
		goto l2a;
	}

}
