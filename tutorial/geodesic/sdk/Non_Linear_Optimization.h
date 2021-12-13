#pragma once
#include "lbfgs.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include "VectorOperations.h"
#include <minmax.h>
using namespace std;
class Non_Linear_Optimization
{
protected:
    vector<lbfgsfloatval_t> m_variables;
	vector<lbfgsfloatval_t> m_Cartesian;
	//vector<lbfgsfloatval_t> m_Cartesian1;
	//vector<lbfgsfloatval_t> m_Cartesian2;
	vector<lbfgsfloatval_t> m_threadPoint;
	vector<lbfgsfloatval_t> m_copy;
	vector<lbfgsfloatval_t> new_m;

	vector<lbfgsfloatval_t> m_TEST;

	lbfgs_parameter_t m_parameters;
	vector<double> m_energysequence;
	vector<double> m_normsequence;
	bool m_printprogress;
public:
    Non_Linear_Optimization() : m_parameters(_defparam)
    {		
		m_printprogress = false;
    }
	bool& GetPrintProgress()
	{
		return m_printprogress;
	}
	//return the number of iterations
	lbfgs_parameter_t& GetParameters()
	{
		return m_parameters;
	}
	virtual double GetTerminationAccuracy() const { return m_parameters.epsilon; }
	const vector<double>& GetEnergySequence() const
	{
		return m_energysequence;
	}
	const vector<double>& GetNormSequence() const
	{
		return m_normsequence;
	}
	vector<double>& GetEnergySequence()
	{
		return m_energysequence;
	}
	vector<double>& GetNormSequence()
	{
		return m_normsequence;
	}
	virtual vector<double> GetVariables() const
	{
		return m_variables;
	}
    virtual int  run()
    { 
		 /*
            Start the L-BFGS optimization; this will invoke the callback functions
            evaluate() and progress() when necessary.
         */
		m_energysequence.clear();
		m_normsequence.clear();
		lbfgsfloatval_t fx = 0;
		int numOfIterations = 0;
        int ret = lbfgs(m_variables.size(), &m_variables[0], &fx, _evaluate, _progress, this, &m_parameters, &numOfIterations);

        /* Report the result. */
		if (m_printprogress)
		{
			if (ret >= 0)
				cerr << "Successfully terminate!\n";
			else if (ret == LBFGSERR_MAXIMUMLINESEARCH)
			{
				cerr << "The line-search routine reaches the maximum number of evaluations.\n";
			}
			else if (ret == LBFGSERR_ROUNDING_ERROR)
			{
				cerr << "No line-search step satisfies the sufficient decrease and curvature conditions." << endl;
			}
			else
			{
				cerr << "L-BFGS optimization terminated with status code = " << ret << endl;
				cerr << "Please see lbfgs.h for reference " << endl;
			}
		}
		return numOfIterations;
	}
	static int SaveSequenceIntoM(const vector<double>& sequence, const char* style, const char* filename, int base = 0)
	{
		if (sequence.empty())
			return base;
		ofstream out(filename);
		out.setf(ios::fixed); 
		out << "clc;clear;close;\n";
		out << "figure(1);\n";
		out << "X = 1:" << sequence.size() << ";\n";
		out << "Y = [";
		for (int i = 0; i < (int)sequence.size(); ++i)
		{
			out << setprecision(10) << sequence[i] << " ";
		}
		out << "];\n";
		out << "plot(X, Y, \'" << style << "\');\n"; 
		out << "hold on" << endl;
		out << "legend(\'" << "???" << "\');\n";
		double xMin = 1;
		double xMax = sequence.size();
		double yMin = *min_element(sequence.begin(), sequence.end());
		double yMax = *max_element(sequence.begin(), sequence.end());
		out << "xlim([" << xMin << " " << xMax << "]);\n";
		out << "ylim([" << yMin << " " << yMax << "]);\n";
		out << "set(gca,\'FontSize\',15);\n";
		out << "set(gca,\'FontName\',\'TimesNewRoman\');\n";
		out << "width = 500;\n";
		out << "height= 400;\n";
		out << "x = 400;\n";
		out << "y = 150;\n";
		out << "set(figure(1), \'Position\', [x y width height]);\n";
		out.close();
		return sequence.size() + base;
	}	
	
	int SaveEnergySequence(const char* style, const char* filename)
	{
		return Non_Linear_Optimization::SaveSequenceIntoM(m_energysequence, style, filename);
	}
	
	int SaveNormSequence(const char* style, const char* filename)
	{
		return Non_Linear_Optimization::SaveSequenceIntoM(m_normsequence, style, filename);
	}
	
protected:
	virtual int progress(
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t *g,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t gnorm,
        const lbfgsfloatval_t step,
        int n,
        int k,
        int ls
        )
    {
		m_energysequence.push_back(fx);
		m_normsequence.push_back(Norm_Inf(g, n));
		if (m_printprogress)
		{
			cerr << " Iteration #" << k << ": ";
			cerr << " Energy = " << setprecision(10) << m_energysequence.back() << ", ";
			cerr << " Error = " << setprecision(10) << m_normsequence.back() << ", ";
			cerr << " step = " << step << "\n";
			cerr << "-----------------------------------------\n";
		}

		return 0;
    }
	virtual lbfgsfloatval_t evaluate(
		const lbfgsfloatval_t *x,
		lbfgsfloatval_t *g,
		const int n,
		const lbfgsfloatval_t step
		) = 0;

    static lbfgsfloatval_t _evaluate(
        void *instance,
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t step
        )
    {
		//if (reinterpret_cast<Non_Linear_Optimization*>(instance)->m_printprogress)
		//	cerr << "\tCall your evaluation function once..........\n";
        return reinterpret_cast<Non_Linear_Optimization*>(instance)->evaluate(x, g, n, step);
    }

	static int _progress(
        void *instance,
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t *g,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t gnorm,
        const lbfgsfloatval_t step,
        int n,
        int k,
        int ls
        )
    {
        return reinterpret_cast<Non_Linear_Optimization*>(instance)->progress(x, g, fx, xnorm, gnorm, step, n, k, ls);
    }   
};