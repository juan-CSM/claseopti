/********************************************************************0******************
Integer Linear Program for the integrated Vehicle-Driver Scheduling Problem
Solver: CPLEX
Autonomous University of Nuevo Leon
omar.ibarrar@uanl.edu.mx
**************************************************************************************/

#include <ilcplex/ilocplex.h>
#include <time.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <vector>
#include <cstdlib>
#include <math.h>

ILOSTLBEGIN

int main() {

	cout << "\t--------- EJEMPLO CPLEX ---------" << endl;
	double time_limit = 180; //time limit in seconds
	double gap_limit = 0.00; //0.01 es 1% de relative gap

							 ///////////////////////
							 ///CPLEX ENVIRONMENT///
							 ///////////////////////

	cout << "Definiendo formulacion..." << endl;

	IloEnv env;
	try {


		//Input

		int I = 2; //fábricas
		int J = 3; //clientes
		vector <int> cfijo;
		vector <int> capacidad;
		vector <int> demanda;
		cfijo.resize(I);
		capacidad.resize(I);
		demanda.resize(J);
		vector <vector <int> > ctrans; 
		ctrans.resize(I);
		for (int i = 0; i < I; i++)
		{
			ctrans[i].resize(J);
		}

		cfijo[0] = 30;
		cfijo[1] = 40;

		capacidad[0] = 100;
		capacidad[1] = 200;

		demanda[0] = 80;
		demanda[1] = 90;
		demanda[2] = 100;
		
		for (int i = 0; i < I; i++) {
			for (int j = 0; j < J; j++) {
				ctrans[i][j] = 10;
			}
		}
		
		//Variables

		IloArray<IloNumVarArray> X(env, I);
		for (int i = 0; i < I; i++) {
			X[i] = IloNumVarArray(env, J, 0, 200, ILOINT);
		}
		
		IloNumVarArray Y(env, I, 0, 1, ILOINT);

		//------------------- Definiendo modelo
		IloModel model(env);

		//-------------------- Definición de restricción

		IloExpr SumVar(env);
		for (int j = 0; j < J; j++) {
			//SumVar = 0;
			for (int i = 0; i < I; i++) {
				SumVar += X[i][j];
			}
			model.add(SumVar >= demanda[j]);
			SumVar.clear();
		}

		for (int i = 0; i < I; i++) {
			//SumVar = 0;
			for (int j = 0; j < J; j++) {
				SumVar += X[i][j];
			}
			model.add(SumVar <= capacidad[i] * Y[i]);
			SumVar.clear();
		}

		//-------------------- Definición de Función objetivo
		IloExpr OBJ(env);
		for (int i = 0; i<I; i++) {
			OBJ += cfijo[i]*Y[i];
			for (int j = 0; j < J; j++)
			{
				OBJ += ctrans[i][j] * X[i][j];
			}
		}

		//------------------- Construccion de modelos y solución
		cout << "Building model for cplex ..." << endl;
		IloCplex cplex(env);
		IloObjective Objetivo = IloMaximize(env, OBJ);
		model.add(Objetivo);
		//------------------- Se acabó

		cplex.setParam(IloCplex::ClockType, 2);
		cplex.setParam(IloCplex::TiLim, time_limit);
		cplex.setParam(IloCplex::EpGap, gap_limit);

		cplex.extract(model);
		cout << "Model extracted for cplex thus, solving ..." << endl;
		cplex.solve();
		cout << "Mejor cota dual == " << cplex.getBestObjValue() << endl;
		cout << "Mejor cota primal == " << cplex.getObjValue() << endl;


		/*for (int i = 0; i < n; i++) {
			cout << cplex.getValue(X[i]);//cout << endl;
		}*/

	}//END try
	catch (IloException& e) {
		cerr << " ERROR: " << e << endl;
	}
	catch (...) {
		cerr << " ERROR" << endl;
	}
	env.end();
	cin.get();
	return 0;
}//fin main