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
	double time_limit = 3600; //time limit in seconds
	double gap_limit = 0.01; //0.01 es 1% de relative gap

							 ///////////////////////
							 ///CPLEX ENVIRONMENT///
							 ///////////////////////

	cout << "Definiendo formulacion..." << endl;

	IloEnv env;
	try {

		//-------------------- Definiendo vector de vectores para los nodos visitados
			
		//vector<vector<int>> grafo{ {2, 3, 4},{3, 5},{5, 6, 7}, {7}, {6, 8}, {8, 9, 10}, {9}, {10}, {10} };

		int flujo[10] = { 100, 0, 0, 0, 0, 0, 0, 0, 0, -100 };

		int ady[10][10] = { 0, 1, 1, 1, 0, 0, 0, 0, 0, 0,
							0, 0, 1, 0, 1, 0, 0, 0, 0, 0,
							0, 0, 0, 0, 1, 1, 1, 0, 0, 0,
							0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
							0, 0, 0, 0, 0, 1, 0, 1, 0, 0,
							0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
							0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
							0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
							0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
							0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

		int costo[10][10] = { 0, 9, 7, 10, 0, 0, 0, 0, 0, 0,
							0, 0, 10, 0, 9, 0, 0, 0, 0, 0,
							0, 0, 0, 0, 7, 8, 5, 0, 0, 0,
							0, 0, 0, 0, 0, 0, 7, 0, 0, 0,
							0, 0, 0, 0, 0, 7, 0, 10, 0, 0,
							0, 0, 0, 0, 0, 0, 0, 1, 3, 0,
							0, 0, 0, 0, 0, 0, 0, 0, 4, 0,
							0, 0, 0, 0, 0, 0, 0, 0, 0, 10,
							0, 0, 0, 0, 0, 0, 0, 0, 0, 6,
							0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

		int cotaS[10][10] = { 0, 80, 60, 80, 0, 0, 0, 0, 0, 0,
							  0, 0, 10, 0, 20, 0, 0, 0, 0, 0,
							  0, 0, 0, 0, 40, 80, 10, 0, 0, 0,
			                  0, 0, 0, 0, 0, 0, 10, 0, 0, 0,
							  0, 0, 0, 0, 0, 15, 0, 40, 0, 0,
							  0, 0, 0, 0, 0, 0, 0, 20, 50, 30,
							  0, 0, 0, 0, 0, 0, 0, 0, 10, 0,
							  0, 0, 0, 0, 0, 0, 0, 0, 0, 90,
							  0, 0, 0, 0, 0, 0, 0, 0, 0, 100,
							  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		
		//Variables

		//IloNumVarArray X(env, 10, 0, 1, ILOINT);
		IloArray<IloNumVarArray> X(env, 10);
		for (int i = 0; i < 10; i++)
			X[i] = IloNumVarArray(env, 10, 0, 100, ILOINT);

		//------------------- Definiendo modelo
		IloModel model(env);

		//-------------------- Definición de restricción

		IloExpr SumVar(env);
		for (int i = 0; i<10; i++) {
			for (int j = 0; j < 10; j++)
			{
				if (ady[i][j]==1)
				{
					SumVar += X[i][j];
				}

				if (ady[j][i]==1)
				{
					SumVar -= X[j][i];
				}
			}
			model.add(SumVar == flujo[i]);
			SumVar.clear();
		}
		
		SumVar.clear();

		for (int i = 0; i < 10; i++) {
			for (int j = 0; j < 10; j++) {
				if (ady[i][j] == 1) {
					model.add(0 <= X[i][j]);
					model.add(X[i][j] <= cotaS[i][j]);
				}
			}
		}
		
		IloExpr OBJ(env);

		for (int i = 0; i < 10; i++) {
			for (int j = 0; j < 10; j++) {
				if (ady[i][j] == 1)
					OBJ += X[i][j] * costo[i][j];
			}
		}
/*
		IloExpr OBJ(env);
		for (int i = 0; i<10; i++) {
			OBJ += X[i] * b[i];
		}
*/
		//------------------- Construccion de modelos y solución
		cout << "Building model for cplex ..." << endl;
		IloCplex cplex(env);
		IloObjective Objetivo = IloMinimize(env, OBJ);
		model.add(Objetivo);
		//------------------- Se acabó

		cplex.setParam(IloCplex::ClockType, 2);
		cplex.setParam(IloCplex::TiLim, time_limit);
		cplex.setParam(IloCplex::EpGap, gap_limit);
		//cplex.setOut( env.getNullStream() );
		//cplex.setParam( IloCplex::HeurFreq, heuristics );
		//cplex.setParam( IloCplex::RINSHeur, heuristics );
		//cplex.setParam( IloCplex::MIPEmphasis, MIPEMPH);

		cplex.extract(model);
		cout << "Model extracted for cplex thus, solving ..." << endl;
		cplex.solve();
		cout << "Mejor cota dual == " << cplex.getBestObjValue() << endl;
		cout << "Mejor cota primal == " << cplex.getObjValue() << endl;

		//IMPRIMIR
		/*
		for(int i = 0; i < 10; i ++){
		for(int j = 0; j < 20; j ++)
		cout << cplex.getValue(X[i][j]) << "\t";
		cout << endl;
		}
		*/

		for (int i = 0; i < 10; i++)
		for (int j= 0;j<10;j++)
		{
			if (ady[i][j] == 1) {
				cout << cplex.getValue(X[i][j]) << "\t";
				cout << endl;
			}
		}
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