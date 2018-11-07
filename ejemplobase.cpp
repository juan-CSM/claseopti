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

int main(){
    
    cout << "\t--------- EJEMPLO CPLEX ---------" << endl;
    double time_limit = 3600; //time limit in seconds
    double gap_limit = 0.01; //0.01 es 1% de relative gap

    ///////////////////////
    ///CPLEX ENVIRONMENT///
    ///////////////////////
	
    cout << "Definiendo formulacion..." << endl;
	
	IloEnv env;
    try {

		//-------------------- Definiendo matriz de variables de 10x20 binarias
		IloArray<IloNumVarArray> X(env, 10);
		for(int i = 0; i < 10; i ++)
			X[i] = IloNumVarArray(env, 20, 0, 1, ILOINT);

		//------------------- Definiendo modelo
		IloModel model(env);

		//-------------------- Definición de restricción
		IloExpr SumVar(env);
		for(int i = 0; i < 10; i ++){
			for(int j = 0; j < 20; j ++)
				SumVar += 2*X[i][j];
		}
		model.add( SumVar <= 100 );
		SumVar.clear();
		
		IloExpr OBJ(env);
		for(int i = 0; i < 20; i ++)
			OBJ += X[0][i];

		//------------------- Construccion de modelos y solución
		cout << "Building model for cplex ..." << endl;
		IloCplex cplex(env);
		IloObjective Objetivo = IloMaximize( env, OBJ );
		model.add(Objetivo);
		//------------------- Se acabó

		cplex.setParam( IloCplex::ClockType, 2 );
		cplex.setParam( IloCplex::TiLim, time_limit );
		cplex.setParam( IloCplex::EpGap, gap_limit );
		//cplex.setOut( env.getNullStream() );
		//cplex.setParam( IloCplex::HeurFreq, heuristics );
		//cplex.setParam( IloCplex::RINSHeur, heuristics );
		//cplex.setParam( IloCplex::MIPEmphasis, MIPEMPH);

		cplex.extract(model);
		cout << "Model extracted for cplex thus, solving ..." << endl;
		cplex.solve();
		cout << "Mejor cota dual == " << cplex.getBestObjValue() << endl;
		cout << "Mejor cota primal == " << cplex.getObjValue() << endl;
		for(int i = 0; i < 10; i ++){
			for(int j = 0; j < 20; j ++)
				cout << cplex.getValue(X[i][j]) << "\t";
			cout << endl;
		}

    }//END try
    catch(IloException& e){
        cerr  << " ERROR: " << e << endl;
    }
    catch(...){
        cerr  << " ERROR" << endl;
    }
    env.end();
    
    return 0;
}//fin main
