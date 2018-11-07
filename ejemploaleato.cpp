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
    double gap_limit = 0; //0.01 es 1% de relative gap
    int numero = 35;
    int cuantos = 123456;
    int b[823457];
    int w[823457];
    int W;
    int sumatemp = 0;
    srand(3731173);
    int intentos = 1000;
    for(int k = 0; k < intentos; k++ ){
    
    for(int i = 0; i<cuantos; i++){
        if(i <cuantos/3){
            b[i] = (rand() % 732 * k) + 20;
            w[i] = b[i]*(rand()%40);
        }else if(i <2*cuantos/3){
            b[i] = (rand() % i * k) + (rand()%20);
            w[i] = b[i]*(rand()%40);
        }else{
            b[i] = (rand() % i * k) * (rand()%20);
            w[i] = b[i];
        }
        sumatemp += w[i];
    }
    
    W = (sumatemp / 1000)* k;

    ///////////////////////
    ///CPLEX ENVIRONMENT///
    ///////////////////////
	
    cout << "Definiendo formulacion..." << endl;
	
	IloEnv env;
    try {

		//-------------------- Definiendo matriz de variables de 10x20 binarias
		IloNumVarArray X(env, cuantos, 0, 1, ILOINT);

		//------------------- Definiendo modelo
		IloModel model(env);

		//-------------------- Definición de restricción
		IloExpr SumVar(env);
		for(int i = 0; i < cuantos; i ++){
				SumVar += w[i]*X[i];
		}
		model.add( SumVar <= W );
		SumVar.clear();
		
		IloExpr OBJ(env);
		for(int i = 0; i < cuantos; i ++)
			OBJ += b[i]*X[i];

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
        cout << "termino"<< endl;
//		for(int i = 0; i < cuantos; i ++){
//            cout << cplex.getValue(X[i]) << "\t";
//			cout << endl;
//		}

    }//END try
    catch(IloException& e){
        cerr  << " ERROR: " << e << endl;
    }
    catch(...){
        cerr  << " ERROR" << endl;
    }
    env.end();
    
}
    
    return 0;
}//fin main
