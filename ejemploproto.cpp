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
    int costo[3] = {30,40,10};
    int b1[3] = {80,90,100};
    int b2[2] = {100,200};

    ///////////////////////
    ///CPLEX ENVIRONMENT///
    ///////////////////////
	
    cout << "Definiendo formulacion..." << endl;
	
	IloEnv env;
    try {

       	//-------------------- Definiendo matriz de variables de 10x20 binarias
      
        IloArray<IloNumVarArray> X(env, 2);
        for(int i = 0; i < 2; i++){
            X[i] = IloNumVarArray(env, 3, 0, 200, ILOINT);
        }
        IloNumVarArray Y(env, 2, 0, 1, ILOINT);

        //------------------- Definiendo modelo
        IloModel model(env);

        //-------------------- Definición de restricción
        IloExpr SumVar(env);
        for(int i = 0; i< 3; i++){
            for(int j = 0; j < 2; j++){
                SumVar += X[j][i];
            }
            model.add(SumVar >= b1[i]);
            SumVar.clear();
        }
        for(int i = 0; i<2; i++){
            for(int j = 0; j<3; j++){
                SumVar += X[i][j];
            }
            model.add(SumVar <= 100*Y[i]);
            SumVar.clear();
        }

        IloExpr OBJ(env);
        for(int i = 0; i < 2; i++){
            for(int j = 0; j < 2; j++){
                OBJ += X[i][j]*costo[2];
            }
            OBJ += Y[i]*costo[i];
        }

		//------------------- Construccion de modelos y solución
		cout << "Building model for cplex ..." << endl;
		IloCplex cplex(env);
		IloObjective Objetivo = IloMinimize( env, OBJ );
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
        for(int i = 0; i < 2; i ++){
            cout << cplex.getValue(Y[i]) << endl;
        }
        for(int i = 0; i < 3; i ++){
            for(int j = 0; j< 2; j++){
                cout<<cplex.getValue(X[i][j])<<"\t";
            }
            cout<<endl;
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
