#include <stdio.h>
#include <string.h>
#include <iostream>
#include <iterator>
#include <vector>
#include <algorithm>

#include "snoptProblem.hpp"

using namespace std;

const double pi = 3.1415926535897;

void userfun(int    *Status, int *n,    double x[],
	     int    *needF,  int *neF,  double F[],
	     int    *needG,  int *neG,  double G[],
	     char      *cu,  int *lencu,
	     int    iu[],    int *leniu,
	     double ru[],    int *lenru) {
  
  // TODO IMPLEMENT ME!

  F[0] =  x[1];
  F[1] =  x[0]*x[0] + 4*x[1]*x[1];
  F[2] = (x[0] - 2)*(x[0] - 2) + x[1]*x[1];
}

// --------- HELPER FUNCTIONS --------- //

void setConfigRobot(double *x, int t, double *q, int dofRobot, int dofEnv){
  int lowOffset = 2*(dofRobot + dofEnv)*t + 1;
  int upOffset  = 2*(dofRobot + dofEnv)*t + dofRobot;
  std::copy(x + lowOffset, x + upOffset, q);
}

void setConfigEnv(double *x, int t, double *q, int dofRobot, int dofEnv){
  int lowOffset = 2*(dofRobot + dofEnv)*t + dofRobot + 1;
  int upOffset  = 2*(dofRobot + dofEnv)*t + dofRobot + dofEnv;
  std::copy(x + lowOffset, x + upOffset, q);
}

void setVelRobot(double *x, int t, double *dq, int dofRobot, int dofEnv){
  int lowOffset = 2*(dofRobot + dofEnv)*t + (dofRobot + dofEnv) + 1;
  int upOffset  = 2*(dofRobot + dofEnv)*t + (dofRobot + dofEnv) + dofRobot;
  std::copy(x + lowOffset, x + upOffset, dq);
}

void setVelEnv(double *x, int t, double *dq, int dofRobot, int dofEnv){
  int lowOffset = 2*(dofRobot + dofEnv)*t + (dofRobot + dofEnv) + dofRobot + 1;
  int upOffset  = 2*(dofRobot + dofEnv)*t + 2*(dofRobot + dofEnv);
  std::copy(x + lowOffset, x + upOffset, dq);
}

void setU(double *x, int t, double *u, int nx, int dofRobot){
  int uStart    = nx + 1;
  int lowOffset = uStart + dofRobot*(t-1);
  int upOffset  = uStart + dofRobot*t - 1;
  std::copy(x + lowOffset, x + upOffset, u);
}

void setLambda(double *x, int t, double *lambda, int nx, int nu, int dofLamb){
  int lambStart = nx + nu + 1;
  int lowOffset = lambStart + dofLamb*(t-1);
  int upOffset  = lambStart + dofLamb*t - 1;
  std::copy(x + lowOffset, x + upOffset, lambda);
}

// ------------------------------------ //

int main(int argc, char **argv) {
  snoptProblemA ToyProb;
  
  // Allocate and initialize;
  int T         =  10;                // planning horizon
  int dt        =  0.5;               // timestep

  int dofRobot  =  3;                 // num dofs of robot
  int dofEnv    =  1;                 // num dofs of environment 
  int dofLamb   =  1;                 // dimension of contact forces

  int nx        =  2*(dofRobot+dofEnv)*(T+1); // num states over time horizon
  int nu        =  dofRobot*T;        // num controls over time horizon
  int nlamb     =  T;

  int neF       =  1 + 2*(dofRobot+dofEnv)*T+T; // num constraints
  int n         =  nx + nu + nlamb;

  int nS = 0, nInf;
  double sInf;

  double *x      = new double[n];
  double *xlow   = new double[n];
  double *xupp   = new double[n];
  double *xmul   = new double[n];
  int    *xstate = new    int[n];

  double *F      = new double[neF];
  double *Flow   = new double[neF];
  double *Fupp   = new double[neF];
  double *Fmul   = new double[neF];
  int    *Fstate = new int[neF];    // vector of inittial states of F(x)

  int    ObjRow  = 0;   // constant value added to the objective
  double ObjAdd  = 0;   // row of F(x) containing the objective

  int Cold = 0, Basis = 1, Warm = 2;

  double *qRobotInit    = new double[dofRobot];
  double *qRobotMin     = new double[dofRobot]; // all zeros
  double *qRobotMax     = new double[dofRobot];
  
  double *qEnvInit      = new double[dofEnv];   // all zeros
  double *qEnvMin       = new double[dofEnv];   // all zeros
  double *qEnvMax       = new double[dofEnv];

  double *dqRobotInit   = new double[dofRobot]; // all zeros
  double *dqRobotMin    = new double[dofRobot]; // all zeros
  double *dqRobotMax    = new double[dofRobot];

  double *dqEnvInit     = new double[dofEnv];   // all zeros
  double *dqEnvMin      = new double[dofEnv];   // all zeros
  double *dqEnvMax      = new double[dofEnv];

  double *lambMin       = new double[dofLamb];  // all zeros
  double *lambMax       = new double[dofLamb];

  double *uMin          = new double[dofRobot];
  double *uMax          = new double[dofRobot];

  // --------- Set up init, max and min  --------- //

  std::fill(qRobotInit, qRobotInit + dofRobot, pi);
  std::fill(qRobotMax, qRobotMax + dofRobot, 2.0*pi);

  std::fill(dqRobotMin, dqRobotMin + dofRobot, -0.35);
  std::fill(dqRobotMax, dqRobotMax + dofRobot, 0.35);

  std::fill(qEnvMax, qEnvMax + dofEnv, 100.0);
  std::fill(dqEnvMax, dqEnvMax + dofEnv, 10.0);

  std::fill(uMin, uMin + dofRobot, -100.0);
  std::fill(uMax, uMax + dofRobot, 100.0);

  std::fill(lambMax, lambMax + dofLamb, 1000.0);

  // --------------------------------------------- //

  // --------- Set up the constraints on x --------- //
  
  for(int t = 0;t < T;t++){
    if(t == 0){
      // Constrain initial config and velocity
      setConfigRobot(xlow, t, qRobotInit, dofRobot, dofEnv);
      setConfigRobot(xupp, t, qRobotInit, dofRobot, dofEnv);

      setVelRobot(xlow, t, dqRobotInit, dofRobot, dofEnv);
      setVelRobot(xupp, t, dqRobotInit, dofRobot, dofEnv);

      setConfigEnv(xlow, t, qEnvInit, dofRobot, dofEnv);
      setConfigEnv(xupp, t, qEnvInit, dofRobot, dofEnv);

      setVelEnv(xlow, t, dqEnvInit, dofRobot, dofEnv);
      setVelEnv(xupp, t, dqEnvInit, dofRobot, dofEnv);

    }else{
      // Constrain joint limits for rest of trajectory
      setConfigRobot(xlow, t, qRobotMin, dofRobot, dofEnv);
      setConfigRobot(xupp, t, qRobotMax, dofRobot, dofEnv);

      setVelRobot(xlow, t, dqRobotMin, dofRobot, dofEnv);
      setVelRobot(xupp, t, dqRobotMax, dofRobot, dofEnv);

      setConfigEnv(xlow, t, qEnvMin, dofRobot, dofEnv);
      setConfigEnv(xupp, t, qEnvMax, dofRobot, dofEnv);

      setVelEnv(xlow, t, dqEnvMin, dofRobot, dofEnv);
      setVelEnv(xupp, t, dqEnvMax, dofRobot, dofEnv);
    }

    if(t > 0){
      // Set torque limits
      setU(xlow, t, uMin, nx, dofRobot);
      setU(xupp, t, uMax, nx, dofRobot);

      // Set lambda constraint forces
      setLambda(xlow, t, lambMin, nx, nu, dofLamb);
      setLambda(xupp, t, lambMax, nx, nu, dofLamb);
    }
  }

  // ----------------------------------------------- //

  // Set bounds on the objective
  Flow[0] = -1e3; 
  Fupp[0] =  1e3; 

  // Load the data for ToyProb ...
  ToyProb.initialize    ("", 1);      // no print file; summary on
  ToyProb.setPrintFile  ("Contact.out"); // oh wait, i want a print file
  ToyProb.setProbName   ("Contact");

  // snopta will compute the Jacobian by finite-differences.
  // snJac will be called  to define the
  // coordinate arrays (iAfun,jAvar,A) and (iGfun, jGvar).
  ToyProb.setIntParameter("Derivative option", 0);
  ToyProb.setIntParameter("Verify level ", 3);

  // Solve the problem.
  // snJac is called implicitly in this case to compute the Jacobian.
  ToyProb.solve(Cold, neF, n, ObjAdd, ObjRow, userfun,
		xlow, xupp, Flow, Fupp,
		x, xstate, xmul, F, Fstate, Fmul,
		nS, nInf, sInf);

  for (int i = 0; i < n; i++){
    cout << "x = " << x[i] << " xstate = " << xstate[i] << endl;
  }

  delete []x;      delete []xlow;   delete []xupp;
  delete []xmul;   delete []xstate;

  delete []F;      delete []Flow;   delete []Fupp;
  delete []Fmul;   delete []Fstate;
}
