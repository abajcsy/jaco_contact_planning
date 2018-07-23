#include <stdio.h>
#include <string.h>
#include <iostream>
#include <iterator>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include "ros/ros.h"

#include "snoptProblem.hpp"

using namespace std;
using namespace Eigen;

const string prefix     = "j2s7s300";
const double pi         = 3.1415926535897;

const int T             = 10.0;         // planning horizon
const double dt         = 0.5;          // timestep
const int dofRobot      = 3;            // num dofs of robot
const int dofEnv        = 1;            // num dofs of environment
const int dofLamb       =  1;           // dimension of contact forces 

const double goal[] = {-0.4, 0.5};
const double obsLow[]   = {-1.0, 0.3};
const double obsHigh[]  = {1.0, 0.3};

// --------- HELPER FUNCTIONS --------- //

VectorXf config2DTo3D(VectorXf& q2D){
  VectorXf q3D(10);
  q3D << q2D(0), pi/2.0, pi/2.0, q2D(1), pi, q2D(2), pi, 0, 0, 0;
  return q3D;
}

VectorXf config3DTo2D(VectorXf& q3D){
  VectorXf q2D(3);
  q2D << q3D(0), q3D(3), q3D(5);
  return q2D;
}

VectorXf vel2DTo3D(VectorXf& dq2D){
  VectorXf dq3D(10);
  dq3D << dq2D(0), 0, 0, dq2D(1), 0, dq2D(2), 0, 0, 0, 0;
  return dq3D;
}

VectorXf vel3DTo2D(VectorXf& dq3D){
  VectorXf dq2D(3);
  dq2D << dq3D(0), dq3D(3), dq3D(5);
  return dq2D;
}

VectorXf control2DTo3D(VectorXf& u2D){
  VectorXf u3D(10);
  u3D << u2D(0), 0, 0, u2D(1), 0, u2D(2), 0, 0, 0, 0;
  return u3D;
}

VectorXf control3DTo2D(VectorXf& u3D){
  VectorXf u2D(3);
  u2D << u3D(0), u3D(3), u3D(5);
  return u2D;
}

VectorXf acc2DTo3D(VectorXf& ddq2D){
  VectorXf ddq3D(10);
  ddq3D << ddq2D(0), 0, 0, ddq2D(1), 0, ddq2D(2), 0, 0, 0, 0;
  return ddq3D;
}

VectorXf acc3DTo2D(VectorXf& ddq3D){
  VectorXf ddq2D(3);
  ddq2D << ddq3D(0), ddq3D(3), ddq3D(5);
  return ddq2D;
}

// --------- GETTER FUNCTIONS --------- //

double getFinalCost(VectorXf& q, VectorXf& dq){
  VectorXf q3D = config2DTo3D(q);
  //string linkName = strcat(prefix, "_link_7");
  //vector<double> eePose = getTransform(robot, q3d, linkName);
  // VectorXf goal(2);
  // goalPose << goal[0], goal[1];
  //double gf = 30*(goalPose - eePose).norm();
  double gf = 0.0;
  return gf;
}

double getRunningCost(VectorXf& q, VectorXf& dq, VectorXf& u){
  return 0.0;
}

VectorXf getConfigRobot(double *x, int t){
  VectorXf qRobot(dofRobot); 
  int lowOffset = 2*(dofRobot + dofEnv)*t + 1;
  int upOffset  = 2*(dofRobot + dofEnv)*t + dofRobot;
  for(size_t ii=lowOffset; ii <= upOffset; ii++)    // TODO check if should be inclusive
    qRobot(ii-lowOffset) = x[ii];
  return qRobot;
}

VectorXf getVelRobot(double *x, int t){
  VectorXf dqRobot(dofRobot); 
  int lowOffset = 2*(dofRobot + dofEnv)*t + (dofRobot + dofEnv) + 1;
  int upOffset  = 2*(dofRobot + dofEnv)*t + (dofRobot + dofEnv) + dofRobot;
  for(size_t ii=lowOffset; ii <= upOffset; ii++)    // TODO check if should be inclusive
    dqRobot(ii-lowOffset) = x[ii];
  return dqRobot;
}

VectorXf getConfigEnv(double *x, int t){
  VectorXf qEnv(dofEnv); //= new double[dofEnv];
  int lowOffset = 2*(dofRobot + dofEnv)*t + dofRobot + 1;
  int upOffset  = 2*(dofRobot + dofEnv)*t + dofRobot + dofEnv;
  for(size_t ii=lowOffset; ii <= upOffset; ii++)    // TODO check if should be inclusive
    qEnv(ii-lowOffset) = x[ii];
  return qEnv;
}

VectorXf getVelEnv(double *x, int t){
  VectorXf dqEnv(dofEnv); 
  int lowOffset = 2*(dofRobot + dofEnv)*t + (dofRobot + dofEnv) + dofRobot + 1;
  int upOffset  = 2*(dofRobot + dofEnv)*t + 2*(dofRobot + dofEnv);
  for(size_t ii=lowOffset; ii <= upOffset; ii++)    // TODO check if should be inclusive
    dqEnv(ii-lowOffset) = x[ii];
  return dqEnv;
}

VectorXf getU(double *x, int t){
  VectorXf u(dofRobot); 
  int nx = 2*(dofRobot+dofEnv)*(T+1);
  int lowOffset = nx + 1 + dofRobot*(t - 1);
  int upOffset  = nx + 1 + dofRobot*t - 1;
  for(size_t ii=lowOffset; ii <= upOffset; ii++)    // TODO check if should be inclusive
    u(ii-lowOffset) = x[ii];
  return u;
}
// ------------------------------------ //

// --------- SETTER FUNCTIONS --------- //

void setConfigRobot(double *x, int t, double *q){
  int lowOffset = 2*(dofRobot + dofEnv)*t + 1;
  int upOffset  = 2*(dofRobot + dofEnv)*t + dofRobot;
  copy(x + lowOffset, x + upOffset, q);
}

void setConfigEnv(double *x, int t, double *q){
  int lowOffset = 2*(dofRobot + dofEnv)*t + dofRobot + 1;
  int upOffset  = 2*(dofRobot + dofEnv)*t + dofRobot + dofEnv;
  copy(x + lowOffset, x + upOffset, q);
}

void setVelRobot(double *x, int t, double *dq){
  int lowOffset = 2*(dofRobot + dofEnv)*t + (dofRobot + dofEnv) + 1;
  int upOffset  = 2*(dofRobot + dofEnv)*t + (dofRobot + dofEnv) + dofRobot;
  copy(x + lowOffset, x + upOffset, dq);
}

void setVelEnv(double *x, int t, double *dq){
  int lowOffset = 2*(dofRobot + dofEnv)*t + (dofRobot + dofEnv) + dofRobot + 1;
  int upOffset  = 2*(dofRobot + dofEnv)*t + 2*(dofRobot + dofEnv);
  copy(x + lowOffset, x + upOffset, dq);
}

void setU(double *x, int t, double *u, int nx){
  int uStart    = nx + 1;
  int lowOffset = uStart + dofRobot*(t-1);
  int upOffset  = uStart + dofRobot*t - 1;
  copy(x + lowOffset, x + upOffset, u);
}

void setLambda(double *x, int t, double *lambda, int nx, int nu){
  int lambStart = nx + nu + 1;
  int lowOffset = lambStart + dofLamb*(t-1);
  int upOffset  = lambStart + dofLamb*t - 1;
  copy(x + lowOffset, x + upOffset, lambda);
}

// ------------------------------------ //

void userfun(int    *Status, int *n,    double x[],
       int    *needF,  int *neF,  double F[],
       int    *needG,  int *neG,  double G[],
       char      *cu,  int *lencu,
       int    iu[],    int *leniu,
       double ru[],    int *lenru) {
  
  // TODO IMPLEMENT ME!
  VectorXf qRobot_T   = getConfigRobot(x, T);
  VectorXf dqRobot_T  = getVelRobot(x, T);
  double obj          = getFinalCost(qRobot_T, dqRobot_T); 

  int kinLen  = (dofRobot + dofEnv)*T;
  int dynLen  = (dofRobot + dofEnv)*T;
  int collLen = T;

  // Compute kinematic, dynamics, and collision constraints
  double *kinConstraints = new double[kinLen];
  double *dynConstraints = new double[dynLen];
  double *collConstraints = new double[collLen];

  for(int t=0; t<T-1; t++){
    VectorXf qRobot_t  = getConfigRobot(x, t);
    VectorXf qRobot_t1 = getConfigRobot(x, t+1); 

    VectorXf dqRobot_t   = getVelRobot(x, t);
    VectorXf dqRobot_t1  = getVelRobot(x, t+1);

    VectorXf dqEnv_t   = getVelEnv(x, t);
    VectorXf dqEnv_t1  = getVelEnv(x, t+1); 

    VectorXf qEnv_t  = getConfigEnv(x, t);
    VectorXf qEnv_t1 = getConfigEnv(x, t+1); 

    VectorXf ddqRobot_t1  = (dqRobot_t1 - dqRobot_t)/dt;
    VectorXf ddqEnv_t1    = (dqEnv_t1 - dqEnv_t)/dt;

    VectorXf qRobot3D_t1  = config2DTo3D(qRobot_t1);
    VectorXf dqRobot3D_t1 = vel2DTo3D(qRobot_t1);

    VectorXf u_t1   = getU(x, t+1);
    VectorXf u3D_t1 = control2DTo3D(u_t1);

    //VectorXf ddqRobot3D_t1_fwd  = forwardDynamics(robot, qRobot3D_t1, dqRobot3D_t1, u3D_t1);
    //VectorXf ddqRobot_t1_fwd    = acc3DTo2D(ddqRobot3D_t1_fwd);

    obj += dt*getRunningCost(qRobot_t, dqRobot_t, u_t1);
  }

  int kinStart  = 1;
  int dynStart  = kinStart + kinLen + 1; 
  int collStart = dynStart + dynLen + 1;

  // Set up the constraint vector
  F[0] = obj;
  copy(F + kinStart, F + kinStart + kinLen, kinConstraints);
  copy(F + dynStart, F + dynStart + dynLen, dynConstraints);
  copy(F + collStart, F + collStart + collLen, collConstraints);
}


int main(int argc, char **argv) {
  snoptProblemA ToyProb;

  string urdf_file = "../urdf/jaco_dynamics.urdf";

  /*urdf::Model robot;
  if (!robot.initString(urdf_file)) {
    ROS_FATAL("Could not initialize robot model");
    return -1;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdf_file, tree)){
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }*/

  // Allocate and initialize;
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
  int signedDistLinkStart = 1 + 2*(dofRobot + dofEnv)*T;

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

  fill(qRobotInit, qRobotInit + dofRobot, pi);
  fill(qRobotMax, qRobotMax + dofRobot, 2.0*pi);

  fill(dqRobotMin, dqRobotMin + dofRobot, -0.35);
  fill(dqRobotMax, dqRobotMax + dofRobot, 0.35);

  fill(qEnvMax, qEnvMax + dofEnv, 100.0);
  fill(dqEnvMax, dqEnvMax + dofEnv, 10.0);

  fill(uMin, uMin + dofRobot, -100.0);
  fill(uMax, uMax + dofRobot, 100.0);

  fill(lambMax, lambMax + dofLamb, 1000.0);

  // --------------------------------------------- //

  // --------- Set up the constraints on x --------- //
  
  for(int t = 0;t < T;t++){
    if(t == 0){
      // Constrain initial config and velocity
      setConfigRobot(xlow, t, qRobotInit);
      setConfigRobot(xupp, t, qRobotInit);

      setVelRobot(xlow, t, dqRobotInit);
      setVelRobot(xupp, t, dqRobotInit);

      setConfigEnv(xlow, t, qEnvInit);
      setConfigEnv(xupp, t, qEnvInit);

      setVelEnv(xlow, t, dqEnvInit);
      setVelEnv(xupp, t, dqEnvInit);

    }else{
      // Constrain joint limits for rest of trajectory
      setConfigRobot(xlow, t, qRobotMin);
      setConfigRobot(xupp, t, qRobotMax);

      setVelRobot(xlow, t, dqRobotMin);
      setVelRobot(xupp, t, dqRobotMax);

      setConfigEnv(xlow, t, qEnvMin);
      setConfigEnv(xupp, t, qEnvMax);

      setVelEnv(xlow, t, dqEnvMin);
      setVelEnv(xupp, t, dqEnvMax);
    }

    if(t > 0){
      // Set torque limits
      setU(xlow, t, uMin, nx);
      setU(xupp, t, uMax, nx);

      // Set lambda constraint forces
      setLambda(xlow, t, lambMin, nx, nu);
      setLambda(xupp, t, lambMax, nx, nu);
    }
  }

  // ----------------------------------------------- //

  // Set bounds on the objective
  Flow[0] = -1e3; 
  Fupp[0] =  1e3; 
  fill(Fupp + signedDistLinkStart, Fupp + signedDistLinkStart + T, 1000.0);

  // Load the data for planContact ...
  ToyProb.initialize    ("", 1);          // no print file; summary on
  ToyProb.setPrintFile  ("Contact.out");  // oh wait, i want a print file
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
