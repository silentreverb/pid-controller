/*----------------------------------------------------------------
  Laplace Inversion Source Code
  Copyright © 2010 James R. Craig, University of Waterloo
----------------------------------------------------------------*/
#ifndef LAP_INV_H
#define LAP_INV_H

#include <math.h>
#include <complex>
#include <iostream>
#include <fstream>

using namespace std;

typedef complex<double>              cmplex;

const double PI          =3.141592653589793238462643;      // pi 
const int    MAX_LAPORDER=60;

int test_case=1;
inline void  upperswap(double &u,const double v){if (v>u){u=v;}}

/***********************************************************************
        LaplaceInversion
************************************************************************
Returns Inverse Laplace transform f(t) of function F(s), 
where s is complex, evaluated at time t

  f(t) = 1/2*PI*i \intfrmto{gamma-i\infty}{gamma+i\infty} exp(st)*F(s) ds

Based upon De Hoog et al., 1982, An improved method for numerical inversion of 
Laplace transforms, SIAM J. Sci. Stat. Comput.

Speed of algorithm is primarily a function of M, the order of the 
Taylor series expansion. 

Inputs: F(s), a function pointer to some complex function of a single 
        complex variable
        t is the desired time of evaluation of f(t)
        tolerance is the required accuracy of f(t)
-----------------------------------------------------------------------*/
double LaplaceInversion(cmplex       (*F)(const cmplex &s),
                        const double &t,
                        const double tolerance)
{
  //--Variable declaration---------------------------------
  int    i,n,m,r;          //counters & intermediate array indices
  int    M(40);            //order of Taylor Expansion (must be less than MAX_LAPORDER)
  double DeHoogFactor(4.0);//DeHoog time factor
  double T;                //Period of DeHoog Inversion formula
  double gamma;            //Integration limit parameter
  cmplex h2M,R2M,z,dz,s;   //Temporary variables

  static cmplex Fctrl [2*MAX_LAPORDER+1];
  static cmplex e     [2*MAX_LAPORDER][MAX_LAPORDER];
  static cmplex q     [2*MAX_LAPORDER][MAX_LAPORDER];
  static cmplex d     [2*MAX_LAPORDER+1];
  static cmplex A     [2*MAX_LAPORDER+2]; 
  static cmplex B     [2*MAX_LAPORDER+2]; 

  //Calculate period and integration limits------------------------------------
  T    =DeHoogFactor*t;    
  gamma=-0.5*log(tolerance)/T;

  //Calculate F(s) at evalution points gamma+IM*i*PI/T for i=0 to 2*M-1--------
  //This is likely the most time consuming portion of the DeHoog algorithm
  Fctrl[0]=0.5*F(gamma); 
  for (i=1; i<=2*M;i++)
  {
    s=cmplex(gamma,i*PI/T);
    Fctrl[i]=F(s);
  }

  //Evaluate e and q ----------------------------------------------------------
  //eqn 20 of De Hoog et al 1982
  for (i=0;i<2*M;i++)
  {
    e[i][0]=0.0;
    q[i][1]=Fctrl[i+1]/Fctrl[i];
  }
  e[2*M][0]=0.0;

  for (r=1;r<=M-1;r++) //one minor correction - does not work for r<=M, as suggested in paper
  {
    for (i=2*(M-r);i>=0;i--)
    {     
      if ((i<2*(M-r)) && (r>1)){
      q[i][r]=q[i+1][r-1]*e[i+1][r-1]/e[i  ][r-1];
      }
      e[i][r]=q[i+1][r  ]-q[i  ][r  ]+e[i+1][r-1];
    }
  }

  //Populate d vector----------------------------------------------------------- 
  d[0]=Fctrl[0];
  for (m=1;m<=M;m++)
  {
    d[2*m-1]=-q[0][m];
    d[2*m  ]=-e[0][m];
  }

  //Evaluate A, B---------------------------------------------------------------
  //Eqn. 21 in De Hoog et al.
  z =cmplex(cos(PI*t/T),sin(PI*t/T));

  A[0]= 0.0; B[0]=1.0; //A_{-1},B_{-1} in De Hoog
  A[1]=d[0]; B[1]=1.0;
  for (n=2;n<=2*M+1; n++) 
  { 
    dz  =d[n-1]*z; 
    A[n]=A[n-1]+dz*A[n-2];
    B[n]=B[n-1]+dz*B[n-2];
  }

  //Eqn. 23 in De Hoog et al.
  h2M=0.5*(1.0+z*(d[2*M-1]-d[2*M]));
  R2M=-h2M*(1.0-sqrt(1.0+(z*d[2*M]/h2M/h2M)));

  //Eqn. 24 in De Hoog et al.
  A[2*M+1]=A[2*M]+R2M*A[2*M-1];
  B[2*M+1]=B[2*M]+R2M*B[2*M-1];

  //Final result: A[2*M]/B[2*M]=sum [F(gamma+itheta)*exp(itheta)]-------------
  return 1.0/T*exp(gamma*t)*(A[2*M+1]/B[2*M+1]).real();
}
//-----------------------------------------------------------------------

#endif /* LAP_INV_H */
