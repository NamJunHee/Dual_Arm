#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>

using namespace std;

#define Linear  1
#define Fifth   5

class spline
{
public:

  struct interpolation
  {
    vector<double> coefs;
    double time_min;
    double time_max;
  };

  vector<interpolation> pattern;

  void put_point(double time, double pos, double vel, double acc, int degree);
  void compute_polynom();
  double result(double t);

private:

  struct data_point
  {
    double time;
    double pos;
    double vel;
    double acc;
    int degree;
  };

  vector<data_point> point;

  vector<double> Fifth_Poly_interpolation(data_point f, data_point f1);
};
