/*
 * File name: orloc.cc
 * Date:      2012/5/10 10:43
 * Author:    Miroslav Kulich
 */

#include <boost/foreach.hpp>
#include "orloc.h"
#define foreach BOOST_FOREACH
using namespace std;
using namespace PlayerCc;

class Line {
public:
  player_point_2d_t p1;
  player_point_2d_t p2;
  double a,b,c;

  Line(const player_point_2d_t &p1, const player_point_2d_t &p2) {
    this->p1 = p1;
    this->p2 = p2;
    a = p2.py-p1.py;
    b = p1.px-p2.px;
    double d = sqrt(a*a+b*b);
    a/=d;
    b/=d;
    c = -(a*p1.px+b*p1.py);
    //      std::cout << "DIST: " << a*p2.px + b*p2.py + c << std::endl;
  }

  double dist(const player_point_2d_t &p) const {
    return fabs(a*p.px + b*p.py + c);
  }
};


struct PolarLine {
  double phi;
  double ro;
  double n;
  PolarLine(double phi, double ro, double n) : phi(phi), ro(ro), n(n) {};
};


typedef vector<PolarLine> PolarLineVector;

inline double dist2(const player_point_2d_t &a, const player_point_2d_t &b) {
  return (a.px-b.px)*(a.px-b.px) + (a.py-b.py)*(a.py-b.py);
}




// transform distances measured with laser into points in Cartesian space
HybNav::Scan HybNav::getPoints(const HybNav::SPosition pos, const LaserProxy &laser) {
  double angle = laser.GetMinAngle();
  double resolution = laser.GetScanRes();
  double ppy = 0.04365; //TODO: this should be read from player laser.GetPose().py

  player_point_2d_t s;
  HybNav::Scan scan;
  for(int i=0;i<laser.GetCount();i++) {
    s.px = pos.x + cos(pos.yaw)*ppy + laser.GetRange(i) * cos(pos.yaw+angle);
    s.py = pos.y + sin(pos.yaw)*ppy + laser.GetRange(i) * sin(pos.yaw+angle);
    scan.push_back(s);
    angle += resolution;
  }
  return scan;
}

// find paramaters of line representing scan points in range <start,end>
PolarLine fitLine(const HybNav::Scan &scan, int start, int end) {
  double x = 0;
  double y = 0;
  double sxx = 0;
  double syy = 0;
  double sxy = 0;
  uint n = end-start+1;//scan.size();


  for(int i=start;i<=end;i++) {
    x+=scan[i].px;
    y+=scan[i].py;
  }
  x/=n;
  y/=n;

  for(int i=start;i<=end;i++) {
    sxx+=(scan[i].px-x)*(scan[i].px-x);
    syy+=(scan[i].py-y)*(scan[i].py-y);
    sxy+=(scan[i].px-x)*(scan[i].py-y);
  }
  double phi = atan2(-2*sxy,(syy-sxx))/2;
  double ro = x*cos(phi)+y*sin(phi);
  return PolarLine(phi,ro,n);
}




//Iterative End Point Fit
void iepf(const HybNav::Scan& scan, uint start, uint end, PolarLineVector &result) {
  Line line(scan[start], scan[end]);
  double max  = 0;
  double d;
  uint imax  = 0;
  for (uint i = start+1;i<end;i++) {
    d= line.dist(scan[i]);
    if (d>max) {
      max =d;
      imax  = i;

    }
  }
  if ( max>HybNav::PRECISION) {
    iepf(scan,start,imax,result);
    iepf(scan,imax,end,result);
  } else {
    if (end-start>10) { // filter lines smaller than 10 points
      PolarLine pl = fitLine(scan,start,end);
      result.push_back(pl);
    }
  }
}


PolarLineVector getLines(HybNav::Scan& scan) {
  PolarLineVector result;
  uint start = 0;
  for(uint i=1;i<scan.size();i++) {
    if ( dist2(scan[i],scan[i-1]) > HybNav::DIST_THRESHOLD_2 ) {
      if ( (i-start) >= HybNav::NUMBER_THRESHOLD ) {
        iepf(scan,start,i-1, result);
      }
      start = i;
    }
  }

  iepf(scan,start,scan.size()-1,result);
  return result;
}


double mod(double angle) {
  double result = angle;
  if (result > 0) {
    while (result > M_PI/2) { result -= M_PI/2; }
    if (result > M_PI/4) { result -= M_PI/2; }
  } else {
    while (result < -M_PI/2) { result += M_PI/2; }
    if (result < -M_PI/4) { result += M_PI/2; }
  }
  return result;
}


void HybNav::angleCorrection(HybNav::SPosition &pos, const LaserProxy &laser) {
  HybNav::Scan scan = getPoints(pos,laser);
  PolarLineVector lines = getLines(scan);
  int n = 0;
  int nn;
  double phi = 0;
  double diff;
  foreach(PolarLine pl, lines) {
    diff = mod(pl.phi);
    nn = pl.n*pl.n;
    if (fabs(diff) < M_PI/9) {
      phi+=nn*diff;
      n+=nn;
    }
  }
  phi = phi / n;

  pos.yaw -=phi; // correct the position
}

