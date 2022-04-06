#ifndef EE4308_HECTOR_CUBIC_HERMITE_SPLINE_H
#define EE4308_HECTOR_CUBIC_HERMITE_SPLINE_H

#include <vector>

#include "common.hpp"
#include "cubic_polynomial.h"
//#include "spline.h"

class CubicHermiteSpline
{
public:
 CubicHermiteSpline() = default;

 CubicHermiteSpline(const std::vector<Position>& points, const std::vector<double>& x_vel,
                    const std::vector<double>& y_vel, const std::vector<double>& time);

 std::vector<Position> samplePoints(const double t_step) const;

 Position evaluate(const double t) const;

 double evaluateXVelocity(const double t) const;

 double evaluateYVelocity(const double t) const;

private:
  std::vector<CubicPolynomial> x_polynomials_;
  std::vector<CubicPolynomial> y_polynomials_;
  std::vector<double> t_;

  CubicPolynomial generatePolynomial(const double y_i, const double y_i_dot, const double y_f, const double y_f_dot,
                                     const double t_f);

  int getIntervalId(const double t) const;

  std::vector<double> calculatePointHeadings(const std::vector<Position>& points);

  double calculateMiddlePointHeading(const Position& before, const Position& middle, const Position& after);
};

#endif // EE4308_HECTOR_CUBIC_HERMITE_SPLINE_H
