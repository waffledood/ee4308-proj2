#include <cmath>
#include <stdexcept>
#include <vector>

#include "cubic_hermite_spline.h"

CubicHermiteSpline::CubicHermiteSpline(const std::vector<Position>& points, const std::vector<double>& x_vel,
                                       const std::vector<double>& y_vel, const std::vector<double>& time) 
{
  if (points.size() != x_vel.size() || x_vel.size() != y_vel.size() || y_vel.size() != time.size()) 
  {
    throw std::invalid_argument(
        "CubicHermiteSpline: arguments must have the same size");
  }

  if (points.size() < 2) {
    throw std::runtime_error("CubicHermiteSpline: size must be at least 2");
  }

  // static const double MAGNITUDE = 0.02;
  // std::vector<double> headings = calculatePointHeadings(points);

  for (int i = 1; i < points.size(); ++i)
  {
    // join i-1 to i
    // double heading_begin = headings.at(i - 1);
    // double heading_end = headings.at(i);
    // double x_i_dot = cos(heading_begin) * MAGNITUDE;
    // double y_i_dot = sin(heading_begin) * MAGNITUDE;
    // double x_f_dot = cos(heading_end) * MAGNITUDE;
    // double y_f_dot = sin(heading_end) * MAGNITUDE;

    double x_i_dot = x_vel.at(i - 1);
    double x_f_dot  = x_vel.at(i);
    double y_i_dot = y_vel.at(i - 1);
    double y_f_dot  = y_vel.at(i);

    CubicPolynomial x_poly =
        generatePolynomial(points.at(i - 1).x, x_i_dot, points.at(i).x, x_f_dot, time.at(i) - time.at(i - 1));
    CubicPolynomial y_poly =
        generatePolynomial(points.at(i - 1).y, y_i_dot, points.at(i).y, y_f_dot, time.at(i) - time.at(i - 1));

    t_ = time;
    x_polynomials_.push_back(x_poly);
    y_polynomials_.push_back(y_poly);
  }
}

std::vector<Position> CubicHermiteSpline::samplePoints(const double t_step) const
{
  std::vector<Position> points;

  for (double t = 0; t < t_.back(); t += t_step)
  {
    points.push_back(evaluate(t));
  }

  return points;
}

Position CubicHermiteSpline::evaluate(const double t) const
{
  int interval = getIntervalId(t);

  double x = x_polynomials_.at(interval).evaluate(t - t_.at(interval));
  double y = y_polynomials_.at(interval).evaluate(t - t_.at(interval));
  return Position(x, y);
}

double CubicHermiteSpline::evaluateXVelocity(const double t) const
{
  int interval = getIntervalId(t);
  return x_polynomials_.at(interval).evaluateFirstDerivative(t - t_.at(interval));
}

double CubicHermiteSpline::evaluateYVelocity(const double t) const
{
  int interval = getIntervalId(t);
  return y_polynomials_.at(interval).evaluateFirstDerivative(t - t_.at(interval));
}

int CubicHermiteSpline::getIntervalId(const double t) const
{
  int interval = 0;

  while ((interval + 1) < (t_.size() - 1) && t_.at(interval + 1) < t)
  {
    ++interval;
  }

  return interval;
}

CubicPolynomial CubicHermiteSpline::generatePolynomial(const double y_i, const double y_i_dot, const double y_f,
                                                       const double y_f_dot, const double t_f)
{
  double a = y_i;
  double b = y_i_dot;
  double c = -(3 / pow(t_f, 2) * y_i) - (2 / t_f * y_i_dot) + (3 / pow(t_f, 2) * y_f) - (1 / t_f * y_f_dot);
  double d =
      (2 / pow(t_f, 3) * y_i) + (1 / pow(t_f, 2) * y_i_dot) - (2 / pow(t_f, 3) * y_f) + ((1 / pow(t_f, 2)) * y_f_dot);

  return CubicPolynomial(a, b, c, d);
}

std::vector<double> CubicHermiteSpline::calculatePointHeadings(const std::vector<Position>& points)
{
  std::vector<double> headings;
  headings.push_back(heading(points.front(), points.at(1)));

  for (int i = 1; i < points.size() - 1; ++i)
  {
    headings.push_back(calculateMiddlePointHeading(points.at(i - 1), points.at(i), points.at(i + 1)));
  }

  headings.push_back(heading(points.at(points.size() - 2), points.back()));
  return headings;
}

double CubicHermiteSpline::calculateMiddlePointHeading(const Position& before, const Position& middle,
                                                       const Position& after)
{
  double heading_before = heading(before, middle);
  double heading_after = heading(middle, after);
  double average_heading = (heading_before + heading_after) / 2;
  return average_heading;
}
