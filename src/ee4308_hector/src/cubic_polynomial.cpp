#include <cmath>
#include <vector>

#include "cubic_polynomial.h"

CubicPolynomial::CubicPolynomial(const double a, const double b, const double c, const double d)
{
  a_ = a;
  b_ = b;
  c_ = c;
  d_ = d;
}

double CubicPolynomial::evaluate(const double x) const
{
  return a_ + b_ * x + c_ * pow(x, 2) + d_ * pow(x, 3);
}

double CubicPolynomial::evaluateFirstDerivative(const double x) const
{
  return b_ + 2 * c_ * x + 3 * d_ * pow(x, 2);
}

double CubicPolynomial::getA() const
{
  return a_;
}

double CubicPolynomial::getB() const
{
  return b_;
}

double CubicPolynomial::getC() const
{
  return c_;
}

double CubicPolynomial::getD() const
{
  return d_;
}
