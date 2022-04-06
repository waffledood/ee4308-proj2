#ifndef EE4308_TURTLE_CUBIC_POLYNOMIAL_H
#define EE4308_TURTLE_CUBIC_POLYNOMIAL_H

/**
 * @brief Represents a cubic polynomial of the form y = a + bx + cx^2 + dx^3.
 *
 */
class CubicPolynomial
{
public:
  /**
   * @brief Constructs a new Cubic Polynomial object
   *
   * @param a The constant coefficient.
   * @param b The linear coefficient.
   * @param c The quadratic coefficient.
   * @param d The cubic coefficient.
   */
  CubicPolynomial(const double a, const double b, const double c, const double d);

  /**
   * @brief Evaluates the polynomial at a given x value.
   *
   * @param x The x value to evaluate the polynomial at.
   * @return The y value of the polynomial at the given x value.
   */
  double evaluate(const double x) const;

  /**
   * @brief Evaluates the derivative of the polynomial at a given x value.
   *
   * @param x The x value to evaluate the derivative of the polynomial at.
   * @return The derivative of the polynomial at the given x value.
   */
  double evaluateFirstDerivative(const double x) const;

  /**
   * @brief Gets the constant coefficient.
   *
   * @return The constant coefficient.
   */
  double getA() const;

  /**
   * @brief Gets the linear coefficient.
   *
   * @return The linear coefficient.
   */
  double getB() const;

  /**
   * @brief Gets the quadratic coefficient.
   *
   * @return The quadratic coefficient.
   */
  double getC() const;

  /**
   * @brief Gets the cubic coefficient.
   *
   * @return The cubic coefficient.
   */
  double getD() const;

private:
  double a_;
  double b_;
  double c_;
  double d_;
};

#endif  // EE4308_TURTLE_CUBIC_POLYNOMIAL_H
