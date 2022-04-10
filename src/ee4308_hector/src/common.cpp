#include "common.hpp"
#include <cmath>
#include <vector>

Index::Index() : i(0), j(0) {};
Index::Index(int i, int j) : i(i), j(j) {};
Position::Position() : x(0), y(0) {};
Position::Position(double x, double y) : x(x), y(y) {};
double sign(double value)
{
    if (value > 0)
        return 1;
    else if (value < 0)
        return -1;
    else
        return 0;
}
double dist_oct(Index src, Index tgt)
{
    return dist_oct(src.i, src.j, tgt.i, tgt.j);
}
double dist_oct(Position src, Position tgt)
{
    return dist_oct(src.x, src.y, tgt.x, tgt.y);
}
double dist_oct(double src_x, double src_y, double tgt_x, double tgt_y)
{
    double abs_Dx = abs(tgt_x - src_x);
    double abs_Dy = abs(tgt_y - src_x);
    double ordinals, cardinals;
    if (abs_Dx > abs_Dy)
    {
        ordinals = abs_Dy; // ordinal (diagonal)
        cardinals = abs_Dx - abs_Dy; // cardinal (vertical or horizontal)
    }
    else
    {
        ordinals = abs_Dx;
        cardinals = abs_Dy - abs_Dx;
    }
    return ordinals * M_SQRT2 + cardinals; // M_SQRT2 is from cmath
}
double dist_euc(Index src, Index tgt)
{
    return dist_euc(src.i, src.j, tgt.i, tgt.j);
}
double dist_euc(Position src, Position tgt)
{
    return dist_euc(src.x, src.y, tgt.x, tgt.y);
}
double dist_euc(double src_x, double src_y, double tgt_x, double tgt_y)
{
    double Dx = tgt_x - src_x;
    double Dy = tgt_y - src_y;
    return sqrt(Dx*Dx + Dy*Dy);
}
double heading(Position src, Position tgt)
{
    return heading(src.x, src.y, tgt.x, tgt.y);
}
double heading(double src_x, double src_y, double tgt_x, double tgt_y)
{
    double Dx = tgt_x - src_x;
    double Dy = tgt_y - src_y;

    return atan2(Dy, Dx);
}
double limit_angle(double angle)
{
    // faster version, but -2PI <= angle < 2PI must be guaranteed (may not happen if any sensor readings surges temporarily due to noise and high speed, e.g. imu ang vel)
    /*
    if (angle >= M_PI)
        return angle - M_PI;
    else if (angle < -M_PI)
        return angle + M_PI;
    */

    // // slower version, but guaranteed to limit the angle in most cases
    // return fmod(angle + M_PI, M_PI*2) - M_PI;
    double result = fmod(angle + M_PI, M_PI*2); // fmod rounds remainders to zero. we want remainders to be +ve like mod() in matlab and % in python
    return result >= 0 ? result - M_PI : result + M_PI;
}


double sat(double calc, double max) {

    if (calc > max) {
        return max;
    } else if (calc < -max) {
        return -max;
    } else {
        return calc;
    }
}
    
double calculate_mean(std::vector<double>& vec)
{
    double sum = 0;
    int size = vec.size();

    for (int i = 0; i < size; i++) {
        sum += vec.at(i);
    }

    return sum / size;
}

double calculate_var(std::vector<double>& vec)
{
    float var = 0, mean;
    int size = vec.size();

    mean = calculate_mean(vec);

    for (int i = 0; i < size; i++) {
        var += (vec.at(i) - mean) * (vec.at(i) - mean);
    }

    return var / (size - 1);
}

double calculate_var_baro(std::vector<double>& vec)
{
    float var = 0;
    int size = vec.size();

    for (int i = 0; i < size; i++) {
        var += (vec.at(i)) * (vec.at(i));
    }

    return var / (size - 1);
}