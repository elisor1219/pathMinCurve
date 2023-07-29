#ifndef POINT_HPP
#define POINT_HPP

struct Point
{
    double x;
    double y;

    // + operator
    Point operator+(const Point & rhs) const
    {
        return {x + rhs.x, y + rhs.y};
    }

    // - operator
    Point operator-(const Point & rhs) const
    {
        return {x - rhs.x, y - rhs.y};
    }

    // * operator
    Point operator*(const double & rhs) const
    {
        return {x * rhs, y * rhs};
    }

    // / operator
    Point operator/(const double & rhs) const
    {
        return {x / rhs, y / rhs};
    }
};

struct PathPoint
{
    Point coneOne;
    Point coneTwo;
};


#endif // POINT_HPP
