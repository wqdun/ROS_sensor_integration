#ifndef __POINT_H__
#define __POINT_H__


class Point2D {
public:
    Point2D()
        : x(0.0)
        , y(0.0) {
    }

    ~Point2D() {
    }


private:
    double x;
    double y;
};

#endif
