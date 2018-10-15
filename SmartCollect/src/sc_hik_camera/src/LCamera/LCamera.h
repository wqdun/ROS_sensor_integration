//
// Created by root on 9/27/17.
//

#ifndef FEATURETRACKING_LCAMERA_H
#define FEATURETRACKING_LCAMERA_H
#include <Eigen/Eigen>
extern"C"
{
class LCamera {
public:
    LCamera() noexcept;
    double mfx,mfy,mcx,mcy,mk1,mk2,mp1,mp2;
    void liftProjective(const Eigen::Vector2d& a, Eigen::Vector3d& b);
};
}


#endif //FEATURETRACKING_LCAMERA_H
