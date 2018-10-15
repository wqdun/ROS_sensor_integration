//
// Created by root on 9/27/17.
//

#include "LCamera.h"
#ifdef __cplusplus
    extern "C"
    {
#endif
        LCamera::LCamera() noexcept
        {
            mfx=520.6;
            mfy=516.2;
            mcx=319.9;
            mcy=241.3;
            mk1=0.1334;
            mk2=-0.3048;
            mp1=-0.0007598;
            mp2=-0.0005401;
        }
        void LCamera::liftProjective(const Eigen::Vector2d& a, Eigen::Vector3d& b)
        {
            double nm_x=(a(0)-mcx)/mfx;
            double nm_y=(a(1)-mcy)/mfy;
            double r2=nm_x*nm_x+nm_y*nm_y;
            double r4=r2*r2;
            double xx=nm_x*(1-mk1*r2-mk2*r4)-2*mp1*nm_x*nm_y-mp2*(r2+2*nm_x*nm_x);
            double yy=nm_y*(1-mk1*r2-mk2*r4)-2*mp2*nm_x*nm_y-mp1*(r2+2*nm_y*nm_y);
            b(0)=xx;
            b(1)=yy;
            b(2)=1;
        }
#ifdef __cplusplus
    }
#endif