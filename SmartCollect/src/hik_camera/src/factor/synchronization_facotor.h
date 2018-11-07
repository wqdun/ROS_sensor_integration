//
// Created by diner on 17-7-14.
//

#ifndef VINS_SYNCHRONIZATION_FACOTOR_H
#define VINS_SYNCHRONIZATION_FACOTOR_H

#include <initial/initial_aligment.hpp>

struct Synchronization_Residual {



    Synchronization_Residual(map<double, ImageFrame> &all_image_frame)
            : all_image_frame(all_image_frame) {}
    template <typename T> bool operator()(const T* const dt,
                                          T* residual) const {

        map<double, ImageFrame>::iterator frame_i;
        map<double, ImageFrame>::iterator frame_j;
        map<double, ImageFrame>::iterator frame_k;

        std::vector<double> dt_buf;
        std::vector<Eigen::Vector3d> acc_buf;
        std::vector<Eigen::Vector3d> gyr_buf;

        for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
        {
            frame_j = next(frame_i);
            for (int i = 0; i < frame_j->second.pre_integration->dt_buf.size(); ++i) {

                dt_buf.push_back(frame_j->second.pre_integration->dt_buf[i]);
                acc_buf.push_back(frame_j->second.pre_integration->acc_buf[i]);
                gyr_buf.push_back(frame_j->second.pre_integration->gyr_buf[i]);
            }
        }

        int nImu = dt / (T) 0.005; // 需要移动几个imu frame




        frame_i = all_image_frame.begin();

        // n>0 说明延时为正， imu数据向左移,舍弃后（新）部分图像
        if (nImu > 0)
        {
            for (int j = 0; j < dt_buf.size() - nImu; ++j) {

                frame_j = next(frame_i);
                int size = frame_j->second.pre_integration->dt_buf.size();
                frame_j->second.pre_integration->clear();
                for (int i = 0; i < size; ++i) {
                    frame_j->second.pre_integration->push_back(dt_buf[j+nImu],acc_buf[j+nImu],gyr_buf[j+nImu]);
                }
                frame_j->second.pre_integration->repropagate();
            }

        }
        else
        {
            frame_k = all_image_frame.end();

            for (int j = dt_buf.size() - nImu; j > 0 ; --j) {
                frame_j = prev(frame_k);
                int size = frame_j->second.pre_integration->dt_buf.size();
                frame_j->second.pre_integration->clear();

                for (int i = 0; i <size; ++i) {
                    frame_j->second.pre_integration->push_back(dt_buf[j],acc_buf[j],gyr_buf[j]);
                }
                frame_j->second.pre_integration->repropagate();
            }
            frame_j->second.pre_integration->repropagate();

        }

        // 视觉VO得到的结果
        Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);

        VectorXd tmp_b(3);
        tmp_b.setZero();
        //移动时间dt后的预积分结果
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();

//        propagate(dt, acc, gyr);
//

//        residual[0] = y_ - (x_+5-c[0])*(x_+5-c[0]);
        return true;
    }
private:
    map<double, ImageFrame> &all_image_frame;

};

class synchronization_facotor {

};


#endif //VINS_SYNCHRONIZATION_FACOTOR_H
