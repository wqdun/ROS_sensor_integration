//
//  feature_tracker.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "feature_tracker.hpp"


extern "C++"
{
int FeatureTracker::n_id = 0;

FeatureTracker::FeatureTracker(camodocal::CameraPtr m_camera,int col,int row,int max_cnt,int min_dis)
        : mask{row, col, CV_8UC1},update_finished{false},img_cnt{0}
{
    this->m_camera = m_camera;
    this->col = col;
    this->row = row;
    this->max_cnt = max_cnt;
    this->min_dis = min_dis;
    printf("init ok\n");
}
/*********************************************************tools function for feature tracker start*****************************************************/
bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

template <typename T>
void reduceVector(vector<T> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
        max_min_pts tmp;
        tmp.min = p;
        tmp.max = p;
        parallax_cnt.push_back(tmp);
    }
}

void FeatureTracker::setMask()
{
    mask.setTo(255);

    // prefer to keep features that are tracked for long time

    vector<pair<pair<int, max_min_pts>, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(make_pair(track_cnt[i], parallax_cnt[i]), make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<pair<int, max_min_pts>, pair<cv::Point2f, int>> &a, const pair<pair<int, max_min_pts>, pair<cv::Point2f, int>> &b)
    {
        return a.first.first > b.first.first;
    });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();
    parallax_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
            //if(it.second.first.x > 10 && it.second.first.y > 10)
        {
            forw_pts.push_back(it.second.first);

            ids.push_back(it.second.second);
            track_cnt.push_back(it.first.first);
            parallax_cnt.push_back(it.first.second);
            cv::circle(mask, it.second.first, 30, 0, -1);
        }
    }
    //for (auto &it: pre_pts)
    //{
    //    cv::circle(mask, it, MIN_DIST, 0, -1);
    //}
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        vector<uchar> status;

        cv::findFundamentalMat(pre_pts, forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = pre_pts.size();
        reduceVector(pre_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        reduceVector(parallax_cnt, status);

        // printf("FM ransac: %d -> %lu: %f\n", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);

    }
}

/*********************************************************tools function for feature tracker ending*****************************************************/


void FeatureTracker::readImage(const cv::Mat &_img, cv::Mat &result, double _cur_time, int _frame_cnt, vector<Point2f> &good_pts, vector<double> &track_len, vector<int>& _track_cnt)
{

    result = _img;
    cur_time = _cur_time;
    if(forw_img.empty())
        pre_img = cur_img = forw_img = _img;
    else
        forw_img = _img;

    forw_pts.clear();

    //track
    {
        if(cur_pts.size()>0)
        {
            vector<uchar> status;
            vector<float> err;

            //TS(time_track);
            calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(31, 31),5);
            //TE(time_track);
            for (int i = 0; i < int(forw_pts.size()); i++)
                if (status[i] && !inBorder(forw_pts[i]))
                    status[i] = 0;
            /////////when tracked points are not enough///////////////
            /*int nTrackedPoints = 0;
            for (int i = 0; i < int(forw_pts.size()); i++)
            {
                if (status[i] > 0)
                    nTrackedPoints++;
            }
            if(nTrackedPoints < 20)
            {
                forw_pts.clear();
                calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21),5,TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01), 0, 2e-04);
                for (int i = 0; i < int(forw_pts.size()); i++)
                    if (status[i] && !inBorder(forw_pts[i]))
                        status[i] = 0;
            }*/

            //////////////////////////////////////////////////////////
            reduceVector(pre_pts, status);
            reduceVector(cur_pts, status);
            reduceVector(forw_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(parallax_cnt, status);

            if (forw_pts.size() >= 8)
            {
                vector<uchar> status;

                cv::findFundamentalMat(cur_pts, forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
                reduceVector(cur_pts, status);
                reduceVector(pre_pts, status);
                reduceVector(forw_pts, status);
                reduceVector(ids, status);
                reduceVector(track_cnt, status);
                reduceVector(parallax_cnt, status);
            }

            if(img_cnt!=0)
            {
                for (int i = 0; i< forw_pts.size(); i++)
                {
                    //cv::line(result, pre_pts[i], forw_pts[i], cvScalar(0), 3, 8, 0);
                    good_pts.push_back(forw_pts[i]);
                    if(forw_pts[i].x < parallax_cnt[i].min.x || forw_pts[i].y < parallax_cnt[i].min.y)
                    {
                        parallax_cnt[i].min = forw_pts[i];
                    }
                    else if(forw_pts[i].x > parallax_cnt[i].max.x || forw_pts[i].y > parallax_cnt[i].max.y)
                    {
                        parallax_cnt[i].max = forw_pts[i];
                    }
                    double parallax = (cv::norm(parallax_cnt[i].max - parallax_cnt[i].min) < 2.0? 0: cv::norm(parallax_cnt[i].max - parallax_cnt[i].min));
                    track_len.push_back(std::min(1.0, 1.0 * parallax/30));
                }
                _track_cnt=track_cnt;
            }
        }
    }

    //detect
    {

        if(img_cnt==0)
        {
            rejectWithF();

            for (int i = 0; i< forw_pts.size(); i++)
            {
                good_pts.push_back(forw_pts[i]);
                if(forw_pts[i].x < parallax_cnt[i].min.x || forw_pts[i].y < parallax_cnt[i].min.y)
                {
                    parallax_cnt[i].min = forw_pts[i];
                }
                else if(forw_pts[i].x > parallax_cnt[i].max.x || forw_pts[i].y > parallax_cnt[i].max.y)
                {
                    parallax_cnt[i].max = forw_pts[i];
                }
                double parallax = (cv::norm(parallax_cnt[i].max - parallax_cnt[i].min) < 2.0? 0: cv::norm(parallax_cnt[i].max - parallax_cnt[i].min));
                track_len.push_back(std::min(1.0, 1.0 * parallax/50));
            }
            _track_cnt=track_cnt;
            for (auto &n : track_cnt)
                n++;

            setMask();
            int n_max_cnt = max_cnt - static_cast<int>(forw_pts.size());

            if(n_max_cnt>0)
            {
                n_pts.clear();
                //forw_img.copyTo(_img);
                //   TS(time_goodfeature);
                //goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.10, MIN_DIST, mask, 3, false, 0.04);
                //////////////////////////////////////////////////////////
                /*UMat eig;
                cornerMinEigenVal(forw_img, eig, 3, 3);
                double maxVal = 0;
                Mat tempmask(forw_img.rows,forw_img.cols,CV_8UC1);
                tempmask.setTo(255);
                minMaxLoc(eig, 0, &maxVal, 0, 0, tempmask);*/
                //////////////////////////////////////////////////////////
                //goodFeaturesToTrack( forw_img, n_pts, 150, 0.05, min_dis, mask);
                goodFeaturesToTrack( forw_img, n_pts, 150, 0.05, min_dis,mask);
                /*if(n_pts.size() + forw_pts.size() < n_max_cnt/2)
                {
                    n_pts.clear();
                    goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.10, min_dis, mask);
                }*/
                // TE(time_goodfeature);
            }
            else
            {
                n_pts.clear();
            }

            addPoints();
            //printf("features num after detect: %d\n",static_cast<int>(forw_pts.size()));
            pre_img = forw_img;
            pre_pts = forw_pts;

            //draw
            for (int i = 0; i < n_pts.size(); i++)
            {
                good_pts.push_back(n_pts[i]);
                track_len.push_back(0);
                _track_cnt.push_back(1);
            }
            //result = mask;

        }
        prev_un_pts = cur_un_pts;
        cur_img = forw_img;
        cur_pts = forw_pts;
        undistortedPoints2();
        prev_time = cur_time;
    }
    if(img_cnt == 0)
    {
        //update id and msg
        /*image_msg.clear();
        int num_new = 0;

        for (unsigned int i = 0;; i++)
        {
            bool completed = false;
            completed |= updateID(i);
            if (!completed)
                break;
        }
        for(int i = 0; i<ids.size(); i++)
        {
//            double x = (cur_pts[i].x - PX)/FOCUS_LENGTH_X;
//            double y = (cur_pts[i].y - PY)/FOCUS_LENGTH_Y;
//            double z = 1.0;

            Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);

            image_msg[(ids[i])] = (Vector3d(b.x() / b.z(), b.y() / b.z(), 1));
        }*/

        image_msg.clear();
        for (unsigned int i = 0;; i++)
        {
            bool completed = false;
            completed |= updateID(i);
            if (!completed)
                break;
        }
        Eigen::Matrix<double, 7, 1> img_msg_matrix;
        for(int i = 0; i < ids.size(); i++)
        {
            img_msg_matrix(0, 0) = cur_un_pts[i].x;
            img_msg_matrix(1, 0) = cur_un_pts[i].y;
            img_msg_matrix(2, 0) = 1;
            img_msg_matrix(3, 0) = cur_pts[i].x;
            img_msg_matrix(4, 0) = cur_pts[i].y;
            img_msg_matrix(5, 0) = pts_velocity[i].x;
            img_msg_matrix(6, 0) = pts_velocity[i].y;
            image_msg[(ids[i])] = img_msg_matrix;
        }

    }
    //finished and tell solver the data is ok

    update_finished = true;
}
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}


vector<cv::Point2f> FeatureTracker::undistortedPoints()
{
    vector<cv::Point2f> un_pts;
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }

    return un_pts;
}

void FeatureTracker::undistortedPoints2()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}
}

void FeatureTracker::goodFeaturesToTrack2( double _maxVal, cv::Mat _image, vector<cv::Point2f>& _corners,
                                           int maxCorners, double qualityLevel, double minDistance,
                                           cv::Mat _mask, int blockSize, int gradientSize,
                                           bool useHarrisDetector, double harrisK)
{
    /*CV_INSTRUMENT_REGION()

    CV_Assert( qualityLevel > 0 && minDistance >= 0 && maxCorners >= 0 );
    CV_Assert( _mask.empty() || (_mask.type() == CV_8UC1 && _mask.sameSize(_image)) );

    CV_OCL_RUN(_image.dims() <= 2 && _image.isUMat(),
               ocl_goodFeaturesToTrack(_image, _corners, maxCorners, qualityLevel, minDistance,
                                    _mask, blockSize, gradientSize, useHarrisDetector, harrisK))*/

    Mat image = _image; Mat eig, tmp;
    /*if (image.empty())
    {
        _corners.release();
        return;
    }*/

    // Disabled due to bad accuracy
    /*CV_OVX_RUN(false && useHarrisDetector && _mask.empty() &&
               !ovx::skipSmallImages<VX_KERNEL_HARRIS_CORNERS>(image.cols, image.rows),
               openvx_harris(image, _corners, maxCorners, qualityLevel, minDistance, blockSize, gradientSize, harrisK))*/

    if( useHarrisDetector )
        cornerHarris( image, eig, blockSize, gradientSize, harrisK );
    else
        cornerMinEigenVal( image, eig, blockSize, gradientSize );

    double maxVal = 0;
    //minMaxLoc( eig, 0, &maxVal, 0, 0, _mask );
    maxVal = _maxVal;
    threshold( eig, eig, maxVal*qualityLevel, 0, THRESH_TOZERO );
    dilate( eig, tmp, Mat());

    Size imgsize = image.size();
    std::vector<const float*> tmpCorners;

    // collect list of pointers to features - put them into temporary image
    Mat mask = _mask;
    for( int y = 1; y < imgsize.height - 1; y++ )
    {
        const float* eig_data = (const float*)eig.ptr(y);
        const float* tmp_data = (const float*)tmp.ptr(y);
        const uchar* mask_data = mask.data ? mask.ptr(y) : 0;

        for( int x = 1; x < imgsize.width - 1; x++ )
        {
            float val = eig_data[x];
            if( val != 0 && val == tmp_data[x] && (!mask_data || mask_data[x]) )
                tmpCorners.push_back(eig_data + x);
        }
    }

    std::vector<Point2f> corners;
    size_t i, j, total = tmpCorners.size(), ncorners = 0;

    if (total == 0)
    {
        _corners.clear();
        return;
    }

    std::sort( tmpCorners.begin(), tmpCorners.end(), greaterThanPtr() );

    if (minDistance >= 1)
    {
        // Partition the image into larger grids
        int w = image.cols;
        int h = image.rows;

        const int cell_size = cvRound(minDistance);
        const int grid_width = (w + cell_size - 1) / cell_size;
        const int grid_height = (h + cell_size - 1) / cell_size;

        std::vector<std::vector<Point2f> > grid(grid_width*grid_height);

        minDistance *= minDistance;

        for( i = 0; i < total; i++ )
        {
            int ofs = (int)((const uchar*)tmpCorners[i] - eig.ptr());
            int y = (int)(ofs / eig.step);
            int x = (int)((ofs - y*eig.step)/sizeof(float));

            bool good = true;

            int x_cell = x / cell_size;
            int y_cell = y / cell_size;

            int x1 = x_cell - 1;
            int y1 = y_cell - 1;
            int x2 = x_cell + 1;
            int y2 = y_cell + 1;

            // boundary check
            x1 = std::max(0, x1);
            y1 = std::max(0, y1);
            x2 = std::min(grid_width-1, x2);
            y2 = std::min(grid_height-1, y2);

            for( int yy = y1; yy <= y2; yy++ )
            {
                for( int xx = x1; xx <= x2; xx++ )
                {
                    std::vector <Point2f> &m = grid[yy*grid_width + xx];

                    if( m.size() )
                    {
                        for(j = 0; j < m.size(); j++)
                        {
                            float dx = x - m[j].x;
                            float dy = y - m[j].y;

                            if( dx*dx + dy*dy < minDistance )
                            {
                                good = false;
                                goto break_out;
                            }
                        }
                    }
                }
            }

            break_out:

            if (good)
            {
                grid[y_cell*grid_width + x_cell].push_back(Point2f((float)x, (float)y));

                corners.push_back(Point2f((float)x, (float)y));
                ++ncorners;

                if( maxCorners > 0 && (int)ncorners == maxCorners )
                    break;
            }
        }
    }
    else
    {
        for( i = 0; i < total; i++ )
        {
            int ofs = (int)((const uchar*)tmpCorners[i] - eig.ptr());
            int y = (int)(ofs / eig.step);
            int x = (int)((ofs - y*eig.step)/sizeof(float));

            corners.push_back(Point2f((float)x, (float)y));
            ++ncorners;
            if( maxCorners > 0 && (int)ncorners == maxCorners )
                break;
        }
    }

    _corners = corners;
    //Mat(corners).convertTo(_corners, _corners.fixedType() ? _corners.type() : CV_32F);
}

