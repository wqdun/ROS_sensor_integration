#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <vector>
#include <cmath>
using namespace cv;
using namespace std;

const double pi = 3.1415926f;
const double RADIAN = 180.0 / pi;

struct line{
    int theta;
    int r;
};

vector<struct line> detectLine(Mat &img, int threshold){
    vector<struct line> lines;
    int diagonal = floor(sqrt(img.rows*img.rows + img.cols*img.cols));
    vector< vector<int> >p(360, vector<int>(diagonal));
    //统计数量
    for (int j = 0; j < img.rows; j++) {
        for (int i = 0; i < img.cols; i++) {
            if (img.at<unsigned char>(j, i) > 0){
                for (int theta = 0; theta < 360; theta++){
                    int r = floor(i*cos(theta / RADIAN) + j*sin(theta / RADIAN));
                    if (r < 0)
                        continue;
                    p[theta][r]++;
                }
            }
        }
    }
    //获得最大值
    for (int theta = 0; theta < 360; theta++){
        for (int r = 0; r < diagonal; r++){
            int thetaLeft = max(0, theta - 1);
            int thetaRight = min(359, theta + 1);
            int rLeft = max(0, r - 1);
            int rRight = min(diagonal - 1, r + 1);
            int tmp = p[theta][r];
            if (tmp > threshold
                && tmp > p[thetaLeft][rLeft] && tmp > p[thetaLeft][r] && tmp > p[thetaLeft][rRight]
                && tmp > p[theta][rLeft] && tmp > p[theta][rRight]
                && tmp > p[thetaRight][rLeft] && tmp > p[thetaRight][r] && tmp > p[thetaRight][rRight]){
                struct line newline;
                newline.theta = theta;
                newline.r = r;
                lines.push_back(newline);
            }
        }
    }
    return lines;
}

void drawLines(Mat &img, const vector<struct line> &lines){
    for (int i = 0; i < lines.size(); i++){
        vector<Point> points;
        int theta = lines[i].theta;
        int r = lines[i].r;

        double ct = cos(theta / RADIAN);
        double st = sin(theta / RADIAN);

        //公式 r = x*ct + y*st
        //计算左边
        int y = int(r / st);
        if (y >= 0 && y < img.rows){
            Point p(0, y);
            points.push_back(p);
        }
        //计算右边
        y = int((r - ct*(img.cols - 1)) / st);
        if (y >= 0 && y < img.rows){
            Point p(img.cols - 1, y);
            points.push_back(p);
        }
        //计算上边
        int x = int(r / ct);
        if (x >= 0 && x < img.cols){
            Point p(x, 0);
            points.push_back(p);
        }
        //计算下边
        x = int((r - st*(img.rows - 1)) / ct);
        if (x >= 0 && x < img.cols){
            Point p(x, img.rows - 1);
            points.push_back(p);
        }
        //画线
        cv::line(img, points[0], points[1], Scalar(255, 0, 0), 5, CV_AA);
    }
}

