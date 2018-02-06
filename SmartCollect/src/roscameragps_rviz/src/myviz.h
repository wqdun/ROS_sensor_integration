#ifndef MYVIZ_H
#define MYVIZ_H

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QWidget>
#include <QLineEdit>
#include <unistd.h>
#include <QPixmap>
#include <QMessageBox>
#include <QEventLoop>
#include <QComboBox>
#include <QTimer>
#include <string>
#include "sc_center/centerMsg.h"
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>


using namespace std;

namespace rviz {
    class Display;
    class RenderPanel;
    class VisualizationManager;
}

// Class "MyViz" implements the top level widget for this example.
class MyViz: public QWidget
{
Q_OBJECT
public:
    MyViz(int paramNum, char **params, QWidget* parent = 0);
    virtual ~MyViz();

    void show_g_data_center_status(const sc_center::centerMsg &center_msg);

public Q_SLOTS:
    void launch_project();
    void set_ip();



public:
    QLineEdit *pMaterIpEdit_;


    QLabel *pConnStatusLabel_;

    QComboBox *pCityCodeBox_;
    QComboBox *pDayNightBox_;
    QLineEdit *pDeviceIdEdit_;
    QLineEdit *pTaskIdEdit_;

    QLabel *pLatLabel_;
    QLabel *pLonLabel_;
    QLabel *pGnssNumLabel_;
    QLabel *pGnssHdopLabel_;
    QLabel *pSpeedLabel_;
    QLabel *pCamFpsLabel_;
    QLabel *pPpsLabel_;

    QEventLoop eventloop;
    ros::NodeHandle *pNode;
    bool isMasterIpSet_;
    // window size
    QSize sizeHint() const;


private:
    int paramNum_;
    char **params_;

    std::vector<std::string> CITY_NAME2CODE = {
        "Beijing-1001",
        "Tianjin-1002",
        "Shanghai-1003",
    };

};

#endif // MYVIZ_H
