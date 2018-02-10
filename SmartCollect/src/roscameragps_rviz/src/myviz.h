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
#include "SmartCollector/clientCmd.h"
#include "sc_server_daemon/serverMsg.h"


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

    void showCenterMsg(const sc_server_daemon::serverMsg &server_msg, const sc_center::centerMsg &center_msg);

public Q_SLOTS:
    void launch_project();
    void set_ip();
    void power_off_cmd();
    void reboot_cmd();
    void collect_ctrl_onclick();
    void monitor_ctrl_onclick();


public:
    int paramNum_;
    char **params_;

    QLineEdit *pMaterIpEdit_;
    QLineEdit *pMasterNameEdit_;
    QLineEdit *pClientPasswdEdit_;
    QPushButton *pSetIpBtn_;

    QLabel *pConnStatusLabel_;

    QComboBox *pCityCodeBox_;
    QComboBox *pDayNightBox_;
    QLineEdit *pDeviceIdEdit_;
    QLineEdit *pTaskIdEdit_;
    QPushButton *pLaunchBtn_;

    QLabel *pLatLabel_;
    QLabel *pLonLabel_;
    QLabel *pGnssNumLabel_;
    QLabel *pGnssHdopLabel_;
    QLabel *pSpeedLabel_;
    QLabel *pCamFpsLabel_;
    QLabel *pPpsLabel_;
    QLabel *pGprmcLabel_;

    QLineEdit *pPasswordEdit_;

    QEventLoop eventloop;
    bool isMasterIpSet_;
    // window size
    QSize sizeHint() const;

    SmartCollector::clientCmd clientCmdMsg_;


private:
    void modifyHostsFile(const std::string &serverIp, const std::string &serverName, const std::string &clientPasswd);
    void run_center_node();

    std::vector<std::string> CITY_NAME2CODE = {
        "Beijing-1001",
        "Tianjin-1002",
        "Shanghai-1003",
    };

    void setLabelColor(QLabel *label, const QColor &color);
    char *string_as_array(string *str);

};

#endif // MYVIZ_H
