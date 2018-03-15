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
#include <QCheckBox>
#include <QCloseEvent>
#include <QFileDialog>

#include <string>
#include "sc_center/centerMsg.h"
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include "SmartCollector/clientCmd.h"
#include "sc_server_daemon/serverMsg.h"
#include <net/if.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include "../../sc_lib_public_tools/src/public_tools.h"
#include "../../sc_lib_public_tools/include/rapidjson/document.h"
#include "../../sc_lib_public_tools/include/rapidjson/prettywriter.h"
#include "../../sc_lib_public_tools/include/rapidjson/ostreamwrapper.h"
#include "../../sc_lib_public_tools/include/rapidjson/istreamwrapper.h"
#include "../../display_had/src/mif_read.h"

using namespace std;

namespace rviz {
    class Display;
    class RenderPanel;
    class VisualizationManager;
}

static char * string_as_array(string *str);
static int getLocalIPs(std::vector<std::string> &IPs);
// Class "MyViz" implements the top level widget for this example.
class MyViz: public QWidget
{
Q_OBJECT
public:
    MyViz(int paramNum, char **params, QWidget* parent = 0);
    virtual ~MyViz();

    void showCenterMsg(const sc_server_daemon::serverMsg &server_msg, const sc_center::centerMsg &center_msg);

public Q_SLOTS:
    void launchProject_onClicked();
    void set_ip();
    void power_off_cmd();
    void reboot_cmd();
    void record_ctrl_onStateChanged(int);
    void monitor_ctrl_onclick();
    void loadMap_onClicked();
    void cleanServer_onClicked();
    void cleanClient_onClicked();
    void setCelltest(int);

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

    QLabel *pGcamGainLabel_;
    QSlider *cell_test_slider;

    QLineEdit *pPasswordEdit_;

    QEventLoop eventloop;
    bool isMasterIpSet_;
    // window size
    QSize sizeHint() const;

    SmartCollector::clientCmd clientCmdMsg_;
    bool isKillMapThread_;

protected:
    void closeEvent(QCloseEvent *event);


private:
    void modifyHostsFile(const std::string &serverIp, const std::string &serverName, const std::string &clientPasswd);
    void run_center_node();
    void loadConfig();
    void dumpConfig();
    void setLabelColor(QLabel *label, const QColor &color);
    void enableProjectSet(bool isEnable);
    void createMapThread();

    std::vector<std::string> CITY_NAME2CODE = {
        "Beijing-1001",
        "Tianjin-1002",
        "Shanghai-1003",
    };
};

#endif // MYVIZ_H
