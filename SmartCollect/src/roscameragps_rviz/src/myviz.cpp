#include <QtGui/QApplication>
#include <QMovie>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QPainter>
#include <QPen>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "myviz.h"
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>
#include "client.h"


//global control
extern bool close_flag;
extern int  save_control;

static bool is_almost_equal(double d1, double d2, double compare_factor) {
    if(compare_factor < 0) {
        compare_factor = -compare_factor;
    }
    return ( (d1-d2) >= -compare_factor && (d1-d2) <= compare_factor);
}

MyViz::MyViz(int paramNum, char **params, QWidget* parent): QWidget(parent) {
    // normal parameters
    paramNum_ = paramNum;
    params_ = params;

    LOG(INFO) << "getenv(DDDDDDDDDD): " << getenv("DDDDDDDDDD");

    clientCmdMsg_.system_cmd = clientCmdMsg_.is_record = 0;

    setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowMinimizeButtonHint | Qt::WindowStaysOnTopHint | Qt::WindowCloseButtonHint);
    this->sizeHint();

    // Construct and lay out labels and slider controls.
    QLabel *pMasterIpLabel = new QLabel("Master IP:");
    pMaterIpEdit_ = new QLineEdit(this);
    pMaterIpEdit_->setPlaceholderText("Server IP");
    pMasterNameEdit_ = new QLineEdit(this);
    pMasterNameEdit_->setPlaceholderText("Server Hostname");
    pClientPasswdEdit_ = new QLineEdit(this);
    pClientPasswdEdit_->setPlaceholderText("Client Password");
    pSetIpBtn_ = new QPushButton("Connect");

    QLabel *pConnStatusLabel = new QLabel("Connection State:");
    pConnStatusLabel_ = new QLabel("Signal Lost");
    pConnStatusLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);

    QLabel *pPrjLabel = new QLabel("Project Info:");
    pCityCodeBox_ = new QComboBox(this);
    for(auto iter = CITY_NAME2CODE.cbegin(); iter != CITY_NAME2CODE.cend(); ++iter) {
        DLOG(INFO) << "City Info: " << *iter;
        pCityCodeBox_->addItem(QWidget::tr(iter->c_str() ) );
    }

    pDayNightBox_ = new QComboBox(this);
    pDayNightBox_->addItem(QWidget::tr("Day") );
    pDayNightBox_->addItem(QWidget::tr("Night") );
    pTaskIdEdit_ = new QLineEdit(this);
    pTaskIdEdit_->setPlaceholderText("Task ID");
    pDeviceIdEdit_ = new QLineEdit(this);
    pDeviceIdEdit_->setPlaceholderText("Device ID");

    pLaunchBtn_ = new QPushButton("Launch Project");

    QLabel *pCollectorLabel = new QLabel("Collector State:");
    pLatLabel_ = new QLabel("Lat:");
    pLonLabel_ = new QLabel("Lon:");
    pGnssNumLabel_ = new QLabel("Gnss Num:");
    pGnssHdopLabel_ = new QLabel("Gnss Hdop:");
    pSpeedLabel_ = new QLabel("Speed:");
    pCamFpsLabel_ = new QLabel("Cam Fps:");
    pPpsLabel_ = new QLabel("PPS:");
    pGprmcLabel_ = new QLabel("GPRMC:");

    pLatLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    pLonLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    pGnssNumLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    pGnssHdopLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    pSpeedLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    pCamFpsLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    pPpsLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    pGprmcLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);

    QLabel *pCollectCtrlLabel = new QLabel("Collector Control:");
    QCheckBox *pRecordCtrl = new QCheckBox(this);
    pRecordCtrl->setText("Recording");

    QPushButton *pMonitorBtn = new QPushButton("Display Monitor");

    QLabel *pSysCtrlLabel = new QLabel("System Control:");
    QPushButton *pCleanServerBtn = new QPushButton("Cleanup Server");
    QPushButton *pCleanClientBtn = new QPushButton("Cleanup Client");
    pPasswordEdit_ = new QLineEdit(this);
    pPasswordEdit_->setPlaceholderText("Server Password");
    QPushButton *pPoweroffBtn = new QPushButton("Power Off");
    QPushButton *pRebootBtn = new QPushButton("Reboot");

    QGridLayout* controls_layout = new QGridLayout();

    int row = -1;
    controls_layout->addWidget(pMasterIpLabel, ++row, 0);
    controls_layout->addWidget(pMaterIpEdit_, row, 1);
    controls_layout->addWidget(pMasterNameEdit_, row, 2);

    controls_layout->addWidget(pClientPasswdEdit_, ++row, 1);
    controls_layout->addWidget(pSetIpBtn_, row, 2);

    controls_layout->addWidget(pConnStatusLabel, ++row, 0);
    controls_layout->addWidget(pConnStatusLabel_, row, 1);
    controls_layout->addWidget(pPrjLabel, ++row, 0);
    controls_layout->addWidget(pCityCodeBox_, row, 1);
    controls_layout->addWidget(pDayNightBox_, row, 2);

    controls_layout->addWidget(pTaskIdEdit_, ++row, 1);
    controls_layout->addWidget(pDeviceIdEdit_, row, 2);

    controls_layout->addWidget(pLaunchBtn_, ++row, 2);

    controls_layout->addWidget(pCollectorLabel, ++row, 0);
    controls_layout->addWidget(pLatLabel_, row, 1);
    controls_layout->addWidget(pLonLabel_, row, 2);
    controls_layout->addWidget(pGnssNumLabel_, ++row, 1);
    controls_layout->addWidget(pGnssHdopLabel_, row, 2);
    controls_layout->addWidget(pSpeedLabel_, ++row, 1);
    controls_layout->addWidget(pCamFpsLabel_, row, 2);
    controls_layout->addWidget(pPpsLabel_, ++row, 1);
    controls_layout->addWidget(pGprmcLabel_, row, 2);


    controls_layout->addWidget(pCollectCtrlLabel, ++row, 0);
    controls_layout->addWidget(pRecordCtrl, row, 1);
    controls_layout->addWidget(pMonitorBtn, row, 2);

    controls_layout->addWidget(pSysCtrlLabel, ++row, 0);
    controls_layout->addWidget(pCleanServerBtn, row, 1);
    controls_layout->addWidget(pCleanClientBtn, row, 2);

    controls_layout->addWidget(pPasswordEdit_, ++row, 0);
    controls_layout->addWidget(pRebootBtn, row, 1);
    controls_layout->addWidget(pPoweroffBtn, row, 2);

    // controls_layout->setMargin(15);
    // controls_layout->setSpacing(10);

    QVBoxLayout* main_layout = new QVBoxLayout();
    main_layout->addLayout(controls_layout);
    setLayout(main_layout);

    // Callback Functions
    connect(pLaunchBtn_, SIGNAL(clicked() ), this, SLOT(launch_project() ) );
    connect(pSetIpBtn_, SIGNAL(clicked() ), this, SLOT(set_ip() ) );
    connect(pPoweroffBtn, SIGNAL(clicked() ), this, SLOT(power_off_cmd() ) );
    connect(pRebootBtn, SIGNAL(clicked() ), this, SLOT(reboot_cmd() ) );
    connect(pRecordCtrl, SIGNAL(stateChanged(int) ), this, SLOT(record_ctrl_onStateChanged(int) ) );
    connect(pMonitorBtn, SIGNAL(clicked() ), this, SLOT(monitor_ctrl_onclick() ) );
    connect(pCleanServerBtn, SIGNAL(clicked() ), this, SLOT(cleanServer_onClicked() ) );
    connect(pCleanClientBtn, SIGNAL(clicked() ), this, SLOT(cleanClient_onClicked() ) );
}

MyViz::~MyViz() {
}

void MyViz::closeEvent(QCloseEvent *event) {
    LOG(INFO) << __FUNCTION__ << " start.";
    std::string clearClientCmd("pkill sc_center_; pkill display_had_; killall rviz");
    LOG(INFO) <<"Run " << clearClientCmd;

    FILE *fpin;
    if(NULL == (fpin = popen(clearClientCmd.c_str(), "r") ) ) {
        LOG(ERROR) << "Failed to " << clearClientCmd;
        exit(1);
    }
    int err = 0;
    if(0 != (err = pclose(fpin) ) ) {
        LOG(INFO) << "Process might not exist, " << clearClientCmd << " returns: " << err;
    }
    LOG(INFO) << "Run: " << clearClientCmd << " end.";

    event->accept();
}

void MyViz::record_ctrl_onStateChanged(int _is_record) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    if(Qt::Checked == _is_record) {
        clientCmdMsg_.is_record = 1;
    }
    else {
        clientCmdMsg_.is_record = 0;
    }
}

void MyViz::launch_project() {
    DLOG(INFO) << __FUNCTION__ << " start.";

    std::string cityName(pCityCodeBox_->currentText().toStdString() );
    std::vector<std::string> parsedCityName;
    (void)boost::split(parsedCityName, cityName, boost::is_any_of("-") );
    if(2 != parsedCityName.size() ) {
        LOG(ERROR) << "Failed to parse " << cityName << ", parsedCityName.size(): " << parsedCityName.size();
        exit(1);
    }

    std::string cityCode(parsedCityName[1]);
    std::string dayOrNight(pDayNightBox_->currentText().toStdString() );
    dayOrNight = ("Day" == dayOrNight)? "1": "2";
    std::string deviceId(pDeviceIdEdit_->text().toStdString() );
    std::string taskId(pTaskIdEdit_->text().toStdString() );
    std::string prjName(cityCode + "-" + dayOrNight + "-" + taskId + deviceId);
    DLOG(INFO) << "Project cityName: " << cityName << "; cityCode: " << cityCode << "; dayOrNight: " << dayOrNight << "; projectName: " << prjName;

    time_t now = time(NULL);
    tm tmNow = { 0 };
    (void)localtime_r(&now, &tmNow);
    char today[50];
    sprintf(today, "%04d%02d%02d", (1900 + tmNow.tm_year), (1 + tmNow.tm_mon), tmNow.tm_mday);
    std::string prj_date(today);
    // get 180208 from 20180208
    prj_date = prj_date.substr(2);

    clientCmdMsg_.project_name = prjName + "-" + prj_date;

    clientCmdMsg_.system_cmd = 0;

    (void)run_center_node();
    return;
}

void MyViz::run_center_node() {
    LOG(INFO) << __FUNCTION__ << " start.";

    // put ENV again, or ROS_MASTER_URI is NULL
    const std::string masterIp(pMaterIpEdit_->text().toStdString() );
    std::string rosMaterUri("ROS_MASTER_URI=http://" + masterIp + ":11311");
    char *rosMaterUriData = string_as_array(&rosMaterUri);
    int err = putenv(rosMaterUriData);
    LOG(INFO) << "Put env: "<< rosMaterUriData << " returns: " << err;

    const std::string exePath(public_tools::PublicTools::safeReadlink("/proc/self/exe") );
    LOG(INFO) << "Get SmartCollector execute path: " << exePath;
    const std::string smartcPath(exePath.substr(0, exePath.find("/devel/") ) );
    LOG(INFO) << "Get SmartCollector path: " << smartcPath;

    const std::string cmd("bash " + smartcPath + "/src/tools/launch_project.sh client " + clientCmdMsg_.project_name);
    LOG(INFO) << "Run: " << cmd << " begin.";
    FILE *fpin;
    if(NULL == (fpin = popen(cmd.c_str(), "r") ) ) {
        LOG(ERROR) << "Failed to " << cmd;
        exit(1);
    }
    if(0 != (err = pclose(fpin) ) ) {
        LOG(ERROR) << "Failed to " << cmd << ", returns: " << err;
        exit(1);
    }
    LOG(INFO) << "Run: " << cmd << " end.";
    return;
}

void MyViz::setLabelColor(QLabel *label, const QColor &color) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    QPalette pe;
    pe.setColor(QPalette::WindowText, color);
    label->setPalette(pe);
}

void MyViz::showCenterMsg(const sc_server_daemon::serverMsg &server_msg, const sc_center::centerMsg &center_msg) {
    DLOG(INFO) << __FUNCTION__ << " start.";

    if(!server_msg.is_server_connected) {
        LOG(WARNING) << "Lost connection to server.";
        pConnStatusLabel_->setText("Signal Lost");
        // TODO: show invalid status
        return;
    }

    static int8_t lastStatus = 0;
    if(1 == server_msg.is_project_already_exist) {
        pLaunchBtn_->setText("Project exist, rename it");
        pLaunchBtn_->setEnabled(true);
    }
    else
    if(0 == server_msg.is_project_already_exist) {
        if(0 != lastStatus) {
            pLaunchBtn_->setText("Launch success");
            pLaunchBtn_->setEnabled(false);
        }
        else {
            // do nothing
        }
    }
    // -1 means initial value
    // else {
    //     pLaunchBtn_->setText("Launch Project");
    //     pLaunchBtn_->setEnabled(true);
    // }
    lastStatus = server_msg.is_project_already_exist;

    pConnStatusLabel_->setText("Connected to " + pMaterIpEdit_->text() );
    pLatLabel_->setText("Lat: " + QString::number(center_msg.latitude, 'f', 6) );
    pLonLabel_->setText("Lon: " + QString::number(center_msg.longitude, 'f', 6) );
    pGnssNumLabel_->setText("Gnss Num: " + QString::number(center_msg.noSV_422) );
    pGnssHdopLabel_->setText("Gnss Hdop: " + QString::number(center_msg.hdop) );
    pSpeedLabel_->setText("Speed: " + QString::number(center_msg.current_speed) + " km/h");
    pCamFpsLabel_->setText("Cam Fps: " + QString::number(center_msg.camera_fps) );
    pPpsLabel_->setText("PPS: " + QString::fromStdString(center_msg.pps_status) );
    pGprmcLabel_->setText("GPRMC: " + QString::fromStdString(center_msg.is_gprmc_valid) );

    if("A" == center_msg.is_gprmc_valid) {
        setLabelColor(pGprmcLabel_, Qt::green);
    }
    else {
        setLabelColor(pGprmcLabel_, Qt::red);
    }
}

static char * string_as_array2(string *str) {
    return str->empty()? NULL: &*str->begin();
}

void *ros_thread(void *pViz) {
    LOG(INFO) << __FUNCTION__ << " start.";
    LOG(INFO) << "getenv(ROS_MASTER_URI): " << getenv("ROS_MASTER_URI");
    LOG(INFO) << "getenv(ROS_IP): " << getenv("ROS_IP");

    MyViz *p_viz = (MyViz *)pViz;
    if(!ros::isInitialized() ) {
        ros::init(p_viz->paramNum_, p_viz->params_, "myviz", ros::init_options::AnonymousName);
    }
    ros::NodeHandle nh;
    Client clientor(nh, p_viz);
    clientor.run();
}

// http://www.cnblogs.com/mickole/articles/3204385.html
static int getLocalIPs(std::vector<std::string> &IPs) {
    struct ifconf ifConf;
    ifConf.ifc_len = 512;
    char buf[512];
    ifConf.ifc_buf = buf;

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockfd < 0) {
        LOG(ERROR) << "Socket error: " << sockfd;
        return -1;
    }
    // get all interfaces
    ioctl(sockfd, SIOCGIFCONF, &ifConf);
    close(sockfd);

    struct ifreq *ifReq = (struct ifreq*)buf;
    // get IPs one by one
    int ipCnt = ifConf.ifc_len / sizeof(*ifReq);
    LOG(INFO) << "Got " << ipCnt << " IPs.";
    if(ipCnt > 5) {
        LOG(ERROR) << "Got too many IPs: " << ipCnt;
        return -1;
    }

    char *ip;
    for(int i = 0; i < ipCnt; ++i) {
        ip = inet_ntoa(((struct sockaddr_in*)&(ifReq->ifr_addr))->sin_addr);
        if(NULL == ip) {
            LOG(ERROR) << "inet_ntoa error: " << ip;
            return -1;
        }
        IPs.push_back(ip);
        LOG(INFO) << "IPs[" << i << "]: " << IPs[i];
        ifReq++;
    }

    return 0;
}

void MyViz::set_ip() {
    LOG(INFO) << __FUNCTION__ << " start.";
    pMaterIpEdit_->setEnabled(false);
    pMasterNameEdit_->setEnabled(false);
    pClientPasswdEdit_->setEnabled(false);
    pSetIpBtn_->setEnabled(false);

    const std::string masterIp(pMaterIpEdit_->text().toStdString() );
    std::string rosMaterUri("ROS_MASTER_URI=http://" + masterIp + ":11311");
    char *rosMaterUriData = string_as_array(&rosMaterUri);
    int err = putenv(rosMaterUriData);
    LOG(INFO) << "Put env: "<< rosMaterUriData << " returns: " << err;

    std::vector<std::string> localIPs;
    if(0 != (err = getLocalIPs(localIPs) ) ) {
        LOG(ERROR) << "Failed to getLocalIPs: " << err;
        exit(0);
    }
    // find right IP whose front 6 chars is same with masterIP
    std::string rosIP("");
    for(auto &_ip: localIPs) {
        if(0 == _ip.find(masterIp.substr(0, 6) ) ) {
            rosIP = _ip;
            break;
        }
    }
    if(rosIP.empty() ) {
        LOG(ERROR) << "Failed to get correspond IP, masterIp: " << masterIp;
        exit(0);
    }

    if(0 != setenv("ROS_IP", rosIP.c_str(), 1) ) {
        LOG(ERROR) << "Failed to set ROS_IP: " << getenv("ROS_IP");
        exit(1);
    }
    LOG(INFO) << "Set ROS_IP: " << getenv("ROS_IP");

    const std::string masterName(pMasterNameEdit_->text().toStdString() );
    const std::string clientPassword(pClientPasswdEdit_->text().toStdString() );

    (void)modifyHostsFile(masterIp, masterName, clientPassword);

    pthread_t ros_thread_id;
    int err_ros_thread = pthread_create(&ros_thread_id, NULL, ros_thread, (void *)this);
    if(0 != err_ros_thread) {
        LOG(ERROR) << "Failed to create ROS thread.";
        exit(1);
    }
}

void MyViz::modifyHostsFile(const std::string &serverIp, const std::string &serverName, const std::string &clientPasswd) {
    LOG(INFO) << __FUNCTION__ << " start, params: " << serverIp << ", " << serverName << ", " << clientPasswd;

    std::string cmd(
        "echo " + clientPasswd + " | sudo -S su >/dev/null 2>&1; "
        + "sudo sed -i '/ " + serverName + "$/d' /etc/hosts; "
        + "sudo sh -c 'echo \"" + serverIp + " " + serverName + "\" >> /etc/hosts'"
    );
    LOG(INFO) <<"Run " << cmd;

    FILE *fpin;
    if(NULL == (fpin = popen(cmd.c_str(), "r") ) ) {
        LOG(ERROR) << "Failed to " << cmd;
        exit(1);
    }
    int err = 0;
    if(0 != (err = pclose(fpin) ) ) {
        LOG(ERROR) << "Failed to " << cmd << ", returns: " << err;
        exit(1);
    }
    LOG(INFO) << "Run: " << cmd << " end.";
    return;
}

void MyViz::power_off_cmd() {
    LOG(INFO) << __FUNCTION__ << " start.";
    const std::string passwd(pPasswordEdit_->text().toStdString() );
    clientCmdMsg_.password = passwd;
    clientCmdMsg_.system_cmd = 1;
}

void MyViz::reboot_cmd() {
    LOG(INFO) << __FUNCTION__ << " start.";
    const std::string passwd(pPasswordEdit_->text().toStdString() );
    clientCmdMsg_.password = passwd;
    clientCmdMsg_.system_cmd = 2;
}

void MyViz::monitor_ctrl_onclick() {
    LOG(INFO) << __FUNCTION__ << " start.";

    // put ENV again, or ROS_MASTER_URI is NULL
    const std::string masterIp(pMaterIpEdit_->text().toStdString() );
    std::string rosMaterUri("ROS_MASTER_URI=http://" + masterIp + ":11311");
    char *rosMaterUriData = string_as_array(&rosMaterUri);
    int err = putenv(rosMaterUriData);
    LOG(INFO) << "Put env: "<< rosMaterUriData << " returns: " << err;

    const std::string rvizExe("export LD_LIBRARY_PATH=/opt/ros/indigo/lib:$LD_LIBRARY_PATH; /opt/ros/indigo/bin/rviz >>/opt/smartc/log/monitor_ctrl_onclick.log 2>&1 &");
    LOG(INFO) <<"Run " << rvizExe;

    FILE *fpin;
    if(NULL == (fpin = popen(rvizExe.c_str(), "r") ) ) {
        LOG(ERROR) << "Failed to " << rvizExe;
        exit(1);
    }
    if(0 != (err = pclose(fpin) ) ) {
        LOG(ERROR) << "Failed to " << rvizExe << ", returns: " << err;
        exit(1);
    }
    LOG(INFO) << "Run: " << rvizExe << " end.";
    return;
}

void MyViz::cleanServer_onClicked() {
    LOG(INFO) << __FUNCTION__ << " start.";
    clientCmdMsg_.system_cmd = 3;
}

void MyViz::cleanClient_onClicked() {
    LOG(INFO) << __FUNCTION__ << " start.";
    std::string clearClientCmd("pkill display_had_; killall rviz");
    LOG(INFO) <<"Run " << clearClientCmd;

    FILE *fpin;
    if(NULL == (fpin = popen(clearClientCmd.c_str(), "r") ) ) {
        LOG(ERROR) << "Failed to " << clearClientCmd;
        exit(1);
    }
    int err = 0;
    if(0 != (err = pclose(fpin) ) ) {
        LOG(INFO) << "Process might not exist, " << clearClientCmd << " returns: " << err;
    }

    LOG(INFO) << "Run: " << clearClientCmd << " end.";
}

int flag_start = 0;
int flag_end = 0;
int flag_exit = 0;

QSize MyViz::sizeHint() const {
    return QSize(600, 300);
}

// https://stackoverflow.com/questions/7352099/stdstring-to-char
// string to char *
static char *string_as_array(string *str) {
    return str->empty()? NULL: &*str->begin();
}