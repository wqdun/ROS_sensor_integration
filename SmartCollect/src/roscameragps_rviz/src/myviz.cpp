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

//global control
extern bool close_flag;
extern int  save_control;

static bool is_almost_equal(double d1, double d2, double compare_factor) {
    if(compare_factor < 0) {
        compare_factor = -compare_factor;
    }
    return ( (d1-d2) >= -compare_factor && (d1-d2) <= compare_factor);
}

// BEGIN_TUTORIAL
// Constructor for MyViz. This does most of the work of the class.
MyViz::MyViz(int paramNum, char **params, QWidget* parent): QWidget(parent) {
    // normal parameters
    paramNum_ = paramNum;
    params_ = params;

    setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowMinimizeButtonHint | Qt::WindowStaysOnTopHint);
    this->sizeHint();

    // Construct and lay out labels and slider controls.
    QLabel *pMasterIpLabel = new QLabel("Master IP:");
    pMaterIpEdit_ = new QLineEdit(this);
    QPushButton *pSetIpBtn = new QPushButton("Commit");

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
    pDeviceIdEdit_ = new QLineEdit(this);
    pTaskIdEdit_ = new QLineEdit(this);

    QPushButton *launchBtn = new QPushButton("Launch Project");

    QLabel *pCollectorLabel = new QLabel("Collector State:");
    pLatLabel_ = new QLabel("Lat:");
    pLonLabel_ = new QLabel("Lon:");
    pGnssNumLabel_ = new QLabel("Gnss Num:");
    pGnssHdopLabel_ = new QLabel("Gnss Hdop:");
    pSpeedLabel_ = new QLabel("Speed:");
    pCamFpsLabel_ = new QLabel("Cam Fps:");
    pPpsLabel_ = new QLabel("PPS:");
    
    pLatLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    pLonLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    pGnssNumLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    pGnssHdopLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    pSpeedLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    pCamFpsLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    pPpsLabel_->setFrameStyle(QFrame::Panel | QFrame::Sunken);

    QGridLayout* controls_layout = new QGridLayout();

    int row = -1;
    controls_layout->addWidget(pMasterIpLabel, ++row, 0);
    controls_layout->addWidget(pMaterIpEdit_, row, 1);
    controls_layout->addWidget(pSetIpBtn, row, 2);

    controls_layout->addWidget(pConnStatusLabel, ++row, 0);
    controls_layout->addWidget(pConnStatusLabel_, row, 1);
    controls_layout->addWidget(pPrjLabel, ++row, 0);
    controls_layout->addWidget(pCityCodeBox_, row, 1);
    controls_layout->addWidget(pDayNightBox_, row, 2);

    QLabel *pDeviceLabel = new QLabel("Device ID:");
    QLabel *pTaskLabel = new QLabel("Task ID:");
    controls_layout->addWidget(pTaskLabel, ++row, 1);
    controls_layout->addWidget(pDeviceLabel, row, 2);
    controls_layout->addWidget(pTaskIdEdit_, ++row, 1);
    controls_layout->addWidget(pDeviceIdEdit_, row, 2);

    controls_layout->addWidget(launchBtn, ++row, 2);

    controls_layout->addWidget(pCollectorLabel, ++row, 0);
    controls_layout->addWidget(pLatLabel_, row, 1);
    controls_layout->addWidget(pLonLabel_, row, 2);
    controls_layout->addWidget(pGnssNumLabel_, ++row, 1);
    controls_layout->addWidget(pGnssHdopLabel_, row, 2);
    controls_layout->addWidget(pSpeedLabel_, ++row, 1);
    controls_layout->addWidget(pCamFpsLabel_, row, 2);
    controls_layout->addWidget(pPpsLabel_, ++row, 1);

    controls_layout->setMargin(15);
    controls_layout->setSpacing(10);


    QVBoxLayout* main_layout = new QVBoxLayout();
    main_layout->addLayout(controls_layout);
    setLayout(main_layout);

    // Callback Functions
    connect(launchBtn, SIGNAL(clicked() ), this, SLOT(launch_project() ) );
    
    connect(pSetIpBtn, SIGNAL(clicked() ), this, SLOT(set_ip() ) );

}

MyViz::~MyViz() {
}

void MyViz::launch_project() {
    DLOG(INFO) << __FUNCTION__ << " start.";
    static ros::Publisher pub_prj_name = pNode->advertise<std_msgs::String>("sc_client_cmd", 0);
    // wait for pub_prj_name created successfully
    static int launchProjectOnce = usleep(200000);

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

    std::string projectName(cityCode + "-" + dayOrNight + "-" + taskId + deviceId);

    std_msgs::String prjInfo;
    prjInfo.data = projectName;
    pub_prj_name.publish(prjInfo);

    DLOG(INFO) << "Project cityName: " << cityName << "; cityCode: " << cityCode << "; dayOrNight: " << dayOrNight;
    LOG(INFO) << "projectName: " << projectName;
}

void MyViz::show_g_data_center_status(const sc_center::centerMsg &center_msg) {
    DLOG(INFO) << __FUNCTION__ << " start.";

    if(!center_msg.is_server_connected) {
        LOG(WARNING) << "Lost connection to server.";
        pConnStatusLabel_->setText("Signal Lost");
        // TODO: show invalid status
        return;
    }
    pConnStatusLabel_->setText("Connected to " + pMaterIpEdit_->text() );
    pLatLabel_->setText("Lat: " + QString::number(center_msg.latitude, 'f', 6) );
    pLonLabel_->setText("Lon: " + QString::number(center_msg.longitude, 'f', 6) );
    pGnssNumLabel_->setText("Gnss Num: " + QString::number(center_msg.noSV_422) );
    pGnssHdopLabel_->setText("Gnss Hdop: " + QString::number(center_msg.hdop) );
    pSpeedLabel_->setText("Speed: " + QString::number(center_msg.current_speed) + " km/h");
    pCamFpsLabel_->setText("Cam Fps: " + QString::number(center_msg.camera_fps) );
    pPpsLabel_->setText("PPS: " + QString::fromStdString(center_msg.pps_status) );
}

sc_center::centerMsg g_data_Infos;

void sub_data_center_CallBack(const sc_center::centerMsg::ConstPtr& pInfos) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    g_data_Infos = *pInfos;
}

void *ros_thread(void *ptr) {
    LOG(INFO) << __FUNCTION__ << " start.";
    MyViz *pMyviz = (MyViz *)ptr;

    ros::NodeHandle nh;
    ros::Subscriber sub_center = nh.subscribe("processed_infor_msg", 0, sub_data_center_CallBack);

    ros::Publisher pub_msg_save = nh.advertise<std_msgs::Int64>("msg_save_control", 1);
    std_msgs::Int64 msg_save_control;

    // TODO: send nh to myviz member
    pMyviz->pNode = &nh;
    // after this you can press project button

    ros::Rate loop_rate(1);
    while(ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
        pMyviz->show_g_data_center_status(g_data_Infos);
    }
}

// https://stackoverflow.com/questions/7352099/stdstring-to-char
// string to char *
static char* string_as_array(string* str) {
    return str->empty() ? NULL : &*str->begin();
}

void MyViz::set_ip() {
    LOG(INFO) << __FUNCTION__ << " start.";
    
    const std::string masterIp(pMaterIpEdit_->text().toStdString() );
    std::string rosMaterUri("ROS_MASTER_URI=http://" + masterIp + ":11311");
    char *rosMaterUriData = string_as_array(&rosMaterUri);
    int err = putenv(rosMaterUriData);
    LOG(INFO) << "Put env: "<< rosMaterUriData << " returns: " << err;


    if(!ros::isInitialized() ) {
        ros::init(paramNum_, params_, "myviz", ros::init_options::AnonymousName);
    }
    pthread_t ros_thread_id;
    int err_ros_thread = pthread_create(&ros_thread_id, NULL, ros_thread, (void *)this);
    if(0 != err_ros_thread) {
        LOG(ERROR) << "Failed to create ROS thread.";
        exit(1);
    }
}


int flag_start = 0;
int flag_end = 0;
int flag_exit = 0;

QSize MyViz::sizeHint() const {
    return QSize(600, 300);
}


