<%@ page contentType="text/html;charset=utf-8" %>
<!DOCTYPE html>
<html lang="en">
<head>
    <%@ include file="include/header.jsp" %>
    <script>
        function pubCtrlParams() {
            isRecordclicked_ = 5;
            var isRecord = document.getElementById("isRecordCheckBox").checked;
            var clientMsg = new ROSLIB.Message({
                system_cmd: 7,
                cmd_arguments: Number(isRecord) + ",20",
            });
            pubCmd_.publish(clientMsg);
        }
    </script>
</head>

<body>
<div class="subnavbar">
    <div class="subnavbar-inner">
        <div class="container">
            <ul class="mainnav">
                <li>
                    <a href="index.jsp">
                        <i class="icon-edit"></i>
                        <span>Setting</span>
                    </a>
                </li>
                <li class="active">
                    <a href="monitor.jsp">
                        <i class="icon-bar-chart"></i>
                        <span>Monitor</span>
                    </a>
                </li>
                <li>
                    <a href="camera.jsp">
                        <i class="icon-facetime-video"></i>
                        <span>Image</span>
                    </a>
                </li>
            </ul>
        </div>
    </div>
</div>

<div class="main-inner">
    <div class="container">
        <div class="row">
            <div class="span12">
                <div class="widget big-stats-container">
                    <table class="table table-bordered table-striped">
                        <thead>
                        <tr>
                            <th>Connection: <a href="#" id="connect" class="alert-link"></a></th>
                            <th>Location: <a href="#" id="location" class="alert-link">"bbb", "cc"</a></th>
                            <th>Satellite: <a href="#" id="num" class="alert-link">""</a></th>
                            <th>Hdop: <a href="#" id="hdop" class="alert-link">""</a></th>
                            <th>Disk usage: <a href="#" id="diskspace" class="alert-link">""</a></th>
                            <th>LIDAR size: <a href="#" id="lidarpkg" class="alert-link">""</a></th>
                        </tr>
                        <tr>
                            <th>Heading: <a href="#" id="heading" class="alert-link">""</a></th>
                            <th>Speed: <a href="#" id="speed" class="alert-link">""km/h</a></th>
                            <th>Camera FPS: <a href="#" id="fps" class="alert-link">""</a></th>
                            <th>PPS: <a href="#" id="pps" class="alert-link">""</a></th>
                            <th>GPRMC: <a href="#" id="gprmc" class="alert-link">""</a></th>
                            <th>Image Number: <a href="#" id="piccounts" class="alert-link">""</a></th>
                        </tr>
                        <tr>
                            <th>IMU status: <a href="#" id="imu_status" class="alert-link">""</a></th>
                            <th>GPS time: <a href="#" id="gps_time" class="alert-link">""</a></th>
                            <th>SC time: <a href="#" id="unix_time" class="alert-link">""</a></th>
                            <th>LIDAR rpm: <a href="#" id="velodyne_rpm" class="alert-link">""</a></th>
                            <!-- <th><input type=button onclick="addVoice();" value="tttt"></th> -->
                        </tr>

                        <tr >
                            <th colspan="6">
                                <p>WARNING<i class="icon-warning-sign" style="font-size: 25px;color:orange;"></i></p>
                                <p id="warning"></p>
                            </th>
                        </tr>
                        </thead>
                    </table>
                </div>
            </div>
        </div>
        <div class="row">
            <div class="span12">
                <div class="widget">
                    <div class="widget-header">
                        <i class="icon-star"></i>
                        <h3>Map</h3>
                    </div>
                    <div class="widget-content">
                        <iframe src="minemap.jsp" frameborder="0" scrolling="no" id="maplayer"
                                onload="this.height='450px'; this.width='100%'"></iframe>
                    </div>
                </div>
            </div>
        </div>

            <div class="row">
            <div class="span12">
                <div class="widget">
                    <div class="widget-header">
                        <i class="icon-list-alt"></i>
                        <h3>Main Camera</h3>
                    </div>
                    <div class="widget-content">
                        <table class="table table-bordered table-striped">
                            <thead>
                            <tr>
                                <th style="text-align:center;"><label class="checkbox">
                                    <input id="isRecordCheckBox" type="checkbox"
                                           onchange="pubCtrlParams();">Collecting</label></th>
                            </tr>
                            </thead>
                        </table>
                        <img src="http://<%=ip%>:8080/stream?topic=/camera/image6666" height="100%" width="100%"
                             alt="http://<%=ip%>:8080/stream?topic=/camera/image6666">
                    </div>
                </div>
            </div>
        </div>
    </div>
</div>

<%@ include file="include/footer.jsp" %>
<script>
    var url_ = window.location.host;
    console.log("window.location.host: " + url_);

    // waiting 5 s for modify take effect: ->server
    var isRecordclicked_ = 0;
    // Connecting to ROS
    var ros_ = new ROSLIB.Ros();

    // If there is an error on the backend, an 'error' emit will be emitted.
    ros_.on('error', function (error) {
        console.log(error);
        document.getElementById('connect').innerHTML = '<font color=red>' + error + '<font>';
    });
    // Find out exactly when we made a connection.
    ros_.on('connection', function () {
        document.getElementById('connect').innerHTML = '<font color=green>Established</font>';
        console.log('Connection made!');
    });
    ros_.on('close', function () {
        console.log('Connection closed.');
        document.getElementById('connect').innerHTML = '<font color=red>Closed</font>';
    });

    // Create a connection to the rosbridge WebSocket server.
    ros_.connect('ws://<%=ip%>:9090');
    pubCmd_ = new ROSLIB.Topic({
        ros: ros_,
        name: '/sc_client_cmd',
        messageType: 'sc_msgs/ClientCmd',
    });

    var centerListener = new ROSLIB.Topic({
        ros: ros_,
        name: '/sc_monitor',
        messageType: 'sc_msgs/MonitorMsg'
    });

    var isAddEventTimeError = false;
    var isAddEventDiskNotEnough = false;
    var isAddEventCameraNumFault = false;
    var isAddEventNoProject = false;
    var isAddEventImuGone = false;

    var isAddVoice = false;
    var voiceCounter = -1;
    centerListener.subscribe(function (message) {
        var unixDateObj = new Date(message.unix_time * 1000);
        var _unix_hour = unixDateObj.getUTCHours();
        var _unix_minute = unixDateObj.getUTCMinutes();
        var _unix_second = unixDateObj.getUTCSeconds();
        document.getElementById('unix_time').innerHTML = _unix_hour + ":" + _unix_minute + ":" + _unix_second;

        // below is IMU related
        if(message.GPStime < 0) {
            console.log("I got no IMU frame.");
            document.getElementById('location').innerHTML = "<font color=red >\"\"</font>";
            document.getElementById('gps_time').innerHTML = "<font color=red>\"\"</font>";
            document.getElementById('heading').innerHTML = "<font color=red >\"\"</font>";
            document.getElementById('speed').innerHTML = "<font color=red >\"\"</font>";
            document.getElementById('imu_status').innerHTML = "<font color=red >\"\"</font>";
            document.getElementById('num').innerHTML = "<font color=red>\"\"</font>";
            document.getElementById('hdop').innerHTML = "<font color=red>\"\"</font>";
        }
        else {
            document.getElementById('location').innerHTML = "<font color=green>" + message.lat_lon_hei.x.toFixed(8) + ", " + message.lat_lon_hei.y.toFixed(8) + "</font>";
            var gpsDateObj = new Date(message.GPStime * 1000);
            var gps_hour = gpsDateObj.getUTCHours();
            var gps_minute = gpsDateObj.getUTCMinutes();
            var gps_second = gpsDateObj.getUTCSeconds();
            document.getElementById('gps_time').innerHTML = gps_hour + ":" + gps_minute + ":" + gps_second;
            document.getElementById('heading').innerHTML = "<font color=green>" + message.pitch_roll_heading.z.toFixed(2) + "</font>";
            if(message.speed > 120) {
                document.getElementById('speed').innerHTML = "<font color=red>" + message.speed.toFixed(2) + "km/h</font>";
            }
            else {
                document.getElementById('speed').innerHTML = "<font color=green>" + message.speed.toFixed(2) + "km/h</font>";
            }
            var imuStatus = message.status & 0xF;
            if(8 == imuStatus) {
                document.getElementById('imu_status').innerHTML = "<font color=yellow>" + message.status.toString(16) + "</font>";
            }
            else if(3 == imuStatus) {
                document.getElementById('imu_status').innerHTML = "<font color=green>" + message.status.toString(16) + "</font>";
            }
            else {
                document.getElementById('imu_status').innerHTML = "<font color=red>" + message.status.toString(16) + "</font>";
            }

            var errSecond = Math.abs(message.unix_time - message.GPStime);
            errSecond = errSecond % (24 * 3600);
            if(errSecond > 1000 && errSecond < 85400) {
                isAddVoice = true;
                if(!isAddEventTimeError) {
                    AddEvent(1003, "SC time error, please set SC time.");
                    isAddEventTimeError = true;
                }
            }

            if(message.no_sv < 0) {
                console.log("GPGGA contains null satellite number.");
                document.getElementById('num').innerHTML = "<font color=red>\"\"</font>";
                document.getElementById('hdop').innerHTML = "<font color=red>\"\"</font>";
            }
            else {
                if(0 == message.no_sv) {
                    document.getElementById('num').innerHTML = "<font color=red>0</font>";
                }
                else {
                    document.getElementById('num').innerHTML = "<font color=green>" + message.no_sv + "</font>";
                }
                document.getElementById('hdop').innerHTML = "<font color=green>" + message.hdop.toFixed(2) + "</font>";
            }
        }

        // below is camera related
        if(message.camera_fps < 0.01) {
            document.getElementById('fps').innerHTML = "<font color=red>" + message.camera_fps.toFixed(2) + "</font>";
        }
        else {
            document.getElementById('fps').innerHTML = "<font color=green>" + message.camera_fps.toFixed(2) + "</font>";
        }

        // below is velodyne related
        if(0 == message.pps_status.length) {
            console.log("I got no pps status.");
            document.getElementById('pps').innerHTML = "<font color=red>Absent</font>";
            document.getElementById('gprmc').innerHTML = "<font color=red>Absent</font>";
            document.getElementById('velodyne_rpm').innerHTML = "<font color=red>\"\"</font>";
        }
        else {
            if() {//TODO;

            }
            document.getElementById('pps').innerHTML = "<font color=green>" + message.pps_status + "</font>";
            if('A' != message.is_gprmc_valid) {
                document.getElementById('gprmc').innerHTML = "<font color=red>" + message.is_gprmc_valid + "</font>";
            }
            else {
                document.getElementById('gprmc').innerHTML = "<font color=green>" + message.is_gprmc_valid + "</font>";
            }
            document.getElementById('velodyne_rpm').innerHTML = message.velodyne_rpm.toFixed(2);
        }

        // below is project monitor related
        if(message.img_num < 0) {
            $("#piccounts")[0].innerHTML = $("#lidarpkg")[0].innerHTML = "<font color=red>\"\"</font>";
            isAddVoice = true;
            if(!isAddEventNoProject) {
                AddEvent(1001, "No active project.");
                isAddEventNoProject = true;
            }
        }
        else {
            $("#piccounts")[0].innerHTML = message.img_num;
            $("#lidarpkg")[0].innerHTML = message.lidar_size + "M";
        }

        // below is sc check
        if(3 != message.sc_check_camera_num) {
            isAddVoice = true;
            if(!isAddEventCameraNumFault) {
                AddEvent(1005, "Camera number is " + message.sc_check_camera_num + ", should be 3.");
                isAddEventCameraNumFault = true;
            }
        }

        if(message.speed > 120) {
            isAddVoice = true;
            if(!isAddEventImuGone) {
                AddEvent(1002, "Speed > 120km/h, please restart IMU.");
                isAddEventImuGone = true;
            }
        }

        $("#diskspace")[0].innerHTML = message.disk_usage;
        var usedPercentage = parseInt(message.disk_usage.split(",")[1]);
        if(usedPercentage > 80) {
            isAddVoice = true;
            if(!isAddEventDiskNotEnough) {
                AddEvent(1004, "Disk free space is not enough: " + message.disk_usage);
                isAddEventDiskNotEnough = true;
            }
        }

        $('#isRecordCheckBox').prop('disabled', ('A' != message.is_gprmc_valid));
        if (isRecordclicked_ != 0) {
            --isRecordclicked_;
            console.log("Waiting modify take effect: " + isRecordclicked_);
        }
        else {
            $("#isRecordCheckBox").prop("checked", message.is_record);
        }

        if(isAddVoice) {
            ++voiceCounter;
            voiceCounter %= 5;
            if(0 == voiceCounter) {
                // addVoice();
            }
        }
    });
</script>

<!-- <audio id="bgMusic" src="ring.mp3" autoplay /> -->
<script>
    function AddEvent(warningId, message) {
        var warningMsg = '<div class="alert alert-error"><a href="#" class="close" data-dismiss="alert">&times;</a>【<b style="color:red">'
                        + warningId + '</b>】' + message + '</div>';
        $('#warning').append(warningMsg);
    }

    // function addVoice() {
    //     bgMusic.volume = 0;
    //     v = 0;
    //     bgMusic.play();
    //     var t = setInterval(function(){
    //         v+= 0.1;
    //         if(v<=1){
    //             bgMusic.volume = v;
    //         }else{
    //             clearInterval(t);
    //         }
    //     },50);
    // }
</script>
</body>

</html>
