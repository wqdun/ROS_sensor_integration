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
                            <td>Connection: <br/><a href="#" id="connect" class="alert-link"></a></td>
                            <td colspan="3">Location: <br/><a href="#" id="location" class="alert-link">"bbb", "cc"</a></td>
                            <td colspan="2" rowspan="2"><p id="warning"></p></td>
                        </tr>
                        <tr>
                            <td>Satellite:<br/> <a href="#" id="num" class="alert-link">""</a></td>
                            <td>Hdop: <br/><a href="#" id="hdop" class="alert-link">""</a></td>
                            <td>Speed: <br/><a href="#" id="speed" class="alert-link">""km/h</a></td>
                            <td>Heading: <br/><a href="#" id="heading" class="alert-link">""km/h</a></td>

                        </tr>
                        <tr>
                            <td>PPS: <br/><a href="#" id="pps" class="alert-link">""</a></td>
                            <td>GPRMC: <br/><a href="#" id="gprmc" class="alert-link">""</a></td>
                            <td>LIDAR Size: <br/><a href="#" id="lidarpkg" class="alert-link">""</a></td>
                            <td>Rawins Size: <br/><a href="#" id="raw_ins" class="alert-link">""</a></td>
                            <td rowspan="3" style="text-align:center;">
                                <img src="http://<%=ip%>:8080/stream?topic=/camera/image0" width="160px" alt="http://<%=ip%>:8080/stream?topic=/camera/image0">
                            </td>

                            <td  rowspan="3">
                                <p id="voff" style="display: block;"><button id="voice_control" onclick="addVoice();">AlermOn</button><i class="icon-volume-off" style="color:red;">×</i>
                                </p>
                                <p id="von"><i class="icon-volume-up" style="color:green;"></i></p>

                                <p id="collect">COLLECT <input id="isRecordCheckBox" type="checkbox" onchange="pubCtrlParams();"></p>

                            </td>
                        </tr>
                        <tr>
                            <td>Camera FPS: <br/><a href="#" id="fps" class="alert-link">""</a></td>
                            <td>IMU Status: <br/><a href="#" id="imu_status" class="alert-link">""</a></td>
                            <td>Image Num: <br/><a href="#" id="piccounts" class="alert-link">""</a></td>
                            <td>LIDAR Rpm: <br/><a href="#" id="velodyne_rpm" class="alert-link">""</a></td>

                        </tr>
                        <tr>
                            <td colspan="2">SC Time: <br/><a href="#" id="unix_time" class="alert-link">""</a></td>
                            <td>Disk Free: <br/><a href="#" id="diskspace" class="alert-link">""</a></td>
                            <td>Image Stamp Size: <br/><a href="#" id="timestamp_size" class="alert-link">""</a></td>
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
                        <iframe src="scmap.jsp" frameborder="0" scrolling="no" id="maplayer"
                                onload="this.height='450px'; this.width='100%'"></iframe>
                    </div>
                </div>
            </div>
        </div>

    </div>
</div>

<%@ include file="include/footer.jsp" %>
<audio id="bgMusic" src="ring.mp3" autoplay></audio>
<script>
    function AddEvent(warningId, message) {
        var warningMsg = '<div class="alert alert-error"><a href="#" class="close" data-dismiss="alert">&times;</a>【<b style="color:red">'
            + warningId + '</b>】' + message + '</div>';
        $('#warning').append(warningMsg);
    }

    $("#von").hide();
    function addVoice() {
        $("#von").show();
        $("#voff").hide();
        bgMusic.play();
    }

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
        console.log('Connection made!');
        document.getElementById('connect').innerHTML = '<font color=green>Established</font>';
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

    var voiceCounter = -1;
    var isWarningAdded = false;

    centerListener.subscribe(function (message) {
        var isStatusError = false;
        var unixDateObj = new Date(message.unix_time * 1000);
        var _unix_hour = unixDateObj.getUTCHours();
        var _unix_minute = unixDateObj.getUTCMinutes();
        var _unix_second = unixDateObj.getUTCSeconds();
        document.getElementById('unix_time').innerHTML = _unix_hour + ":" + _unix_minute + ":" + _unix_second;

        var warningMap = new Map();

        // below is IMU related
        if (message.GPStime < 0) {
            console.log("I got no IMU frame.");
            document.getElementById('heading').innerHTML = "<font color=red >\"\"</font>";
            document.getElementById('location').innerHTML = "<font color=red >\"\"</font>";
            document.getElementById('speed').innerHTML = "<font color=red >\"\"</font>";
            document.getElementById('imu_status').innerHTML = "<font color=red >\"\"</font>";
            document.getElementById('num').innerHTML = "<font color=red>\"\"</font>";
            document.getElementById('hdop').innerHTML = "<font color=red>\"\"</font>";
        }
        else {
            document.getElementById('heading').innerHTML = "<font color=green>" + message.pitch_roll_heading.z.toFixed(2) + "</font>";
            document.getElementById('location').innerHTML = "<font color=green>" + message.lat_lon_hei.x.toFixed(8) + ", " + message.lat_lon_hei.y.toFixed(8) + "</font>";
            if (message.speed > 120) {
                document.getElementById('speed').innerHTML = "<font color=red>" + message.speed.toFixed(2) + "km/h</font>";
            }
            else {
                document.getElementById('speed').innerHTML = "<font color=green>" + message.speed.toFixed(2) + "km/h</font>";
            }
            var imuStatus = message.status & 0xF;
            if (8 === imuStatus) {
                document.getElementById('imu_status').innerHTML = "<font color=#B8860B>" + message.status.toString(16) + "</font>";
                isStatusError = true;
            }
            else if (3 === imuStatus) {
                document.getElementById('imu_status').innerHTML = "<font color=green>" + message.status.toString(16) + "</font>";
            }
            else {
                document.getElementById('imu_status').innerHTML = "<font color=red>" + message.status.toString(16) + "</font>";
                isStatusError = true;
            }

            var errSecond = Math.abs(message.unix_time - message.GPStime);
            errSecond = errSecond % (24 * 3600);
            if (errSecond > 1000 && errSecond < 85400) {
                warningMap.set(1003, "SC time error, please set SC time or check GPS time");
            }

            if (message.no_sv < 0) {
                console.log("GPGGA contains null satellite number.");
                document.getElementById('num').innerHTML = "<font color=red>\"\"</font>";
                document.getElementById('hdop').innerHTML = "<font color=red>\"\"</font>";
            }
            else {
                if (0 === message.no_sv) {
                    document.getElementById('num').innerHTML = "<font color=red>0</font>";
                }
                else {
                    document.getElementById('num').innerHTML = "<font color=green>" + message.no_sv + "</font>";
                }
                document.getElementById('hdop').innerHTML = "<font color=green>" + message.hdop.toFixed(2) + "</font>";
            }
        }

        // below is camera related
        if (message.camera_fps < 0.01) {
            document.getElementById('fps').innerHTML = "<font color=red>" + message.camera_fps.toFixed(2) + "</font>";
        }
        else {
            document.getElementById('fps').innerHTML = "<font color=green>" + message.camera_fps.toFixed(2) + "</font>";
        }

        // below is velodyne related
        if (0 === message.pps_status.length) {
            console.log("I got no pps status.");
            document.getElementById('pps').innerHTML = "<font color=red>Absent</font>";
            document.getElementById('gprmc').innerHTML = "<font color=red>Absent</font>";
            document.getElementById('velodyne_rpm').innerHTML = "<font color=red>\"\"</font>";
        }
        else {
            if (message.pps_status.indexOf("PPS locked") < 0) {
                document.getElementById('pps').innerHTML = "<font color=red>" + message.pps_status + "</font>";
            }
            else {
                document.getElementById('pps').innerHTML = "<font color=green>" + message.pps_status + "</font>";
            }

            if ('A' !== message.is_gprmc_valid) {
                document.getElementById('gprmc').innerHTML = "<font color=red>" + message.is_gprmc_valid + "</font>";
            }
            else {
                document.getElementById('gprmc').innerHTML = "<font color=green>" + message.is_gprmc_valid + "</font>";
            }

            document.getElementById('velodyne_rpm').innerHTML = "<font color=green>" + message.velodyne_rpm.toFixed(2) + "</font>";
        }

        // below is project monitor related
        console.log("message.img_num: " + message.img_num);
        if (0 === message.project_info.city_code) {
            warningMap.set(1001, "No active project");
            $("#piccounts")[0].innerHTML = $("#lidarpkg")[0].innerHTML = "<font color=red>\"\"</font>";
        }
        else {
            $("#piccounts")[0].innerHTML = (message.img_num < 0)? 0: message.img_num;
            $("#lidarpkg")[0].innerHTML = (((message.lidar_size - 1) < 0)? message.lidar_size: (message.lidar_size - 1)) + "M";
        }

        if(message.raw_ins_size < 1024) {
            document.getElementById('raw_ins').innerHTML = message.raw_ins_size + "B";
        }
        else if(message.raw_ins_size < 1048576) {
            document.getElementById('raw_ins').innerHTML = (message.raw_ins_size / 1024).toFixed(2) + "KB";
        }
        else {
            document.getElementById('raw_ins').innerHTML = (message.raw_ins_size / 1048576).toFixed(2) + "MB";
        }

        if(message.timestamp_size <= 0) {
            document.getElementById('timestamp_size').innerHTML = "<font color=red>" + message.timestamp_size + "B</font>";
        }
        else
        if(message.timestamp_size < 1024) {
            document.getElementById('timestamp_size').innerHTML = message.timestamp_size + "B";
        }
        else
        if(message.timestamp_size < 1048576) {
            document.getElementById('timestamp_size').innerHTML = (message.timestamp_size / 1024).toFixed(2) + "KB";
        }
        else {
            document.getElementById('timestamp_size').innerHTML = (message.timestamp_size / 1048576).toFixed(2) + "MB";
        }

        // below is sc check
        // if (3 !== message.sc_check_camera_num) {
        //     warningMap.set(1005, "Camera number is " + message.sc_check_camera_num + ", should be 3");
        // }

        if (message.speed > 120) {
            warningMap.set(1002, "Speed > 120km/h, please restart IMU");
        }

        var diskUsageArr = message.disk_usage.split(",");
        var freePercentage = 100 - parseInt(diskUsageArr[1]);
        if (freePercentage > 20) {
            document.getElementById('diskspace').innerHTML = "<font color=green>" + diskUsageArr[0] + ", " + freePercentage + "%</font>";
        }
        else {
            document.getElementById('diskspace').innerHTML = "<font color=red>" + diskUsageArr[0] + ", " + freePercentage + "%</font>";
        }
        if (freePercentage < 10) {
            warningMap.set(1004, "Disk free space is not enough: " + freePercentage);
        }

        $('#isRecordCheckBox').prop('disabled', ('A' !== message.is_gprmc_valid));
        if (isRecordclicked_ !== 0) {
            --isRecordclicked_;
            console.log("Waiting modify take effect: " + isRecordclicked_);
        }
        else {
            $("#isRecordCheckBox").prop("checked", message.is_record);
        }

        if (!isWarningAdded) {
            warningMap.forEach(function (value, key, map) {
                AddEvent(key, value);
            });
            isWarningAdded = true;
        }

        if (0 !== warningMap.size || isStatusError) {
            ++voiceCounter;
            voiceCounter %= 10;
            console.log("NOISE!");
            bgMusic.play();
        }
    });
</script>
</body>
</html>
