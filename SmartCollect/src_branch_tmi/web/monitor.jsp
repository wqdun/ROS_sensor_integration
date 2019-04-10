<%@ page contentType="text/html;charset=utf-8" %>
<!DOCTYPE html>
<html lang="en">
<head>
    <%@ include file="include/header.jsp" %>
    <script>
        function PubCtrlParams() {
            recordClickedDelayCounter_ = 5;
            isStartRecord_ = !isStartRecord_;
            SetCollectButton(isStartRecord_);

            var clientMsg = new ROSLIB.Message({
                system_cmd: 7,
                cmd_arguments: Number(isStartRecord_) + ",20",
            });
            pubCmd_.publish(clientMsg);
        }

        function SetCollectButton(isRecording) {
            if (isRecording) {
                document.getElementById("collect_button").innerHTML = "Pause";
                document.getElementById("collect_button").setAttribute("class","btn btn-large btn-danger");
            }
            else {
                document.getElementById("collect_button").innerHTML = "Start";
                document.getElementById("collect_button").setAttribute("class","btn btn-large btn-success");
            }
        }

        function AlermOn(){
            bgMusic.play();
        }

        function UpdateImuStatus(_message) {
            var _isImuError = false;

            if (_message.GPStime < 0) {
                _isImuError = true;
                console.log("I got no IMU GPFPD frame.");
                document.getElementById('heading').innerHTML = "<font color=red>\"\"</font>";
                document.getElementById('location').innerHTML = "<font color=red>\"\"</font>";
                document.getElementById('speed').innerHTML = "<font color=red>\"\"</font>";
                document.getElementById('imu_status').innerHTML = "<font color=red>\"\"</font>";
                document.getElementById('gps_time').innerHTML = "<font color=red>\"\"</font>";
                document.getElementById('num').innerHTML = "<font color=red>\"\"</font>";
                document.getElementById('hdop').innerHTML = "<font color=red>\"\"</font>";
            }
            else {
                document.getElementById('heading').innerHTML = "<font color=green>" + _message.pitch_roll_heading.z.toFixed(2) + "</font>";
                document.getElementById('location').innerHTML = "<font color=green>" + _message.lat_lon_hei.x.toFixed(8) + ", " + _message.lat_lon_hei.y.toFixed(8) + "</font>";
                if (_message.speed > 120) {
                    _isImuError = true;
                    document.getElementById('speed').innerHTML = "<font color=red>" + _message.speed.toFixed(2) + "km/h</font>";
                }
                else {
                    document.getElementById('speed').innerHTML = "<font color=green>" + _message.speed.toFixed(2) + "km/h</font>";
                }
                var imuStatus = _message.status & 0xF;
                if (3 === imuStatus || 0xC === imu_status) {
                    document.getElementById('imu_status').innerHTML = "<font color=green>" + _message.status.toString(16).toUpperCase() + "</font>";
                }
                else {
                    _isImuError = true;
                    document.getElementById('imu_status').innerHTML = "<font color=red>" + _message.status.toString(16) + "</font>";
                }

                var gpsDateObj = new Date(_message.GPStime * 1000);
                var _gps_hour = gpsDateObj.getUTCHours();
                var _gps_minute = gpsDateObj.getUTCMinutes();
                var _gps_second = gpsDateObj.getUTCSeconds();
                var errSecond = Math.abs(_message.unix_time - _message.GPStime);
                errSecond = errSecond % (24 * 3600);
                if (errSecond > 1000 && errSecond < 85400) {
                    _isImuError = true;
                    document.getElementById('gps_time').innerHTML = "<font color=red>" + _gps_hour + ":" + _gps_minute + ":" + _gps_second + "</font>";
                }
                else {
                    document.getElementById('gps_time').innerHTML = "<font color=green>" + _gps_hour + ":" + _gps_minute + ":" + _gps_second + "</font>";
                }

                if (_message.no_sv < 0) {
                    _isImuError = true;
                    console.log("GPGGA contains no satellite number.");
                    document.getElementById('num').innerHTML = "<font color=red>\"\"</font>";
                    document.getElementById('hdop').innerHTML = "<font color=red>\"\"</font>";
                }
                else {
                    if (0 === _message.no_sv) {
                        _isImuError = true;
                        document.getElementById('num').innerHTML = "<font color=red>0</font>";
                    }
                    else {
                        document.getElementById('num').innerHTML = "<font color=green>" + _message.no_sv + "</font>";
                    }
                    document.getElementById('hdop').innerHTML = "<font color=green>" + _message.hdop.toFixed(2) + "</font>";
                }
            }

            return _isImuError;
        }

        function UpdateCameraStatus(_message) {
            var _isCameraError = false;

            if (_message.camera_fps < 0.01) {
                _isCameraError = true;
                document.getElementById('fps').innerHTML = "<font color=red>" + _message.camera_fps.toFixed(2) + "</font>";
            }
            else {
                document.getElementById('fps').innerHTML = "<font color=green>" + _message.camera_fps.toFixed(2) + "</font>";
            }

            return _isCameraError;
        }

        function UpdateLidarStatus(_message) {
            var _isLidarError = false;

            if (0 === _message.pps_status.length) {
                _isLidarError = true;
                console.log("I got no pps status.");
                document.getElementById('pps').innerHTML = "<font color=red>\"\"</font>";
                document.getElementById('gprmc').innerHTML = "<font color=red>\"\"</font>";
                document.getElementById('velodyne_rpm').innerHTML = "<font color=red>\"\"</font>";
            }
            else {
                if (_message.pps_status.indexOf("PPS locked") < 0) {
                    _isLidarError = true;
                    document.getElementById('pps').innerHTML = "<font color=red>" + _message.pps_status + "</font>";
                }
                else {
                    document.getElementById('pps').innerHTML = "<font color=green>" + _message.pps_status + "</font>";
                }

                if ('A' !== _message.is_gprmc_valid) {
                    _isLidarError = true;
                    document.getElementById('gprmc').innerHTML = "<font color=red>" + _message.is_gprmc_valid + "</font>";
                }
                else {
                    document.getElementById('gprmc').innerHTML = "<font color=green>" + _message.is_gprmc_valid + "</font>";
                }

                document.getElementById('velodyne_rpm').innerHTML = "<font color=green>" + _message.velodyne_rpm.toFixed(2) + "</font>";
            }

            return _isLidarError;
        }

        function UpdateFileSize(_message) {
            console.log("_message.img_num: " + _message.img_num);
            var _isFileSizeError = false;

            if (0 === _message.project_info.city_code) {
                _isFileSizeError = true;
                document.getElementById('project_info').innerHTML = "<font color=red>\"\"</font>";
                document.getElementById('lidarpkg').innerHTML = "<font color=red>\"\"</font>";
                document.getElementById('piccounts').innerHTML = "<font color=red>\"\"</font>";
                document.getElementById('raw_ins').innerHTML = "<font color=red>\"\"</font>";
                document.getElementById('timestamp_size').innerHTML = "<font color=red>\"\"</font>";
            }
            else {
                document.getElementById('project_info').innerHTML = "<font color=green>" + _message.project_info.city_code + "-"
                                                                                         + _message.project_info.daynight_code + "-"
                                                                                         + _message.project_info.task_id + "-"
                                                                                         + _message.project_info.device_id + "</font>";
                var lidarSize = (((_message.lidar_size - 1) < 0)? _message.lidar_size: (_message.lidar_size - 1)) + "M";
                document.getElementById('lidarpkg').innerHTML = "<font color=green>" + lidarSize + "</font>";
                var picCount = (_message.img_num < 0)? 0: _message.img_num;
                document.getElementById('piccounts').innerHTML = "<font color=green>" + picCount + "</font>";

                if(_message.raw_ins_size <= 0) {
                    _isFileSizeError = true;
                    document.getElementById('raw_ins').innerHTML = "<font color=red>0B</font>";
                }
                else {
                    document.getElementById('raw_ins').innerHTML = "<font color=green>" + Byte2HumanReadable(_message.raw_ins_size) + "</font>";
                }

                if(_message.timestamp_size <= 0) {
                    _isFileSizeError = true;
                    document.getElementById('timestamp_size').innerHTML = "<font color=red>0B</font>";
                }
                else {
                    document.getElementById('timestamp_size').innerHTML = "<font color=green>" + Byte2HumanReadable(_message.timestamp_size) + "</font>";
                }
            }

            return _isFileSizeError;
        }

        function UpdateHardwareStatus(_message) {
            var _isHardwareError = false;

            if (3 === _message.sc_check_camera_num) {
                document.getElementById('camera_num').innerHTML = "<font color=green>3</font>";
            }
            else {
                _isHardwareError = true;
                document.getElementById('camera_num').innerHTML = "<font color=red>" + _message.sc_check_camera_num + "( ≠ 3)</font>";
            }

            var diskUsageArr = _message.disk_usage.split(",");
            var freePercentage = 100 - parseInt(diskUsageArr[1]);
            if (freePercentage > 20) {
                document.getElementById('diskspace').innerHTML = "<font color=green>" + diskUsageArr[0] + ", " + freePercentage + "%</font>";
            }
            else {
                _isHardwareError = true;
                document.getElementById('diskspace').innerHTML = "<font color=red>" + diskUsageArr[0] + ", " + freePercentage + "%</font>";
            }

            return _isHardwareError;
        }

        function Byte2HumanReadable(sizeInByte) {
            if(sizeInByte <= 0) {
                return "0B";
            }

            if(sizeInByte < 1024) {
                return sizeInByte + "B";
            }

            if(sizeInByte < 1048576) {
                return (sizeInByte / 1024).toFixed(2) + "KB";
            }

            return (sizeInByte / 1048576).toFixed(2) + "MB";
        }

    </script>

</head>

<body>
<div class="modal fade" id="optModal" tabindex="-1" role="dialog" aria-labelledby="myModalLabel" aria-hidden="true">
    <div class="modal-dialog">
      <div class="modal-content">
        <div class="modal-header">
        </div>
        <div class="modal-body">
          <label id="optname" align="center" ><font size=4>Open the audio alarm ?</font></label>
        </div>
        <div class="modal-footer">
            <button type="button" class="btn btn-inverse" data-dismiss="modal" id="cancel" >OFF</button>
            <button type="button" class="btn btn-primary" data-dismiss="modal" id="cmtBtn" onclick="AlermOn();">ON</button>
        </div>
      </div>
    </div>
</div>

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
                            <td colspan="1">Project: <br/><a href="#" id="project_info" class="alert-link">""</a></td>
                            <td colspan="1">Location: <br/><a href="#" id="location" class="alert-link">"bbb", "cc"</a></td>
                            <td colspan="1">Camera Num: <br/><a href="#" id="camera_num" class="alert-link">""</a></td>
                            <td colspan="2" rowspan="4" style="text-align:center;">
                                <img src="http://<%=ip%>:8080/stream?topic=/camera/image6666" width="340px" alt="http://<%=ip%>:8080/stream?topic=/camera/image6666">
                            </td>
                        </tr>
                        <tr>
                            <td>Satellite:<br/> <a href="#" id="num" class="alert-link">""</a></td>
                            <td>HDOP: <br/><a href="#" id="hdop" class="alert-link">""</a></td>
                            <td>Speed: <br/><a href="#" id="speed" class="alert-link">""km/h</a></td>
                            <td>Heading: <br/><a href="#" id="heading" class="alert-link">""km/h</a></td>

                        </tr>
                        <tr>
                            <td>PPS: <br/><a href="#" id="pps" class="alert-link">""</a></td>
                            <td>GPRMC: <br/><a href="#" id="gprmc" class="alert-link">""</a></td>
                            <td>LIDAR Size: <br/><a href="#" id="lidarpkg" class="alert-link">""</a></td>
                            <td>Rawins Size: <br/><a href="#" id="raw_ins" class="alert-link">""</a></td>

                        </tr>
                        <tr>
                            <td>Camera FPS: <br/><a href="#" id="fps" class="alert-link">""</a></td>
                            <td>IMU Status: <br/><a href="#" id="imu_status" class="alert-link">""</a></td>
                            <td>Image Num: <br/><a href="#" id="piccounts" class="alert-link">""</a></td>
                            <td>LIDAR Rpm: <br/><a href="#" id="velodyne_rpm" class="alert-link">""</a></td>

                        </tr>
                        <tr>
                            <td colspan="1">SC Time: <br/><a href="#" id="unix_time" class="alert-link">""</a></td>
                            <td colspan="1">GPS Time: <br/><a href="#" id="gps_time" class="alert-link">""</a></td>
                            <td>Disk Free: <br/><a href="#" id="diskspace" class="alert-link">""</a></td>
                            <td>Image Stamp Size: <br/><a href="#" id="timestamp_size" class="alert-link">""</a></td>
                            <td colspan="2" style="text-align:center;">
                                <button class="btn btn-large btn-success" id="collect_button" onclick="PubCtrlParams();">Start</button>
                            </td>
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
    $('#optModal').modal({
        show:true,
        backdrop:'static'
    });

    var url_ = window.location.host;
    console.log("window.location.host: " + url_);

    // waiting 5 s for modify take effect: ->server
    var recordClickedDelayCounter_ = 0;
    var isStartRecord_ = false;

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

    var gonnaRingCounter = 0;
    var isGonnaRing = false;
    centerListener.subscribe(function (message) {
        var unixDateObj = new Date(message.unix_time * 1000);
        var _unix_hour = unixDateObj.getUTCHours();
        var _unix_minute = unixDateObj.getUTCMinutes();
        var _unix_second = unixDateObj.getUTCSeconds();
        document.getElementById('unix_time').innerHTML = "<font color=green>" + _unix_hour + ":" + _unix_minute + ":" + _unix_second + "</font>";

        var isImuError = UpdateImuStatus(message);
        var isCameraError = UpdateCameraStatus(message);
        var isLidarError = UpdateLidarStatus(message);
        var isFileSizeError = UpdateFileSize(message);
        var isHardwareError = UpdateHardwareStatus(message);

        // document.getElementById("collect_button").disabled = ('A' !== message.is_gprmc_valid);
        if (recordClickedDelayCounter_ !== 0) {
            --recordClickedDelayCounter_;
            console.log("Waiting modify take effect: " + recordClickedDelayCounter_);
        }
        else {
            SetCollectButton(message.is_record);
        }

        if (isImuError || isCameraError || isLidarError || isFileSizeError || isHardwareError) {
            ++gonnaRingCounter;
            gonnaRingCounter %= 10;
            if (0 === gonnaRingCounter) {
                isGonnaRing = true;
            }
            // else do nothing
        }
        else {
            gonnaRingCounter = 0;
            isGonnaRing = false;
        }

        if (isGonnaRing) {
            console.log("FIX ME!!! " + gonnaRingCounter);
            bgMusic.play();
        }

    });
</script>
</body>
</html>
