<%@ page contentType="text/html;charset=utf-8" %>
<!DOCTYPE html>
<html lang="en">
<head>
    <%@ include file="include/header.jsp" %>
    <script>

        function pubCtrlParams() {
            isRecordclicked_ = 5;
            var isRecord = document.getElementById("isRecordCheckBox").checked;
            console.log("typeof(isRecord): " + typeof(isRecord) + ", isRecord: " + isRecord);
            var camGain = $("#ex1").val();
            console.log("typeof(camGain): " + typeof(camGain) + ", camGain: " + camGain);

            var clientMsg = new ROSLIB.Message({
                system_cmd: 7,
                cmd_arguments: Number(isRecord) + "," + camGain,
            });

            pubCmd_.publish(clientMsg);
        }

    </script>
</head>

<body>

<div class="modal fade" id="myModal" tabindex="-1" role="dialog" aria-labelledby="myModalLabel" aria-hidden="true">
    <input type="hidden" id="position" />
    <input type="hidden" id="pinfo" />
    <div class="modal-dialog">
        <div class="modal-content">
            <div class="modal-header">
                <button type="button" class="close" data-dismiss="modal" aria-hidden="true">
                    &times;
                </button>
                <h4 class="modal-title" id="myModalLabel">
                    记录车道变化信息
                </h4>
            </div>
            <div class="modal-body">
                <select class="form-control"  style = "width:60px;" id='startLane'>
                    <option>1</option>
                    <option selected>2</option>
                    <option>3</option>
                    <option>4</option>
                    <option>5</option>
                    <option>6</option>
                    <option>7</option>
                </select>
                变
                <select class="form-control"  style = "width:60px;" id='endLane'>
                    <option>1</option>
                    <option>2</option>
                    <option selected>3</option>
                    <option>4</option>
                    <option>5</option>
                    <option>6</option>
                    <option>7</option>
                </select>
            </div>
            <div class="modal-footer">
                <button type="button" class="btn btn-default" data-dismiss="modal">
                    取消
                </button>
                <button type="button" class="btn btn-primary" data-dismiss="modal" id="laneupdate">
                    提交
                </button>
            </div>
        </div>
    </div>
</div>

<div class="subnavbar">
    <div class="subnavbar-inner">
        <div class="container">
            <ul class="mainnav">
                <li >
                    <a href="check.jsp">
                        <i class="icon-exclamation-sign"></i>
                        <span>设备检查</span>
                    </a>
                </li>
                <li>
                    <a href="index.jsp">
                        <i class="icon-edit"></i>
                        <span>系统设置</span>
                    </a>
                </li>
                <li class="active">
                    <a href="monitor.jsp">
                        <i class="icon-bar-chart"></i>
                        <span>运行监控</span>
                    </a>
                </li>
                <li>
                    <a href="camera.jsp">
                        <i class="icon-facetime-video"></i>
                        <span>全景相机</span>
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
                            <th>连接状态:<a href="#" id="connect" class="alert-link"></a></th>
                            <th>经纬度：<a href="#" id="location" class="alert-link">"bbb", "cc"</a></th>
                            <th>卫星数：<a href="#" id="num" class="alert-link">""</a></th>
                            <th>卫星水平定位精度：<a href="#" id="hdop" class="alert-link">""</a></th>
                            <th>磁盘空间：<a href="#" id="diskspace" class="alert-link">""</a></th>
                            <th>激光包大小：<a href="#" id="lidarpkg" class="alert-link">""</a></th>
                        </tr>
                        <tr>
                            <th>方向：<a href="#" id="heading" class="alert-link">""</a></th>
                            <th>速度：<a href="#" id="speed" class="alert-link">""公里/小时</a></th>
                            <th>相机存储频率：<a href="#" id="fps" class="alert-link">""</a></th>
                            <th>PPS：<a href="#" id="pps" class="alert-link">""</a></th>
                            <th>GPRMC：<a href="#" id="gprmc" class="alert-link">""</a></th>
                            <th>照片数量：<a href="#" id="piccounts" class="alert-link">""</a><button onclick="addEvent(1001, 'test_message');">test</button></th>
                        </tr>

                        <tr >
                            <th colspan="6">
                                <p> WARNING  <i class="icon-warning-sign" style="font-size: 25px;color:orange;"></i></p>
                                <p id="warning">
                                <!-- <div class="alert alert-error">
                                    <a href="#" class="close" data-dismiss="alert">
                                        &times;
                                    </a>
                                    【<b style="color:red">1001</b>】GPS信号过弱【2018-10-28 14:23:48】
                                </div>

                                <div class="alert alert-error">
                                    <a href="#" class="close" data-dismiss="alert">
                                        &times;
                                    </a>
                                    【<b style="color:red">1002</b>】经纬度输出异常.【2018-10-28 14:23:48】
                                </div>

                                <div class="alert alert-error">
                                    <a href="#" class="close" data-dismiss="alert">
                                        &times;
                                    </a>
                                    【<b style="color:red">1003</b>】主板断开连接.【2018-10-28 14:23:48】
                                </div> -->
                                </p>
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
                        <h3>相机</h3>
                    </div>
                    <div class="widget-content">
                        <table class="table table-bordered table-striped">
                            <thead>
                            <tr>
                                <th style="text-align:center;"><label class="checkbox">
                                    <input id="isRecordCheckBox" type="checkbox"
                                           onchange="pubCtrlParams();">采集</label></th>
                                <th style="text-align:center;">1<input id="ex1" type="text"
                                                                       data-slider-min="1"
                                                                       data-slider-max="50" data-slider-step="1"
                                                                       data-slider-value='20'/>50
                                </th>
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


    var pinfo = "";
    var markposition = "";
    function marklane() {
        $('#position').val(markposition);
        $('#pinfo').val(pinfo);
    }
    $('#laneupdate').click(function () {
        $.post("http://10.42.0.1/log.jsp", {"pinfo":$('#pinfo').val(),"position":$('#position').val(),"type":"updateLane","startLane":$('#startLane').val(),"endLane":$('#endLane').val()});
    })

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
        document.getElementById('connect').innerHTML = '<font color=green>连接成功</font>';
        console.log('Connection made!');
    });
    ros_.on('close', function () {
        console.log('Connection closed.');
        document.getElementById('connect').innerHTML = '<font color=red>连接关闭</font>';
        addEvent(1001, 'test_message');
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
    var voiceCounter = -1;
    centerListener.subscribe(function (message) {
        markposition = message.lat_lon_hei.y+","+message.lat_lon_hei.x+","+message.lat_lon_hei.z;
        pinfo = message.project_info.city_code + "-" + message.project_info.daynight_code + "-" + message.project_info.task_id + "" + message.project_info.device_id;
        console.log(centerListener.name + '::heading: ' + message.pitch_roll_heading.z);

        var gpsTimeMinute = (message.GPStime / 60) % 60;
        var unixTimeMinute = (message.unix_time / 60) % 60;
        var errMinute = Math.abs(gpsTimeMinute - unixTimeMinute);
        if(errMinute > 20) {
            console.log("errMinute: " + errMinute);
            if(!isAddEventTimeError) {
                addEvent(1001, "GPS time is inconsistent with Unix time, please set Unix time.");
                isAddEventTimeError = true;
            }
        }

        // text is "sc_integrate_imu_recorder node not running"
        if (message.hdop_novatel < 0) {
            var errMsg = 'IMU节点未运行！';
            document.getElementById('location').innerHTML = "<font color=red >" + errMsg + "</font>";
            document.getElementById('num').innerHTML = "<font color=red >" + errMsg + "</font>";
            document.getElementById('hdop').innerHTML = "<font color=red >" + errMsg + "</font>";
        }
        else {
            document.getElementById('location').innerHTML = message.lat_lon_hei.x.toFixed(8) + ", " + message.lat_lon_hei.y.toFixed(8);
            document.getElementById('num').innerHTML = message.nsv1_num;
            document.getElementById('hdop').innerHTML = message.hdop_novatel.toFixed(2);
        }
        // -2 means not running
        if (message.speed <= -1.6) {
            var errMsg = 'IMU节点未运行！';
            document.getElementById('heading').innerHTML = "<font color=red >" + errMsg + "</font>";
            document.getElementById('speed').innerHTML = "<font color=red >" + errMsg + "</font>";
        }
        else {
            document.getElementById('heading').innerHTML = message.pitch_roll_heading.z.toFixed(2);
            document.getElementById('speed').innerHTML = message.speed.toFixed(2) + "公里/小时";
        }

        if (message.camera_fps <= -1.6) {
            var errMsg = '相机节点未运行！';
            document.getElementById('fps').innerHTML = "<font color=red >" + errMsg + "</font>";
        }
        else {
            document.getElementById('fps').innerHTML = message.camera_fps.toFixed(2);
        }

        if (message.pps_status.indexOf("not") > 0) {
            var errMsg = '激光雷达节点未运行！';
            document.getElementById('pps').innerHTML = "<font color=red >" + errMsg + "</font>";
            document.getElementById('gprmc').innerHTML = "<font color=red >" + errMsg + "</font>";
        }
        else {
            document.getElementById('pps').innerHTML = message.pps_status;
            document.getElementById('gprmc').innerHTML = message.is_gprmc_valid;
        }

        if ('A' == message.is_gprmc_valid) {
            $('#isRecordCheckBox').prop('disabled', false);
        }
        else {
            $('#isRecordCheckBox').prop('disabled', true);
        }

        $("#diskspace")[0].innerHTML = message.disk_usage;
        var usedPercentage = parseInt(message.disk_usage.split(",")[1]);
        if(usedPercentage > 80) {
            if(!isAddEventDiskNotEnough) {
                addEvent(1002, "Disk free space is not enough.");
                isAddEventDiskNotEnough = true;
            }
        }
        if (-2 == message.img_num) {
            var errMsg = '没有活动工程！';
            $("#piccounts")[0].innerHTML = $("#lidarpkg")[0].innerHTML = "<font color=red >" + errMsg + "</font>";
        }
        else {
            $("#piccounts")[0].innerHTML = message.img_num;
            $("#lidarpkg")[0].innerHTML = message.lidar_size + "M";
        }

        console.log("is_record: " + message.is_record);
        console.log("cam_gain: " + typeof(message.cam_gain) + ", " + message.cam_gain);
        if (isRecordclicked_ != 0) {
            --isRecordclicked_;
            console.log("Waiting modify take effect: " + isRecordclicked_);
        }
        else {
            $("#isRecordCheckBox").prop("checked", message.is_record);
            $("#ex1").slider("setValue", message.cam_gain);
            console.log("cam_gain on server is: " + $("#ex1").slider("getValue"));
        }

        if(isAddEventTimeError || isAddEventDiskNotEnough) {
            ++voiceCounter;
            voiceCounter %= 5;
            if(0 == voiceCounter) {
                addVoice();
            }
        }
    });

    $('#ex1').slider({
        tooltip: 'always'
    }).on('change', function (e) {
        console.log(e.value.oldValue + '--' + e.value.newValue);
        pubCtrlParams();
    });

</script>

<audio id="bgMusic" src="ring.mp3" autoplay />
<script>
    function addEvent(warningId, message) {
        var warningMsg = '<div class="alert alert-error"><a href="#" class="close" data-dismiss="alert">&times;</a>【<b style="color:red">'
                        + warningId + '</b>】' + message + '</div>';
        $('#warning').append(warningMsg);
    }

    function addVoice() {
        bgMusic.volume = 0;
        v = 0;
        bgMusic.play();
        var t = setInterval(function(){
            v+= 0.1;
            if(v<=1){
                bgMusic.volume = v;
            }else{
                clearInterval(t);
            }
        },50);
    }

</script>
</body>

</html>
