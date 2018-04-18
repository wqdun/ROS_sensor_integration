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

        var nodeParamsMsg = new ROSLIB.Message({
            is_record: Number(isRecord),
            cam_gain: Number(camGain),
        });

        var clientMsg = new ROSLIB.Message({
            node_params: nodeParamsMsg,
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
          <li >
            <a href="index.jsp">
              <i class="icon-th-large"></i>
              <span>系统设置</span>
            </a>
          </li>
          <li class="active">
            <a href="monitor.jsp">
              <i class="icon-facetime-video"></i>
              <span>运行监控</span>
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
                <th>经纬度：<a href="#" id="location" class="alert-link">"", ""</a></th>
                <th>卫星数：<a href="#" id="num" class="alert-link">""</a></th>
                <th>卫星水平定位精度：<a href="#" id="hdop" class="alert-link">""</a></th>
                <th>保留位置：<a href="#" id="reserved" class="alert-link">""</a></th>
              </tr>
              <tr>
                <th>方向：<a href="#" id="heading" class="alert-link">""</a></th>
                <th>速度：<a href="#" id="speed" class="alert-link">""公里/小时</a></th>
                <th>相机存储频率：<a href="#" id="fps" class="alert-link">""</a></th>
                <th>PPS：<a href="#" id="pps" class="alert-link">""</a></th>
                <th>GPRMC：<a href="#" id="gprmc" class="alert-link">""</a></th>
              </tr>
              </thead>
               </table>
          </div>
        </div>
      </div>
      <div class="row">
        <div class="span6">
          <div class="widget">
            <div class="widget-header">
              <i class="icon-star"></i>
              <h3>地图</h3>
            </div>
            <div class="widget-content">
              <iframe src="include/map.html" frameborder="0" scrolling="no" id="maplayer"
                onload="this.height=450;this.width=500"></iframe>
            </div>
          </div>
        </div>

        <div class="span6">
          <div class="widget">
            <div class="widget-header">
              <i class="icon-list-alt"></i>
              <h3>相机</h3>
            </div>
            <div class="widget-content">
              <img src="http://<%=ip%>:8080/stream?topic=/camera/image" height="100%" width="100%" alt="http://<%=ip%>:8080/stream?topic=/camera/image">
              <table class="table table-bordered table-striped">
                <thead>
                <tr >
                  <th style="text-align:center;"><label class="checkbox">
                    <input id="isRecordCheckBox" type="checkbox"
                      onchange="pubCtrlParams();">采集</label></th>
                  <th style="text-align:center;">1<input id="ex1" type="text"
                    data-slider-min="1"
                    data-slider-max="50" data-slider-step="1"
                    data-slider-value='20' />50</th>
                </tr>
                </thead>
              </table>
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
        document.getElementById('connect').innerHTML = '<font color=green>连接成功</font>';
        console.log('Connection made!');
    });
    ros_.on('close', function () {
        console.log('Connection closed.');
        document.getElementById('connect').innerHTML = '<font color=red>连接关闭</font>';
    });

    // Create a connection to the rosbridge WebSocket server.
    ros_.connect('ws://<%=ip%>:9090');
    pubCmd_ = new ROSLIB.Topic({
        ros: ros_,
        name: '/sc_client_cmd',
        messageType: 'sc_server_daemon/clientCmd',
    });

    var centerListener = new ROSLIB.Topic({
        ros: ros_,
        name: '/sc_monitor',
        messageType: 'sc_server_daemon/monitorMsg'
    });
    centerListener.subscribe(function (message) {
        console.log(centerListener.name + '::heading: ' + message.pitch_roll_heading.z);

        // text is "sc_integrate_imu_recorder node not running"
        if(message.hdop.indexOf("not") > 0) {
            var errMsg = 'IMU组合解算数据采集节点未运行！';
            document.getElementById('location').innerHTML = "<font color=red >" + errMsg + "</font>";
            document.getElementById('num').innerHTML = "<font color=red >" + errMsg + "</font>";
            document.getElementById('hdop').innerHTML = "<font color=red >" + errMsg + "</font>";
        }
        else {
            document.getElementById('location').innerHTML = message.latitude + ", " + message.longitude;
            document.getElementById('num').innerHTML = message.noSV_422;
            document.getElementById('hdop').innerHTML = message.hdop;
        }
        // -2 means not running
        if (message.speed <= -1.6) {
            var errMsg = 'IMU实时数据采集节点未运行！';
            document.getElementById('heading').innerHTML = "<font color=red >" + errMsg + "</font>";
            document.getElementById('speed').innerHTML = "<font color=red >" + errMsg + "</font>";
        }
        else {
            document.getElementById('heading').innerHTML = message.pitch_roll_heading.z;
            document.getElementById('speed').innerHTML = message.speed.toFixed(2) + "公里/小时";
        }

        if (message.camera_fps <= -1.6) {
            var errMsg = '相机节点未运行！';
            document.getElementById('fps').innerHTML = "<font color=red >" + errMsg + "</font>";
        }
        else {
            document.getElementById('fps').innerHTML = message.camera_fps.toFixed(2);
        }

        if(message.pps_status.indexOf("not") > 0) {
            var errMsg = '激光雷达数据采集节点未运行！';
            document.getElementById('pps').innerHTML = "<font color=red >" + errMsg + "</font>";
            document.getElementById('gprmc').innerHTML = "<font color=red >" + errMsg + "</font>";
        }
        else {
            document.getElementById('pps').innerHTML = message.pps_status;
            document.getElementById('gprmc').innerHTML = message.is_gprmc_valid;
        }
    });

    var nodeCtrlParamListener = new ROSLIB.Topic({
        ros: ros_,
        name: '/sc_node_params',
        messageType: 'sc_server_daemon/nodeParams'
    });
    nodeCtrlParamListener.subscribe(function(message) {
        console.log("is_record: " + message.is_record);
        console.log("cam_gain: " + typeof(message.cam_gain) + ", " + message.cam_gain);
        if(isRecordclicked_ != 0) {
            --isRecordclicked_;
            console.log("Waiting modify take effect: " + isRecordclicked_);
            return;
        }
        $("#isRecordCheckBox").prop("checked", message.is_record);
        $("#ex1").slider("setValue", message.cam_gain);
        console.log("cam_gain on server is: " + $("#ex1").slider("getValue"));
    })

    $('#ex1').slider({
        tooltip: 'always'
    }).on('change', function (e) {
        console.log(e.value.oldValue + '--' + e.value.newValue);
        pubCtrlParams();
    });

  </script>
</body>

</html>