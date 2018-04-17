<%@ page contentType="text/html;charset=utf-8" %>
<!DOCTYPE html>
<html lang="en">
<head>
  <%@ include file="include/header.jsp" %>
  <script>
    function projlayer() {
        var cityCode = document.getElementById('city').value.substr(-4);
        var dayNightCode = 1;
        if ('白天' != document.getElementById('dayORnight').value) {
            dayNightCode = 2;
        }

        var dateToday = new Date();
        var dateSys = dateToday.getDate();
        var monthSys = dateToday.getMonth() + 1;
        var yearSys = dateToday.getFullYear();
        if (monthSys >= 1 && monthSys <= 9) {
            monthSys = "0" + monthSys;
        }
        if (dateSys >= 0 && dateSys <= 9) {
            dateSys = "0" + dateSys;
        }
        var tdateSys = (yearSys + "").substr(2) + monthSys + dateSys;

        var deviceId = document.getElementById('deviceID').value.substr(-4);

        var projectNameWithDate = cityCode + "-" + dayNightCode + "-" + document.getElementById('pid').value + deviceId + "-" + tdateSys;
        console.log('projectNameWithDate: ' + projectNameWithDate);
        $("#projname").html(projectNameWithDate);
    }

    function createProject() {
        console.log("Submit a new project.");
        var projectNameWithDate = $('#projname').text();
        var clientMsg = new ROSLIB.Message({
            project_name: projectNameWithDate,
            system_cmd: 0,
        });

        pubCmd_.publish(clientMsg);
    }

    function pubCtrlParams() {
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

    function setCookie(name, value) {
        var Days = 30;
        var exp = new Date();
        exp.setTime(exp.getTime() + Days * 24 * 60 * 60 * 1000);
        document.cookie = name + "=" + escape(value) + ";expires=" + exp.toGMTString();
    }

    function getCookie(name) {
        var arr, reg = new RegExp("(^| )" + name + "=([^;]*)(;|$)");
        if (arr = document.cookie.match(reg))
            return unescape(arr[2]);
        else
            return null;
    }

  </script>
</head>

<body>

  <div class="modal fade" id="myModal" tabindex="-1" role="dialog" aria-labelledby="myModalLabel" aria-hidden="true">
    <div class="modal-dialog">
      <div class="modal-content">
        <div class="modal-header">
          <button type="button" class="close" data-dismiss="modal" aria-hidden="true">
            &times;
          </button>
          <h4 class="modal-title" id="myModalLabel">
              新建工程
          </h4>
        </div>
        <div class="modal-body">
          提交新工程：<label id="projname"></label>
        </div>
        <div class="modal-footer">
          <button type="button" class="btn btn-default" data-dismiss="modal">
            取消
          </button>
          <button type="button" class="btn btn-primary" onclick="createProject();" data-dismiss="modal">
            提交
          </button>
        </div>
      </div>
    </div>
  </div>

  <div class="modal fade" id="optModal" tabindex="-1" role="dialog" aria-labelledby="myModalLabel" aria-hidden="true">
    <div class="modal-dialog">
      <div class="modal-content">
        <div class="modal-header">
          <button type="button" class="close" data-dismiss="modal" aria-hidden="true">
            &times;
          </button>
          <h4 class="modal-title" id="optModalLabel">
            操作提交
          </h4>
        </div>
        <div class="modal-body">
          确认要提交操作？<label id="optname"></label>
          <input type="hidden" id="optid" value="0"/>
        </div>
        <div class="modal-footer">
          <button type="button" class="btn btn-default" data-dismiss="modal">
            取消
          </button>
          <button type="button" class="btn btn-primary" data-dismiss="modal" id="cmtBtn">
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
          <li class="active">
            <a href="index.jsp">
              <i class="icon-th-large"></i>
              <span>系统设置</span>
            </a>
          </li>
          <li>
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
          <div class="widget ">
            <div class="widget-header">
              <i class="icon-user"></i>
              <h3>系统设置</h3>
            </div>
            <div class="widget-content">
              <div class="tabbable">
                <div class="tab-content">
                  <div class="tab-pane active" id="profile">
                    <form id="edit-profile" class="form-horizontal"/>
                    <div class="control-group">
                      <label class="control-label">工程信息</label>
                      <div class="controls">
                        <select class="form-control" id='city'>
                          <option>北京-1001</option>
                          <option>天津-1002</option>
                        </select>
                        <select class="form-control" id='dayORnight'>
                          <option>白天</option>
                          <option>晚上</option>
                        </select>
                      </div>
                    </div>
                    <div class="control-group">
                      <label class="control-label"></label>
                      <div class="controls">
                        <input class="form-control" placeholder="任务ID" id='pid'>
                        <select class="form-control" id='deviceID'>
                          <option>设备号-0001</option>
                          <option>设备号-0002</option>
                          <option>设备号-0003</option>
                          <option>设备号-0004</option>
                          <option>设备号-0005</option>
                          <option>设备号-0006</option>
                          <option>设备号-0007</option>
                          <option>设备号-0008</option>
                          <option>设备号-0009</option>
                          <option>设备号-0010</option>
                          <option>设备号-0011</option>
                        </select>
                      </div>
                    </div>
                    <div class="control-group">
                      <label class="control-label"></label>
                      <div class="controls">
                        <button type="button" class="btn btn-inverse" id="newProject" data-toggle="modal"
                            data-target="#myModal" onclick="projlayer();">新建工程
                        </button>
                        <button class="btn btn-inverse" id="clear" data-toggle="modal"
                            data-target="#optModal">关闭工程
                        </button>
                      </div>
                    </div>
                    <br/>
                    <div class="form-actions">
                      <button class="btn btn-inverse" id="fixData" data-toggle="modal"
                          data-target="#optModal">整理数据
                      </button>
                      <button class="btn btn-inverse" id="reboot" data-toggle="modal"
                          data-target="#optModal">重启
                      </button>
                      <button class="btn btn-inverse" id="shutdown" data-toggle="modal"
                          data-target="#optModal">关机
                      </button>
                    </div>
                    </form>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>

  <%@ include file="include/footer.jsp" %>
  <script>

    console.log("Task ID should not empty or contains -.");
    $('#newProject').prop('disabled', true);

    var ros_ = new ROSLIB.Ros();
    ros_.connect('ws://<%=ip%>:9090');
    var pubCmd_ = new ROSLIB.Topic({
        ros: ros_,
        name: '/sc_client_cmd',
        messageType: 'sc_server_daemon/clientCmd',
    });

    var projectInfoListener = new ROSLIB.Topic({
        ros: ros_,
        name: '/sc_project_info',
        messageType: 'sc_server_daemon/projectInfoMsg'
    });
    projectInfoListener.subscribe(function (message) {
        console.log("city_code: " + message.city_code);
        if (0 == message.city_code) {
            console.log("No project is committed to server.");
            // enable input
            $('#city').prop('disabled', false);
            $('#dayORnight').prop('disabled', false);
            $('#pid').prop('disabled', false);
            $('#deviceID').prop('disabled', false);
            // $('#newProject').prop('disabled', false);
        }
        else {
            // disable input
            $('#city').prop('disabled', true);
            $('#dayORnight').prop('disabled', true);
            $('#pid').prop('disabled', true);
            $('#deviceID').prop('disabled', true);
            $('#newProject').prop('disabled', true);

            for (var i = 0; i < city.options.length; ++i) {
                if (city.options[i].text.indexOf(message.city_code) > 0) {
                    city.options[i].selected = true;
                    break;
                }
            }

            console.log("daynight_code: " + typeof(message.daynight_code) + ", " + message.daynight_code);
            dayORnight.options[message.daynight_code - 1].selected = true;

            $("#pid").val(message.task_id);

            console.log("device_id: " + message.device_id);
            for (var i = 0; i < deviceID.options.length; ++i) {
                if (deviceID.options[i].text.indexOf(message.device_id) > 0) {
                    deviceID.options[i].selected = true;
                    break;
                }
            }

        }
    })

    $('#pid').bind('input propertychange', function() {
        if ($('#pid').val() == '' || $('#pid').val().indexOf("-") >= 0) {
            console.log("Task ID should not empty or contains -.");
            $('#newProject').prop('disabled', true);
        }
        else {
            $('#newProject').prop('disabled', false);
        }
    });

    $('#clear').click(function () {
        $("#optname").html('关闭工程');
        $("#optid").text("3");
    })
    $('#reboot').click(function () {
        $("#optname").html('重启');
        $("#optid").text("2");
    })
    $('#shutdown').click(function () {
        $("#optname").html('关机');
        $("#optid").text("1");
    })

    $('#cmtBtn').click(function () {
        var optid = $("#optid").text();
        var clientMsg = new ROSLIB.Message({
            project_name: '',
            system_cmd: Number(optid),
        });
        pubCmd_.publish(clientMsg);
    })

  </script>
</body>

</html>
