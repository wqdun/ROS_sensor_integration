<%@ page contentType="text/html;charset=utf-8" %>
<!DOCTYPE html>
<html lang="en">
<head>
  <%@ include file="include/header.jsp" %>
  <%@ include file="include/footer.jsp" %>
  <script>
    function projlayer() {
        var cityCode = $("#city").val().substr(-4);
        var dayNightCode = 1;
        if ('Day' !== $("#dayORnight").val()) {
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

    function smartcCheck_onclick() {
        var scCheckProjectName = "9999-1-T0001-000001";
        console.log('scCheckProjectName: ' + scCheckProjectName);
        $("#projname").html(scCheckProjectName);
    }

    function createProject() {
        console.log("Submit a new project.");
        var projectNameWithDate = $('#projname').text();
        var clientMsg = new ROSLIB.Message({
            system_cmd: 6,
            cmd_arguments: projectNameWithDate,
        });

        pubCmd_.publish(clientMsg);
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
              New Project
          </h4>
        </div>
        <div class="modal-body">
          Submit Project: <label id="projname"></label>
        </div>
        <div class="modal-footer">
          <button type="button" class="btn btn-default" data-dismiss="modal">
            Cancel
          </button>
          <button type="button" class="btn btn-primary" onclick="createProject();" data-dismiss="modal">
            Submit
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
            Submit
          </h4>
        </div>
        <div class="modal-body">
          Confirm Operation: <label id="optname"></label>
          <input type="hidden" id="optid" value="0"/>
        </div>
        <div class="modal-footer">
          <button type="button" class="btn btn-default" data-dismiss="modal">
            Cancel
          </button>
          <button type="button" class="btn btn-primary" data-dismiss="modal" id="cmtBtn">
            Submit
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
                        <i class="icon-edit"></i>
                        <span>Setting</span>
                    </a>
                </li>
                <li>
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
          <div class="widget ">
            <div class="widget-header">
              <i class="icon-user"></i>
              <h3>SETTINGS</h3>
            </div>
            <div class="widget-content">
              <div class="tabbable">
                <div class="tab-content">
                  <div class="tab-pane active" id="profile">
                    <div id="edit-profile" class="form-horizontal"/>
                    <div class="control-group">
                      <label class="control-label">Project Setting</label>
                      <div class="controls">
                        <select class="form-control" id='city'>
                          <option>China-1001</option>
                          <option selected>Japan-1002</option>
                          <option disabled>Test-9999</option>
                        </select>
                        <select class="form-control" id='dayORnight'>
                          <option>Day</option>
                          <option>Night</option>
                        </select>
                      </div>
                    </div>
                    <div class="control-group">
                      <label class="control-label"></label>
                      <div class="controls">
                        <input class="form-control" placeholder="Task ID" maxlength=1 id='pid'>
                        <select class="form-control" id='deviceID'>
                          <option>SC0001</option>
                          <option>SC0002</option>
                          <option>SC0003</option>
                          <option>SC0004</option>
                          <option>SC0005</option>
                          <option>SC0006</option>
                          <option>SC0007</option>
                          <option>SC0008</option>
                          <option>SC0009</option>
                          <option>SC0010</option>
                          <option>SC0011</option>
                          <option>SC0012</option>
                          <option selected>SC0013</option>
                          <option>SC0014</option>
                          <option>SC0015</option>
                          <option>SC0016</option>
                          <option>SC0017</option>
                        </select>
                      </div>
                    </div>
                    <div class="control-group">
                      <label class="control-label"></label>
                      <div class="controls">
                        <button class="btn btn-inverse" id="smartcCheck" data-toggle="modal"
                            data-target="#myModal" onclick="smartcCheck_onclick();">SmartC Check
                        </button>
                        <button type="button" class="btn btn-inverse" id="newProject" data-toggle="modal"
                            data-target="#myModal" onclick="projlayer();">Build Project
                        </button>
                        <button class="btn btn-inverse" id="clear" data-toggle="modal"
                            data-target="#optModal">Close Project
                        </button>
                      </div>
                    </div>

                    <div class="control-group">
                      <label class="control-label">Collected Projects</label>
                      <div class="controls">
                        <select class="selectpicker" multiple id='dirname'>
                        </select>
                        <div class="progress progress-striped active">
                        <div class="progress-bar progress-bar-success" role="progressbar"
                            aria-valuenow="60" aria-valuemin="0" aria-valuemax="100"
                            style="width: 100%;" id="pathbar">
                          <span class="sr-only" id="pathcontent">100%</span>
                        </div>
                      </div>
                      </div>
                    </div>

                    <br/>
                    <div class="form-actions">
                      <button class="btn btn-inverse" id="removeData" data-toggle="modal"
                          data-target="#optModal">Delete Data
                      </button>
                      <button class="btn btn-inverse" id="fixData" data-toggle="modal"
                          data-target="#optModal">Collate Data
                      </button>
                      <button class="btn btn-inverse" id="reboot" data-toggle="modal"
                          data-target="#optModal">Reboot OS
                      </button>
                      <button class="btn btn-inverse" id="shutdown" data-toggle="modal"
                          data-target="#optModal">Shutdown OS
                      </button>
                    </div>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>

  <script>
    console.log("Task ID should not empty or contains -.");
    $('#newProject').prop('disabled', true);

  // below from http://www.cnblogs.com/yuzhongwusan/archive/2012/09/03/2669022.html
  var browser = {
    versions: function() {
      var u = navigator.userAgent, app = navigator.appVersion;
      return {
        // IE内核
        trident: u.indexOf('Trident') > -1,
        // opera内核
        presto: u.indexOf('Presto') > -1,
        // 苹果、谷歌内核
        webKit: u.indexOf('AppleWebKit') > -1,
        // 火狐内核
        gecko: u.indexOf('Gecko') > -1 && u.indexOf('KHTML') === -1,
        // 是否为移动终端
        mobile: !!u.match(/AppleWebKit.*Mobile.*/)||!!u.match(/AppleWebKit/),
        // ios终端
        ios: !!u.match(/\(i[^;]+;( U;)? CPU.+Mac OS X/),
        // android终端或者uc浏览器
        android: u.indexOf('Android') > -1 || u.indexOf('Linux') > -1,
        // 是否为iPhone或者QQHD浏览器
        iPhone: u.indexOf('iPhone') > -1 || u.indexOf('Mac') > -1,
        // 是否iPad
        iPad: u.indexOf('iPad') > -1,
        // 是否web应该程序，没有头部与底部
        webApp: u.indexOf('Safari') === -1
      };
    } (),
    language: (navigator.browserLanguage || navigator.language).toLowerCase()
  };
  if(navigator.userAgent.toLowerCase().indexOf('mobile') > -1) {
    console.log("I am a mobile browser.");
    $('.selectpicker').selectpicker('mobile');
  }
  else {
    console.log("I am a desktop browser.");
  }

    var ros_ = new ROSLIB.Ros();
    ros_.connect('ws://<%=ip%>:9090');
    var pubCmd_ = new ROSLIB.Topic({
        ros: ros_,
        name: '/sc_client_cmd',
        messageType: 'sc_msgs/ClientCmd',
    });

    var doItOnce = 0;
    var serverListener_ = new ROSLIB.Topic({
      ros: ros_,
      name: '/sc_monitor',
      messageType: 'sc_msgs/MonitorMsg'
    });
    serverListener_.subscribe(
      function(message) {
        console.log("I am listening " + serverListener_.name);
        console.log("Process status: " + message.process_num + "/" + message.total_file_num);
        var fixPercent = message.process_num / (message.total_file_num + 0.00000001) * 100;
        fixPercent = fixPercent.toFixed(2);
        console.log("fixPercent: " + fixPercent);
        $("#pathbar").css("width", fixPercent + "%");
        $("#pathcontent").html(fixPercent + "%  (" + message.process_num + "/" + message.total_file_num + ")");

        console.log("city_code: " + message.project_info.city_code);
        if(0 !== message.project_info.city_code || message.total_file_num !== message.process_num) {
          console.log("You cannot fix data when project running or being fixed.");
          $('#fixData').prop('disabled', true);
        }
        else {
          $('#fixData').prop('disabled', false);
        }

        // $('#smartcCheck').prop('disabled', !$('#newProject').prop("disabled"));
        if(0 === message.project_info.city_code) {
            console.log("No project running.");
            // enable input
            $('#city').prop('disabled', false);
            $('#dayORnight').prop('disabled', false);
            $('#pid').prop('disabled', false);
            $('#deviceID').prop('disabled', false);
            $('#smartcCheck').prop('disabled', false);
            // $('#newProject').prop('disabled', false);
            $('#removeData').prop('disabled', false);
            $('#reboot').prop('disabled', false);
            $('#shutdown').prop('disabled', false);
        }
        else {
            // disable input
            $('#city').prop('disabled', true);
            $('#dayORnight').prop('disabled', true);
            $('#pid').prop('disabled', true);
            $('#deviceID').prop('disabled', true);
            $('#smartcCheck').prop('disabled', true);
            $('#newProject').prop('disabled', true);
            $('#removeData').prop('disabled', true);
            $('#reboot').prop('disabled', true);
            $('#shutdown').prop('disabled', true);

            for (var i = 0; i < city.options.length; ++i) {
                if (city.options[i].text.indexOf(message.project_info.city_code) > 0) {
                    city.options[i].selected = true;
                    break;
                }
            }

            dayORnight.options[message.project_info.daynight_code - 1].selected = true;
            $("#pid").val(message.project_info.task_id);

            for(var i = 0; i < deviceID.options.length; ++i) {
                if(deviceID.options[i].text.indexOf(message.project_info.device_id) > 0) {
                    deviceID.options[i].selected = true;
                    break;
                }
            }
        }

        if(0 === doItOnce) {
          doItOnce = 1;
          console.log("I found " + message.projects.length + " projects in " + serverListener_.name);
            for(x in message.projects) {
                if(0 === x % 2) {
                    $("#dirname").append("<option data-content=\"<span class='label label-success'>" +  message.projects[x] + "</span>\">" + message.projects[x] + "</option>");
                }
                else {
                  $("#dirname").append("<option data-content=\"<span class='label label-info'>" +  message.projects[x] + "</span>\">" + message.projects[x] + "</option>");
                }
            }
            $("#dirname").selectpicker('refresh');
        }
      }
    );

    $('#pid').bind('input propertychange', function() {
      $('#newProject').prop('disabled', $('#pid').val().length !== 1 || $('#pid').val().indexOf("-") >= 0 || 2 === document.getElementById("city").selectedIndex);
    });
    $('#city').bind('input propertychange', function() {
      $('#newProject').prop('disabled', $('#pid').val().length !== 1 || $('#pid').val().indexOf("-") >= 0 || 2 === document.getElementById("city").selectedIndex);
    });

    $('#shutdown').click(function () {
        $("#optname").html('Shutdown OS?');
        $("#optid").text("1");
    });
    $('#reboot').click(function () {
        $("#optname").html('Reboot OS?');
        $("#optid").text("2");
    });
    $('#clear').click(function () {
        $("#optname").html('Close Project?');
        $("#optid").text("3");
    });

    $('#fixData').click(function () {
        $("#optname").html('Collate Data: ' + $("#dirname").selectpicker('val'));
        $("#optid").text("4");
    });
    $('#removeData').click(function () {
        $("#optname").html('Delete Data: ' + $("#dirname").selectpicker('val'));
        $("#optid").text("5");
        // $('.selectpicker option:selected').remove();
        // $('.selectpicker').selectpicker('refresh');
    });

    $('#cmtBtn').click(function() {
        var projectSelected = $("#dirname").selectpicker('val');
        console.log("projectSelected: " + projectSelected);
        var projects = "";
        if(projectSelected) {
          for(x in projectSelected) {
            console.log("projectSelected[" + x + "]: " + projectSelected[x]);
            projects += projectSelected[x] + ",";
          }
        }
        console.log("projects: " + projects);
        projects = projects.substr(0, projects.length - 1);
        console.log("projects: " + projects);

        var clientMsg = new ROSLIB.Message({
          system_cmd: Number($("#optid").text() ),
          cmd_arguments: projects
        });
        pubCmd_.publish(clientMsg);
    })

  </script>
</body>

</html>
