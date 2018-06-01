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
        <li >
          <a href="monitor.jsp">
            <i class="icon-facetime-video"></i>
            <span>运行监控</span>
          </a>
        </li>
        <li class="active">
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
      <div class="span6">
        <div class="widget">
          <div class="widget-header">
            <i class="icon-list-alt"></i>
            <h3>相机1</h3>
          </div>
          <div class="widget-content">
            <img src="http://<%=ip%>:8080/stream?topic=/camera/image1" height="100%" width="100%" alt="http://<%=ip%>:8080/stream?topic=/camera/image1">
          </div>
        </div>
      </div>

      <div class="span6">
        <div class="widget">
          <div class="widget-header">
            <i class="icon-list-alt"></i>
            <h3>相机2</h3>
          </div>
          <div class="widget-content">
            <img src="http://<%=ip%>:8080/stream?topic=/camera/image2" height="100%" width="100%" alt="http://<%=ip%>:8080/stream?topic=/camera/image2">
          </div>
        </div>
      </div>

      <div class="span6">
        <div class="widget">
          <div class="widget-header">
            <i class="icon-list-alt"></i>
            <h3>相机3</h3>
          </div>
          <div class="widget-content">
            <img src="http://<%=ip%>:8080/stream?topic=/camera/image3" height="100%" width="100%" alt="http://<%=ip%>:8080/stream?topic=/camera/image3">
          </div>
        </div>
      </div>

      <div class="span6">
        <div class="widget">
          <div class="widget-header">
            <i class="icon-list-alt"></i>
            <h3>相机4</h3>
          </div>
          <div class="widget-content">
            <img src="http://<%=ip%>:8080/stream?topic=/camera/image4" height="100%" width="100%" alt="http://<%=ip%>:8080/stream?topic=/camera/image4">
          </div>
        </div>
      </div>

      <div class="span6">
        <div class="widget">
          <div class="widget-header">
            <i class="icon-list-alt"></i>
            <h3>相机5</h3>
          </div>
          <div class="widget-content">
            <img src="http://<%=ip%>:8080/stream?topic=/camera/image5" height="100%" width="100%" alt="http://<%=ip%>:8080/stream?topic=/camera/image5">
          </div>
        </div>
      </div>
    </div>
  </div>
</div>

<%@ include file="include/footer.jsp" %>

</body>

</html>
