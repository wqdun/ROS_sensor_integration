<%@ page contentType="text/html;charset=utf-8" %>
<!DOCTYPE html>
<html lang="en">
<head>
  <%@ include file="include/header.jsp" %>
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
                <li>
                    <a href="monitor.jsp">
                        <i class="icon-bar-chart"></i>
                        <span>Monitor</span>
                    </a>
                </li>
                <li class="active">
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
      <div class="span6">
        <div class="widget">
          <div class="widget-header">
            <i class="icon-list-alt"></i>
            <h3>Camera 1</h3>
          </div>
          <div class="widget-content">
            <a href="img.jsp?camerid=5555">
            <img src="http://<%=ip%>:8080/stream?topic=/camera/image5555" height="100%" width="100%" alt="http://<%=ip%>:8080/stream?topic=/camera/image5555">
            </a>
          </div>
        </div>
      </div>

      <div class="span6">
        <div class="widget">
          <div class="widget-header">
            <i class="icon-list-alt"></i>
            <h3>Camera 2</h3>
          </div>
          <div class="widget-content">
            <a href="img.jsp?camerid=6666">
            <img src="http://<%=ip%>:8080/stream?topic=/camera/image6666" height="100%" width="100%" alt="http://<%=ip%>:8080/stream?topic=/camera/image6666">
            </a>
          </div>
        </div>
      </div>

      <div class="span6">
        <div class="widget">
          <div class="widget-header">
            <i class="icon-list-alt"></i>
            <h3>Camera 3</h3>
          </div>
          <div class="widget-content">
            <a href="img.jsp?camerid=7777">
            <img src="http://<%=ip%>:8080/stream?topic=/camera/image7777" height="100%" width="100%" alt="http://<%=ip%>:8080/stream?topic=/camera/image7777">
            </a>
          </div>
        </div>
      </div>
    </div>
  </div>
</div>

<%@ include file="include/footer.jsp" %>

</body>

</html>
