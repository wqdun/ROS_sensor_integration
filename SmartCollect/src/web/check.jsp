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
                <li class="active">
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
                <li>
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
                <div class="widget ">
                    <div class="widget-header">
                        <i class="icon-cog"></i>
                        <h3>设备检查</h3>
                    </div>
                    <div class="widget-content">
                        <div class="tabbable">
                            <div class="tab-content">
                                <div class="tab-pane active" id="profile">



                                    <table class="table table-bordered table-striped" >
                                        <tr>
                                            <th style='text-align: center;width:20%;vertical-align: middle;'><h4>惯导</h4></th>
                                            <th style='text-align: center;width:20%;vertical-align: middle;'><i class="icon-ok-sign" style="color: green;font-size:25px"></i></th>
                                            <th colspan="2"></th>
                                        </tr>
                                        <tr>
                                            <th style='text-align: center;vertical-align: middle;'><h4>激光</h4></th>
                                            <th style='text-align: center;;vertical-align: middle;'><i class="icon-remove-sign" style="color: red;font-size:25px"></i></th>
                                            <th colspan="2">
                                                <font color="red"><h4>【数据包输出异常】</h4></font><p >无数据包输出</p>
                                                <font color="red"><h4>【PPS异常】</h4></font><p >PPS信号异常PPS Error</p>
                                            </th>
                                        </tr>
                                        <tr>
                                            <th style='text-align: center;;vertical-align: middle;'><h4>相机</h4></th>
                                            <th style='text-align: center;;vertical-align: middle;'><i class="icon-remove-sign" style="color: red;font-size:25px"></i></th>
                                            <th colspan="2">
                                                <font color="red"><h4>【相机输出异常】</h4></font><p >左相机数据输出错误</p>
                                            </th>
                                        </tr>
                                        <tr>
                                            <th style='text-align: center;;vertical-align: middle;'><h4>存储设备</h4></th>
                                            <th style='text-align: center;;vertical-align: middle;'><i class="icon-remove-sign" style="color: red;font-size:25px"></i></th>
                                            <th colspan="2">
                                                <font color="red"><h4>【存储空间异常】</h4></font><p >存储空间不足</p>
                                            </th>
                                        </tr>
                                        <tr>
                                            <th style='text-align: center;;vertical-align: middle;'><h4>通讯检查</h4></th>
                                            <th style='text-align: center;;vertical-align: middle;'><i class="icon-remove-sign" style="color: red;font-size:25px"></i></th>
                                            <th colspan="2">
                                                <font color="red"><h4>【PAD与主板连接异常】</h4></font><p >SOCKET连接关闭</p>
                                            </th>
                                        </tr>


                                    </table>

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

<%@ include file="include/footer.jsp" %>
<script>
    $("[data-toggle ='tooltip']").tooltip({html : true });
</script>

</body>

</html>
