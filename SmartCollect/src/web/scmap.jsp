<!DOCTYPE html>
<html lang="en">
<head>
 <meta http-equiv="content-type" content="text/html; charset=UTF-8">
 <title>scmap</title>
 <%@ include file="include/header.jsp" %>
 <script type="text/javascript" src="js/jquery-3.1.1.js"></script>
 <script type="text/javascript" src="js/ol.js"></script>
 <script type="text/javascript" src="build/eventemitter2.min.js"></script>
 <script type="text/javascript" src="build/roslib.js"></script>
 <script type="text/javascript" src="map.js"></script>
 <style>
  html, body, #map {
   width: 100%;
   height: 100%;
  }

  html, body {
   margin: 0;
   padding: 0;
  }
 </style>
</head>
<body>
<div id="map">
 <table class="table table-bordered table-striped">
  <thead>
  <tr>
   <!--<th style="text-align:center;"><label class="checkbox"><input id="isDisplayLayer1" type="checkbox" checked="true" onclick="layer1Display();">显示采集图层-->
   <!--</label></th>-->
   <!--<th style="text-align:center;"><label class="checkbox"><input id="isDisplayLayer2" type="checkbox" checked="true" onclick="layer2Display();">显示计划图层-->
   <!--</label></th>-->
  </tr>
  </thead>
 </table>
</div>
</body>
</html>
