<%@ page contentType="text/html;charset=utf-8" %>
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>minemap</title>
  <link rel="stylesheet" href="//10.42.0.1:8888/minemapapi/v1.3/minemap.css">
  <%@ include file="include/header.jsp" %>
  <script src="//10.42.0.1:8888/minemapapi/v1.3/minemap.js"></script>
  <script src="build/eventemitter2.min.js"></script>
  <script src="build/roslib.js"></script>
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
</div>
<script>
  minemap.domainUrl = '//10.42.0.1:8888';
  minemap.spriteUrl = '//10.42.0.1:8888/minemapapi/v1.3/sprite/sprite';
  minemap.serviceUrl = '//10.42.0.1:8888/service';
  minemap.accessToken = '25cc55a69ea7422182d00d6b7c0ffa93';
  minemap.solution = 2365;

  var map = new minemap.Map({
    container: 'map',
    style: "//10.42.0.1:8888/service/solu/style/id/2365",
    center: [116.38, 39.90],
    zoom: 15,
    pitch: 0,
  });
  map.addControl(new minemap.Navigation(),"bottom-right");
  map.addControl(new minemap.Scale(),"top-right");

  map.on("load", function() {
    map.flyTo({
      center: [116.2394822, 40.0719897],
      pitch: 0,
    });

    map.addSource("unrecordedTrackSource", {
      "type": "geojson",
      "data": {
        "type": "FeatureCollection",
        "features": [],
      },
    });
    map.addLayer({
      "id": "unrecordedTrackLines",
      "type": "line",
      "source": "unrecordedTrackSource",
      "layout": {
        "line-join": "round",
        "line-cap": "round"
      },
      "paint": {
        "line-color": "#FF9912",
        "line-width": 1
      }
    });

    map.addSource("recordedTrackSource", {
      "type": "geojson",
      "data": {
        "type": "FeatureCollection",
        "features": [],
      },
    });
    map.addLayer({
      "id": "recordedTrackLines",
      "type": "line",
      "source": "recordedTrackSource",
      "layout": {
        "line-join": "round",
        "line-cap": "round"
      },
      "paint": {
        "line-color": "#ff0000",
        "line-width": 2
      }
    });

    map.addSource("arrowSource", {
      "type": "geojson",
      "data": {
        "type": "Feature",
        "properties": {},
        "geometry": {
          "type": "LineString",
          "coordinates": [
            [116.284939+ 0.42761*0.01, 39.920658+ 0.72274*0.01],
            [116.284939+ 0.45097*0.01, 39.920658+ 0.16086*0.01],
          ],
        }
      }
    });
    map.addLayer({
      "id": "arrow",
      "type": "symbol",
      "source": "arrowSource",
      "layout": {
        "symbol-placement": "line",
        "icon-allow-overlap": true,
        "icon-image": "direction-1-18",       //指定箭头图标
        "text-field": "{title}",
        "text-offset": [0, 0.6],
        "text-anchor": "top"
      },
      "paint": {
        "icon-color": "#ff0000"
      }
    });

    if(typeof map.getSource("arrowSource") == "undefined") {
        console.log("recordedTrackSource undefined.");
      }
      else {
        map.getSource("arrowSource").setData({
          "type": "Feature",
          "properties": {},
          "geometry": {
            "type": "LineString",
            "coordinates": [
              [116.284939+ 0.26367*0.01, 39.920658+ 0.60582*0.01],
                        [116.284939+ 0.79526*0.01, 39.920658+ 0.93414*0.01],
                        // [116.284939+ 0.88778*0.01, 39.920658+ 0.98129*0.01],
                        // [116.284939+ 0.01739*0.01, 39.920658+ 0.33856*0.01]
            ],
          }
        });
      }

      map.flyTo({
          // center: [unrecordedTrackMsg.lines2D[linesNum - 1].line2D[pointNumOfLastLine - 1].x, unrecordedTrackMsg.lines2D[linesNum - 1].line2D[pointNumOfLastLine - 1].y],
          center: [116.284939+ 0.42761*0.01, 39.920658+ 0.72274*0.01],
        });



  });

  console.log("Connecting to ROS: rosbridge WebSocket server...");
  var ros_ = new ROSLIB.Ros();
  ros_.connect('ws://<%=ip%>:9090');

  var baseMapListener_ = new ROSLIB.Topic({
    ros: ros_,
    name: '/sc_base_map',
    messageType: 'sc_msgs/Lines2D'
  });
  baseMapListener_.subscribe(
    function(baseMapMsg) {
      var lineLength = baseMapMsg.lines2D.length;
      console.log("Found " + lineLength + " lines in " + baseMapListener_.name);
      if(lineLength <= 0) {
        console.log("Found 0 line in " + baseMapListener_.name);
        baseMapListener_.unsubscribe();
        return;
      }

      console.log("1st point: " + baseMapMsg.lines2D[0].line2D[0].x + ", " + baseMapMsg.lines2D[0].line2D[0].y);
      var baseMaplinesFeatureArry = new Array();
      for(var i = 0; i < lineLength; ++i) {
        var points = baseMapMsg.lines2D[i];
        var pointsArry = new Array();
        for(var j = 0; j < points.line2D.length; ++j) {
          var onepoint = new Array();
          onepoint[0] = points.line2D[j].x;
          onepoint[1] = points.line2D[j].y;
          pointsArry[j] = onepoint;
        }

        var lineFeature = {
          "type": "Feature",
          "properties": {},
          "geometry": {
            "type": "LineString",
            "coordinates": pointsArry
          },
        };
        baseMaplinesFeatureArry.push(lineFeature);
      }

      map.addSource("baseLineSource", {
        "type": "geojson",
        "data": {
          "type": "FeatureCollection",
          "features": baseMaplinesFeatureArry
        }
      });
      map.addLayer({
        "id": "baseMapLines",
        "type": "line",
        "source": "baseLineSource",
        "layout": {
          "line-join": "round",
          "line-cap": "round"
        },
        "paint": {
          "line-color": "#000000",
          "line-width": 1
        }
      });

      console.log("Only subscribe base map once.");
      baseMapListener_.unsubscribe();
    }
  );

  var planMapListener_ = new ROSLIB.Topic({
    ros: ros_,
    name: '/sc_plan_map',
    messageType: 'sc_msgs/Lines2D'
  });
  planMapListener_.subscribe(
    function(planMapMsg) {
      var lineLength = planMapMsg.lines2D.length;
      console.log("Found " + lineLength + " lines in " + planMapListener_.name);
      if(lineLength <= 0) {
        console.log("Found 0 line in " + planMapListener_.name);
        planMapListener_.unsubscribe();
        return;
      }

      console.log("1st point: " + planMapMsg.lines2D[0].line2D[0].x + ", " + planMapMsg.lines2D[0].line2D[0].y);
      var planMaplinesFeatureArry = new Array();
      for(var i = 0; i < lineLength; ++i) {
        var points = planMapMsg.lines2D[i];
        var pointsArry = new Array();
        for(var j = 0; j < points.line2D.length; ++j) {
          var onepoint = new Array();
          onepoint[0] = points.line2D[j].x;
          onepoint[1] = points.line2D[j].y;
          pointsArry[j] = onepoint;
        }

        var lineFeature = {
          "type": "Feature",
          "properties": {},
          "geometry": {
            "type": "LineString",
            "coordinates": pointsArry
          },
        };
        planMaplinesFeatureArry.push(lineFeature);
      }

      map.addSource("planLineSource", {
        "type": "geojson",
        "data": {
          "type": "FeatureCollection",
          "features": planMaplinesFeatureArry
        }
      });
      map.addLayer({
        "id": "planMapLines",
        "type": "line",
        "source": "planLineSource",
        "layout": {
          "line-join": "round",
          "line-cap": "round"
        },
        "paint": {
          "line-color": "#000000",
          "line-width": 1
        }
      });

      console.log("Only subscribe plan map once.");
      planMapListener_.unsubscribe();
    }
  );

  var unrecordedPointNumOfLastLineLast_ = -1;
  var unrecordedTrackListener_ = new ROSLIB.Topic({
    ros: ros_,
    name: '/sc_unrecorded_track',
    messageType: 'sc_msgs/Lines2D'
  });
  unrecordedTrackListener_.subscribe(
    function(unrecordedTrackMsg) {

      if(typeof map.getSource("arrowSource") == "undefined") {
        console.log("recordedTrackSource undefined.");
      }
      else {
        map.getSource("arrowSource").setData({
          "type": "Feature",
          "properties": {},
          "geometry": {
            "type": "LineString",
            "coordinates": [
              [116.284939+ 0.42761*0.01, 39.920658+ 0.72274*0.01],
              [116.284939+ 0.45097*0.01, 39.920658+ 0.16086*0.01],
            ],
          }
        });
      }

      var lineLength = unrecordedTrackMsg.lines2D.length;
      console.log("Found " + lineLength + " lines in " + unrecordedTrackListener_.name);
      if(lineLength <= 0) {
        console.log("Found 0 line in " + unrecordedTrackListener_.name);
        return;
      }

      console.log("1st point: " + unrecordedTrackMsg.lines2D[0].line2D[0].x + ", " + unrecordedTrackMsg.lines2D[0].line2D[0].y);
      var linesNum = unrecordedTrackMsg.lines2D.length;
      var pointNumOfLastLine = unrecordedTrackMsg.lines2D[linesNum - 1].line2D.length;
      if(unrecordedPointNumOfLastLineLast_ != pointNumOfLastLine) {
        console.log("I am not recording.");
        map.flyTo({
          // center: [unrecordedTrackMsg.lines2D[linesNum - 1].line2D[pointNumOfLastLine - 1].x, unrecordedTrackMsg.lines2D[linesNum - 1].line2D[pointNumOfLastLine - 1].y],
          center: [116.284939+ 0.42761*0.01, 39.920658+ 0.72274*0.01],
        });
      }
      // else {nothing}
      unrecordedPointNumOfLastLineLast_ = pointNumOfLastLine;

      var unrecordedTrackFeatureArry = new Array();
      for(var i = 0; i < lineLength; ++i) {
        var points = unrecordedTrackMsg.lines2D[i];
        var pointsArry = new Array();
        for(var j = 0; j < points.line2D.length; ++j) {
          var onepoint = new Array();
          onepoint[0] = points.line2D[j].x;
          onepoint[1] = points.line2D[j].y;
          pointsArry[j] = onepoint;
        }

        var lineFeature = {
          "type": "Feature",
          "properties": {},
          "geometry": {
            "type": "LineString",
            "coordinates": pointsArry
          },
        };
        unrecordedTrackFeatureArry.push(lineFeature);
      }

      if(typeof map.getSource("unrecordedTrackSource") == "undefined") {
        console.log("unrecordedTrackSource undefined.");
        return;
      }
      console.log("unrecordedTrackSource defined.");
      map.getSource("unrecordedTrackSource").setData({
        "type": "FeatureCollection",
        "features": unrecordedTrackFeatureArry
      });
    }
  );

  var recordedPointNumOfLastLineLast_ = -1;
  var recordedTrackListener_ = new ROSLIB.Topic({
    ros: ros_,
    name: '/sc_recorded_track',
    messageType: 'sc_msgs/Lines2D'
  });
  recordedTrackListener_.subscribe(
    function(recordedTrackMsg) {
      var lineLength = recordedTrackMsg.lines2D.length;
      console.log("Found " + lineLength + " lines in " + recordedTrackListener_.name);
      if(lineLength <= 0) {
        console.log("Found 0 line in " + recordedTrackListener_.name);
        return;
      }

      console.log("1st point: " + recordedTrackMsg.lines2D[0].line2D[0].x + ", " + recordedTrackMsg.lines2D[0].line2D[0].y);
      var linesNum = recordedTrackMsg.lines2D.length;
      var pointNumOfLastLine = recordedTrackMsg.lines2D[linesNum - 1].line2D.length;
      if(recordedPointNumOfLastLineLast_ != pointNumOfLastLine) {
        console.log("I am recording.");
        map.flyTo({
          center: [recordedTrackMsg.lines2D[linesNum - 1].line2D[pointNumOfLastLine - 1].x, recordedTrackMsg.lines2D[linesNum - 1].line2D[pointNumOfLastLine - 1].y],
        });
      }
      // else {nothing}
      recordedPointNumOfLastLineLast_ = pointNumOfLastLine;

      var recordedTrackFeatureArry = new Array();
      for(var i = 0; i < lineLength; ++i) {
        var points = recordedTrackMsg.lines2D[i];
        var pointsArry = new Array();
        for(var j = 0; j < points.line2D.length; ++j) {
          var onepoint = new Array();
          onepoint[0] = points.line2D[j].x;
          onepoint[1] = points.line2D[j].y;
          pointsArry[j] = onepoint;
        }

        var lineFeature = {
          "type": "Feature",
          "properties": {},
          "geometry": {
            "type": "LineString",
            "coordinates": pointsArry
          },
        };
        recordedTrackFeatureArry.push(lineFeature);
      }

      if(typeof map.getSource("recordedTrackSource") == "undefined") {
        console.log("recordedTrackSource undefined.");
        return;
      }
      console.log("recordedTrackSource defined.");
      map.getSource("recordedTrackSource").setData({
        "type": "FeatureCollection",
        "features": recordedTrackFeatureArry
      });



    }
  );

</script>
</body>