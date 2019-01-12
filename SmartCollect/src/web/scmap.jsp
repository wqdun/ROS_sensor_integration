<!DOCTYPE html>
<html lang="en">
<head>
    <meta http-equiv="content-type" content="text/html; charset=UTF-8">
    <title>scmap</title>
    <%@ include file="include/header.jsp" %>

    <script type="text/javascript" src="js/ol.js"></script>
    <script type="text/javascript" src="build/eventemitter2.min.js"></script>
    <script type="text/javascript" src="build/roslib.js"></script>
    <script type="text/javascript">
    // 坐标转换，将WGS84坐标系转换成Web Mercator坐标
    var transform = function (coordinate) {
        var coor = ol.proj.transform(coordinate, 'EPSG:4326', 'EPSG:900913');
        return coor;
    };

    // 将WGS84坐标系的WKT字符串转换成Openlayer的Geometry对象
    var getGeometry = function (wkt) {
        if (wkt == null) {
            return null;
        }
        var wktFormat = new ol.format.WKT()
        return wktFormat.readGeometry(wkt, {
            // 输入
            "dataProjection": 'EPSG:4326',
            // 输出
            "featureProjection": 'EPSG:900913'
        });
    };
    </script>

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
console.log("Connecting to ROS: rosbridge WebSocket server...");
var ros_ = new ROSLIB.Ros();
ros_.connect('ws://<%=ip%>:9090');

var currentLocation_ = [116.2394822, 40.0719897];
var map = new ol.Map({
    target: 'map',
    // layer图层
    layers : [],
    view: new ol.View({
        center: transform(currentLocation_),
        zoom: 15,
        minZoom: 0,
        maxZoom: 21
    })
});

// 我们需要一个vector的layer来放置图标
var layer = new ol.layer.Vector({
    source: new ol.source.Vector()
});

var _anchor = new ol.Feature({
    geometry: new ol.geom.Point(transform(currentLocation_))
});
// 设置样式，在样式中就可以设置图标
_anchor.setStyle(new ol.style.Style({
    image: new ol.style.Icon({
        src: '../img/loc.png',
        scale:10
    })
}));
// 添加到之前的创建的layer中去
layer.getSource().addFeature(_anchor);

// 监听地图层级变化
map.getView().on('change:resolution', function() {
    var style = _anchor.getStyle();
    // 重新设置图标的缩放率，基于层级10来做缩放
    _anchor.setStyle(style);
})
map.addLayer(layer);

var recordedTrackLayer_ = new ol.layer.Vector({
    source: new ol.source.Vector(),
    style: new ol.style.Style({
        stroke: new ol.style.Stroke({
            color: '#000000',
            width: 2,
        })
    })
});
map.addLayer(recordedTrackLayer_);

var recordedTrackListener_ = new ROSLIB.Topic({
    ros: ros_,
    name: '/sc_recorded_track',
    messageType: 'sc_msgs/Lines2D'
});

var recordedPointNumOfLastLineLast_ = -1;
recordedTrackListener_.subscribe(
    function (recordedTrackMsg) {
        var lineLength = recordedTrackMsg.lines2D.length;
        console.log("Found " + lineLength + " lines in " + recordedTrackListener_.name);
        if (lineLength <= 0) {
            return;
        }

        var linesNum = recordedTrackMsg.lines2D.length;
        var pointNumOfLastLine = recordedTrackMsg.lines2D[linesNum - 1].line2D.length;
        if (recordedPointNumOfLastLineLast_ != pointNumOfLastLine) {
            console.log("I am recording.");
            currentLocation_ = [recordedTrackMsg.lines2D[linesNum - 1].line2D[pointNumOfLastLine - 1].x, recordedTrackMsg.lines2D[linesNum - 1].line2D[pointNumOfLastLine - 1].y];
        }
        // else {nothing}
        recordedPointNumOfLastLineLast_ = pointNumOfLastLine;

        for (var i = 0; i < lineLength; ++i) {
            var points = recordedTrackMsg.lines2D[i];
            var addLineWkt = "LINESTRING(";
            for (var j = 0; j < points.line2D.length; ++j) {
                addLineWkt += points.line2D[j].x + " " + points.line2D[j].y;
            }
            addLineWkt += ")";
            console.log("addLineWkt: " + addLineWkt);
            recordedTrackLayer_.getSource().addFeature(new ol.Feature(getGeometry(addLineWkt)));
        }
    }
);

var unrecordedTrackLayer_ = new ol.layer.Vector({
    source: new ol.source.Vector(),
    style: new ol.style.Style({
        stroke: new ol.style.Stroke({
            color: '#FF0000',
            width: 2,
        })
    })
});
map.addLayer(unrecordedTrackLayer_);
var unrecordedPointNumOfLastLineLast_ = -1;
var unrecordedTrackListener_ = new ROSLIB.Topic({
    ros: ros_,
    name: '/sc_unrecorded_track',
    messageType: 'sc_msgs/Lines2D'
});
unrecordedTrackListener_.subscribe(
    function (unrecordedTrackMsg) {
        var lineLength = unrecordedTrackMsg.lines2D.length;
        console.log("Found " + lineLength + " lines in " + unrecordedTrackListener_.name);
        if (lineLength <= 0) {
            return;
        }

        var linesNum = unrecordedTrackMsg.lines2D.length;
        var pointNumOfLastLine = unrecordedTrackMsg.lines2D[linesNum - 1].line2D.length;
        if (unrecordedPointNumOfLastLineLast_ != pointNumOfLastLine) {
            console.log("I am not recording.");
            currentLocation_ = [unrecordedTrackMsg.lines2D[linesNum - 1].line2D[pointNumOfLastLine - 1].x, unrecordedTrackMsg.lines2D[linesNum - 1].line2D[pointNumOfLastLine - 1].y];
        }
        // else {nothing}
        unrecordedPointNumOfLastLineLast_ = pointNumOfLastLine;

        for (var i = 0; i < lineLength; ++i) {
            var points = unrecordedTrackMsg.lines2D[i];
            var addLineWkt = "LINESTRING(";
            for (var j = 0; j < points.line2D.length; ++j) {
                addLineWkt += points.line2D[j].x + " " + points.line2D[j].y;
            }
            addLineWkt += ")";
            console.log("addLineWkt: " + addLineWkt);
            unrecordedTrackLayer_.getSource().addFeature(new ol.Feature(getGeometry(addLineWkt)));
        }
    }
);

</script>
</body>
</html>
