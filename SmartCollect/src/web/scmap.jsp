<!DOCTYPE html>
<html lang="en">
<head>
    <meta http-equiv="content-type" content="text/html; charset=UTF-8">
    <title>scmap</title>
    <%@ include file="include/header.jsp" %>

    <link href="css/ol.css" rel="stylesheet" type="text/css" />
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


var sdMapLayer = function() {
    // 获取标准地图图层，用于地图底图显示
    var sdMapLayer = new ol.layer.Tile({
        source : new ol.source.XYZ({
            wrapX : true,
            projection : 'EPSG:900913',
            url:'http://<%=ip%>/roadmap/{z}/{x}/{y}.png'
        })
    });
    return sdMapLayer;
}

var map = new ol.Map({
    target: 'map',
    // layer图层
    layers : [ sdMapLayer() ],
    view: new ol.View({
        center: transform(currentLocation_),
        zoom: 10,
        minZoom: 6,
        maxZoom: 17,
        rotation: 0 * Math.PI / 180
    })
});

var scaleLineControl = new ol.control.ScaleLine({
    units : 'metric',
    target : 'scalebar',
    className : 'ol-scale-line'
});
map.addControl(scaleLineControl);

var arrowFeature = new ol.Feature({
    geometry: new ol.geom.Point(transform(currentLocation_))
});
// 我们需要一个vector的layer来放置图标
arrowFeature.set("rotation", 0);


var arrowLayer = new ol.layer.Vector({
    source : new ol.source.Vector({
        features: [arrowFeature]
    }),
    style :function(feature) {
        var r = feature.get("rotation");
        console.log("rotation: " + r);
        // 设置点样式
        var style = new ol.style.Style({
            image: new ol.style.Icon({
                src: '/img/loc.png',
                rotation : r,
                scale: 0.04
            })
        })
        return [ style ];
    }
});

map.addLayer(arrowLayer);

var centerListener = new ROSLIB.Topic({
    ros: ros_,
    name: '/sc_monitor',
    messageType: 'sc_msgs/MonitorMsg'
});
var currentHeading_ = 0;
centerListener.subscribe(function (message) {
    currentHeading_ = message.pitch_roll_heading.z;
    currentHeading_ += map.getView().getRotation() * 180 / Math.PI;
    console.log("map.getView().getRotation(): " + map.getView().getRotation());
    console.log('currentHeading_: ' + currentHeading_);
});

var recordedTrackLayer_ = new ol.layer.Vector({
    source: new ol.source.Vector(),
    style: new ol.style.Style({
        stroke: new ol.style.Stroke({
            color: '#FF0000',
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
        var startTime = new Date().getTime() / 1000;

        var linesNum = recordedTrackMsg.lines2D.length;
        console.log("Found " + linesNum + " lines in " + recordedTrackListener_.name);
        if (linesNum <= 0) {
            return;
        }

        var pointNumOfLastLine = recordedTrackMsg.lines2D[linesNum - 1].line2D.length;
        if (recordedPointNumOfLastLineLast_ != pointNumOfLastLine) {
            console.log("I am recording.");
            currentLocation_ = [recordedTrackMsg.lines2D[linesNum - 1].line2D[pointNumOfLastLine - 1].x, recordedTrackMsg.lines2D[linesNum - 1].line2D[pointNumOfLastLine - 1].y];

            var currentPoint = new ol.geom.Point(transform(currentLocation_));
            arrowFeature.setGeometry(currentPoint);
            var lastHeading = arrowFeature.get("rotation");

            arrowFeature.set("rotation", currentHeading_ * Math.PI / 180);


            map.getView().setCenter(transform(currentLocation_));
        }
        // else {nothing}
        recordedPointNumOfLastLineLast_ = pointNumOfLastLine;

        var recordedLinesFeatures = [];
        var allPointsNum = 0;
        for (var i = 0; i < linesNum; ++i) {
            var points = recordedTrackMsg.lines2D[i];
            var lineFeatures = [];
            allPointsNum += points.line2D.length;
            for (var j = 0; j < points.line2D.length; ++j) {
                lineFeatures.push(points.line2D[j].x + " " + points.line2D[j].y);
            }
            var lineWkt = 'LINESTRING(' +lineFeatures.join(',')+ ')';
            recordedLinesFeatures.push(new ol.Feature(getGeometry(lineWkt)));
        }
        recordedTrackLayer_.getSource().clear();
        recordedTrackLayer_.getSource().addFeatures(recordedLinesFeatures);
        console.log("recordedTrackLayer_.getSource().getFeatures().length: " + recordedTrackLayer_.getSource().getFeatures().length);

        var endTime = new Date().getTime() / 1000;
        console.log(startTime + " --> " + endTime + "; recordedTrack cost " + (endTime - startTime) + "; allPointsNum: " + allPointsNum);
    }
);

var unrecordedTrackLayer_ = new ol.layer.Vector({
    source: new ol.source.Vector(),
    style: new ol.style.Style({
        stroke: new ol.style.Stroke({
            color: '#0000FF',
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
        var startTime = new Date().getTime() / 1000;

        var linesNum = unrecordedTrackMsg.lines2D.length;
        console.log("Found " + linesNum + " lines in " + unrecordedTrackListener_.name);
        if (linesNum <= 0) {
            return;
        }

        var pointNumOfLastLine = unrecordedTrackMsg.lines2D[linesNum - 1].line2D.length;
        if (unrecordedPointNumOfLastLineLast_ != pointNumOfLastLine) {
            console.log("I am not recording.");
            currentLocation_ = [unrecordedTrackMsg.lines2D[linesNum - 1].line2D[pointNumOfLastLine - 1].x, unrecordedTrackMsg.lines2D[linesNum - 1].line2D[pointNumOfLastLine - 1].y];
            var currentPoint = new ol.geom.Point(transform(currentLocation_));
            arrowFeature.setGeometry(currentPoint);
            arrowFeature.set("rotation", currentHeading_ * Math.PI / 180);

            map.getView().setCenter(transform(currentLocation_));
        }
        // else {nothing}
        unrecordedPointNumOfLastLineLast_ = pointNumOfLastLine;

        var unrecordedLinesFeatures = [];
        var allPointsNum = 0;
        for (var i = 0; i < linesNum; ++i) {
            var points = unrecordedTrackMsg.lines2D[i];
            var lineFeatures = [];
            allPointsNum += points.line2D.length;
            for (var j = 0; j < points.line2D.length; ++j) {
                lineFeatures.push(points.line2D[j].x + " " + points.line2D[j].y);
            }
            var lineWkt = 'LINESTRING(' +lineFeatures.join(',')+ ')';

            unrecordedLinesFeatures.push(new ol.Feature(getGeometry(lineWkt)));
        }
        unrecordedTrackLayer_.getSource().clear();
        unrecordedTrackLayer_.getSource().addFeatures(unrecordedLinesFeatures);
        console.log("unrecordedTrackLayer_.getSource().getFeatures().length: " + unrecordedTrackLayer_.getSource().getFeatures().length);

        var endTime = new Date().getTime() / 1000;
        console.log(startTime + " --> " + endTime + "; unrecordedTrack cost " + (endTime - startTime) + "; allPointsNum: " + allPointsNum);
    }
);

</script>
</body>
</html>
