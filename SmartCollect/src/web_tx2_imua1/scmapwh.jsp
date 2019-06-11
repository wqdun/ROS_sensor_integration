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
<div id="map"></div>

<script>
console.log("Connecting to ROS: rosbridge WebSocket server...");
var ros_ = new ROSLIB.Ros();
ros_.connect('ws://<%=ip%>:9090');

var currentLocation_ = [116.2394822, 40.0719897];
var map = new ol.Map({
    target: 'map',
    // layer图层
    // layers : [sdMapLayer()],
    view: new ol.View({
        center: transform(currentLocation_),
        zoom: 15,
        minZoom: 0,
        maxZoom: 21
    })
});

var feature = new ol.Feature({
    geometry: new ol.geom.Point(transform(currentLocation_))
});
// 我们需要一个vector的layer来放置图标
var layer = new ol.layer.Vector({
    source: new ol.source.Vector({
    features : [feature]
  }),
  style : new ol.style.Style({        //设置点样式
    image: new ol.style.Icon({
      src: '/img/loc.png',
      scale: 0.04
    })
  })
});


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



var recordedTrackMsg = {lines2D:[{line2D:[{x:116.247660429,y:40.068108911},{x:116.247660429,y:40.069398911}]},{line2D:[{x:116.246630429,y:40.070098911},{x:116.246430429,y:40.070498911}]}]};

var lineFeatures = [];
function test() {
        var lineLength = recordedTrackMsg.lines2D.length;
    var point = new ol.geom.Point(transform([116.247660429,40.068108911]));
    feature.setGeometry(point);
    map.getView().setCenter(transform([116.247660429,40.068108911]));
        for (var i = 0; i < lineLength; ++i) {
            var points = recordedTrackMsg.lines2D[i];

            for (var j = 0; j < points.line2D.length; ++j) {
                addLineWkt = points.line2D[j].x + " " + points.line2D[j].y;
        lineFeatures.push(addLineWkt);
            }
      var lineWkt = 'LINESTRING(' +lineFeatures.join(',')+ ')';
            console.log("addLineWkt: " + lineWkt);
            recordedTrackLayer_.getSource().addFeature(new ol.Feature(getGeometry(lineWkt)));
        }
    }

var unrecordedTrackLayer_ = new ol.layer.Vector({
    source: new ol.source.Vector(),
    style: new ol.style.Style({
        stroke: new ol.style.Stroke({
            color: '#FF0000',
            width: 5,
        })
    })
});

</script>
</body>
</html>
