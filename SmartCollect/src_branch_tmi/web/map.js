console.log("********************************");

// 坐标转换，将WGS84坐标系转换成Web Mercator坐标
var transform = function(coordinate) {
    var coor = ol.proj.transform(coordinate,'EPSG:4326', 'EPSG:900913');
    return coor ;
};

// 将WGS84坐标系的WKT字符串转换成Openlayer的Geometry对象
var getGeometry = function(wkt) {
    if (wkt == null) {
        return null;
    }
    var wktFormat = new ol.format.WKT()
    return wktFormat.readGeometry(wkt, {
        // 输入
        "dataProjection" : 'EPSG:4326',
        // 输出
        "featureProjection" : 'EPSG:900913'
    });
};

console.log("Connecting to ROS: rosbridge WebSocket server...");
var ros_ = new ROSLIB.Ros();
ros_.connect('ws://<%=ip%>:9090');

var map = new ol.Map({
    // 对应html里的id
    target : 'map',
    // layer图层
    // layers : [],
    // 视图
    view : new ol.View({
        center : transform([ 116.247660429, 40.068108911 ]),
        zoom : 15,
        minZoom : 0,
        maxZoom : 21
    })
});

// 将线对象放入数组
var lineFeatures = [];
// 线图层
var lineVector = new ol.layer.Vector({
    source : new ol.source.Vector({
        features : lineFeatures
    }),
    // 设置线样式
    style : new ol.style.Style({
        stroke : new ol.style.Stroke({
            color : '#000000',
            width : 1,
        })
    })
});
map.addLayer(lineVector);

var recordedTrackListener_ = new ROSLIB.Topic({
    ros: ros_,
    name: '/sc_recorded_track',
    messageType: 'sc_msgs/Lines2D'
});

var recordedPointNumOfLastLineLast_ = -1;
var currentLocation_ = [116.2394822, 40.0719897];
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

        var recordedTrackFeatureArry = new Array();

        for (var i = 0; i < lineLength; ++i) {
            var points = recordedTrackMsg.lines2D[i];
            var addLineWkt = "LINESTRING(";
            for (var j = 0; j < points.line2D.length; ++j) {
                addLineWkt += points.line2D[j].x + " " + points.line2D[j].y;
            }
            addLineWkt += ")";
            console.log("addLineWkt: " + addLineWkt);
            lineVector.getSource().addFeature(new ol.Feature(getGeometry(addLineWkt)));
        }
    }
)


