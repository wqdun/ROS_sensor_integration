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
    <script>
    function removeLayer(layerId) {
        /**
         * 删除图层的方法，目前的geojson数据通常是以图层的方式进行添加
         * 那么当删除刚刚添加的数据需要调用以下方法：
         * map.removerLayer(layerId)  layerId是上面定义的图层id 即 lines。
         */
        map.removeLayer(layerId);
    }

    function addLayer(layerId) {
        /**
         * 添加图层的方法，目前的geojson数据的添加方式采用以下方式添加
         * map.addLayer(layerObj)    layerObj是图层的描述。如下所示：
         */
        map.addLayer({
            "id": "lines",
            "type": "line",
            "source": "lineSource",
            "layout": {
                "line-join": "round",
                "line-cap": "round"
            },
            "paint": {
                "line-color": "#ff0000",
                "line-width": 6
            }
        });
    }
    </script>
</head>
<body>
<div id="map">
    <button onclick="removeLayer('lines')">删除线layer</button>
    <button onclick="addLayer('lines')">添加线图层</button>
</div>
<script>

    console.log("Connecting to ROS...");
    // Connecting to ROS
    var ros_ = new ROSLIB.Ros();
    // Create a connection to the rosbridge WebSocket server.
    ros_.connect('ws://<%=ip%>:9090');

    var centerListener = new ROSLIB.Topic({
        ros: ros_,
        name: '/sc_monitor',
        messageType: 'sc_server_daemon/monitorMsg'
    });
    centerListener.subscribe(function (message) {
        console.log(centerListener.name + '::heading: ' + message.pitch_roll_heading.z);
    });

    var nodeCtrlParamListener = new ROSLIB.Topic({
        ros: ros_,
        name: '/sc_node_params',
        messageType: 'sc_server_daemon/nodeParams'
    });
    var lon_ = 116.46;
    var lat_ = 39.92;
    nodeCtrlParamListener.subscribe(function(message) {
        console.log("is_record: " + message.is_record);
            // var int = self.setInterval("moveMap()", 500);
            // function moveMap() {
            lon_ += 0.00003;
            lat_ += 0.00003;
            // map.panBy([move_, move_]);

            map.flyTo({
                center: [lon_, lat_],
                zoom: 15
            });

            map.addSource("lineSource2", {
                "type": "geojson",
                "data": {
                "type": "Feature",
                "properties": {},
                "geometry": {
                "type": "LineString",
                "coordinates": [
                    [116.46+ 0.26087*0.01, 39.92+ 0.42890*0.02],
                    [116.46+ 0.03136*0.01, 39.92+ 0.48131*0.01],
                    [116.46+ 0.99157*0.01, 39.92+ 0.99328*0.01],
                    [116.46+ 0.94665*0.01, 39.92+ 0.19273*0.01],
                    [116.46+ 0.97511*0.01, 39.92+ 0.06375*0.01],
                    [116.46+ 0.70278*0.01, 39.92+ 0.03362*0.01]
                ]
                }
        }
    });

    });

    minemap.domainUrl = '//10.42.0.1:8888';
    minemap.spriteUrl = '//10.42.0.1:8888/minemapapi/v1.3/sprite/sprite';
    minemap.serviceUrl = '//10.42.0.1:8888/service';
    minemap.accessToken = '25cc55a69ea7422182d00d6b7c0ffa93';
    minemap.solution = 2365;

    var map = new minemap.Map({
        container: 'map',
        style: "//10.42.0.1:8888/service/solu/style/id/2365",
        center: [116.46,39.92],
        zoom: 15,
        pitch: 0
    });

    // map.addSource("lineSource2", {
    //     "type": "geojson",
    //     "data": {
    //         "type": "Feature",
    //         "properties": {},
    //         "geometry": {
    //             "type": "LineString",
    //             "coordinates": [
    //                 [116.46+ 0.26087*0.01, 39.92+ 0.42890*0.02],
    //                 [116.46+ 0.03136*0.01, 39.92+ 0.48131*0.01],
    //                 [116.46+ 0.99157*0.01, 39.92+ 0.99328*0.01],
    //                 [116.46+ 0.94665*0.01, 39.92+ 0.19273*0.01],
    //                 [116.46+ 0.97511*0.01, 39.92+ 0.06375*0.01],
    //                 [116.46+ 0.70278*0.01, 39.92+ 0.03362*0.01]
    //             ]
    //         }
    //     }
    // });
    // map.flyTo({
    //     center: [116.46,39.92],
    //     zoom: 15
    // });

    // var move_ = 0.1;
    // var lon_ = 116.46;
    // var lat_ = 39.92;

    // var int = self.setInterval("moveMap()", 500);
    // function moveMap() {
    //     lon_ += 0.00001;
    //     lat_ += 0.00001;
    //     // map.panBy([move_, move_]);

    //     map.flyTo({
    //         center: [lon_, lat_],
    //         zoom: 15
    //     });
    // }

    map.on("load", function () {
        map.addSource("lineSource", {
            "type": "geojson",
            "data": {
                "type": "Feature",
                "properties": {},
                "geometry": {
                    "type": "LineString",
                    "coordinates": [
                        [116.46+ 0.26087*0.01, 39.92+ 0.42890*0.02],
                        [116.46+ 0.03136*0.01, 39.92+ 0.48131*0.01],
                        [116.46+ 0.99157*0.01, 39.92+ 0.99328*0.01],
                        [116.46+ 0.94665*0.01, 39.92+ 0.19273*0.01],
                        [116.46+ 0.97511*0.01, 39.92+ 0.06375*0.01],
                        [116.46+ 0.70278*0.01, 39.92+ 0.03362*0.01]
                    ]
                }
            }
        });

        map.addLayer({
            "id": "lines",
            "type": "line",
            "source": "lineSource",
            "layout": {
                "line-join": "round",
                "line-cap": "round"
            },
            "paint": {
                "line-color": "#00ff00",
                "line-width": 6
            }
        });

    })


    //     for(var i = 0; i < 0; ++i) {
    //         if(i == 800) {
    //             alert("now 800");
    //         }
    //         map.addLayer({
    //             "id": "lines",
    //             "type": "line",
    //             "source": "lineSource",
    //             "layout": {
    //                 "line-join": "round",
    //                 "line-cap": "round"
    //             },
    //             "paint": {
    //                 "line-color": "#ff0000",
    //                 "line-width": i % 12
    //             }
    //         });
    //     }
    // })
</script>
</body>