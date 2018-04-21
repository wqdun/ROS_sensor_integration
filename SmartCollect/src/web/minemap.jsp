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
    var soruceIndex = 0;
    var movepoint = 0.001;
    minemap.domainUrl = '//10.42.0.1:8888';
    minemap.spriteUrl = '//10.42.0.1:8888/minemapapi/v1.3/sprite/sprite';
    minemap.serviceUrl = '//10.42.0.1:8888/service';
    minemap.accessToken = '25cc55a69ea7422182d00d6b7c0ffa93';
    minemap.solution = 2365;

    var map = new minemap.Map({
        container: 'map',
        style: "//10.42.0.1:8888/service/solu/style/id/2365",
        center: [116.46, 39.92],
        zoom: 15,
        pitch: 0
    });



    console.log("Connecting to ROS...");
    var ros_ = new ROSLIB.Ros();
    // Create a connection to the rosbridge WebSocket server.
    ros_.connect('ws://<%=ip%>:9090');

    var serverdListener_ = new ROSLIB.Topic({
        ros: ros_,
        name: '/sc_monitor',
        messageType: 'sc_msgs/MonitorMsg'
    });
    serverdListener_.subscribe(
        function(monitorMsg) {
            soruceIndex++;
            movepoint = movepoint+0.001;
            console.log("monitorMsg.lat_lon_hei.y: " + monitorMsg.lat_lon_hei.y);
            if(monitorMsg.lat_lon_hei.y <= 0) {
                console.log("GPS lost.");
            }
            else {
                map.flyTo({
                    center: [monitorMsg.lat_lon_hei.y+movepoint, monitorMsg.lat_lon_hei.x],
                    // zoom: 15
                })
            }
            console.log("Only subscribe once.");
            // serverdListener_.unsubscribe();



             map.removeLayer("points");
             map.addSource("pointSource"+soruceIndex, {
                "type": "geojson",
                "data": {
                    "type": "FeatureCollection",
                    "features": [{
                        "type": "Feature",
                        "geometry": {
                            "type": "Point",
                            "coordinates": [monitorMsg.lat_lon_hei.y+movepoint, monitorMsg.lat_lon_hei.x],
                        },
                        "properties": {
                            // "title": "当前位置",
                            "icon": "marker-15-6",
                            "color": "#ff0000"
                        }
                    }]
                }
            });
            map.addLayer({
            "id": "points",
            "type": "symbol",
            "source": "pointSource"+soruceIndex,
            "layout": {
                "icon-image": "{icon}",
                "text-field": "{title}",
                "text-offset": [0, 0.6],
                "text-anchor": "top",
                "icon-allow-overlap": true,  //图标允许压盖
                "text-allow-overlap": true,   //图标覆盖文字允许压盖
            },
            "paint": {
                "text-color":{
                    'type': 'identity',
                    'property': 'color'
                },
                "icon-color":{
                    'type': 'identity',
                    'property': 'color'
                }
            }
        });

        }
    );

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

            var linefeatureArry = new Array();
            var linefeatureArry1 = new Array();
            for(var i = 0; i < lineLength; i++) {
                var points = baseMapMsg.lines2D[i];
                var posarry = new Array();
                for(var j = 0; j < points.line2D.length; j++) {
                    var onepoint = new Array();
                    onepoint[0] = points.line2D[j].x;
                    onepoint[1] = points.line2D[j].y;
                    posarry[j] = onepoint;
                }

                var lineFeature = {
                    "type": "Feature",
                    "properties": {},
                    "geometry": {
                        "type": "LineString",
                        "coordinates": posarry
                    }
                };
                if(i>=50){
                linefeatureArry.push(lineFeature);
            }

                if(i<50){
                    linefeatureArry1.push(lineFeature);
                }
            }




            // map.addSource("pointSource", {
            //     "type": "geojson",
            //     "data": {
            //         "type": "FeatureCollection",
            //         "features": [{
            //             "type": "Feature",
            //             "geometry": {
            //                 "type": "Point",
            //                 "coordinates": [116, 40],
            //             },
            //             "properties": {
            //                 "title": "大学",
            //                 "icon": "marker-15-6",
            //                 "color": "#ff0000"
            //             }
            //         }]
            //     }
            // });

            // map.addLayer({
            //     "id": "points",
            //     "type": "symbol",
            //     "source": "pointSource",
            //     "layout": {
            //         "icon-image": "{icon}",
            //         "text-field": "{title}",
            //         "text-offset": [0, 0.6],
            //         "text-anchor": "top",
            //         "icon-allow-overlap": true,
            //         "text-allow-overlap": true,
            //     },
            //     "paint": {
            //         "text-color": {
            //             'type': 'identity',
            //             'property': 'color'
            //         },
            //         "icon-color": {
            //             'type': 'identity',
            //             'property': 'color'
            //         }
            //     }
            // });


            // map.addSource("lineSource", {
            //     "type": "geojson",
            //     "data": {
            //         "type": "FeatureCollection",
            //         "features": linefeatureArry
            //     }
            // });
            // map.addSource("lineSource1", {
            //     "type": "geojson",
            //     "data": {
            //         "type": "FeatureCollection",
            //         "features": linefeatureArry1
            //     }
            // });
            // map.addLayer({
            //     "id": "lines",
            //     "type": "line",
            //     "source": "lineSource",
            //     "layout": {
            //         "line-join": "round",
            //         "line-cap": "round"
            //     },
            //     "paint": {
            //         "line-color": "#ff0000",
            //         "line-width": 4
            //     }
            // });
            // map.addLayer({
            //     "id": "lines2",
            //     "type": "line",
            //     "source": "lineSource1",
            //     "layout": {
            //         "line-join": "round",
            //         "line-cap": "round"
            //     },
            //     "paint": {
            //         "line-color": "#00ff00",
            //         "line-width": 1
            //     }
            // });

            console.log("Only subscribe once.");
            baseMapListener_.unsubscribe();
        }
    );



</script>
</body>