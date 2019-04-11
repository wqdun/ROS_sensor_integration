getGeometry = function(wkt) {// 将WGS84坐标系的WKT字符串转换成Openlayer的Geometry对象
    if (wkt == null) {
        return null;
    }
    var wktFormat = new ol.format.WKT()
    return wktFormat.readGeometry(wkt, {
        "dataProjection" : 'EPSG:4326',// 输入
        "featureProjection" : 'EPSG:900913'// 输出
    });
};
sdMapLayer = function() {// 获取标准地图图层，用于地图底图显示
    var sdMapLayer = new ol.layer.Tile({
        source : new ol.source.XYZ({
            wrapX : true,
            projection : 'EPSG:900913',
            tileUrlFunction : MapUtils.tileUrlFunction
        })
    });
    return sdMapLayer;
}
var map = new ol.Map({          //ol地图
    target : 'mapViewer',   //对应html里id
    layers : [ sdMapLayer ],    //layer图层
    view : new ol.View({       //视图
        center : [ 108.39745, 35.90877 ],
        zoom : 5,
        minZoom : 0,
        maxZoom : 21
    })
});
var lineFeatures = [];               //将线对象放入数组
for(int i=0;i<a.length;i++){
    var geom =getGeometry(a[i]);
    lineFeatures.push(new ol.Feature(geom));
}
lineVector = new ol.layer.Vector({        //线图层
    source : new ol.source.Vector({
        features : lineFeatures
    }),
    style : new ol.style.Style({           //设置线样式
        stroke : new ol.style.Stroke({
            width : 5,
            color : '#FF0000'
        })
    })
});
var pointFeatures = [];
for(int i=0;i<b.length;i++){              //将点对象放入数组
    var geom =getGeometry(b[i]);
    pointFeatures.push(new ol.Feature(geom));
}
pointVector = new ol.layer.Vector({           //点图层
    source : new ol.source.Vector({
        features : pointFeatures
    }),
    style : new ol.style.Style({        //设置点样式
        image : new ol.style.Circle({
            radius : 1,
            fill : new ol.style.Fill({
                color : 'rgba(255, 255, 255, 0.8)'
            }),
            stroke : new ol.style.Stroke({
                color : 'red',
                width : 2
            })
        })
    })
});

map.addLayer(lineVector);
map.addLayer(pointVector);
map.render();

//map.removeLayer(lineVector); 移除图层