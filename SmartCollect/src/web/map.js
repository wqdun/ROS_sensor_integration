$(function() {
	transform = function(coordinate) {// 坐标转换，将WGS84坐标系转换成Web Mercator坐标
		var coor = ol.proj.transform(coordinate,'EPSG:4326', 'EPSG:900913');		
		return coor ;
	};
	tileUrlFunction = function(tileCoord) {// 获取趣驾地图的瓦片
		/**/
		var funDriverUrl = 'http://image.fundrive.com.cn/en/{z}/{p}{t}.png';
		var x = tileCoord[1];
		var y = -tileCoord[2] - 1;
		var z = tileCoord[0];
		var nGrade = Math.ceil((z - 3) / 4);
		var nPreRow = 0, nPreCol = 0, nPreSize = 0;
		var path = "";
		for (var i = 0; i < nGrade; i++) {
			// 每级最大网格数为 16* 16 = 256(相关于子目录数和文件数)
			var nSize = 1 << (4 * (nGrade - i));// 计算当前目录包括的单元数(网格距离）
			var nRow = parseInt((x - nPreRow * nPreSize) / nSize); // 得到行，列值
			var nCol = parseInt((y - nPreCol * nPreSize) / nSize);
			path += ((nRow > 9) ? nRow : "0" + nRow) + "" + ((nCol > 9) ? nCol : "0" + nCol) + "/";
			nPreRow = nRow;
			nPreCol = nCol;
			nPreSize = nSize;
		}
		var id = (((x) & ((1 << 20) - 1)) + (((y) & ((1 << 20) - 1)) * Math.pow(2, 20)) + (((z) & ((1 << 8) - 1)) * Math.pow(2, 40)));
		return funDriverUrl.replace('{z}', tileCoord[0].toString()).replace('{p}', path.toString()).replace('{t}', id.toString());
	}
	sdMapLayer = function() {// 获取标准地图图层，用于地图底图显示
		var sdMapLayer = new ol.layer.Tile({
			source : new ol.source.XYZ({
				wrapX : true,
				projection : 'EPSG:900913',
				tileUrlFunction : tileUrlFunction
			})
		});
		return sdMapLayer;
	}
	var map = new ol.Map({          //ol地图
		target : 'map',   //对应html里id
		layers : [ sdMapLayer() ],    //layer图层
		view : new ol.View({       //视图
			center : transform([ 116.247660429, 40.068108911 ]),
			zoom : 15,
			minZoom : 0,
			maxZoom : 21
		})
	});	
	var getGeometry = function(wkt) {// 将WGS84坐标系的WKT字符串转换成Openlayer的Geometry对象
		if (wkt == null) {
			return null;
		}
		var wktFormat = new ol.format.WKT()
		return wktFormat.readGeometry(wkt, {
			"dataProjection" : 'EPSG:4326',// 输入
			"featureProjection" : 'EPSG:900913'// 输出
		});
	};
	var point = [[116.247660429,40.068108911],[116.247530429,40.068298911],[116.247430429,40.068498911],[116.247380429,40.068598911]];
	var pointWkt = 'POINT(' + point[0][0]+' '+point[0][1] + ')';
	var pointFeatures = [];
	pointFeatures.push(new ol.Feature(getGeometry(pointWkt)));
	console.log(getGeometry(pointWkt));
	var pointVector = new ol.layer.Vector({           //点图层
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
							width : 5
						})
					})
				})
			});
	
	$("#addPoint").click(function(){
		var addPointWkt = 'POINT(' + point[1][0]+' '+point[2][1] + ')';
		pointVector.getSource().addFeature(new ol.Feature(getGeometry(addPointWkt)));
		//map.render();
	});

	map.addLayer(pointVector);
	
});