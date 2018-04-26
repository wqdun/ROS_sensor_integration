<%@ page contentType="text/html;charset=utf-8" %>
<!DOCTYPE html>
<html lang="en">
<head>
  <%@ include file="include/header.jsp" %>
  <script>
    function projlayer() {
        var cityCode = $("#city").val().substr(-4);
        var dayNightCode = 1;
        if ('白天' != $("#dayORnight").val()) {
            dayNightCode = 2;
        }

        var dateToday = new Date();
        var dateSys = dateToday.getDate();
        var monthSys = dateToday.getMonth() + 1;
        var yearSys = dateToday.getFullYear();
        if (monthSys >= 1 && monthSys <= 9) {
            monthSys = "0" + monthSys;
        }
        if (dateSys >= 0 && dateSys <= 9) {
            dateSys = "0" + dateSys;
        }
        var tdateSys = (yearSys + "").substr(2) + monthSys + dateSys;
        var deviceId = document.getElementById('deviceID').value.substr(-4);
        var projectNameWithDate = cityCode + "-" + dayNightCode + "-" + document.getElementById('pid').value + deviceId + "-" + tdateSys;
        console.log('projectNameWithDate: ' + projectNameWithDate);
        $("#projname").html(projectNameWithDate);
    }

    function createProject() {
        console.log("Submit a new project.");
        var projectNameWithDate = $('#projname').text();
        var clientMsg = new ROSLIB.Message({
            project_name: projectNameWithDate,
            system_cmd: 0,
        });

        pubCmd_.publish(clientMsg);
    }

  </script>
</head>

<body>

  <div class="modal fade" id="myModal" tabindex="-1" role="dialog" aria-labelledby="myModalLabel" aria-hidden="true">
    <div class="modal-dialog">
      <div class="modal-content">
        <div class="modal-header">
          <button type="button" class="close" data-dismiss="modal" aria-hidden="true">
            &times;
          </button>
          <h4 class="modal-title" id="myModalLabel">
              新建工程
          </h4>
        </div>
        <div class="modal-body">
          提交新工程：<label id="projname"></label>
        </div>
        <div class="modal-footer">
          <button type="button" class="btn btn-default" data-dismiss="modal">
            取消
          </button>
          <button type="button" class="btn btn-primary" onclick="createProject();" data-dismiss="modal">
            提交
          </button>
        </div>
      </div>
    </div>
  </div>

  <div class="modal fade" id="optModal" tabindex="-1" role="dialog" aria-labelledby="myModalLabel" aria-hidden="true">
    <div class="modal-dialog">
      <div class="modal-content">
        <div class="modal-header">
          <button type="button" class="close" data-dismiss="modal" aria-hidden="true">
            &times;
          </button>
          <h4 class="modal-title" id="optModalLabel">
            提交操作
          </h4>
        </div>
        <div class="modal-body">
          确认要提交操作？<label id="optname"></label>
          <input type="hidden" id="optid" value="0"/>
        </div>
        <div class="modal-footer">
          <button type="button" class="btn btn-default" data-dismiss="modal">
            取消
          </button>
          <button type="button" class="btn btn-primary" data-dismiss="modal" id="cmtBtn">
            提交
          </button>
        </div>
      </div>
    </div>
  </div>

  <div class="subnavbar">
    <div class="subnavbar-inner">
      <div class="container">
        <ul class="mainnav">
          <li class="active">
            <a href="index.jsp">
              <i class="icon-th-large"></i>
              <span>系统设置</span>
            </a>
          </li>
          <li>
            <a href="monitor.jsp">
              <i class="icon-facetime-video"></i>
              <span>运行监控</span>
            </a>
          </li>
        </ul>
      </div>
    </div>
  </div>

  <div class="main-inner">
    <div class="container">
      <div class="row">
        <div class="span12">
          <div class="widget ">
            <div class="widget-header">
              <i class="icon-user"></i>
              <h3>系统设置</h3>
            </div>
            <div class="widget-content">
              <div class="tabbable">
                <div class="tab-content">
                  <div class="tab-pane active" id="profile">
                    <div id="edit-profile" class="form-horizontal"/>
                    <div class="control-group">
                      <label class="control-label">工程信息</label>
                      <div class="controls">
                        <select class="form-control" id='city'>
                          <option>北京-北京-1001</option>
                          <option>上海-上海-1002</option>
                          <option>天津-天津-1013</option>
                          <option>重庆-重庆-1008</option>
                          <option>海南省-三亚-1003</option>
                          <option>海南省-海口市-1017</option>
                          <option>海南省-文昌市-1018</option>
                          <option>海南省-琼海市-1019</option>
                          <option>海南省-万宁市-1020</option>
                          <option>海南省-五指山市-1021</option>
                          <option>海南省-东方市-1022</option>
                          <option>海南省-儋州市-1023</option>
                          <option>海南省-临高县-1024</option>
                          <option>海南省-澄迈县-1025</option>
                          <option>海南省-定安县-1026</option>
                          <option>海南省-屯昌县-1027</option>
                          <option>海南省-昌江黎族自治县-1028</option>
                          <option>海南省-白沙黎族自治县-1029</option>
                          <option>海南省-琼中黎族苗族自治县-1030</option>
                          <option>海南省-陵水黎族自治县-1031</option>
                          <option>海南省-保亭黎族苗族自治县-1032</option>
                          <option>海南省-乐东黎族自治县-1033</option>
                          <option>河北省-石家庄市-1101</option>
                          <option>河北省-唐山市-1102</option>
                          <option>河北省-秦皇岛市-1103</option>
                          <option>河北省-邯郸市-1104</option>
                          <option>河北省-邢台市-1105</option>
                          <option>河北省-保定市-1106</option>
                          <option>河北省-张家口市-1107</option>
                          <option>河北省-承德市-1108</option>
                          <option>河北省-沧州市-1109</option>
                          <option>河北省-廊坊市-1110</option>
                          <option>河北省-衡水市-1111</option>
                          <option>山西省-太原市-1201</option>
                          <option>山西省-大同市-1202</option>
                          <option>山西省-阳泉市-1203</option>
                          <option>山西省-长治市-1204</option>
                          <option>山西省-晋城市-1205</option>
                          <option>山西省-朔州市-1206</option>
                          <option>山西省-晋中市-1207</option>
                          <option>山西省-运城市-1208</option>
                          <option>山西省-忻州市-1209</option>
                          <option>山西省-临汾市-1210</option>
                          <option>山西省-吕梁市-1211</option>
                          <option>内蒙古-呼和浩特-1301</option>
                          <option>内蒙古-包头市-1302</option>
                          <option>内蒙古-乌海市-1303</option>
                          <option>内蒙古-赤峰市-1304</option>
                          <option>内蒙古-通辽市-1305</option>
                          <option>内蒙古-鄂尔多斯-1306</option>
                          <option>内蒙古-呼伦贝尔-1307</option>
                          <option>内蒙古-巴彦淖尔-1308</option>
                          <option>内蒙古-乌兰察布-1309</option>
                          <option>内蒙古-兴安盟-1310</option>
                          <option>内蒙古-锡林郭勒-1311</option>
                          <option>内蒙古-阿拉善盟-1312</option>
                          <option>辽宁省-沈阳市-1401</option>
                          <option>辽宁省-大连市-1402</option>
                          <option>辽宁省-鞍山市-1403</option>
                          <option>辽宁省-抚顺市-1404</option>
                          <option>辽宁省-本溪市-1405</option>
                          <option>辽宁省-丹东市-1406</option>
                          <option>辽宁省-锦州市-1407</option>
                          <option>辽宁省-营口市-1408</option>
                          <option>辽宁省-阜新市-1409</option>
                          <option>辽宁省-辽阳市-1410</option>
                          <option>辽宁省-盘锦市-1411</option>
                          <option>辽宁省-铁岭市-1412</option>
                          <option>辽宁省-朝阳市-1413</option>
                          <option>辽宁省-葫芦岛市-1414</option>
                          <option>吉林省-长春市-1501</option>
                          <option>吉林省-吉林市-1502</option>
                          <option>吉林省-四平市-1503</option>
                          <option>吉林省-辽源市-1504</option>
                          <option>吉林省-通化市-1505</option>
                          <option>吉林省-白山市-1506</option>
                          <option>吉林省-松原市-1507</option>
                          <option>吉林省-白城市-1508</option>
                          <option>吉林省-延边朝鲜族自治州-1509</option>
                          <option>黑龙江-哈尔滨市-1601</option>
                          <option>黑龙江-齐齐哈尔-1602</option>
                          <option>黑龙江-鸡西市-1603</option>
                          <option>黑龙江-鹤岗市-1604</option>
                          <option>黑龙江-双鸭山市-1605</option>
                          <option>黑龙江-大庆市-1606</option>
                          <option>黑龙江-伊春市-1607</option>
                          <option>黑龙江-佳木斯市-1608</option>
                          <option>黑龙江-七台河市-1609</option>
                          <option>黑龙江-牡丹江市-1610</option>
                          <option>黑龙江-黑河市-1611</option>
                          <option>黑龙江-绥化市-1612</option>
                          <option>黑龙江-大兴安岭-1613</option>
                          <option>江苏省-南京-1010</option>
                          <option>江苏省-苏州-1012</option>
                          <option>江苏省-徐州市-1701</option>
                          <option>江苏省-常州市-1702</option>
                          <option>江苏省-无锡市-1703</option>
                          <option>江苏省-南通市-1704</option>
                          <option>江苏省-连云港市-1705</option>
                          <option>江苏省-淮安市-1706</option>
                          <option>江苏省-盐城市-1707</option>
                          <option>江苏省-扬州市-1708</option>
                          <option>江苏省-镇江市-1709</option>
                          <option>江苏省-泰州市-1710</option>
                          <option>江苏省-宿迁市-1711</option>
                          <option>浙江省-杭州-1009</option>
                          <option>浙江省-宁波市-1801</option>
                          <option>浙江省-温州市-1802</option>
                          <option>浙江省-嘉兴市-1803</option>
                          <option>浙江省-湖州市-1804</option>
                          <option>浙江省-绍兴市-1805</option>
                          <option>浙江省-金华市-1806</option>
                          <option>浙江省-衢州市-1807</option>
                          <option>浙江省-舟山市-1808</option>
                          <option>浙江省-台州市-1809</option>
                          <option>浙江省-丽水市-1810</option>
                          <option>安徽省-合肥市-1901</option>
                          <option>安徽省-芜湖市-1902</option>
                          <option>安徽省-蚌埠市-1903</option>
                          <option>安徽省-淮南市-1904</option>
                          <option>安徽省-马鞍山市-1905</option>
                          <option>安徽省-淮北市-1906</option>
                          <option>安徽省-铜陵市-1907</option>
                          <option>安徽省-安庆市-1908</option>
                          <option>安徽省-黄山市-1909</option>
                          <option>安徽省-滁州市-1910</option>
                          <option>安徽省-阜阳市-1911</option>
                          <option>安徽省-宿州市-1912</option>
                          <option>安徽省-巢湖市-1913</option>
                          <option>安徽省-六安市-1914</option>
                          <option>安徽省-亳州市-1915</option>
                          <option>安徽省-池州市-1916</option>
                          <option>安徽省-宣城市-1917</option>
                          <option>福建省-厦门-1015</option>
                          <option>福建省-福州市-2001</option>
                          <option>福建省-莆田市-2002</option>
                          <option>福建省-三明市-2003</option>
                          <option>福建省-泉州市-2004</option>
                          <option>福建省-漳州市-2005</option>
                          <option>福建省-南平市-2006</option>
                          <option>福建省-龙岩市-2007</option>
                          <option>福建省-宁德市-2008</option>
                          <option>江西省-南昌市-2101</option>
                          <option>江西省-景德镇市-2102</option>
                          <option>江西省-萍乡市-2103</option>
                          <option>江西省-九江市-2104</option>
                          <option>江西省-新余市-2105</option>
                          <option>江西省-鹰潭市-2106</option>
                          <option>江西省-赣州市-2107</option>
                          <option>江西省-吉安市-2108</option>
                          <option>江西省-宜春市-2109</option>
                          <option>江西省-抚州市-2110</option>
                          <option>江西省-上饶市-2111</option>
                          <option>山东省-青岛-1011</option>
                          <option>山东省-济南市-2201</option>
                          <option>山东省-淄博市-2202</option>
                          <option>山东省-枣庄市-2203</option>
                          <option>山东省-东营市-2204</option>
                          <option>山东省-烟台市-2205</option>
                          <option>山东省-潍坊市-2206</option>
                          <option>山东省-济宁市-2207</option>
                          <option>山东省-泰安市-2208</option>
                          <option>山东省-威海市-2209</option>
                          <option>山东省-日照市-2210</option>
                          <option>山东省-莱芜市-2211</option>
                          <option>山东省-临沂市-2212</option>
                          <option>山东省-德州市-2213</option>
                          <option>山东省-聊城市-2214</option>
                          <option>山东省-滨州市-2215</option>
                          <option>山东省-菏泽市-2216</option>
                          <option>河南省-郑州市-2301</option>
                          <option>河南省-开封市-2302</option>
                          <option>河南省-洛阳市-2303</option>
                          <option>河南省-平顶山市-2304</option>
                          <option>河南省-安阳市-2305</option>
                          <option>河南省-鹤壁市-2306</option>
                          <option>河南省-新乡市-2307</option>
                          <option>河南省-焦作市-2308</option>
                          <option>河南省-濮阳市-2309</option>
                          <option>河南省-许昌市-2310</option>
                          <option>河南省-漯河市-2311</option>
                          <option>河南省-三门峡市-2312</option>
                          <option>河南省-南阳市-2313</option>
                          <option>河南省-商丘市-2314</option>
                          <option>河南省-信阳市-2315</option>
                          <option>河南省-周口市-2316</option>
                          <option>河南省-驻马店市-2317</option>
                          <option>河南省-济源市-2318</option>
                          <option>湖北省-武汉-1014</option>
                          <option>湖北省-黄石市-2401</option>
                          <option>湖北省-十堰市-2402</option>
                          <option>湖北省-宜昌市-2403</option>
                          <option>湖北省-襄樊市-2404</option>
                          <option>湖北省-鄂州市-2405</option>
                          <option>湖北省-荆门市-2406</option>
                          <option>湖北省-孝感市-2407</option>
                          <option>湖北省-荆州市-2408</option>
                          <option>湖北省-黄冈市-2409</option>
                          <option>湖北省-咸宁市-2410</option>
                          <option>湖北省-随州市-2411</option>
                          <option>湖北省-恩施-2412</option>
                          <option>湖北省-湛江市-2413</option>
                          <option>湖北省-襄阳市-2414</option>
                          <option>湖北省-天门市-2415</option>
                          <option>湖北省-仙桃市-2416</option>
                          <option>湖北省-神农架林区-2417</option>
                          <option>湖南省-长沙市-2501</option>
                          <option>湖南省-株洲市-2502</option>
                          <option>湖南省-湘潭市-2503</option>
                          <option>湖南省-衡阳市-2504</option>
                          <option>湖南省-邵阳市-2505</option>
                          <option>湖南省-岳阳市-2506</option>
                          <option>湖南省-常德市-2507</option>
                          <option>湖南省-张家界市-2508</option>
                          <option>湖南省-益阳市-2509</option>
                          <option>湖南省-郴州市-2510</option>
                          <option>湖南省-永州市-2511</option>
                          <option>湖南省-怀化市-2512</option>
                          <option>湖南省-娄底市-2513</option>
                          <option>湖南省-湘西-2514</option>
                          <option>广东省-深圳-1004</option>
                          <option>广东省-广州-1006</option>
                          <option>广东省-韶关市-2601</option>
                          <option>广东省-珠海市-2602</option>
                          <option>广东省-汕头市-2603</option>
                          <option>广东省-佛山市-2604</option>
                          <option>广东省-江门市-2605</option>
                          <option>广东省-湛江市-2606</option>
                          <option>广东省-茂名市-2607</option>
                          <option>广东省-肇庆市-2608</option>
                          <option>广东省-惠州市-2609</option>
                          <option>广东省-梅州市-2610</option>
                          <option>广东省-汕尾市-2611</option>
                          <option>广东省-河源市-2612</option>
                          <option>广东省-阳江市-2613</option>
                          <option>广东省-清远市-2614</option>
                          <option>广东省-东莞市-2615</option>
                          <option>广东省-中山市-2616</option>
                          <option>广东省-潮州市-2617</option>
                          <option>广东省-揭阳市-2618</option>
                          <option>广东省-云浮市-2619</option>
                          <option>广西-南宁市-2701</option>
                          <option>广西-柳州市-2702</option>
                          <option>广西-桂林市-2703</option>
                          <option>广西-梧州市-2704</option>
                          <option>广西-北海市-2705</option>
                          <option>广西-防城港市-2706</option>
                          <option>广西-钦州市-2707</option>
                          <option>广西-贵港市-2708</option>
                          <option>广西-玉林市-2709</option>
                          <option>广西-百色市-2710</option>
                          <option>广西-贺州市-2711</option>
                          <option>广西-河池市-2712</option>
                          <option>广西-来宾市-2713</option>
                          <option>广西-崇左市-2714</option>
                          <option>四川省-成都-1007</option>
                          <option>四川省-自贡市-2801</option>
                          <option>四川省-攀枝花市-2802</option>
                          <option>四川省-泸州市-2803</option>
                          <option>四川省-德阳市-2804</option>
                          <option>四川省-绵阳市-2805</option>
                          <option>四川省-广元市-2806</option>
                          <option>四川省-遂宁市-2807</option>
                          <option>四川省-内江市-2808</option>
                          <option>四川省-乐山市-2809</option>
                          <option>四川省-南充市-2810</option>
                          <option>四川省-眉山市-2811</option>
                          <option>四川省-宜宾市-2812</option>
                          <option>四川省-广安市-2813</option>
                          <option>四川省-达州市-2814</option>
                          <option>四川省-雅安市-2815</option>
                          <option>四川省-巴中市-2816</option>
                          <option>四川省-资阳市-2817</option>
                          <option>四川省-阿坝-2818</option>
                          <option>四川省-甘孜-2819</option>
                          <option>四川省-凉山-2820</option>
                          <option>贵州省-贵阳市-2901</option>
                          <option>贵州省-六盘水市-2902</option>
                          <option>贵州省-遵义市-2903</option>
                          <option>贵州省-安顺市-2904</option>
                          <option>贵州省-铜仁地区-2905</option>
                          <option>贵州省-黔西南-2906</option>
                          <option>贵州省-毕节地区-2907</option>
                          <option>贵州省-黔东南-2908</option>
                          <option>贵州省-黔南-2909</option>
                          <option>云南省-昆明市-3001</option>
                          <option>云南省-曲靖市-3002</option>
                          <option>云南省-玉溪市-3003</option>
                          <option>云南省-保山市-3004</option>
                          <option>云南省-昭通市-3005</option>
                          <option>云南省-丽江市-3006</option>
                          <option>云南省-普洱市-3007</option>
                          <option>云南省-临沧市-3008</option>
                          <option>云南省-楚雄-3009</option>
                          <option>云南省-红河-3010</option>
                          <option>云南省-文山-3011</option>
                          <option>云南省-西双版纳-3012</option>
                          <option>云南省-大理-3013</option>
                          <option>云南省-德宏-3014</option>
                          <option>云南省-怒江-3015</option>
                          <option>云南省-迪庆-3016</option>
                          <option>陕西省-西安-1016</option>
                          <option>陕西省-铜川市-3101</option>
                          <option>陕西省-宝鸡市-3102</option>
                          <option>陕西省-咸阳市-3103</option>
                          <option>陕西省-渭南市-3104</option>
                          <option>陕西省-延安市-3105</option>
                          <option>陕西省-汉中市-3106</option>
                          <option>陕西省-榆林市-3107</option>
                          <option>陕西省-安康市-3108</option>
                          <option>陕西省-商洛市-3109</option>
                          <option>甘肃省-兰州市-3201</option>
                          <option>甘肃省-嘉峪关市-3202</option>
                          <option>甘肃省-金昌市-3203</option>
                          <option>甘肃省-白银市-3204</option>
                          <option>甘肃省-天水市-3205</option>
                          <option>甘肃省-武威市-3206</option>
                          <option>甘肃省-张掖市-3207</option>
                          <option>甘肃省-平凉市-3208</option>
                          <option>甘肃省-酒泉市-3209</option>
                          <option>甘肃省-庆阳市-3210</option>
                          <option>甘肃省-定西市-3211</option>
                          <option>甘肃省-陇南市-3212</option>
                          <option>甘肃省-临夏-3213</option>
                          <option>甘肃省-甘南-3214</option>
                          <option>青海省-西宁市-3301</option>
                          <option>青海省-海东地区-3302</option>
                          <option>青海省-海北-3303</option>
                          <option>青海省-黄南-3304</option>
                          <option>青海省-海南-3305</option>
                          <option>青海省-果洛-3306</option>
                          <option>青海省-玉树-3307</option>
                          <option>青海省-海西-3308</option>
                          <option>宁夏-银川市-3401</option>
                          <option>宁夏-石嘴山市-3402</option>
                          <option>宁夏-吴忠市-3403</option>
                          <option>宁夏-固原市-3404</option>
                          <option>宁夏-中卫市-3405</option>
                          <option>新疆-乌鲁木齐-3501</option>
                          <option>新疆-克拉玛依-3502</option>
                          <option>新疆-吐鲁番-3503</option>
                          <option>新疆-哈密地区-3504</option>
                          <option>新疆-昌吉-3505</option>
                          <option>新疆-博尔塔拉-3506</option>
                          <option>新疆-巴音郭楞-3507</option>
                          <option>新疆-阿克苏-3508</option>
                          <option>新疆-克孜勒-3509</option>
                          <option>新疆-喀什地区-3510</option>
                          <option>新疆-和田地区-3511</option>
                          <option>新疆-伊犁-3512</option>
                          <option>新疆-塔城地区-3513</option>
                          <option>新疆-阿勒泰-3514</option>
                          <option>新疆-五家渠市-3515</option>
                          <option>新疆-图木舒克市-3516</option>
                          <option>新疆-石河子市-3517</option>
                          <option>西藏-拉萨市-1005</option>
                          <option>西藏-昌都地区-3601</option>
                          <option>西藏-山南地区-3602</option>
                          <option>西藏-日喀则-3603</option>
                          <option>西藏-那曲地区-3604</option>
                          <option>西藏-阿里地区-3605</option>
                          <option>西藏-林芝地区-3606</option>
                          <option>香港-香港-3701</option>
                          <option>澳门-澳门-3801</option>
                          <option>台湾-台湾-3901</option>
                          <option>东沙群岛-东沙群岛-4001</option>
                          <option>西沙群岛-西沙群岛-4101</option>
                          <option>中沙群岛-中沙群岛-4201</option>
                          <option>测试-测试-9999</option>
                        </select>
                        <select class="form-control" id='dayORnight'>
                          <option>白天</option>
                          <option>晚上</option>
                        </select>
                      </div>
                    </div>
                    <div class="control-group">
                      <label class="control-label"></label>
                      <div class="controls">
                        <input class="form-control" placeholder="任务ID" maxlength=1 id='pid'>
                        <select class="form-control" id='deviceID'>
                          <option>设备号-0001</option>
                          <option>设备号-0002</option>
                          <option>设备号-0003</option>
                          <option>设备号-0004</option>
                          <option>设备号-0005</option>
                          <option>设备号-0006</option>
                          <option>设备号-0007</option>
                          <option>设备号-0008</option>
                          <option>设备号-0009</option>
                          <option>设备号-0010</option>
                          <option>设备号-0011</option>
                        </select>
                      </div>
                    </div>
                    <div class="control-group">
                      <label class="control-label"></label>
                      <div class="controls">
                        <button type="button" class="btn btn-inverse" id="newProject" data-toggle="modal"
                            data-target="#myModal" onclick="projlayer();">新建工程
                        </button>
                        <button class="btn btn-inverse" id="clear" data-toggle="modal"
                            data-target="#optModal">关闭工程
                        </button>
                      </div>
                    </div>

                    <div class="control-group">
                      <label class="control-label">选择工程</label>
                      <div class="controls">
                        <select class="selectpicker" multiple id='dirname'>
                        </select>
                        <div class="progress progress-striped active">
                        <div class="progress-bar progress-bar-success" role="progressbar"
                            aria-valuenow="60" aria-valuemin="0" aria-valuemax="100"
                            style="width: 100%;" id="pathbar">
                          <span class="sr-only" id="pathcontent">100%</span>
                        </div>
                      </div>
                      </div>
                    </div>

                    <br/>
                    <div class="form-actions">
                      <button class="btn btn-inverse" id="removeData" data-toggle="modal"
                          data-target="#optModal">删除数据
                      </button>
                      <button class="btn btn-inverse" id="fixData" data-toggle="modal"
                          data-target="#optModal">整理数据
                      </button>
                      <button class="btn btn-inverse" id="reboot" data-toggle="modal"
                          data-target="#optModal">重启
                      </button>
                      <button class="btn btn-inverse" id="shutdown" data-toggle="modal"
                          data-target="#optModal">关机
                      </button>
                    </div>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>

  <%@ include file="include/footer.jsp" %>
  <script>
    console.log("Task ID should not empty or contains -.");
    $('#newProject').prop('disabled', true);

  // below from http://www.cnblogs.com/yuzhongwusan/archive/2012/09/03/2669022.html
  var browser = {
    versions: function() {
      var u = navigator.userAgent, app = navigator.appVersion;
      return {
        // IE内核
        trident: u.indexOf('Trident') > -1,
        // opera内核
        presto: u.indexOf('Presto') > -1,
        // 苹果、谷歌内核
        webKit: u.indexOf('AppleWebKit') > -1,
        // 火狐内核
        gecko: u.indexOf('Gecko') > -1 && u.indexOf('KHTML') == -1,
        // 是否为移动终端
        mobile: !!u.match(/AppleWebKit.*Mobile.*/)||!!u.match(/AppleWebKit/),
        // ios终端
        ios: !!u.match(/\(i[^;]+;( U;)? CPU.+Mac OS X/),
        // android终端或者uc浏览器
        android: u.indexOf('Android') > -1 || u.indexOf('Linux') > -1,
        // 是否为iPhone或者QQHD浏览器
        iPhone: u.indexOf('iPhone') > -1 || u.indexOf('Mac') > -1,
        // 是否iPad
        iPad: u.indexOf('iPad') > -1,
        // 是否web应该程序，没有头部与底部
        webApp: u.indexOf('Safari') == -1
      };
    } (),
    language: (navigator.browserLanguage || navigator.language).toLowerCase()
  }
  if(navigator.userAgent.toLowerCase().indexOf('mobile') > -1) {
    console.log("I am a mobile browser.");
    $('.selectpicker').selectpicker('mobile');
  }
  else {
    console.log("I am a desktop browser.");
  }

    var ros_ = new ROSLIB.Ros();
    ros_.connect('ws://<%=ip%>:9090');
    var pubCmd_ = new ROSLIB.Topic({
        ros: ros_,
        name: '/sc_client_cmd',
        messageType: 'sc_msgs/ClientCmd',
    });

    var projectArrListener_ = new ROSLIB.Topic({
        ros: ros_,
        name: '/sc_project_array',
        messageType: 'sc_msgs/ProjectArr'
    });
    projectArrListener_.subscribe(
        function(_projectArrMsg) {
            console.log("I found " + _projectArrMsg.projects.length + " projects in " + projectArrListener_.name);
            for(x in _projectArrMsg.projects) {
                console.log("Project[" + x + "]: " + _projectArrMsg.projects[x]);
                if(0 == x % 2) {
                    $("#dirname").append("<option data-content=\"<span class='label label-success'>" +  _projectArrMsg.projects[x] + "</span>\">" + _projectArrMsg.projects[x] + "</option>");
                }
                else {
                  $("#dirname").append("<option data-content=\"<span class='label label-info'>" +  _projectArrMsg.projects[x] + "</span>\">" + _projectArrMsg.projects[x] + "</option>");
                }
            }
            $("#dirname").selectpicker('refresh');
            projectArrListener_.unsubscribe();
        }
    )

    var centerListener = new ROSLIB.Topic({
      ros: ros_,
      name: '/sc_monitor',
      messageType: 'sc_msgs/MonitorMsg'
    });
    centerListener.subscribe(
      function(message) {
        console.log("I am listening: " + centerListener.name);
        console.log("Process status: " + message.process_num + "/" + message.total_file_num);
        var fixPercent = message.process_num / (message.total_file_num + 0.00000001) * 100;
        fixPercent = fixPercent.toFixed(2);
        console.log("fixPercent: " + fixPercent);
        $("#pathbar").css("width", fixPercent + "%");
        $("#pathcontent").html(fixPercent + "%  (" + message.process_num + "/" + message.total_file_num + ")");

        console.log("city_code: " + message.project_info.city_code);
        if(0 != message.project_info.city_code || message.total_file_num != message.process_num) {
          console.log("You cannot fix data when project running or being fixed.");
          $('#fixData').prop('disabled', true);
        }
        else {
          $('#fixData').prop('disabled', false);
        }

        if(0 == message.project_info.city_code) {
            console.log("No project running.");
            // enable input
            $('#city').prop('disabled', false);
            $('#dayORnight').prop('disabled', false);
            $('#pid').prop('disabled', false);
            $('#deviceID').prop('disabled', false);
            $('#newProject').prop('disabled', false);
            $('#removeData').prop('disabled', false);
            $('#reboot').prop('disabled', false);
            $('#shutdown').prop('disabled', false);
        }
        else {
            // disable input
            $('#city').prop('disabled', true);
            $('#dayORnight').prop('disabled', true);
            $('#pid').prop('disabled', true);
            $('#deviceID').prop('disabled', true);
            $('#newProject').prop('disabled', true);
            $('#removeData').prop('disabled', true);
            $('#reboot').prop('disabled', true);
            $('#shutdown').prop('disabled', true);

            for (var i = 0; i < city.options.length; ++i) {
                if (city.options[i].text.indexOf(message.project_info.city_code) > 0) {
                    city.options[i].selected = true;
                    break;
                }
            }

            console.log("daynight_code: " + typeof(message.project_info.daynight_code) + ", " + message.project_info.daynight_code);
            dayORnight.options[message.project_info.daynight_code - 1].selected = true;

            $("#pid").val(message.project_info.task_id);

            console.log("device_id: " + message.project_info.device_id);
            for(var i = 0; i < deviceID.options.length; ++i) {
                if(deviceID.options[i].text.indexOf(message.project_info.device_id) > 0) {
                    deviceID.options[i].selected = true;
                    break;
                }
            }
        }
      }
    );

    $('#pid').bind('input propertychange', function() {
        if ($('#pid').val() == '' || $('#pid').val().indexOf("-") >= 0) {
            console.log("Task ID should not empty or contains -.");
            $('#newProject').prop('disabled', true);
        }
        else {
            $('#newProject').prop('disabled', false);
        }
    });

    $('#shutdown').click(function () {
        $("#optname").html('关机');
        $("#optid").text("1");
    })
    $('#reboot').click(function () {
        $("#optname").html('重启');
        $("#optid").text("2");
    })
    $('#clear').click(function () {
        $("#optname").html('关闭工程');
        $("#optid").text("3");
    })

    $('#fixData').click(function () {
        $("#optname").html('整理工程: ' + $("#dirname").selectpicker('val'));
        $("#optid").text("4");
    })
    $('#removeData').click(function () {
        $("#optname").html('删除数据: ' + $("#dirname").selectpicker('val'));
        $("#optid").text("5");
        // $('.selectpicker option:selected').remove();
        // $('.selectpicker').selectpicker('refresh');
    })

    $('#cmtBtn').click(function() {
        var projectSelected = $("#dirname").selectpicker('val');
        console.log("projectSelected: " + projectSelected);
        var projects = "";
        if(projectSelected) {
          for(x in projectSelected) {
            console.log("projectSelected[" + x + "]: " + projectSelected[x]);
            projects += projectSelected[x] + ",";
          }
        }
        console.log("projects: " + projects);
        projects = projects.substr(0, projects.length - 1);
        console.log("projects: " + projects);

        var clientMsg = new ROSLIB.Message({
          system_cmd: Number($("#optid").text() ),
          cmd_arguments: projects
        });
        pubCmd_.publish(clientMsg);
    })

  </script>
</body>

</html>
