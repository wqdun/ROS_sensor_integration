<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8" %>
<%@ page import="org.apache.log4j.Logger" %>
<%@ page import="java.util.*,java.io.*,java.text.*,java.net.*" %>
<!DOCTYPE html PUBLIC "-//WAPFORUM//DTD XHTML Mobile 1.0//EN" "http://www.wapforum.org/DTD/xhtml-mobile10.dtd">
<html>
<head>
    <title>打印日志</title>

</head>

<body>
<%
    String location = "";
    if (request.getParameter("position") != null && request.getParameter("position").trim().length() > 0) {
        location = request.getParameter("position");
    }
    String pinfo = "";
    if (request.getParameter("pinfo") != null && request.getParameter("pinfo").trim().length() > 0) {
        pinfo = request.getParameter("pinfo");
    }


    String type = "";
    if(request.getParameter("type")!=null && request.getParameter("type").trim().length()>0 && pinfo!=null && pinfo.length()>0 && location.length()>10){
        SimpleDateFormat formatter = new SimpleDateFormat("yyMMdd");
        SimpleDateFormat sdf=new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
        pinfo = pinfo+"-"+formatter.format(new Date());

        try
        {
            String path="/opt/smartc/record/"+pinfo+"/Rawdata/Event/";
//            String path="D:\\demo\\"+pinfo+"\\Rawdata\\Event\\";
            System.out.println(path);
            File file=new File(path);
            if(!file.exists()){
                file.mkdirs();
            }
            File eventfile = new File(path+"event.txt");
            FileOutputStream fout=new FileOutputStream(eventfile,true); //如果追加方式用true
            StringBuffer sb=new StringBuffer();
            sb.append(location+","+request.getParameter("startLane")+","+request.getParameter("endLane")+","+sdf.format(new Date())+"\n");
            fout.write(sb.toString().getBytes("utf-8"));//注意需要转换对应的字符集
            fout.close();
        }
        catch(IOException ex)
        {
            System.out.println(ex.getStackTrace());
            ex.printStackTrace();
        }

//
//        Logger log = Logger.getRootLogger();
//        log.info(pinfo+ "," + new Date() + "," + new Date().getTime() + "," + location+","+request.getParameter("startLane")+","+request.getParameter("endLane"));
//        System.out.println(pinfo+ "," + new Date() + "," + new Date().getTime() + "," + location+","+request.getParameter("startLane")+","+request.getParameter("endLane"));
    }
%>
</body>
</html>
