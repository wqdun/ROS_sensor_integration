<%@ page contentType="text/html;charset=utf-8" %>
<!DOCTYPE html>
<html lang="en">
<head>
    <%@ include file="include/header.jsp" %>
</head>

<body>
<%
    String camerid = request.getParameter("camerid");
%>


<center>
    <a href="javascript:history.go(-1)">
        <img style="position: absolute;left:0px;top:0px;width: 100%;height: 100%;z-index: -1;" src="http://<%=ip%>:8080/stream?topic=/camera/image<%=camerid%>"/>
    </a>
</center>


<%@ include file="include/footer.jsp" %>

</body>

</html>