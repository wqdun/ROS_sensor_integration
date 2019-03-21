killall hdop_teller_node
killall ntd_info_process_node
killall display_had_node
kill -9 $(pidof SmartCollector)
kill -9 $(pidof roscameragpsimg)
kill -9 $(pidof nodelet)
kill -9 $(pidof myviz)
kill -9 $(pidof rviz)
echo 2 >> /tmp/log

echo 1 >> /tmp/log
