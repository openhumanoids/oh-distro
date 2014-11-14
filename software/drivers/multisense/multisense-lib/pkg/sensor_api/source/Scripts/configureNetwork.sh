echo 16777215 > /proc/sys/net/core/rmem_max
echo 16777215 > /proc/sys/net/core/wmem_max
ifconfig eth0 mtu 7200

