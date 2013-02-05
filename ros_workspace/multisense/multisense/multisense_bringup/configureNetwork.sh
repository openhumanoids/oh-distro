ifconfig eth0 10.10.72.56

echo 16777215 > /proc/sys/net/core/rmem_max
echo 16777215 > /proc/sys/net/core/wmem_max
ifconfig eth0 mtu 7200

