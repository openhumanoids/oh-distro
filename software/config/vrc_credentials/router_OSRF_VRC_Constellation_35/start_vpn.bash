#!/bin/bash
set -e
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ $UID != 0 ]; then
  echo "You're not root.  Run this script under sudo."
  exit 1
fi

echo "Killing other openvpn connections..."
killall openvpn || true
openvpn --config  $DIR/openvpn.config >/dev/null 2>&1 &

# Wait for tun0 to come up, then add a static route to the 10.0.0.0/24 network, which is the VPC on the other side
# of the router.
while ! ifconfig tun0 || test -z "`ifconfig tun0 | grep 'inet addr'`" 2>/dev/null; do
  echo "Waiting for tun0 to come up..."
  sleep 1
done

echo "Adding route to 10.0.0.0/24 network"
route add -net 10.0.0.0 netmask 255.255.255.0 gw 11.8.0.2

echo "VPN ready.  To kill it:"
echo "    sudo killall openvpn"
