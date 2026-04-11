#!/bin/bash
# Setup WiFi Access Point for ESP32 robots inside Docker container
#
# Prerequisites:
#   - Container running with --privileged and --net=host
#   - MT7612U WiFi dongle plugged in
#
# Usage:
#   sudo ./setup_wifi_ap.sh [interface_name]
#   sudo ./setup_wifi_ap.sh wlan1
#
# This script:
#   1. Identifies the WiFi dongle interface
#   2. Assigns a static IP (192.168.4.1)
#   3. Starts hostapd (creates plato-arena WiFi network)
#   4. Starts dnsmasq (DHCP server for ESP32 robots)

set -e

IFACE=${1:-wlan1}
SSID="plato-arena"
PASSWORD="platopod123"
IP="192.168.4.1"
DHCP_RANGE_START="192.168.4.10"
DHCP_RANGE_END="192.168.4.50"
CHANNEL=6

echo "=== Plato Pod WiFi AP Setup ==="
echo "Interface: $IFACE"
echo "SSID: $SSID"
echo "IP: $IP"

# Check interface exists
if ! ip link show "$IFACE" &>/dev/null; then
    echo "ERROR: Interface $IFACE not found."
    echo "Available wireless interfaces:"
    iw dev | grep Interface
    exit 1
fi

# Check AP mode support
if ! iw list | grep -A5 "Supported interface modes" | grep -q "AP"; then
    echo "ERROR: Interface $IFACE does not support AP mode."
    echo "Check chipset — MT7612U or RT5572 recommended."
    exit 1
fi

# Stop any existing services
systemctl stop hostapd 2>/dev/null || true
systemctl stop dnsmasq 2>/dev/null || true
killall hostapd 2>/dev/null || true
killall dnsmasq 2>/dev/null || true

# Configure interface
ip link set "$IFACE" down
ip addr flush dev "$IFACE"
ip addr add "$IP/24" dev "$IFACE"
ip link set "$IFACE" up

# Write hostapd config
cat > /tmp/hostapd.conf <<EOF
interface=$IFACE
ssid=$SSID
hw_mode=g
channel=$CHANNEL
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=$PASSWORD
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
EOF

# Write dnsmasq config
cat > /tmp/dnsmasq.conf <<EOF
interface=$IFACE
dhcp-range=$DHCP_RANGE_START,$DHCP_RANGE_END,255.255.255.0,24h
bind-interfaces
server=8.8.8.8
domain-needed
bogus-priv
EOF

# Start hostapd
echo "Starting hostapd..."
hostapd -B /tmp/hostapd.conf

# Start dnsmasq
echo "Starting dnsmasq..."
dnsmasq -C /tmp/dnsmasq.conf

echo ""
echo "=== WiFi AP Ready ==="
echo "SSID: $SSID"
echo "Password: $PASSWORD"
echo "Server IP: $IP"
echo "DHCP range: $DHCP_RANGE_START — $DHCP_RANGE_END"
echo ""
echo "ESP32 firmware should connect to:"
echo "  WIFI_SSID = \"$SSID\""
echo "  WIFI_PASS = \"$PASSWORD\""
echo "  AGENT_IP  = \"$IP\""
echo "  AGENT_PORT = 8888"
