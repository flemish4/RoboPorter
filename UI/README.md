# RoboPorter UI
So this is where the Roboporter UI files are stored. It is broken down into two different sections:
1. Backend UI
2. Front End Software

## Backend UI
The Backend Software is written in python and is /backend.py This software runs on the second RPi and acts as the link between the UI and the server python code. It is mostly just a piece of software the forwards connections from one program to the other and handles web socket connections.

## Front End 
The front end is a web interface built using the Bootstrap framework. To get it working the following steps must be followed on the Raspberry Pi:
1. Install Apache Web Server 
2. Install PHP 7 or greater
3. Install MySql 
4. Install PHPMyAdmin for database admin tasks
5. Upload the database stored in /database
6. Change `/php/include.php` to be the correct database details for the database created in step3
7. To setup the wireless access point install hostapd
8. Then install dnsmasq for a DHCP server and DNS server for the access point created in step 7
9. Install bridge-utilities  
10. Configure hostapd by adding the following code to `/etc/hostapd/hostapd.conf` 
```
ctrl_interface=/var/run/hostapd
###############################
# Basic Config
###############################
macaddr_acl=0
auth_algs=1
# Most modern wireless drivers in the kernel need driver=nl80211
driver=nl80211
##########################
# Local configuration...
##########################
interface=wlan0
bridge=br0
hw_mode=g
channel=1
ssid=RoboPorter
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=3
wpa_passphrase=R0b0p0rter
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP

```
 You can also change the password and SSID at this point.
11. You also need to add `DAEMON_CONF="/etc/hostapd/hostapd.conf"` to `/etc/default/hostapd`
12. Setup bridge in `/etc/network/interfaces` so it looks like this:

```
# This file describes the network interfaces available on your system
# and how to activate them. For more information, see interfaces(5).

# The loopback network interface
auto lo
iface lo inet loopback

# The primary network interface
#auto eth0
#iface eth0 inet static
#       address 192.168.0.2
#       netmask 255.255.255.0
#       network 192.168.0.0
#       broadcast 192.168.0.255
#       gateway 192.168.0.1

auto br0
iface br0 inet static
        address 192.168.0.2
        netmask 255.255.255.0
        network 192.168.0.0
        broadcast 192.168.0.255
        gateway 192.168.0.1
        bridge-ports eth0 wlan0

 ``` 
This bridge simply connects the eth0 and wlan0 interfaces together onto the same subnet 
13. Configure dnsmasq `/etc/dnsmasq.conf` so that the following lines look like this: `address=/roboporter.local/192.168.0.2` and 
`dchp-range=192.168.0.5,192.168.0.150,12h`. This redirects roboporter.local to the server and sets the range of IP addresses for the DHCP server.

14. Reboot using `sudo reboot` 
15. Install FTP server vsftpd -- explained in log book. To be typed up later

## Implemented
* Control Page
* Mapping info and clickable map 
* Added ability to change a user password and create a new user
* Added a graphical output for the ultrasonic sensors
* Home Screen withquick view debug and status
* Battery indicator on home screen 

## Work to follow
* Finish mapping UI
* Finish settings
