# RoboPorter UI
So this is where the Roboporter UI files are stored. It is broken down into two different sections:
1. Backend UI
2. Front End Software

## Backend UI
The Backend Software is written in python and is /backend.py This software runs on the second RPi and acts as the link between the UI and the server python code. It is mostly just a piece of software the forwards connections from one program to the other and handles web socket connections.

## Front End 
The front end is a web interface built using the Bootstrap framework. 

## Setup to get UI working
To get the UI working the following steps must be followed on the Raspberry Pi:
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

auto br0
iface br0 inet static
        address 192.168.0.1
        netmask 255.255.255.0
        network 192.168.0.0
        broadcast 192.168.0.255
        gateway 192.168.0.1
        bridge-ports eth0 wlan0
``` 
This bridge simply connects the eth0 and wlan0 interfaces together onto the same subnet.

13. Configure DNSMASQ `/etc/dnsmasq.conf` so that the following lines look like this: `address=/roboporter.local/192.168.0.1` and 
`dchp-range=192.168.0.5,192.168.0.150,12h`. This redirects roboporter.local to the server and sets the range of IP addresses for the DHCP server.

14. Reboot using `sudo reboot` 
15. Install FTP server vsftpd -- explained in log book. To be typed up later remember to change conf to allow write commands
16. User group creation
17. Copy UI files over to UI PI and copy server.py over to the Control PI. The UI files need to go in `/var/www/html` and the server.py file needs to go in `/home/pi`
18. `Server.py` and `backend.py` need to be setup as services so they run on boot. To do this go to `/lib/systemd/system/portercontrol.service` on the Control PI and `/lib/systemd/system/porterUI.service` on the UI Pi. Then copy 
``` 
[Unit]
Description=Roboporter Service
After=multi-user.target

[Service]
Type=idle
User=pi
ExecStart=/usr/bin/python  /home/pi/server.py >/home/pi/sample.log 2>&1

[Install]
WantedBy=multi-user.target

```
to the Control PI and 

```
[Unit]
Description=Roboporter UI Service
After=multi-user.target

[Service]
Type=idle
User=pi
ExecStart=/usr/bin/python  /var/www/html/backend.py >/home/pi/sample.log 2>&1

[Install]
WantedBy=multi-user.target
``` 
to the UI pi.

19. Finally you then need to run `sudo systemctl daemon-reload` on each Pi then `sudo systemctl enable portercontrol.service` and `sudo systemctl enable porterUI.service` on the approproate PIs. The status of each service can be viewed using `sudo systemctl status portercontrol` and `sudo systemctl status porterUI`
20. Need to discuss static IP for Control Pi

## Install MJPG-Streamer

1. Install the initial dependancies 
`sudo apt-get install libjpeg8-dev imagemagick libv4l-dev`
2. Add missing videodev 
`sudo ln -s /usr/include/linux/videodev2.h /usr/include/linux/videodev.h`
3. Download MJPG-Streamer 
`wget http://sourceforge.net/code-snapshots/svn/m/mj/mjpg-streamer/code/mjpg-streamer-code-182.zip`
4. Unzip File 
`unzip mjpg-streamer-code-182.zip`

5. Make Instalation 
```
sudo cp mjpg_streamer /usr/local/bin
sudo cp output_http.so input_file.so input_uvc.so /usr/local/lib/
sudo cp -R www /usr/local/www
```
6. A patch is required to fix install. See `MJPG_Streamer/input_uvc_patch.txt`
place the file in: 
`/home/pi/mjpg-streamer-code/mjpg-streamer`

7. Run the following code to install the patch:

```
cd mjpg-streamer-code/mjpg-streamer
patch -p0 < input_uvc_patch.txt
make USE_LIBV4L2=true clean all
sudo make DESTDIR=/usr/local install
```
8. Start the server using:
```
/usr/local/bin/mjpg_streamer -i "/usr/local/lib/input_uvc.so" -o "/usr/local/lib/output_http.so -w /usr/local/www"
```
9. To make server start on boot you need to copy `MJPG-Streamer/boot.sh` to the root directory. Then add the following code to `/etc/rc.local` 
```
/home/pi/boot.sh
```

REFERENCE;

`https://blog.miguelgrinberg.com/post/how-to-build-and-run-mjpg-streamer-on-the-raspberry-pi`
`https://jacobsalmela.com/2014/05/31/raspberry-pi-webcam-using-mjpg-streamer-over-internet/`

## Implemented
* Control Page
* Mapping info and clickable map 
* Added ability to change a user password and create a new user
* Added a graphical output for the ultrasonic sensors
* Home Screen withquick view debug and status
* Battery indicator on home screen 
* Streaming Webcam to interface

## Work to follow
* Finish mapping UI
* Finish settings
