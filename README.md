# ros-qr-listener
''ros-qr-listener'' is a ROS wrapper for zbarlight to read QR codes.

# Installation

1. Instal Python Imaging Library (PIL);
2. Install ZBar bar code reader <http://zbar.sourceforge.net/>:
- Debian: ''apt-get install libzbar0 libzbar-dev'';
3. Install ''zbarlight'' <https://github.com/Polyconseil/zbarlight/>;
4. Install ''cv_bridge'' <http://wiki.ros.org/cv_bridge>.

# Example

Launch ''example.launch'' in launch folder:
- Requires ''uvc_camera'' <http://wiki.ros.org/uvc_camera>;
- Subscribes to ''image_raw'' topic;
- Publishes to ''qrcode'' topic.
