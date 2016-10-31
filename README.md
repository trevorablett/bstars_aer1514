# ros-qr-listener
*ros-qr-listener* is a ROS wrapper of [zbarlight](https://github.com/Polyconseil/zbarlight/) QR code reader.

# Installation

1. Instal [PIL](http://python-pillow.org/);

    ```sh
    $ pip install Pillow
    ```
    
2. Install [ZBar](http://zbar.sourceforge.net/) bar code reader:

    ```sh
    $ apt-get install libzbar0 libzbar-dev
    ```
    
3. Install [zbarlight](https://github.com/Polyconseil/zbarlight/):

    ```sh
    $ pip install zbarlight
    ``` 
    
4. Install [cv_bridge](http://wiki.ros.org/cv_bridge):

    ```sh
    $ apt-get install ros-<rosdistro>-cv-bridge
    ```
    Replace *``<rosdistro>``* with your ROS distribution (e.g., indigo).

# Example

> Launch *example.launch* in launch folder:

- Requires [uvc_camera](http://wiki.ros.org/uvc_camera);

    ```sh
    $ apt-get install ros-<rosdistro>-cv-bridge
    ```
    
- Subscribes to *image_raw* topic;
- Publishes to *qrcode* topic.
