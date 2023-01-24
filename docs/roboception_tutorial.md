 # Roboception tutorial 

 ## Installation

- install [rc_genicam_api](https://github.com/roboception/rc_genicam_api)
- install [openCV](https://www.google.com/search?q=opencv+cmake+install&oq=opencv+&aqs=chrome.2.69i59l3j69i57j0i20i263i512j69i60l3.3103j0j4&sourceid=chrome&ie=UTF-8)
- install [rc_visard_opencv_example](https://github.com/roboception/rc_visard_opencv_example)

## Connection

- connect the Roboception camera through ethernet
- the IP address of the device should be 169.254.7.156
- in network setting, put the address as 169.254.7.56 with a mask 255.255.0.0
- then, first ping yourself 169.254.7.56
- then, ping the camera 169.254.7.156 (for some reason it does not work before pinging first yourself)

## Uses

- then, we can use some of the tools of the rc_genicam_api. Note that the device ID is 02940273
    - to see available devices
        - $ cd rc_genicam_api/build/tools
        - $ ./gc_config -l
    - to get images from the camera
        - $ cd rc_genicam_api/build/tools
        - $ ./gc_stream 02940273 


