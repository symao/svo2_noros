Introduction
---
test_svo2 is a sample project to use SVO2.0(http://rpg.ifi.uzh.ch/svo2.html).

One can also find ros sample code from https://github.com/uzh-rpg/rpg_svo_example.

This project contains mono/stereo demo with/without ros.

test environment: Ubuntu16.04+ros kinetic

NOTE: If you want to run it in Ubuntu14.04+indigo, your should replace the svo binaries in svo2.0.tar.gz. You can download [here](http://rpg.ifi.uzh.ch/svo2.html)

How to build
---
```
cd <your_catkin_workspace>/src
git clone https://github.com/symao/test_svo2.git
cd test_svo2
tar -xzf svo2.0.tar.gz
cd ../..
catkin_make
```

Run demo
---
1. run with euroc data with ROS
- Download rosbag data from [here](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- modify launch/euroc_stereo.launch, change the *.bag file path to your own path
- run: roslaunch test_svo2 euroc_stereo.launch

2. run with svo sample data with ROS
- Download the test bag file from [here](http://rpg.ifi.uzh.ch/datasets/airground_rig_s3_2013-03-18_21-38-48.bag)
- modify launch/svo_pinhole.launch, change the *.bag file path to your own path
- run: roslaunch test_svo2 svo_pinhole.launch

3. run with stereo video file
- run: roslaunch test_svo2 mynteye.launch

you can modify to run your own video files, do not forget to use your own calib params.


Q&A
---
1. libopencv*3.so.3.2 not found
Ros kinetic use opencv 3.3.1, while svo2.0 need opencv 3.2, create softlink by 'ln -s libopencv*3.so.3.3.1 libopen*3.so.3.2', this is not a good solution, but it works.

2. when running svo stereo demo, print error 'seed sigma is nan!', and warning 'critical', such as below
```
[ INFO] [1514192558.630363968]: Stereo: Triangulated 100 features,170 failed.
[ INFO] [1514192558.632994742]: Init: Selected first frame.
[ERROR] [1514192559.198140335]: seed sigma is nan!-0.316199, sq-nan, check-convergence = 0
[ERROR] [1514192559.198215167]: seed sigma is nan!-0.231193, sq-nan, check-convergence = 0
[ERROR] [1514192559.198291805]: seed sigma is nan!-0.16796, sq-nan, check-convergence = 0
[ERROR] [1514192559.198720866]: seed sigma is nan!-0.316199, sq-nan, check-convergence = 0
[ERROR] [1514192559.198759543]: seed sigma is nan!-0.231193, sq-nan, check-convergence = 0
[ERROR] [1514192559.202572936]: seed sigma is nan!-0.316199, sq-nan, check-convergence = 0
[ERROR] [1514192559.202618447]: seed sigma is nan!-0.231193, sq-nan, check-convergence = 0
[ERROR] [1514192559.206280775]: seed sigma is nan!-0.316199, sq-nan, check-convergence = 0
[ERROR] [1514192559.206331176]: seed sigma is nan!-0.231193, sq-nan, check-convergence = 0
[ WARN] [1514192559.293642765]: critical
```

I haven't solve it. [here](https://github.com/uzh-rpg/rpg_svo_example/issues/5) are the author's answers. It means something wrong with the depth filter. However, even if i get this error, the track trajectory seems good for me sometimes. May God bless you.

If anyone know how to solve this, please contact to me(maoshuyuan123@gmail.com), thank you so much.