Introduction
---
svo2_noros is a sample project without ROS to use SVO2.0 binaries(http://rpg.ifi.uzh.ch/svo2.html).

One can also find ros sample code from https://github.com/uzh-rpg/rpg_svo_example.

This project contains mono/stereo demo.

test environment: Ubuntu16.04+opencv3.3.1

NOTE: If you want to run it in Ubuntu14.04+indigo, your should replace the svo binaries in svo2.0.tar.gz. You can download [here](http://rpg.ifi.uzh.ch/svo2.html)

How to build
---
```
git clone https://github.com/symao/svo2_noros.git
cd svo2_noros
tar -xzf svo2.0.tar.gz
mkdir build
cd build
cmake ..
make -j
```

Run demo
---
Modify 'calib_file' in param/euroc_stereo_imu.yaml to your own calib path. Same as mono config.
> calib_file: /home/symao/workspace/svo2_noros/calib/euroc_stereo.yaml
- Run mono:
> ./demo_mono_euroc param_file imgts_file img_dir
- Run stereo:
> ./demo_stereo_euroc param_file imgts_file left_img_dir right_img_dir

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