# Romeo two handed manipulation

This is an augmented reality demonstration developped for the robot Romeo: a two handed-manipulation for holding a tray, in order to solve a ball-in-maze game in augmented reality.

![alt tag](http://img.youtube.com/vi/-wIzJ2Ckifg/maxresdefault.jpg)

The two arms, 14 joints in total, are holding a tray from two handles. A known picture is placed on the tray, it is detected automatically and then tracked using the template tracker in ViSP. Furthermore, this algorithm computes the 6D pose of the picture with respect to the camera. A virtual maze is added in augmented reality on the top of the tray and its pose is directly linked with the pose of the picture. The aim of the game is to roll the virtual ball from its actual position to the end of the maze. The main software used to develop this framework are the Aldebaran SDK C++, ViSP, ViSPNaoqi, Panda3D, Metapod3 and OpenCV.


## How to compile:
You will need:
* [ViSP](https://visp.inria.fr/)
* [visp_naoqi](https://github.com/lagadic/visp_naoqi) compiled with naoqi 2.3
* [romeo_tk](https://github.com/lagadic/romeo_tk)
* [Panda3D](https://www.panda3d.org/)

  ```
  $ qibuild configure --release -c toolchain_romeo_2_3 -Dvisp_naoqi_DIR=/udd/fspindle/soft/romeo/workspace_eutelsat/visp_naoqi/build-toolchain_romeo_2_3-release/sdk/cmake -DVISP_DIR=/local/soft/ViSP/visp-build-release/ -DROMEO_TK_DIR=/udd/fspindle/soft/romeo/workspace_eutelsat/romeo_tk/build-toolchain_romeo_2_3-release/sdk/cmake/
  
  $ cd build_toolchain_romeo_2_3
  $ make
```
