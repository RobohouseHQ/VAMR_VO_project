
# Monocular visual-odometry (VO) project

##### Table of Contents
1. [Introduction](#intro)
2. [Running the pipeline](#runpipeline)
3. [Features](#features)
4. [Sample results](#results)
5. [References](#references)
1. [Contributors](#contributors)
1. [License](#license)

<a name="intro"></a>
## Introduction
This repository contains a MATLAB implementation of a Monocular (single camera) Visual Odometry pipeline developed as part of the  (Vision Algorithms for Mobile Robotics)[http://rpg.ifi.uzh.ch/teaching.html] class at ETH Zürich. 

<a name="runpipeline"></a>
## Running the pipeline
### Requirements
The code has been developed and tested using MATLAB version 2020b and 2021b. In addition, the following toolboxes are needed:
* image_toolbox
* optimization_toolbox
* statistics_toolbox
* video_and_image_blockset

The screencasts that showcase some sample results [below](#results) have been recorded on a laptop with the following specs:
* Ubuntu 20.04
* Cores: 12
* CPU frequency: 2.6 GHz
* RAM: 16 GB

### Instructions
1. Clone this repository somewhere on your machine.
2. Download at least one of the datasets (KITTI, Malaga, Parking) from [here](http://rpg.ifi.uzh.ch/teaching.html) and place it inside a folder data which you should create at the root of this repository.
3. Open MATLAB and open `main.m`, make sure that the parameters at the top are correct:
```
ds = 0; % 0: KITTI, 1: Malaga, 2: parking
parking_path = 'data/parking';
kitti_path = 'data/kitti';
malaga_path = 'data/malaga-urban-dataset-extract-07';
```
4. Once you are ready, run `main.m`


<!-- <a name="references"><a/>
## References
<a id="1">[1]</a> L. Kneip, D. Scaramuzza, and R. Siegwart. A novel parametrization ofthe perspective-three-point problem for a direct computation of absolutecamera position and orientation.  In CVPR 2011, pages 2969–2976, June 2011.

<a id="2">[2]</a> Bruce D. Lucas and Takeo Kanade. An iterative image registration technique with an application to stereo vision. In Proceedings of the 7th International Joint Conference on Artificial Intelligence - Volume2, IJCAI’81, page 674–679, San Francisco, CA, USA, 1981. Morgan Kaufmann Publishers Inc.

<a id="3">[3]</a> Simon Baker, Ralph Gross, and Iain Matthews. Lucas-kanade 20 years on: A unifying framework: Part 3. Int. J. Comput. Vis, 56, 12 2003.

<a id="4">[4]</a> R. I. Hartley. In defense of the eight-point algorithm. IEEE Transactionson Pattern Analysis and Machine Intelligence, 19(6):580–593, June 1997.

<a id="5">[5]</a> Martin A. Fischler and Robert C. Bolles. Random sample consensus: A paradigm for model fitting with applications to image analysis and automated cartography. Commun. ACM, 24(6):381–395, June 1981

<a id="6">[6]</a> D. Nister. An efficient solution to the five-point relative pose problem. IEEE Transactions on Pattern Analysis and Machine Intelligence, 26(6):756–770, June 2004

<a id="7">[7]</a> Heng Yang, Pasquale Antonante, Vasileios Tzoumas, and Luca Carlone. Graduated non-convexity for robust spatial perception: From non-minimal solvers to global outlier rejection. IEEE Robotics and Automation Letters, 5(2):1127–1134, Apr 2020.

<a id="8">[8]</a> Y.I Abdel-Aziz, H.M. Karara, and Michael Hauck. Direct linear transformation from comparator coordinates into object space coordinates in close-range photogrammetry. Photogrammetric Engineering Remote Sensing, 81(2):103–107, 2015.

<a id="9">[9]</a> Zichao Zhang and Davide Scaramuzza. A tutorial on quantitativetrajectory evaluation for visual(-inertial) odometry. In IEEE/RSJ Int. Conf. Intell. Robot. Syst. (IROS), 2018. -->


<a name="contributors"><a/>
### Contributors
#### Authors
+ Patricia Apostol
+ Rayan Armani
+ David Oort Alonso
+ Max Martinez Ruts

<a name="license"><a/>
## License
This project is licensed under the MIT License - see the [LICENSE](https://github.com/antonioterpin/visual-odometry-mono/blob/main/LICENSE) file for details.
