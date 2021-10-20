# DSOR filter #

The Dynamic Statistical Outlier Removal (DSOR) filter can de-noise LiDAR point clouds corrupted by falling snow. It can also be used as a general purpose outlier removal filter and is fully ROS compatible.

#### This is a header only file and you can just include the dsor.hpp file in your project to use it ####

<img src="snow1.gif" width="400" height="200"/> ➡ <img src="snow2.gif" width="400" height="200"/>


### Build example node ###
```
$ cd <catkin_ws>/src
$ git clone git@bitbucket.org:autonomymtu/dsor_filter.git
$ catkin build dsor_filter
```

### Launch example node ###
```
$ roslaunch dsor_filter example.launch
```

### Publication ###
Please consider citing our work if you find it useful!
```
@misc{kurup2021dsor,
      title={DSOR: A Scalable Statistical Filter for Removing Falling Snow from LiDAR Point Clouds in Severe Winter Weather}, 
      author={Akhil Kurup and Jeremy Bos},
      year={2021},
      eprint={2109.07078},
      archivePrefix={arXiv}
}
```

### Maintainer ###

* Akhil Kurup <amkurup@mtu.edu>


### The WADS dataset ###

Details about WADS can be found here: [WADS](https://bitbucket.org/autonomymtu/wads)
