rtabmap
=======

[![RTAB-Map Logo](https://raw.githubusercontent.com/introlab/rtabmap/master/guilib/src/images/RTAB-Map100.png)](http://introlab.github.io/rtabmap)

[![Release][release-image]][releases]
[![Downloads][downloads-image]][downloads]
[![License][license-image]][license]

[release-image]: https://img.shields.io/badge/release-0.21.4-green.svg?style=flat
[releases]: https://github.com/introlab/rtabmap/releases

[downloads-image]: https://img.shields.io/github/downloads/introlab/rtabmap/total?label=downloads
[downloads]: https://github.com/introlab/rtabmap/releases

[license-image]: https://img.shields.io/badge/license-BSD-green.svg?style=flat
[license]: https://github.com/introlab/rtabmap/blob/master/LICENSE

RTAB-Map library and standalone application.

 * For more information (e.g., papers, major updates), visit [RTAB-Map's home page](http://introlab.github.io/rtabmap).
 * For installation instructions and examples, visit [RTAB-Map's wiki](https://github.com/introlab/rtabmap/wiki).

To use RTAB-Map under ROS, visit the [rtabmap](http://wiki.ros.org/rtabmap) page on the ROS wiki.

### Acknowledgements
This project is supported by [IntRoLab - Intelligent / Interactive / Integrated / Interdisciplinary Robot Lab](https://introlab.3it.usherbrooke.ca/), Sherbrooke, Québec, Canada.

<a href="https://introlab.3it.usherbrooke.ca/">
<img src="https://github.com/introlab/16SoundsUSB/blob/master/images/IntRoLab.png" alt="IntRoLab" height="100">
</a>

#### CI Latest

  <table>
    <tbody>
        <tr>
           <td>Linux</td>
           <td><a href="https://github.com/introlab/rtabmap/actions/workflows/cmake.yml"><img src="https://github.com/introlab/rtabmap/actions/workflows/cmake.yml/badge.svg" alt="Build Status"/> <br> <a href="https://github.com/introlab/rtabmap/actions/workflows/cmake-ros.yml"><img src="https://github.com/introlab/rtabmap/actions/workflows/cmake-ros.yml/badge.svg" alt="Build Status"/> <br> <a href="https://github.com/introlab/rtabmap/actions/workflows/docker.yml"><img src="https://github.com/introlab/rtabmap/actions/workflows/docker.yml/badge.svg" alt="Build Status"/>
           </td>
        </tr>
        <tr>
           <td>Windows</td>
           <td><a href="https://ci.appveyor.com/project/matlabbe/rtabmap/branch/master"><img src="https://ci.appveyor.com/api/projects/status/hr73xspix9oqa26h/branch/master?svg=true" alt="Build Status"/>
           </td>
        </tr>
     </tbody>
  </table>
 
 #### ROS Binaries
 
 `ros-$ROS_DISTRO-rtabmap`
 
 <table>
    <tbody>
        <tr>
           <td rowspan="1">ROS 1</td>
            <td>Noetic</td>
            <td><a href="http://build.ros.org/job/Nbin_ufv8_uFv8__rtabmap__ubuntu_focal_arm64__binary/"><img src="http://build.ros.org/buildStatus/icon?job=Nbin_ufv8_uFv8__rtabmap__ubuntu_focal_arm64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td rowspan="3">ROS 2</td>
            <td>Humble</td>
            <td><a href="http://build.ros2.org/job/Hbin_uJ64__rtabmap__ubuntu_jammy_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__rtabmap__ubuntu_jammy_amd64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td>Jazzy</td>
            <td><a href="http://build.ros2.org/job/Jbin_uN64__rtabmap__ubuntu_noble_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Jbin_uN64__rtabmap__ubuntu_noble_amd64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td>Rolling</td>
            <td><a href="http://build.ros2.org/job/Rbin_uJ64__rtabmap__ubuntu_jammy_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Rbin_uJ64__rtabmap__ubuntu_jammy_amd64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
           <td>Docker</td>
           <td>
             <a href="https://hub.docker.com/r/introlab3it/rtabmap">rtabmap</a>
           </td>
           <td><img src="https://img.shields.io/docker/pulls/introlab3it/rtabmap" alt="Docker Pulls"/></td>
        </tr>
    </tbody>
</table>

#### 调试说明

联合ros一起构建：

```bash
# 1 首先清除通过apt安装的rtabmap以及rtabmap-ros
sudo apt autoremove ros-humble-rtabmap-ros

# 2 安装rtabmap源码
cd ~/ros_ws/src
git clone https://github.com/introlab/rtabmap.git --branch humble-devel
git clone https://github.com/introlab/rtabmap_ros.git --branch ros2

# 3 先查看需要安装哪些库
rosdep update
rosdep install --from-paths src --ignore-src -r --simulate

# 4 确认无误后安装这些库
rosdep install --from-paths src --ignore-src -r -y

# 5 开始编译
# 为什么rtabmap不是ROS2包还可以使用colcon build编译？
# colcon是构建工具，重点在于按照依赖关系依次调用构建系统完成一系列功能包的构建。cmake是构建系统，针对一个单独的包进行构建。colcon不是只编译ROS节点，它支持cmake编译方式。
# colcon 会先扫描包，寻找符合条件的包（关键在于有没有package.xml），之后会自动按依赖排序，并依次调用cmake编译某个包。
# 具体拓展请查看 https://fishros.com/d2lros2/#/humble/chapt2/basic/1.使用gcc编译ROS2节点
export MAKEFLAGS="-j6" # 等同于 make -j6
# 5.1 以release编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
# 5.2 以debug编译
# --symlink-install 可以让你修改源码后无需重新构建全部，只需重启调试
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select rtabmap
# rtabmap编译后的产物会放在~/ros_ws/install/rtabmap/[include|lib|share]中

# 6 启动
source ~/ros_ws/install/setup.bash
```

单独构建rtabmap：

```bash
# 1 依赖安装
sudo apt-get update
sudo apt-get install libsqlite3-dev libpcl-dev libopencv-dev git cmake libproj-dev libqt5svg5-dev

# 2 g2o依赖安装
git clone https://github.com/RainerKuemmerle/g2o.git 
cd g2o
mkdir build
cd build
cmake -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_OPENGL=OFF ..
make -j4
sudo make install

# 3 opencv安装
cd opencv
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
sudo make install

# 4 rtabmap编译
git clone --branch humble-devel https://github.com/introlab/rtabmap.git rtabmap
cd rtabmap/build
cmake ..
make -j4
sudo make install
```