#include<aris_core.h>
#include<aris_control.h>
#include<aris_dynamic.h>
#include<aris_sensor.h>
#include<aris_server.h>

/// \mainpage aris简介
///
/// \section introduction_sec 功能简介
/// aris主要包含两个功能：
/// - 机器人建模、仿真、规划、运动学与动力学
///		- 可以让用户针对串联机构、并联机构、混连机构、六轴机械手、SCARA机器人、Sterwart等任意机器人机构建模
///		- 可以自动建立位置正反解、速度正反解等运动学模型
///		- 可以自动建立机器人加速度->力或力->加速度等动力学正逆模型
///		- 运动学位置模型需要迭代计算，不建议在实时循环中直接使用，用户可以自己针对机构进行重载，从而实时使用，
///		- 运动学速度和动力学模型计算时间确定，且效率超高，Sterwart机构仅需0.03ms左右
///		- 可以生成Adams模型等，便于用户比对结果
/// - 机器人控制系统
///		- 支持linux-xenomai实时系统，以及etherlab库，可以作为实时控制器使用
///		- 支持用户自己编写命令控制机器人（比如moveC,moveL等）
///
/// 机器人建模请参考 \ref dynamic_model_page "机器人建模"
/// 
/// \section install_sec 安装
/// aris使用Cmake作为构建工具，可以跨平台使用。源码完全基于标准C++ 14编写，并在以下编译器下进行过测试：
/// - Visual Studio 2015
/// - gcc&g++ 5.4.1
/// - clang&clang++ 3.8
///
/// \subsection  Windows
/// 在Windows下master时钟不实时，同时无法使用EtherCat模块。Windows平台推荐使用cmake-gui等CMake工具来构建。
/// 
/// \subsection  普通Linux
/// 在普通Linux下master时钟不实时也无法使用EtherCat模块。普通Linux平台需要用户预装cmake软件。
/// 假设当前目录在aris的源码目录下，那么以下指令为使用gcc和g++的安装过程：
/// ~~~~~~~~~~~~~~~~~
/// mkdir build
/// cd build
/// cmake .. -DCMAKE_CXX_COMPILER=g++ -DCMAKE_C_COMPILER=gcc
/// make install -j4
/// ~~~~~~~~~~~~~~~~~
///
/// 以下指令为使用clang和clang++的安装过程
/// ~~~~~~~~~~~~~~~~~
/// mkdir build
/// cd build
/// cmake .. -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang
/// make install -j4
/// ~~~~~~~~~~~~~~~~~
/// 
/// \subsection  实时Linux
/// 在实时Linux下，aris可以xenomai内核以及etherlab，这需要在编译时额外使用以下选项：
/// ~~~~~~~~~~~~~~~~~
/// mkdir build
/// cd build
/// cmake .. -DUSE_XENOMAI=ON -DUSE_ETHERLAB=ON
/// make install
/// ~~~~~~~~~~~~~~~~~
///
///

