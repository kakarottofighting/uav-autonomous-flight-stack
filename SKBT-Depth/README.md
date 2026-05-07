# SKBT-Depth 开发调试记录

## 单独编译特定功能包

用于先生成自定义消息头文件，防止直接 `catkin_make` 报错。

工作空间根目录下执行，不是在功能包目录下：

```bash
catkin_make --only-pkg-with-deps my_package
```

继续编译整体工作空间：

```bash
rm -rf build
catkin_make
```

## 启用 GDB

`CMakeLists.txt` 添加：

```cmake
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
```

launch 添加：

```xml
launch-prefix="xterm -e gdb -ex run --args"
```

## about usb_cam

launch 文件地址：

```text
/opt/ros/noetic/share/usb_cam/launch/usb_cam-test.launch
```

需要修改其中的相机端口号。

相机端口号查询：

```bash
ls /dev/video*
```

测试：

```bash
roslaunch usb_cam usb_cam-test.launch
```

## Git

创建并切换到一个新的分支：

```bash
git checkout -b feature-branch
```

添加本地修改：

```bash
git add .
git commit -m "备注"
```

添加到远程仓库：

```bash
git push -u origin feature-branch
```

