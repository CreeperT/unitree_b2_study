# websocket debug
## 问题1：显示错误：Node '/ws_msgs_manage_node' has already been added to an executor
**原因：** 使用了函数spin_until_future_complete，造成执行器冲突。  
**解决：** 修改回调函数，在客户端的异步请求里采用一个lambda函数完成回调；超时等待改用wait_for替代spin_until_future_complete。


## 问题2：发生了段错误
**原因：** 通过查看堆栈，发现该段错误是在执行DeviceCtrlCmdSendback函数时，使用cJSON_Duplicate函数尝试复制一个cJSON对象时发生的，这意味着程序正在试图操作一个无效、已损坏或已被释放的cJSON对象。  
**解决：** 通过添加错误信息输出发现，程序在进行到ros2客户端回调的lambda函数时马上发生段错误，可能是lambda函数捕获的cJSON对象出错，于是在这之前先将要捕获的cJSON对象复制了一份，让lambda函数捕获复制的cJSON对象，成功解决。

## 问题3：无论通信是否成功，均会触发错误处理函数DeviceInternalCommunicationErrorProcess
**原因：** wait_for函数阻塞当前线程，与客户端的异步处理逻辑有冲突，且判断条件为
```cpp
wait_for(std::chrono::seconds(1)) != std::future_status::ready
```
可能因为其他原因误判  
**解决：** 使用了一个定时器来处理通信超时等待错误，同时将判断条件改为
```cpp
wait_for(std::chrono::seconds(1)) == std::future_status::timeout
```
为了使用`wait_for`函数，首先要创建一个future共享指针加入lambda回调函数，然后在外面重载
```cpp
auto shared_future_ptr = std::make_shared<rclcpp::Client<robot_msgs::srv::ChangeCtrlModeCmd>::SharedFuture>();
// 在lambda回调函数中
*shared_future_ptr = future;
// lambda回调函数结束后重载
auto future = ChangeCtrlModeCmd_client->async_send_request(request, res_callback);
*shared_future_ptr = future.future;
// 定时器调用
auto timer = this->create_wall_timer(std::chrono::milliseconds(500),
            [this, json_fun_copy, shared_future_ptr](){
                if (shared_future_ptr->valid() && 
                    shared_future_ptr->wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                {
                    DeviceInternalCommunicationErrorProcess(json_fun_copy);
                    return;
                }
            }); 
```

## 问题4：有很多cJSON对象没有判断是否为空

**解决：** 添加了判断cJSON对象是否为空的处理语句


# dddmr_lego_loam debug
## 问题1：~~lego_loam_fa节点显示ERROR: Could not get robot_center to , check your odom topic and baselink_frame~~ pcl::Voxel滤波器问题和Eigen对象内存未对齐问题
**原因：**  
通过在`featureAssociation.cpp`中添加调试信息，发现在`runFeatureAssociation`函数执行`_input_channel.receive(projection)`语句时发生了堵塞。  
再细查，发现`imageProjection`中的`_output_channel.send( std::move(out) )`函数也没有正确执行。  
继续调试，发现堵塞发生在`imageProjection`的`groundRemoval`函数的`dsf_patched_ground_.filter(*patched_ground_)`语句。退出码为`-11`，是**段错误**。  
~~参考[一篇博客](https://blog.csdn.net/weixin_42325783/article/details/134369931)，分析可能是pcl库和eigen库共同引起的bug。~~  
`FeatureAssociation::extractFeatures`函数中，Voxel滤波器downSizeFilter对地面点云滤波后得到空点云。  
`pcl::transformPointsCloud`函数引发的段错误，通过gdb查看堆栈发现错误发生在`_mm_load_pd_`指令处。参考[一篇知乎文章](https://zhuanlan.zhihu.com/p/510724305)可以得知这是内存未对齐引发的。

**解决：**  
~~修改了滤波函数，在滤波部分跑通了。但仍然发生段错误。~~   
将Voxel滤波器换成了ApproximateVoxel滤波器。不需要再根据那篇博客的方法了。  
修改了`mapOptiminaztion.h`的代码。
```cpp
// 修改前
Eigen::Affine3d trans_m2ci_af3_, trans_c2s_af3_, trans_s2c_af3_, trans_c2b_af3_;

// 修改后
EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 添加这个宏
alignas(32) Eigen::Affine3d trans_m2ci_af3_, trans_c2s_af3_, trans_s2c_af3_, trans_c2b_af3_; // 确保Eigen对象以32位正确对齐
```
顺利解决，可以运行了。
## 问题2：pose_graph_editor节点运行出错
**原因：**  
通过gdb工具发现`pose_graph_editor`模块在运行`pcl::io::loadPCDFile`函数时发生段错误。  
**解决：**  
使用更安全的加载方式，即先以通用格式读取，再转换到具体点类型。  
具体如下：  
```cpp
// 新建一个中间点云
pcl::PCLPointCloud2 cloud2;
// 以通用格式读取
if (pcl::io::loadPCDFile(edges_path, cloud2) == -1)
{
    RCLCPP_ERROR(this->get_logger(), "Failed to read poses PCD file: %s", poses_path.c_str());
}
// 再通过中间点云转换到目标点云
pcd_edges_.reset(new pcl::PointCloud<PointXYZ>());
pcl::fromPCLPointCloud2(cloud2, *pcd_edges_);
// 对之后的读取部分同理替换
```

# hdl_localization_ros2
## 问题1：报错terminate called after throwing an instance of 'std::runtime_error' what():  can't compare times with different time sources
参考该[issue](https://github.com/pyc5714/hdl-localization-ROS2/issues/4)，在`pose_estimator.hpp`头文件中进行修改


## 问题2：找不到odom到base_link的tf转换
**原因：**   
1. 机器狗自己并不发布tf变换，需要手动发布。  
2. 发布的odom到base_link的tf变换时间戳不同步，比记录时间晚了89秒。  

**解决：**  
1. 通过获取`/dog_odom`话题的消息，手动发布了odom到base_link的tf变换。
2. 原来发布的odom到base_link的tf变换中，时间戳使用了`/dog_odom`话题的时间戳。实际上，`/dog_odom`话题由机器狗的另一台底层控制电脑发布，其时间戳与开发电脑不一致，晚了89秒。于是将该tf变换中的时间戳改为使用系统时间`this->now()`，成功解决。

## 问题3：map到odom的tf变换有问题
