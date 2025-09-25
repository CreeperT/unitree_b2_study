# websocket debug
## 问题1：显示错误：Node '/ws_msgs_manage_node' has already been added to an executor
**原因：** 使用了函数spin_until_future_complete，造成执行器冲突。  
**解决：** 修改回调函数，在客户端的异步请求里采用一个lambda函数完成回调；超时等待改用wait_for替代spin_until_future_complete。


## 问题2：发生了段错误
**原因：** 通过查看堆栈，发现该段错误是在执行DeviceCtrlCmdSendback函数时，使用cJSON_Duplicate函数尝试复制一个cJSON对象时发生的，这意味着程序正在试图操作一个无效、已损坏或已被释放的cJSON对象。  
**解决：** 通过添加错误信息输出发现，程序在进行到ros2客户端回调的lambda函数时马上发生段错误，可能是lambda函数捕获的cJSON对象出错，于是在这之前先将要捕获的cJSON对象复制了一份，让lambda函数捕获复制的cJSON对象，成功解决。

## 问题3：无论通信是否成功，均会触发错误处理函数DeviceInternalCommunicationErrorProcess
**原因：** wait_for函数阻塞当前线程，与客户端的异步处理逻辑有冲突，且判断条件为
>`wait_for(std::chrono::seconds(1)) != std::future_status::ready`

可能因为其他原因误判  
**解决：** 使用了一个定时器来处理通信超时等待错误，同时将判断条件改为
>`wait_for(std::chrono::seconds(1)) == std::future_status::timeout`

## 问题4：有很多cJSON对象没有判断是否为空

**解决：** 添加了判断cJSON对象是否为空的处理语句


# dddmr_lego_loam debug
## 问题1：lego_loam_fa节点显示ERROR: Could not get robot_center to , check your odom topic and baselink_frame
**原因：**  
通过在`featureAssociation.cpp`中添加调试信息，发现在`runFeatureAssociation`函数执行`_input_channel.receive(projection)`语句时发生了堵塞。  
再细查，发现`imageProjection`中的`_output_channel.send( std::move(out) )`函数也没有正确执行。  
继续调试，发现堵塞发生在`imageProjection`的`groundRemoval`函数的`dsf_patched_ground_.filter(*patched_ground_)`语句。退出码为`-11`，是**段错误**。  
根据[一篇博客](https://blog.csdn.net/weixin_42325783/article/details/134369931)，分析可能是pcl库和eigen库共同引起的bug。

**解决：**  
修改了滤波函数，在滤波部分跑通了。但仍然发生段错误。