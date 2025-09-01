## 环境

1. #### 安装 ROS

   安装所需类型的 ROS:ROS 多个类型:**Desktop-Full**、**Desktop**、**ROS-Base**。这里介绍较为常用的 Desktop-Full(官方推荐)安装: ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception

   ```bash
   sudo apt install ros-noetic-desktop-full
   ```

2. #### 安装构建依赖

   ```bash
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

   > | 包名                           | 作用                                                                     |
   > | ------------------------------ | ------------------------------------------------------------------------ |
   > | `python3-rosdep`               | 安装并管理 ROS 包的系统依赖（就是我们刚才说的 `rosdep` 工具本体）。      |
   > | `python3-rosinstall`           | 用于从 `.rosinstall` 文件导入多个 ROS 包仓库（老工具，现在用得少了）。   |
   > | `python3-rosinstall-generator` | 根据 ROS 发行版生成 `.rosinstall` 文件（配合 `wstool` 使用）。           |
   > | `python3-wstool`               | 管理 ROS 工作空间中多个 Git/SVN 仓库的工具（类似 `vcstool`，但老一点）。 |
   > | `build-essential`              | Ubuntu 下的基础构建工具包，包含 `gcc`、`g++`、`make` 等编译工具。        |
   >
   > `rosdep` 是 ROS（Robot Operating System）中的一个命令行工具，它的主要作用是 **帮助用户自动安装 ROS 包所需的系统依赖项**。
   >
   > ###### 📌 一句话总结：
   >
   > > `rosdep` 就是 ROS 用来“一键安装依赖”的工具。
   >
   > ***
   >
   > ###### 📘 举个例子：
   >
   > 假设你下载了一个 ROS 包，它依赖于某些系统库（比如 OpenCV、Boost 等），但你电脑上没装这些库。这个时候你就可以用：
   >
   > ```bash
   > rosdep install --from-paths src --ignore-src -r -y
   > ```
   >
   > 这条命令会自动检查 `src` 目录下所有 ROS 包的依赖，然后帮你用 `apt` 或其他包管理器安装缺失的系统依赖。
   >
   > ***
   >
   > ###### 🧩 核心功能：
   >
   > - 检查并安装 ROS 包的 **系统依赖**（注意：不是 ROS 包本身，而是它们依赖的底层库）。
   > - 支持多平台（Ubuntu、Debian、macOS 等）。
   > - 依赖信息来自 `package.xml` 中的 `<depend>` 标签。
   >
   > ***
   >
   > ###### 🧪 常见命令：
   >
   > | 命令                            | 说明               |
   > | ------------------------------- | ------------------ |
   > | `rosdep update`                 | 更新依赖数据库     |
   > | `rosdep install <package_name>` | 安装指定包的依赖   |
   > | `rosdep check <package_name>`   | 检查依赖是否已满足 |
   >
   > ***
   >
   > ###### ✅ 使用前提：
   >
   > - 安装好 ROS 后，第一次使用前需要运行：
   >   ```bash
   >   sudo rosdep init
   >   rosdep update
   >   ```

## ROS 程序（C++版本）

实现流程大致如下：

1. 先创建一个工作空间；

   > ```bash
   > mkdir -p 自定义空间名称/src
   > cd 自定义空间名称
   > catkin_make
   > ```
   >
   > 上述命令，首先会创建一个工作空间以及一个 src 子目录，然后再进入工作空间调用 catkin_make 命令编译。
   >
   > > `catkin_make` 就是 ROS 1 的 “一键编译” 命令，相当于把 `cmake` + `make` 包装成了傻瓜式操作,识别`Cmakelist.txt`文件。
   > >
   > > `catkin_make` 是 ROS 1 的工具；ROS 2 用的是 `colcon build`。

2. 再创建一个 ros 功能包；

   > ```bash
   > cd src
   > catkin_create_pkg 自定义ROS包名 roscpp rospy std_msgs
   > ```
   >
   > 上述命令，会在工作空间下生成一个功能包，该功能包依赖于 roscpp、rospy 与 std_msgs，其中 roscpp 是使用 C++实现的库，而 rospy 则是使用 python 实现的库，std_msgs 是标准消息库，创建 ROS 功能包时，一般都会依赖这三个库实现。

3. 编辑源文件；

   > ###### 进入 ros 包的 src 目录编辑源文件
   >
   > ```bash
   > cd 自定义的包
   > ```
   >
   > C++源码实现(文件名自定义)
   >
   > ```c++
   > #include "ros/ros.h"
   >
   > int main(int argc, char *argv[])
   > {
   >     //执行 ros 节点初始化
   >     ros::init(argc,argv,"hello");
   >     //创建 ros 节点句柄(非必须)
   >     ros::NodeHandle n;
   >     //控制台输出 hello world
   >     ROS_INFO("hello world!");
   >
   >     return 0;
   > }
   > ```

4. 编辑配置文件；

   > 编辑 ros 包下的 Cmakelist.txt 文件
   >
   > ```Cmakelist.txt
   > add_executable(步骤3的源文件名
   >   src/步骤3的源文件名.cpp
   > )
   > target_link_libraries(步骤3的源文件名
   >   ${catkin_LIBRARIES}
   > )
   > ```

5. 编译并执行。

   > ```bash
   > cd 自定义空间名称
   > # 编译
   > catkin_make
   > # 执行
   > roscore
   > cd 工作空间
   > source ./devel/setup.bash
   > rosrun 包名 C++节点
   > ```

##### Python

[1.3.3 HelloWorld 实现 B(Python) · Autolabor-ROS 机器人入门课程《ROS 理论与实践》零基础教程](http://www.autolabor.com.cn/book/ROSTutorials/chapter1/13-rosji-cheng-kai-fa-huan-jing-da-jian/132-helloworldpython724829.html)

## ROS 架构

[1.5 ROS 架构 · Autolabor-ROS 机器人入门课程《ROS 理论与实践》零基础教程](http://www.autolabor.com.cn/book/ROSTutorials/chapter1/15-ben-zhang-xiao-jie.html)

## ROS 通信

### 话题通信---发布订阅模式

#### 涉及到三个角色:

- ROS Master (管理者)
- Talker (发布者)
- Listener (订阅者)

在模型实现中，ROS master 不需要实现，而连接建立也已经被封装了，需要关注的关键点有三个:

1. 发布方

2. 接收方

3. 数据(此处为普通文本)

4. 句柄解释

   > 先把结论放在最前面：  
   > **ros::NodeHandle（简称 nh）是节点与整个 ROS 运行系统之间的「会话句柄」，没有它就无法在 ROS 里做“注册话题 / 服务 / 参数 / 时钟”等任何实质操作。底层它持有一个** **`ros::NodeHandle::Impl`** **对象，该对象里保存了**
   >
   > 1. **与 master 的连接（XML-RPC）**
   > 2. **本节点的订阅/发布列表、TCPROS/UDPROS 连接**
   > 3. **回调队列、线程池、参数缓存**
   >
   > 换句话说，**NodeHandle 只是把 roscpp 庞大的 C++ 内部实现包装成了一个面向用户的“把手”**；你握住这个把手，才能拧动 ROS 的齿轮。
   >
   > ***
   >
   > ###### 1. 为什么必须“先 init，再 NodeHandle”？
   >
   > - `ros::init(...)`  
   >   只做三件事：解析命令行、设置节点名、在进程内**创建**一个**全局静态**的 `ros::NodeHandle::Impl` 并把它**注册为单例**。  
   >   此时**还没有**真正连到 master。
   >
   > - `ros::NodeHandle nh;`  
   >   在构造函数里把上面那个**单例 Impl 指针**拷进来，同时给 Impl 里的计数器 +1。  
   >   首次构造时，会触发 `start()`：
   >   - 建立 XML-RPC 连接到 rosmaster
   >   - 启动内部线程：网络收发线程、回调线程、时钟线程等  
   >     因而只有**第一次** NodeHandle 构造才真正“点火”，后面的 NodeHandle 只是共享同一份资源。
   >
   > ***
   >
   > ###### 2. NodeHandle 能做什么？（用户视角）
   >
   > | 成员函数                | 实际动作（由 Impl 完成）                                                                              |
   > | ----------------------- | ----------------------------------------------------------------------------------------------------- |
   > | `nh.subscribe(...)`     | 把话题名、类型、回调封装成 `Subscription` 对象，通过 XML-RPC 向 master 注册订阅，再在 TCPROS 端口监听 |
   > | `nh.advertise(...)`     | 创建 `Publisher` 对象，向 master 注册发布者，等待订阅者反向连接                                       |
   > | `nh.setParam(...)`      | XML-RPC 调用 master，把键值写进参数服务器                                                             |
   > | `nh.serviceClient(...)` | 建立到目标服务的 TCPROS 连接                                                                          |
   > | `nh.ok()`               | 检查全局标志位 `g_shutting_down`                                                                      |
   >
   > 所有这些“动作”都通过 **`Impl`** 里的 XML-RPC 客户端或 TCPROS 连接对象完成，NodeHandle 只是转发。
   >
   > ***
   >
   > ###### 3. 底层数据结构长什么样？（简化版）
   >
   > ```cpp
   > namespace ros
   > {
   > class NodeHandle
   > {
   > public:
   >     NodeHandle(const std::string& ns = std::string(), const M_string& remappings = M_string());
   >     Subscriber subscribe(...);
   >     Publisher  advertise(...);
   >     ...
   > private:
   >     boost::shared_ptr<NodeHandle::Impl> impl_;  // 真正干活的
   > };
   >
   > class NodeHandle::Impl
   > {
   > public:
   >     std::string              name_;           // 节点名
   >     std::string              namespace_;      // 私有命名空间
   >     PollSetPtr               poll_set_;       // epoll/select 网络复用
   >     XMLRPCManagerPtr         xmlrpc_manager_; // 与 master 通信
   >     ConnectionManagerPtr     connection_manager_; // TCPROS/UDPROS 连接池
   >     CallbackQueuePtr         callback_queue_; // spin() 时用的回调队列
   >     ...
   > };
   > }
   > ```
   >
   > - 整个进程里只有**一个** `Impl` 实例，所有 NodeHandle 共享。
   > - 当最后一个 NodeHandle 析构时，`Impl` 的引用计数归零，`Impl` 的析构会关闭所有 socket、线程，完成优雅退出。
   >
   > ***
   >
   > ###### 4. 小结（背下来即可）
   >
   > | 问题                | 答案                                                                                      |
   > | ------------------- | ----------------------------------------------------------------------------------------- |
   > | NodeHandle 是什么？ | 节点与 ROS 之间的会话句柄，封装了网络连接、注册表、回调队列等。                           |
   > | 为什么必须有它？    | 所有话题/服务/参数/时钟的 API 都以它作为入口；没有它就无法告诉 ROS “我是谁、我要干什么”。 |
   > | 底层实现？          | 一个共享的 `Impl` 单例，内部持 XML-RPC 客户端、TCPROS 连接池、线程池、回调队列。          |
   >
   > 一句话：  
   > **NodeHandle 是 ROS 给你的一把“钥匙”，拿着它才能开门进 ROS 的世界。**

#### **流程:**

1. 编写发布方实现;

   > ```c++
   > /*
   >     需求: 实现基本的话题通信，一方发布数据，一方接收数据，
   >          实现的关键点:
   >          1.发送方
   >          2.接收方
   >          3.数据(此处为普通文本)
   >
   >          PS: 二者需要设置相同的话题
   >
   >
   >     消息发布方:
   >         循环发布信息:HelloWorld 后缀数字编号
   >
   >     实现流程:
   >         1.包含头文件
   >         2.初始化 ROS 节点:命名(唯一)
   >         3.实例化 ROS 句柄
   >         4.实例化 发布者 对象
   >         5.组织被发布的数据，并编写逻辑发布数据
   >
   > */
   > // 1.包含头文件
   > #include "ros/ros.h"
   > #include "std_msgs/String.h" //普通文本类型的消息
   > #include <sstream>
   >
   > int main(int argc, char  *argv[])
   > {
   >     //设置编码
   >     setlocale(LC_ALL,"");
   >
   >     //2.初始化 ROS 节点:命名(唯一)
   >     // 参数1和参数2 后期为节点传值会使用
   >     // 参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一
   >     ros::init(argc,argv,"talker");
   >     //3.实例化 ROS 句柄
   >     ros::NodeHandle nh;//该类封装了 ROS 中的一些常用功能
   >
   >     //4.实例化 发布者 对象
   >     //泛型: 发布的消息类型
   >     //参数1: 要发布到的话题
   >     //参数2: 队列中最大保存的消息数，超出此阀值时，先进的先销毁(时间早的先销毁)
   >     ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",10);
   >
   >     //5.组织被发布的数据，并编写逻辑发布数据
   >     //数据(动态组织)
   >     std_msgs::String msg;
   >     // msg.data = "你好啊！！！";
   >     std::string msg_front = "Hello 你好！"; //消息前缀
   >     int count = 0; //消息计数器
   >
   >     //逻辑(一秒10次)
   >     ros::Rate r(1);
   >
   >     //节点不死
   >     while (ros::ok())
   >     {
   >         //使用 stringstream 拼接字符串与编号
   >         std::stringstream ss;
   >         ss << msg_front << count;
   >         msg.data = ss.str();
   >         //发布消息
   >         pub.publish(msg);
   >         //加入调试，打印发送的消息
   >         ROS_INFO("发送的消息:%s",msg.data.c_str());
   >
   >         //根据前面制定的发送贫频率自动休眠 休眠时间 = 1/频率；
   >         r.sleep();
   >         count++;//循环结束前，让 count 自增
   >         //暂无应用
   >         ros::spinOnce();
   >     }
   >
   >
   >     return 0;
   > }
   > ```

2. 编写订阅方实现；

   > ```c++
   > /*
   >     需求: 实现基本的话题通信，一方发布数据，一方接收数据，
   >          实现的关键点:
   >          1.发送方
   >          2.接收方
   >          3.数据(此处为普通文本)
   >
   >
   >     消息订阅方:
   >         订阅话题并打印接收到的消息
   >
   >     实现流程:
   >         1.包含头文件
   >         2.初始化 ROS 节点:命名(唯一)
   >         3.实例化 ROS 句柄
   >         4.实例化 订阅者 对象
   >         5.处理订阅的消息(回调函数)
   >         6.设置循环调用回调函数
   >
   > */
   > // 1.包含头文件
   > #include "ros/ros.h"
   > #include "std_msgs/String.h"
   >
   > void doMsg(const std_msgs::String::ConstPtr& msg_p){
   >     ROS_INFO("我听见:%s",msg_p->data.c_str());
   >     // ROS_INFO("我听见:%s",(*msg_p).data.c_str());
   > }
   > int main(int argc, char  *argv[])
   > {
   >     setlocale(LC_ALL,"");
   >     //2.初始化 ROS 节点:命名(唯一)
   >     ros::init(argc,argv,"listener");
   >     //3.实例化 ROS 句柄
   >     ros::NodeHandle nh;
   >
   >     //4.实例化 订阅者 对象
   >     ros::Subscriber sub = nh.subscribe<std_msgs::String>("chatter",10,doMsg);
   >     //5.处理订阅的消息(回调函数)
   >
   >     //     6.设置循环调用回调函数
   >     ros::spin();//循环读取接收的数据，并调用回调函数处理
   >
   >     return 0;
   > }
   > ```

3. 编辑配置文件；

   ```Cmakelist.txt
   add_executable(Hello_pub
     src/Hello_pub.cpp
   )
   add_executable(Hello_sub
     src/Hello_sub.cpp
   )

   target_link_libraries(Hello_pub
     ${catkin_LIBRARIES}
   )
   target_link_libraries(Hello_sub
     ${catkin_LIBRARIES}
   )
   ```

4. 编译并执行。

   1.启动 roscore;

   2.启动发布节点;

   3.启动订阅节点。

   运行结果与引言部分的演示案例 1 类似。

5. Python:[2.1.3 话题通信基本操作 B(Python) · Autolabor-ROS 机器人入门课程《ROS 理论与实践》零基础教程](http://www.autolabor.com.cn/book/ROSTutorials/di-2-zhang-ros-jia-gou-she-ji/22hua-ti-tong-xin/213-hua-ti-tong-xin-zhi-python-shi-xian.html)

#### 话题通信自定义 msg + 话题通信自定义 msg 调用(C++)

在 ROS 通信协议中，数据载体是一个较为重要组成部分，ROS 中通过`std_msgs`封装了一些原生的数据类型,比如:String、Int32、Int64、Char、Bool、Empty.... 但是，这些数据一般只包含一个 data 字段，结构的单一意味着功能上的局限性，当传输一些复杂的数据，比如: 激光雷达的信息... std_msgs 由于描述性较差而显得力不从心，这种场景下可以使用自定义的消息类型

msgs 只是简单的文本文件，每行具有字段类型和字段名称，可以使用的字段类型有：

- int8, int16, int32, int64 (或者无符号类型: uint\*)
- float32, float64
- string
- time, duration
- other msg files
- variable-length array[] and fixed-length array[C]

ROS 中还有一种特殊类型：`Header`，标头包含时间戳和 ROS 中常用的坐标帧信息。会经常看到 msg 文件的第一行具有`Header标头`。

**需求:**创建自定义消息，该消息包含人的信息:姓名、身高、年龄等。

**流程:**

1. 按照固定格式创建`msg`文件
2. 编辑配置文件
3. 编译生成可以被`Python`或`C++`调用的中间文件

### 服务通信

服务通信也是ROS中一种极其常用的通信模式，服务通信是基于**请求响应**模式的，是一种应答机制。也即: 一个节点A向另一个节点B发送请求，B接收处理请求并产生响应结果返回给A。比如如下场景:

> 机器人巡逻过程中，控制系统分析传感器数据发现可疑物体或人... 此时需要拍摄照片并留存。

在上述场景中，就使用到了服务通信。

- 一个节点需要向相机节点发送拍照请求，相机节点处理请求，并返回处理结果

与上述应用类似的，服务通信更适用于对时时性有要求、具有一定逻辑处理的应用场景。



服务通信较之于话题通信更简单些，理论模型如下图所示，该模型中涉及到三个角色:

- ROS master(管理者)
- Server(服务端)
- Client(客户端)

ROS Master 负责保管 Server 和 Client 注册的信息，并匹配话题相同的 Server 与 Client ，帮助 Server 与 Client 建立连接，连接建立后，Client 发送请求信息，Server 返回响应信息。

#### 整个流程由以下步骤实现:

##### 0.Server注册

Server 启动后，会通过RPC在 ROS Master 中注册自身信息，其中包含提供的服务的名称。ROS Master 会将节点的注册信息加入到注册表中。

##### 1.Client注册

Client 启动后，也会通过RPC在 ROS Master 中注册自身信息，包含需要请求的服务的名称。ROS Master 会将节点的注册信息加入到注册表中。

##### 2.ROS Master实现信息匹配

ROS Master 会根据注册表中的信息匹配Server和 Client，并通过 RPC 向 Client 发送 Server 的 **TCP** 地址信息。

##### 3.Client发送请求

Client 根据步骤2 响应的信息，使用 TCP 与 Server 建立网络连接，并发送请求数据。

##### 4.Server发送响应

Server 接收、解析请求的数据，并产生响应结果返回给 Client。

> 注意:
>
> 1.客户端请求被处理时，需要保证服务器已经启动；
>
> 2.服务端和客户端都可以存在多个。

#### 服务通信自定义srv + 服务通信自定义srv调用(C++)

##### **需求:**

> 服务通信中，客户端提交两个整数至服务端，服务端求和并响应结果到客户端，请创建服务器与客户端通信的数据载体。

##### **流程:**

srv 文件内的可用数据类型与 msg 文件一致，且定义 srv 实现流程与自定义 msg 实现流程类似:

1. 按照固定格式创建srv文件
2. 编辑配置文件
3. 编译生成中间文件



### 参数服务器

参数服务器在ROS中主要用于实现不同节点之间的数据共享。参数服务器相当于是独立于所有节点的一个公共容器，可以将数据存储在该容器中，被不同的节点调用，当然不同的节点也可以往其中存储数据，关于参数服务器的典型应用场景如下:

> 导航实现时，会进行路径规划，比如: 全局路径规划，设计一个从出发点到目标点的大致路径。本地路径规划，会根据当前路况生成时时的行进路径

上述场景中，全局路径规划和本地路径规划时，就会使用到参数服务器：

- 路径规划时，需要参考小车的尺寸，我们可以将这些尺寸信息存储到参数服务器，全局路径规划节点与本地路径规划节点都可以从参数服务器中调用这些参数

参数服务器，一般适用于存在数据共享的一些应用场景。

------

#### **概念**

以共享的方式实现不同节点之间数据交互的通信模式。

#### **作用**

存储一些多节点共享的数据，类似于全局变量。

#### **案例**

实现参数增删改查操作。

#### 理论模型

参数服务器实现是最为简单的，该模型如下图所示,该模型中涉及到三个角色:

- ROS Master (管理者)
- Talker (参数设置者)
- Listener (参数调用者)

ROS Master 作为一个公共容器保存参数，Talker 可以向容器中设置参数，Listener 可以获取参数。

整个流程由以下步骤实现:

#### 1.Talker 设置参数

Talker 通过 RPC 向参数服务器发送参数(包括参数名与参数值)，ROS Master 将参数保存到参数列表中。

#### 2.Listener 获取参数

Listener 通过 RPC 向参数服务器发送参数查找请求，请求中包含要查找的参数名。

#### 3.ROS Master 向 Listener 发送参数值

ROS Master 根据步骤2请求提供的参数名查找参数值，并将查询结果通过 RPC 发送给 Listener。

------

参数可使用数据类型:

- 32-bit integers
- booleans
- strings
- doubles
- iso8601 dates
- lists
- base64-encoded binary data
- 字典

> 注意:参数服务器不是为高性能而设计的，因此最好用于存储静态的非二进制的简单数据

### 常用命令

机器人系统中启动的节点少则几个，多则十几个、几十个，不同的节点名称各异，通信时使用话题、服务、消息、参数等等都各不相同，一个显而易见的问题是: 当需要自定义节点和其他某个已经存在的节点通信时，如何获取对方的话题、以及消息载体的格式呢？

在 ROS 同提供了一些实用的命令行工具，可以用于获取不同节点的各类信息，常用的命令如下:

- rosnode : 操作节点
- rostopic : 操作话题
- rosservice : 操作服务
- rosmsg : 操作msg消息
- rossrv : 操作srv消息
- rosparam : 操作参数

------

#### **作用**

和之前介绍的文件系统操作命令比较，文件操作命令是静态的，操作的是磁盘上的文件，而上述命令是动态的，在ROS程序启动后，可以动态的获取运行中的节点或参数的相关信息。
