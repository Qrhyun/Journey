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

## ROS通信机制进阶

### 常用API(c++)

首先，建议参考官方API文档或参考源码:

- ROS节点的初始化相关API;

- NodeHandle 的基本使用相关API;

- 话题的发布方，订阅方对象相关API;

- 服务的服务端，客户端对象相关API;

- 时间相关API;

- 日志输出相关API

  

- 1. 初始化

  ```c++
  /** @brief ROS初始化函数。
   *
   * 该函数可以解析并使用节点启动时传入的参数(通过参数设置节点名称、命名空间...) 
   *
   * 该函数有多个重载版本，如果使用NodeHandle建议调用该版本。 
   *
   * \param argc 参数个数
   * \param argv 参数列表
   * \param name 节点名称，需要保证其唯一性，不允许包含命名空间
   * \param options 节点启动选项，被封装进了ros::init_options
   *
   */
  void init(int &argc, char **argv, const std::string& name, uint32_t options = 0);
  ```

+ 2. 话题与服务相关对象

     在 roscpp 中，话题和服务的相关对象一般由 NodeHandle 创建。

  + 发布对象

    + 对象获取

      ```c++
      /**
      * \brief 根据话题生成发布对象
      *
      * 在 ROS master 注册并返回一个发布者对象，该对象可以发布消息
      *
      * 使用示例如下:
      *
      *   ros::Publisher pub = handle.advertise<std_msgs::Empty>("my_topic", 1);
      *
      * \param topic 发布消息使用的话题
      *
      * \param queue_size 等待发送给订阅者的最大消息数量
      *
      * \param latch (optional) 如果为 true,该话题发布的最后一条消息将被保存，并且后期当有订阅者连接时会将该消息发送给订阅者
      *
      * \return 调用成功时，会返回一个发布对象
      *
      *
      */
      template <class M>
      Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false)
      ```

    + **消息发布函数:**

      ```c++
      /**
      * 发布消息          
      */
      template <typename M>
      void publish(const M& message) const
      ```

  + 订阅对象

    + 对象获取

      ```c++
      /**
         * \brief 生成某个话题的订阅对象
         *
         * 该函数将根据给定的话题在ROS master 注册，并自动连接相同主题的发布方，每接收到一条消息，都会调用回调
         * 函数，并且传入该消息的共享指针，该消息不能被修改，因为可能其他订阅对象也会使用该消息。
         * 
         * 使用示例如下:
      
      void callback(const std_msgs::Empty::ConstPtr& message)
      {
      }
      
      ros::Subscriber sub = handle.subscribe("my_topic", 1, callback);
      
         *
      * \param M [template] M 是指消息类型
      * \param topic 订阅的话题
      * \param queue_size 消息队列长度，超出长度时，头部的消息将被弃用
      * \param fp 当订阅到一条消息时，需要执行的回调函数
      * \return 调用成功时，返回一个订阅者对象，失败时，返回空对象
      * 
      
      void callback(const std_msgs::Empty::ConstPtr& message){...}
      ros::NodeHandle nodeHandle;
      ros::Subscriber sub = nodeHandle.subscribe("my_topic", 1, callback);
      if (sub) // Enter if subscriber is valid
      {
      ...
      }
      
      */
      template<class M>
      Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr<M const>&), const TransportHints& transport_hints = TransportHints())
      ```

      

  + 服务对象

    + **对象获取:**

      ```c++
      /**
      * \brief 生成服务端对象
      *
      * 该函数可以连接到 ROS master，并提供一个具有给定名称的服务对象。
      *
      * 使用示例如下:
      \verbatim
      bool callback(std_srvs::Empty& request, std_srvs::Empty& response)
      {
      return true;
      }
      
      ros::ServiceServer service = handle.advertiseService("my_service", callback);
      \endverbatim
      *
      * \param service 服务的主题名称
      * \param srv_func 接收到请求时，需要处理请求的回调函数
      * \return 请求成功时返回服务对象，否则返回空对象:
      \verbatim
      bool Foo::callback(std_srvs::Empty& request, std_srvs::Empty& response)
      {
      return true;
      }
      ros::NodeHandle nodeHandle;
      Foo foo_object;
      ros::ServiceServer service = nodeHandle.advertiseService("my_service", callback);
      if (service) // Enter if advertised service is valid
      {
      ...
      }
      \endverbatim
      
      */
      template<class MReq, class MRes>
      ServiceServer advertiseService(const std::string& service, bool(*srv_func)(MReq&, MRes&))
      ```

      

  + 客户端对象

    + **对象获取：**

      ```c++
      /** 
        * @brief 创建一个服务客户端对象
        *
        * 当清除最后一个连接的引用句柄时，连接将被关闭。
        *
        * @param service_name 服务主题名称
        */
       template<class Service>
       ServiceClient serviceClient(const std::string& service_name, bool persistent = false, 
                                   const M_string& header_values = M_string())
      ```

    + **请求发送函数:**

      ```c++
      /**
         * @brief 发送请求
         * 返回值为 bool 类型，true，请求处理成功，false，处理失败。
         */
        template<class Service>
        bool call(Service& service)
      ```

    + **等待服务函数1:**

      ```c++
      /**
       * ros::service::waitForService("addInts");
       * \brief 等待服务可用，否则一致处于阻塞状态
       * \param service_name 被"等待"的服务的话题名称
       * \param timeout 等待最大时常，默认为 -1，可以永久等待直至节点关闭
       * \return 成功返回 true，否则返回 false。
       */
      ROSCPP_DECL bool waitForService(const std::string& service_name, ros::Duration timeout = ros::Duration(-1));
      ```

    + **等待服务函数2:**

      ```c++
      /**
      * client.waitForExistence();
      * \brief 等待服务可用，否则一致处于阻塞状态
      * \param timeout 等待最大时常，默认为 -1，可以永久等待直至节点关闭
      * \return 成功返回 true，否则返回 false。
      */
      bool waitForExistence(ros::Duration timeout = ros::Duration(-1));
      ```

+ 3. 回旋函数

     在ROS程序中，频繁的使用了 ros::spin() 和 ros::spinOnce() 两个回旋函数，可以用于处理回调函数。

     - spinOnce()

       ```c++
       /**
        * \brief 处理一轮回调
        *
        * 一般应用场景:
        *     在循环体内，处理所有可用的回调函数
        * 
        */
       ROSCPP_DECL void spinOnce();
       ```

     - spin()

       ```c++
       /** 
        * \brief 进入循环处理回调 
        */
       ROSCPP_DECL void spin();
       ```

     - 二者比较

       ```c++
       相同点:二者都用于处理回调函数；
       
       不同点:ros::spin() 是进入了循环执行回调函数，而 ros::spinOnce() 只会执行一次回调函数(没有循环)，在 ros::spin() 后的语句不会执行到，而 ros::spinOnce() 后的语句可以执行。
       ```

+ 4. 时间

      ROS中时间相关的API是极其常用，比如:获取当前时刻、持续时间的设置、执行频率、休眠、定时器...都与时间相关。

     + 获取时刻

       ```c++
       ros::init(argc,argv,"hello_time");
       ros::NodeHandle nh;//必须创建句柄，否则时间没有初始化，导致后续API调用失败
       ros::Time right_now = ros::Time::now();//将当前时刻封装成对象
       ROS_INFO("当前时刻:%.2f",right_now.toSec());//获取距离 1970年01月01日 00:00:00 的秒数
       ROS_INFO("当前时刻:%d",right_now.sec);//获取距离 1970年01月01日 00:00:00 的秒数
       
       ros::Time someTime(100,100000000);// 参数1:秒数  参数2:纳秒
       ROS_INFO("时刻:%.2f",someTime.toSec()); //100.10
       ros::Time someTime2(100.3);//直接传入 double 类型的秒数
       ROS_INFO("时刻:%.2f",someTime2.toSec()); //100.30
       ```

     + 持续时间

       设置一个时间区间(间隔):

       ```c++
       ROS_INFO("当前时刻:%.2f",ros::Time::now().toSec());
       ros::Duration du(10);//持续10秒钟,参数是double类型的，以秒为单位
       du.sleep();//按照指定的持续时间休眠
       ROS_INFO("持续时间:%.2f",du.toSec());//将持续时间换算成秒
       ROS_INFO("当前时刻:%.2f",ros::Time::now().toSec());
       ```

     + 持续时间与时刻运算

       为了方便使用，ROS中提供了时间与时刻的运算:

       ```c++
       ROS_INFO("时间运算");
       ros::Time now = ros::Time::now();
       ros::Duration du1(10);
       ros::Duration du2(20);
       ROS_INFO("当前时刻:%.2f",now.toSec());
       //1.time 与 duration 运算
       ros::Time after_now = now + du1;
       ros::Time before_now = now - du1;
       ROS_INFO("当前时刻之后:%.2f",after_now.toSec());
       ROS_INFO("当前时刻之前:%.2f",before_now.toSec());
       
       //2.duration 之间相互运算
       ros::Duration du3 = du1 + du2;
       ros::Duration du4 = du1 - du2;
       ROS_INFO("du3 = %.2f",du3.toSec());
       ROS_INFO("du4 = %.2f",du4.toSec());
       //PS: time 与 time 不可以运算
       // ros::Time nn = now + before_now;//异常
       ```

     + 设置运行频率

       ```c++
       ros::Rate rate(1);//指定频率
       while (true)
       {
           ROS_INFO("-----------code----------");
           rate.sleep();//休眠，休眠时间 = 1 / 频率。
       }
       ```

     + 定时器

       ```c++
       ros::NodeHandle nh;//必须创建句柄，否则时间没有初始化，导致后续API调用失败
       
        // ROS 定时器
        /**
       * \brief 创建一个定时器，按照指定频率调用回调函数。
       *
       * \param period 时间间隔
       * \param callback 回调函数
       * \param oneshot 如果设置为 true,只执行一次回调函数，设置为 false,就循环执行。
       * \param autostart 如果为true，返回已经启动的定时器,设置为 false，需要手动启动。
       */
        //Timer createTimer(Duration period, const TimerCallback& callback, bool oneshot = false,
        //                bool autostart = true) const;
       
        // ros::Timer timer = nh.createTimer(ros::Duration(0.5),doSomeThing);
        ros::Timer timer = nh.createTimer(ros::Duration(0.5),doSomeThing,true);//只执行一次
       
        // ros::Timer timer = nh.createTimer(ros::Duration(0.5),doSomeThing,false,false);//需要手动启动
        // timer.start();
        ros::spin(); //必须 spin
       ```

       - 定时器的回调函数

         ```c++
         void doSomeThing(const ros::TimerEvent &event){
             ROS_INFO("-------------");
             ROS_INFO("event:%s",std::to_string(event.current_real.toSec()).c_str());
         }
         ```

+ 5. 其他函数

     在发布实现时，一般会循环发布消息，循环的判断条件一般由节点状态来控制，C++中可以通过 ros::ok() 来判断节点状态是否正常，而 python 中则通过 rospy.is_shutdown() 来实现判断，导致节点退出的原因主要有如下几种:

     - 节点接收到了关闭信息，比如常用的 ctrl + c 快捷键就是关闭节点的信号；
     - 同名节点启动，导致现有节点退出；
     - 程序中的其他部分调用了节点关闭相关的API(C++中是ros::shutdown()，python中是rospy.signal_shutdown())

     另外，日志相关的函数也是极其常用的，在ROS中日志被划分成如下级别:

     - DEBUG(调试):只在调试时使用，此类消息不会输出到控制台；
     - INFO(信息):标准消息，一般用于说明系统内正在执行的操作；
     - WARN(警告):提醒一些异常情况，但程序仍然可以执行；
     - ERROR(错误):提示错误信息，此类错误会影响程序运行；
     - FATAL(严重错误):此类错误将阻止节点继续运行。

  + .节点状态判断

    ```c++
    /** \brief 检查节点是否已经退出
     *
     *  ros::shutdown() 被调用且执行完毕后，该函数将会返回 false
     *
     * \return true 如果节点还健在, false 如果节点已经火化了。
     */
    bool ok();
    ```

  + 节点关闭函数

    ```c++
    /*
    *   关闭节点
    */
    void shutdown();
    ```

  + 日志函数

    ```c++
    ROS_DEBUG("hello,DEBUG"); //不会输出
    ROS_INFO("hello,INFO"); //默认白色字体
    ROS_WARN("Hello,WARN"); //默认黄色字体
    ROS_ERROR("hello,ERROR");//默认红色字体
    ROS_FATAL("hello,FATAL");//默认红色字体
    ```

### ROS中的头文件与源文件

1. 设置头文件，可执行文件作为源文件；
2. 分别设置头文件，源文件与可执行文件。

在ROS中关于头文件的使用，核心内容在于CMakeLists.txt文件的配置，不同的封装方式，配置上也有差异。

#### 自定义头文件调用

**需求:**设计头文件，可执行文件本身作为源文件。

**流程:**

1. 编写头文件；
2. 编写可执行文件(同时也是源文件)；
3. 编辑配置文件并执行。

##### 1.头文件

在功能包下的 include/功能包名 目录下新建头文件: hello.h，示例内容如下:

```cpp
#ifndef _HELLO_H
#define _HELLO_H

namespace hello_ns{

class HelloPub {

public:
    void run();
};

}

#endif
```

**注意:**

在`VScode`中，为了后续包含头文件时不抛出异常，请配置`.vscode`下` c_cpp_properties.json`的`includepath`属性

```
"/home/用户/工作空间/src/功能包/include/**"
```

##### 2.可执行文件

在 src 目录下新建文件:hello.cpp，示例内容如下:

```cpp
#include "ros/ros.h"
#include "test_head/hello.h"

namespace hello_ns {

void HelloPub::run(){
    ROS_INFO("自定义头文件的使用....");
}

}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"test_head_node");
    hello_ns::HelloPub helloPub;
    helloPub.run();
    return 0;
}
```

##### 3.配置文件

配置`CMakeLists.txt`文件，头文件相关配置如下:

```makefile
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
```

可执行配置文件配置方式与之前一致:

```makefile
add_executable(hello src/hello.cpp)

add_dependencies(hello ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hello
  ${catkin_LIBRARIES}
)
```

最后，编译并执行，控制台可以输出自定义的文本信息。

## ROS运行管理

ROS是多进程(节点)的分布式框架，一个完整的ROS系统实现：

> 可能包含多台主机；
> 每台主机上又有多个工作空间(workspace)；
> 每个的工作空间中又包含多个功能包(package)；
> 每个功能包又包含多个节点(Node)，不同的节点都有自己的节点名称；
> 每个节点可能还会设置一个或多个话题(topic)...

在多级层深的ROS系统中，其实现与维护可能会出现一些问题，比如，如何关联不同的功能包，繁多的ROS节点应该如何启动？功能包、节点、话题、参数重名时应该如何处理？不同主机上的节点如何通信？

### ROS元功能包

**场景:**完成ROS中一个系统性的功能，可能涉及到多个功能包，比如实现了机器人导航模块，该模块下有地图、定位、路径规划...等不同的子级功能包。那么调用者安装该模块时，需要逐一的安装每一个功能包吗？

显而易见的，逐一安装功能包的效率低下，在ROS中，提供了一种方式可以将不同的功能包打包成一个功能包，当安装某个功能模块时，直接调用打包后的功能包即可，该包又称之为元功能包(metapackage)。

##### 概念

MetaPackage是Linux的一个文件管理系统的概念。是ROS中的一个虚包，里面没有实质性的内容，但是它依赖了其他的软件包，通过这种方法可以把其他包组合起来，我们可以认为它是一本书的目录索引，告诉我们这个包集合中有哪些子包，并且该去哪里下载。

例如：

- `sudo apt install ros-noetic-desktop-full`命令安装ros时就使用了元功能包，该元功能包依赖于ROS中的其他一些功能包，安装该包时会一并安装依赖。

还有一些常见的`MetaPackage`：navigation moveit! turtlebot3 ....	

##### 作用

方便用户的安装，我们只需要这一个包就可以把其他相关的软件包组织到一起安装了。

##### 实现

**首先:**新建一个功能包

**然后:**修改**package.xml** ,内容如下:

```xml
 <exec_depend>被集成的功能包</exec_depend>
 .....
 <export>
   <metapackage />
 </export>
```

**最后:**修改 CMakeLists.txt,内容如下:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(demo)
find_package(catkin REQUIRED)
catkin_metapackage()
```

PS:CMakeLists.txt 中不可以有换行。

### ROS节点运行管理launch文件

关于 launch 文件的使用我们已经不陌生了，在第一章内容中，就曾经介绍到:

> 一个程序中可能需要启动多个节点，比如:ROS 内置的小乌龟案例，如果要控制乌龟运动，要启动多个窗口，分别启动 roscore、乌龟界面节点、键盘控制节点。**如果每次都调用 rosrun 逐一启动，显然效率低下**，如何优化?

采用的优化策略便是使用`roslaunch` 命令集合`launch`文件启动管理节点，并且在后续教程中，也多次使用到了 `launch`文件。

##### **概念**

launch 文件是一个 XML 格式的文件，可以启动本地和远程的多个节点，还可以在参数服务器中设置参数。

##### **作用**

简化节点的配置与启动，提高ROS程序的启动效率。

##### **使用**

以 turtlesim 为例演示

###### 1.新建launch文件

在功能包下添加 launch目录, 目录下新建 xxxx.launch 文件，编辑 launch 文件

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node"     name="myTurtle" output="screen" />
    <node pkg="turtlesim" type="turtle_teleop_key"  name="myTurtleContro" output="screen" />
</launch>
```

###### 2.调用 launch 文件

```
roslaunch 包名 xxx.launch
```

**注意:**roslaunch 命令执行launch文件时，首先会判断是否启动了 roscore,如果启动了，则不再启动，否则，会自动调用 roscore

**PS:**本节主要介绍launch文件的使用语法，launch 文件中的标签，以及不同标签的一些常用属性。

#### launch文件标签之launch

`<launch>`标签是所有 launch 文件的根标签，充当其他标签的容器

###### 1.属性

- `deprecated = "弃用声明"`

  告知用户当前 launch 文件已经弃用

###### 2.子级标签

所有其它标签都是launch的子级

#### launch文件标签之node

`<node>`标签用于指定 ROS 节点，是最常见的标签，需要注意的是: roslaunch 命令不能保证按照 node 的声明顺序来启动节点(节点的启动是多进程的)

###### 1.属性

- pkg="包名"

  节点所属的包

- type="nodeType"

  节点类型(与之相同名称的可执行文件)

- name="nodeName"

  节点名称(在 ROS 网络拓扑中节点的名称)

- args="xxx xxx xxx" (可选)

  将参数传递给节点

- machine="机器名"

  在指定机器上启动节点

- respawn="true | false" (可选)

  如果节点退出，是否自动重启

- respawn_delay=" N" (可选)

  如果 respawn 为 true, 那么延迟 N 秒后启动节点

- required="true | false" (可选)

  该节点是否必须，如果为 true,那么如果该节点退出，将杀死整个 roslaunch

- ns="xxx" (可选)

  在指定命名空间 xxx 中启动节点

- clear_params="true | false" (可选)

  在启动前，删除节点的私有空间的所有参数

- output="log | screen" (可选)

  日志发送目标，可以设置为 log 日志文件，或 screen 屏幕,默认是 log

###### 2.子级标签

- env 环境变量设置
- remap 重映射节点名称
- rosparam 参数设置
- param 参数设置

#### launch文件标签之include

`include`标签用于将另一个 xml 格式的 launch 文件导入到当前文件

###### 1.属性

- file="$(find 包名)/xxx/xxx.launch"

  要包含的文件路径

- ns="xxx" (可选)

  在指定命名空间导入文件

###### 2.子级标签

- env 环境变量设置
- arg 将参数传递给被包含的文件

#### launch文件标签之remap

用于话题重命名

###### 1.属性

- from="xxx"

  原始话题名称

- to="yyy"

  目标名称

###### 2.子级标签

- 无

#### todo....

### ROS工作空间覆盖

所谓工作空间覆盖，是指不同工作空间中，存在重名的功能包的情形。

> ROS 开发中，会自定义工作空间且自定义工作空间可以同时存在多个，可能会出现一种情况: 虽然特定工作空间内的功能包不能重名，但是自定义工作空间的功能包与内置的功能包可以重名或者不同的自定义的工作空间中也可以出现重名的功能包，那么调用该名称功能包时，会调用哪一个呢？比如：自定义工作空间A存在功能包 turtlesim，自定义工作空间B也存在功能包 turtlesim，当然系统内置空间也存在turtlesim，如果调用turtlesim包，会调用哪个工作空间中的呢？

#### **实现**

0.新建工作空间A与工作空间B，两个工作空间中都创建功能包: turtlesim。

1.在 ~/.bashrc 文件下**追加**当前工作空间的 bash 格式如下:

```
source /home/用户/路径/工作空间A/devel/setup.bash
source /home/用户/路径/工作空间B/devel/setup.bash
```

2.新开命令行:`source .bashrc`加载环境变量

3.查看ROS环境环境变量`echo $ROS_PACKAGE_PATH`

结果:自定义工作空间B:自定义空间A:系统内置空间

4.调用命令:`roscd turtlesim`会进入自定义工作空间B

#### **原因**

ROS 会解析 .bashrc 文件，并生成 ROS_PACKAGE_PATH ROS包路径，该变量中按照 .bashrc 中配置设置工作空间优先级，在设置时需要遵循一定的原则:ROS_PACKAGE_PATH 中的值，和 .bashrc 的配置顺序相反--->后配置的优先级更高，如果更改自定义空间A与自定义空间B的source顺序，那么调用时，将进入工作空间A。

#### **结论**

功能包重名时，会按照 ROS_PACKAGE_PATH 查找，配置在前的会优先执行。

#### **隐患**

存在安全隐患，比如当前工作空间B优先级更高，意味着当程序调用 turtlesim 时，不会调用工作空间A也不会调用系统内置的 turtlesim，如果工作空间A在实现时有其他功能包依赖于自身的 turtlesim，而按照ROS工作空间覆盖的涉及原则，那么实际执行时将会调用工作空间B的turtlesim，从而导致执行异常，出现安全隐患。

------

BUG 说明:

> 当在 .bashrc 文件中 source 多个工作空间后，可能出现的情况，在 ROS PACKAGE PATH 中只包含两个工作空间，可以删除自定义工作空间的 build 与 devel 目录，重新 catkin_make，然后重新载入 .bashrc 文件，问题解决。

### ROS节点名称重名

[4.4 ROS节点名称重名 · Autolabor-ROS机器人入门课程《ROS理论与实践》零基础教程](http://www.autolabor.com.cn/book/ROSTutorials/5/42-rosjie-dian-ming-cheng-zhong-ming.html)

> 场景:ROS 中创建的节点是有名称的，C++初始化节点时通过API:`ros::init(argc,argv,"xxxx");`来定义节点名称，在Python中初始化节点则通过 `rospy.init_node("yyyy")` 来定义节点名称。在ROS的网络拓扑中，是不可以出现重名的节点的，因为假设可以重名存在，那么调用时会产生混淆，这也就意味着，不可以启动重名节点或者同一个节点启动多次，的确，在ROS中如果启动重名节点的话，之前已经存在的节点会被直接关闭，但是如果有这种需求的话，怎么优化呢？

在ROS中给出的解决策略是使用命名空间或名称重映射

**命名空间就是为名称添加前缀，名称重映射是为名称起别名**。这两种策略都可以解决节点重名问题，两种策略的实现途径有多种:

- rosrun 命令
- launch 文件
- 编码实现

以上三种途径都可以通过命名空间或名称重映射的方式，来避免节点重名，本节将对三者的使用逐一演示，三者要实现的需求类似。

### todo...

## ROS常用组件

在ROS中内置一些比较实用的工具，通过这些工具可以方便快捷的实现某个功能或调试程序，从而提高开发效率，本章主要介绍ROS中内置的如下组件:

- TF坐标变换，实现不同类型的坐标系之间的转换；
- rosbag 用于录制ROS节点的执行过程并可以重放该过程；
- rqt 工具箱，集成了多款图形化的调试工具。

本章预期达成的学习目标:

- 了解 TF 坐标变换的概念以及应用场景；
- 能够独立完成TF案例:小乌龟跟随；
- 可以使用 rosbag 命令或编码的形式实现录制与回放；
- 能够熟练使用rqt中的图形化工具。

### TF坐标变换

机器人系统上，有多个传感器，如激光雷达、摄像头等，有的传感器是可以感知机器人周边的物体方位(或者称之为:坐标，横向、纵向、高度的距离信息)的，以协助机器人定位障碍物，可以直接将物体相对该传感器的方位信息，等价于物体相对于机器人系统或机器人其它组件的方位信息吗？显示是不行的，这中间需要一个转换过程。

**tf:**TransForm Frame,坐标变换

**坐标系:**ROS 中是通过坐标系统开标定物体的，确切的将是通过右手坐标系来标定的。

![img](http://www.autolabor.com.cn/book/ROSTutorials/assets/%E5%8F%B3%E6%89%8B%E5%9D%90%E6%A0%87%E7%B3%BB.jpg)

##### **作用**

在 ROS 中用于实现不同坐标系之间的点或向量的转换。

##### **案例**

**小乌龟跟随案例：**如本章引言部分演示。

##### 说明

在ROS中坐标变换最初对应的是tf，不过在 hydro 版本开始, tf 被弃用，迁移到 tf2,后者更为简洁高效，tf2对应的常用功能包有:

tf2_geometry_msgs:可以将ROS消息转换成tf2消息。

tf2: 封装了坐标变换的常用消息。

tf2_ros:为tf2提供了roscpp和rospy绑定，封装了坐标变换常用的API。

------

**另请参考:**

- http://wiki.ros.org/tf2

####  坐标msg消息

订阅发布模型中数据载体 msg 是一个重要实现，首先需要了解一下，在坐标转换实现中常用的 msg:`geometry_msgs/TransformStamped`和`geometry_msgs/PointStamped`

**前者用于传输坐标系相关位置信息，后者用于传输某个坐标系内坐标点的信息**。在坐标变换中，频繁的需要使用到坐标系的相对关系以及坐标点信息。

##### `1.geometry_msgs/TransformStamped`

命令行键入:`rosmsg info geometry_msgs/TransformStamped`

```bash
std_msgs/Header header                     #头信息
  uint32 seq                                #|-- 序列号
  time stamp                                #|-- 时间戳
  string frame_id                            #|-- 坐标 ID
string child_frame_id                    #子坐标系的 id
geometry_msgs/Transform transform        #坐标信息
  geometry_msgs/Vector3 translation        #偏移量
    float64 x                                #|-- X 方向的偏移量
    float64 y                                #|-- Y 方向的偏移量
    float64 z                                #|-- Z 方向上的偏移量
  geometry_msgs/Quaternion rotation        #四元数
    float64 x                                
    float64 y                                
    float64 z                                
    float64 w
```

四元数用于表示坐标的相对姿态

##### `2.geometry_msgs/PointStamped`

命令行键入:`rosmsg info geometry_msgs/PointStamped`

```bash
std_msgs/Header header                    #头
  uint32 seq                                #|-- 序号
  time stamp                                #|-- 时间戳
  string frame_id                            #|-- 所属坐标系的 id
geometry_msgs/Point point                #点坐标
  float64 x                                    #|-- x y z 坐标
  float64 y
  float64 z
```

#### 静态坐标变换

**实现分析:**

1. 坐标系相对关系，可以通过发布方发布
2. 订阅方，订阅到发布的坐标系相对关系，再传入坐标点信息(可以写死)，然后借助于 tf 实现坐标变换，并将结果输出

**实现流程:**C++ 与 Python 实现流程一致

1. 新建功能包，添加依赖
2. 编写发布方实现
3. 编写订阅方实现
4. 执行并查看结果

C++实现

##### 1.创建功能包

创建项目功能包依赖于 tf2、tf2_ros、tf2_geometry_msgs、roscpp rospy std_msgs geometry_msgs

##### 2.发布方

```cpp
/* 
    静态坐标变换发布方:
        发布关于 laser 坐标系的位置信息 

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建静态坐标转换广播器
        4.创建坐标系信息
        5.广播器发布坐标系信息
        6.spin()
*/


// 1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"static_brocast");
    // 3.创建静态坐标转换广播器
    tf2_ros::StaticTransformBroadcaster broadcaster;
    // 4.创建坐标系信息
    geometry_msgs::TransformStamped ts;
    //----设置头信息
    ts.header.seq = 100;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = "base_link";
    //----设置子级坐标系
    ts.child_frame_id = "laser";
    //----设置子级相对于父级的偏移量
    ts.transform.translation.x = 0.2;
    ts.transform.translation.y = 0.0;
    ts.transform.translation.z = 0.5;
    //----设置四元数:将 欧拉角数据转换成四元数
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0);
    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();
    // 5.广播器发布坐标系信息
    broadcaster.sendTransform(ts);
    ros::spin();
    return 0;
}
```

配置文件此处略。

##### 3.订阅方

```cpp
/*  
    订阅坐标系信息，生成一个相对于 子级坐标系的坐标点数据，转换成父级坐标系中的坐标点

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建 TF 订阅节点
        4.生成一个坐标点(相对于子级坐标系)
        5.转换坐标点(相对于父级坐标系)
        6.spin()
*/
//1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"tf_sub");
    ros::NodeHandle nh;
    // 3.创建 TF 订阅节点
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Rate r(1);
    while (ros::ok())
    {
    // 4.生成一个坐标点(相对于子级坐标系)
        geometry_msgs::PointStamped point_laser;
        point_laser.header.frame_id = "laser";
        point_laser.header.stamp = ros::Time::now();
        point_laser.point.x = 1;
        point_laser.point.y = 2;
        point_laser.point.z = 7.3;
    // 5.转换坐标点(相对于父级坐标系)
        //新建一个坐标点，用于接收转换结果  
        //--------------使用 try 语句或休眠，否则可能由于缓存接收延迟而导致坐标转换失败------------------------
        try
        {
            geometry_msgs::PointStamped point_base;
            point_base = buffer.transform(point_laser,"base_link");
            ROS_INFO("转换后的数据:(%.2f,%.2f,%.2f),参考的坐标系是:",point_base.point.x,point_base.point.y,point_base.point.z,point_base.header.frame_id.c_str());

        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("程序异常.....");
        }


        r.sleep();  
        ros::spinOnce();
    }


    return 0;
}
```

配置文件此处略。

##### 4.执行

可以使用命令行或launch文件的方式分别启动发布节点与订阅节点，如果程序无异常，控制台将输出，坐标转换后的结果。

#### 动态坐标变换

所谓动态坐标变换，是指两个坐标系之间的相对位置是变化的。

**需求描述:**

启动 turtlesim_node,该节点中窗体有一个世界坐标系(左下角为坐标系原点)，乌龟是另一个坐标系，键盘控制乌龟运动，将两个坐标系的相对位置动态发布。

**实现分析:**

1. 乌龟本身不但可以看作坐标系，也是世界坐标系中的一个坐标点
2. 订阅 turtle1/pose,可以获取乌龟在世界坐标系的 x坐标、y坐标、偏移量以及线速度和角速度
3. 将 pose 信息转换成 坐标系相对信息并发布

**实现流程:**C++ 与 Python 实现流程一致

1. 新建功能包，添加依赖

2. 创建坐标相对关系发布方(同时需要订阅乌龟位姿信息)

3. 创建坐标相对关系订阅方

4. 执行

5. 方案A:C++实现

   ##### 1.创建功能包

   创建项目功能包依赖于 tf2、tf2_ros、tf2_geometry_msgs、roscpp rospy std_msgs geometry_msgs、turtlesim

   ##### 2.发布方

   ```cpp
   /*  
       动态的坐标系相对姿态发布(一个坐标系相对于另一个坐标系的相对姿态是不断变动的)
   
       需求: 启动 turtlesim_node,该节点中窗体有一个世界坐标系(左下角为坐标系原点)，乌龟是另一个坐标系，键盘
       控制乌龟运动，将两个坐标系的相对位置动态发布
   
       实现分析:
           1.乌龟本身不但可以看作坐标系，也是世界坐标系中的一个坐标点
           2.订阅 turtle1/pose,可以获取乌龟在世界坐标系的 x坐标、y坐标、偏移量以及线速度和角速度
           3.将 pose 信息转换成 坐标系相对信息并发布
   
       实现流程:
           1.包含头文件
           2.初始化 ROS 节点
           3.创建 ROS 句柄
           4.创建订阅对象
           5.回调函数处理订阅到的数据(实现TF广播)
               5-1.创建 TF 广播器
               5-2.创建 广播的数据(通过 pose 设置)
               5-3.广播器发布数据
           6.spin
   */
   // 1.包含头文件
   #include "ros/ros.h"
   #include "turtlesim/Pose.h"
   #include "tf2_ros/transform_broadcaster.h"
   #include "geometry_msgs/TransformStamped.h"
   #include "tf2/LinearMath/Quaternion.h"
   
   void doPose(const turtlesim::Pose::ConstPtr& pose){
       //  5-1.创建 TF 广播器
       static tf2_ros::TransformBroadcaster broadcaster;
       //  5-2.创建 广播的数据(通过 pose 设置)
       geometry_msgs::TransformStamped tfs;
       //  |----头设置
       tfs.header.frame_id = "world";
       tfs.header.stamp = ros::Time::now();
   
       //  |----坐标系 ID
       tfs.child_frame_id = "turtle1";
   
       //  |----坐标系相对信息设置
       tfs.transform.translation.x = pose->x;
       tfs.transform.translation.y = pose->y;
       tfs.transform.translation.z = 0.0; // 二维实现，pose 中没有z，z 是 0
       //  |--------- 四元数设置
       tf2::Quaternion qtn;
       qtn.setRPY(0,0,pose->theta);
       tfs.transform.rotation.x = qtn.getX();
       tfs.transform.rotation.y = qtn.getY();
       tfs.transform.rotation.z = qtn.getZ();
       tfs.transform.rotation.w = qtn.getW();
   
   
       //  5-3.广播器发布数据
       broadcaster.sendTransform(tfs);
   }
   
   int main(int argc, char *argv[])
   {
       setlocale(LC_ALL,"");
       // 2.初始化 ROS 节点
       ros::init(argc,argv,"dynamic_tf_pub");
       // 3.创建 ROS 句柄
       ros::NodeHandle nh;
       // 4.创建订阅对象
       ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose",1000,doPose);
       //     5.回调函数处理订阅到的数据(实现TF广播)
       //        
       // 6.spin
       ros::spin();
       return 0;
   }
   ```

   配置文件此处略。

   ##### 3.订阅方

   ```cpp
   //1.包含头文件
   #include "ros/ros.h"
   #include "tf2_ros/transform_listener.h"
   #include "tf2_ros/buffer.h"
   #include "geometry_msgs/PointStamped.h"
   #include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
   
   int main(int argc, char *argv[])
   {
       setlocale(LC_ALL,"");
       // 2.初始化 ROS 节点
       ros::init(argc,argv,"dynamic_tf_sub");
       ros::NodeHandle nh;
       // 3.创建 TF 订阅节点
       tf2_ros::Buffer buffer;
       tf2_ros::TransformListener listener(buffer);
   
       ros::Rate r(1);
       while (ros::ok())
       {
       // 4.生成一个坐标点(相对于子级坐标系)
           geometry_msgs::PointStamped point_laser;
           point_laser.header.frame_id = "turtle1";
           point_laser.header.stamp = ros::Time();
           point_laser.point.x = 1;
           point_laser.point.y = 1;
           point_laser.point.z = 0;
       // 5.转换坐标点(相对于父级坐标系)
           //新建一个坐标点，用于接收转换结果  
           //--------------使用 try 语句或休眠，否则可能由于缓存接收延迟而导致坐标转换失败------------------------
           try
           {
               geometry_msgs::PointStamped point_base;
               point_base = buffer.transform(point_laser,"world");
               ROS_INFO("坐标点相对于 world 的坐标为:(%.2f,%.2f,%.2f)",point_base.point.x,point_base.point.y,point_base.point.z);
   
           }
           catch(const std::exception& e)
           {
               // std::cerr << e.what() << '\n';
               ROS_INFO("程序异常:%s",e.what());
           }
   
   
           r.sleep();  
           ros::spinOnce();
       }
   
   
       return 0;
   }
   ```

   配置文件此处略。

   ##### 4.执行

   可以使用命令行或launch文件的方式分别启动发布节点与订阅节点，如果程序无异常，与演示结果类似。

   可以使用 rviz 查看坐标系相对关系。

#### 坐标系关系查看

在机器人系统中，涉及的坐标系有多个，为了方便查看，ros 提供了专门的工具，可以用于生成显示坐标系关系的 pdf 文件，该文件包含树形结构的坐标系图谱。

##### 准备

首先调用`rospack find tf2_tools`查看是否包含该功能包，如果没有，请使用如下命令安装:

```
sudo apt install ros-noetic-tf2-tools
```

##### 使用

##### 生成 pdf 文件

启动坐标系广播程序之后，运行如下命令:

```
rosrun tf2_tools view_frames.py
```

会产生类似于下面的日志信息:

```
[INFO] [1592920556.827549]: Listening to tf data during 5 seconds...
[INFO] [1592920561.841536]: Generating graph in frames.pdf file...
```

查看当前目录会生成一个 frames.pdf 文件

##### 查看文件

可以直接进入目录打开文件，或者调用命令查看文件:`evince frames.pdf`

内如如图所示:![img](http://www.autolabor.com.cn/book/ROSTutorials/assets/12%E5%9D%90%E6%A0%87%E5%8F%98%E6%8D%A2.PNG)

#### TF2与TF

##### 1.TF2与TF比较_简介

- TF2已经替换了TF，TF2是TF的超集，建议学习 TF2 而非 TF
- TF2 功能包的增强了内聚性，TF 与 TF2 所依赖的功能包是不同的，TF 对应的是`tf`包，TF2 对应的是`tf2`和`tf2_ros`包，在 TF2 中不同类型的 API 实现做了分包处理。
- TF2 实现效率更高，比如在:TF2 的静态坐标实现、TF2 坐标变换监听器中的 Buffer 实现等

##### 2.TF2与TF比较_静态坐标变换演示

接下来，我们通过静态坐标变换来演示TF2的实现效率。

###### 2.1启动 TF2 与 TF 两个版本的静态坐标变换

TF2 版静态坐标变换:`rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 /base_link /laser`

TF 版静态坐标变换:`rosrun tf static_transform_publisher 0 0 0 0 0 0 /base_link /laser 100`

会发现，TF 版本的启动中最后多一个参数，该参数是指定发布频率

###### 2.2运行结果比对

使用`rostopic`查看话题，包含`/tf`与`/tf_static`, 前者是 TF 发布的话题，后者是 TF2 发布的话题，分别调用命令打印二者的话题消息

`rostopic echo /tf`: 当前会**循环输出坐标系信息**

`rostopic echo /tf_static`: **坐标系信息只有一次**

###### 2.3结论

如果是静态坐标转换，那么不同坐标系之间的相对状态是固定的，既然是固定的，那么没有必要重复发布坐标系的转换消息，很显然的，tf2 实现较之于 tf 更为高效

### rosbag

机器人传感器获取到的信息，有时我们可能需要时时处理，有时可能只是采集数据，事后分析，比如:

> 机器人导航实现中，可能需要绘制导航所需的全局地图，地图绘制实现，有两种方式，方式1：可以控制机器人运动，将机器人传感器感知到的数据时时处理，生成地图信息。方式2：同样是控制机器人运动，将机器人传感器感知到的数据留存，事后，再重新读取数据，生成地图信息。两种方式比较，显然方式2使用上更为灵活方便。

在ROS中关于数据的留存以及读取实现，提供了专门的工具: rosbag。

#### **概念**

是用于录制和回放 ROS 主题的一个工具集。

#### **作用**

实现了数据的复用，方便调试、测试。

#### **本质**

**rosbag本质也是ros的节点**，当录制时，rosbag是一个订阅节点，可以订阅话题消息并将订阅到的数据写入磁盘文件；**当重放时，rosbag是一个发布节点**，可以读取磁盘文件，发布文件中的话题消息。

#### rosbag使用_命令行

**需求:**

ROS 内置的乌龟案例并操作，操作过程中使用 rosbag 录制，录制结束后，实现重放

**实现:**

1.准备

创建目录保存录制的文件

```
mkdir ./xxx
cd xxx
```

2.开始录制

```
rosbag record -a -O 目标文件
```

操作小乌龟一段时间，结束录制使用 ctrl + c，在创建的目录中会生成bag文件。

3.查看文件

```
rosbag info 文件名
```

4.回放文件

```
rosbag play 文件名
```

重启乌龟节点，会发现，乌龟按照录制时的轨迹运动。

#### rosbag使用_编码

命令实现不够灵活，可以使用编码的方式，增强录制与回放的灵活性,本节将通过简单的读写实现演示rosbag的编码实现。

------

方案A:C++实现

##### 1.写 bag

```cpp
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/String.h"


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"bag_write");
    ros::NodeHandle nh;
    //创建bag对象
    rosbag::Bag bag;
    //打开
    bag.open("/home/rosdemo/demo/test.bag",rosbag::BagMode::Write);
    //写
    std_msgs::String msg;
    msg.data = "hello world";
    bag.write("/chatter",ros::Time::now(),msg);
    bag.write("/chatter",ros::Time::now(),msg);
    bag.write("/chatter",ros::Time::now(),msg);
    bag.write("/chatter",ros::Time::now(),msg);
    //关闭
    bag.close();

    return 0;
}
```

##### 2.读bag

```cpp
/*  
    读取 bag 文件：

*/
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

int main(int argc, char *argv[])
{

    setlocale(LC_ALL,"");

    ros::init(argc,argv,"bag_read");
    ros::NodeHandle nh;

    //创建 bag 对象
    rosbag::Bag bag;
    //打开 bag 文件
    bag.open("/home/rosdemo/demo/test.bag",rosbag::BagMode::Read);
    //读数据
    for (rosbag::MessageInstance const m : rosbag::View(bag))
    {
        std_msgs::String::ConstPtr p = m.instantiate<std_msgs::String>();
        if(p != nullptr){
            ROS_INFO("读取的数据:%s",p->data.c_str());
        }
    }

    //关闭文件流
    bag.close();
    return 0;
}
```

### rqt工具箱

之前，在 ROS 中使用了一些实用的工具,比如: ros_bag 用于录制与回放、tf2_tools 可以生成 TF 树 ..... 这些工具大大提高了开发的便利性，但是也存在一些问题: 这些工具的启动和使用过程中涉及到一些命令操作，应用起来不够方便，在ROS中，提供了rqt工具箱，在调用工具时以图形化操作代替了命令操作，应用更便利，提高了操作效率，优化了用户体验。

------

#### **概念**

ROS基于 QT 框架，针对机器人开发提供了一系列可视化的工具，这些工具的集合就是rqt

#### **作用**

可以方便的实现 ROS 可视化调试，并且在同一窗口中打开多个部件，提高开发效率，优化用户体验。

#### **组成**

rqt 工具箱组成有三大部分

- rqt——核心实现，开发人员无需关注
- rqt_common_plugins——rqt 中常用的工具套件
- rqt_robot_plugins——运行中和机器人交互的插件(比如: rviz)

#### rqt安装启动与基本使用

##### 1.安装

- 一般只要你安装的是desktop-full版本就会自带工具箱

- 如果需要安装可以以如下方式安装

  ```
  $ sudo apt-get install ros-noetic-rqt
  $ sudo apt-get install ros-noetic-rqt-common-plugins
  ```

##### 2.启动

`rqt`的启动方式有两种:

- 方式1:`rqt`
- 方式2:`rosrun rqt_gui rqt_gui`

##### 3.基本使用

启动 rqt 之后，可以通过 plugins 添加所需的插件![img](http://www.autolabor.com.cn/book/ROSTutorials/assets/13rqt%E5%B7%A5%E5%85%B7%E7%AE%B1.PNG)

#### rqt常用插件:rqt_graph

**简介:**可视化显示计算图

**启动:**可以在 rqt 的 plugins 中添加，或者使用`rqt_graph`启动

#### rqt常用插件:rqt_console

**简介:**rqt_console 是 ROS 中用于显示和过滤日志的图形化插件

**准备:**编写 Node 节点输出各个级别的日志信息

```cpp
/*  
    ROS 节点:输出各种级别的日志信息

*/
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"log_demo");
    ros::NodeHandle nh;

    ros::Rate r(0.3);
    while (ros::ok())
    {
        ROS_DEBUG("Debug message d");
        ROS_INFO("Info message oooooooooooooo");
        ROS_WARN("Warn message wwwww");
        ROS_ERROR("Erroe message EEEEEEEEEEEEEEEEEEEE");
        ROS_FATAL("Fatal message FFFFFFFFFFFFFFFFFFFFFFFFFFFFF");
        r.sleep();
    }


    return 0;
}
```

**启动:**

可以在 rqt 的 plugins 中添加，或者使用`rqt_console`启动

#### rqt常用插件:rqt_plot

**简介:**图形绘制插件，可以以 2D 绘图的方式绘制发布在 topic 上的数据

**准备:**启动 turtlesim 乌龟节点与键盘控制节点，通过 rqt_plot 获取乌龟位姿

**启动:**可以在 rqt 的 plugins 中添加，或者使用`rqt_plot`启动

#### rqt常用插件:rqt_bag

**简介:**录制和重放 bag 文件的图形化插件

**准备:**启动 turtlesim 乌龟节点与键盘控制节点

**启动:**可以在 rqt 的 plugins 中添加，或者使用`rqt_bag`启动

## 机器人系统仿真

机器人一般价格不菲，为了降低机器人学习、调试成本，在ROS中提供了系统的机器人仿真实现，通过仿真，可以实现大部分需求

本章会介绍:

- 如何创建并显示机器人模型；
- 如何搭建仿真环境；
- 如何实现机器人模型与仿真环境的交互。

本章预期的学习目标如下:

- 能够独立使用URDF创建机器人模型，并在Rviz和Gazebo中分别显示；
- 能够使用Gazebo搭建仿真环境；
- 能够使用机器人模型中的传感器(雷达、摄像头、编码器...)获取仿真环境数据。

**机器人系统仿真：**是通过计算机对实体机器人系统进行模拟的技术，在 ROS 中，**仿真实现涉及的内容主要有三:对机器人建模(URDF)、创建仿真环境(Gazebo)以及感知环境(Rviz)等系统性实现。**

#### 相关组件

##### URDF

**URDF**是 Unified Robot Description Format 的*首字母缩写，直译为**统一(标准化)机器人描述格式**，可以以一种 XML 的方式描述机器人的部分结构，比如底盘、摄像头、激光雷达、机械臂以及不同关节的自由度.....,该文件**可以被 C++ 内置的解释器转换成可视化的机器人模型**，是 ROS 中实现机器人仿真的重要组件

##### rviz

RViz 是 ROS Visualization Tool 的首字母缩写，直译为**ROS的三维可视化工具**。它的主要目的是**以三维方式显示ROS消息，可以将数据进行可视化表达，图形化的显示机器人各种传感器感知到的环境信息**。例如:可以显示机器人模型，可以无需编程就能表达激光测距仪（LRF）传感器中的传感 器到障碍物的距离，RealSense、Kinect或Xtion等三维距离传感器的点云数据（PCD， Point Cloud Data），从相机获取的图像值等

以“ros- [ROS_DISTRO] -desktop-full”命令安装ROS时，RViz会默认被安装。

运行使用命令`rviz`或`rosrun rviz rviz`

***如果rviz没有安装，请调用如下命令自行安装:***

```
sudo apt install ros-[ROS_DISTRO]-rviz
```

##### gazebo

Gazebo是一款**3D动态模拟器，用于显示机器人模型并创建仿真环境,能够在复杂的室内和室外环境中准确有效地模拟机器人**。与游戏引擎提供高保真度的视觉模拟类似，Gazebo提供高保真度的物理模拟，其提供一整套传感器模型，以及对用户和程序非常友好的交互方式。

以“ros- [ROS_DISTRO] -desktop-full”命令安装ROS时，gzebo会默认被安装。

运行使用命令`gazebo`或`rosrun gazebo_ros gazebo`

**注意1:**在 Ubuntu20.04 与 ROS Noetic 环境下，gazebo 启动异常以及解决

- **问题1:**VMware: vmw_ioctl_command error Invalid argument(无效的参数)

  **解决:**

  `echo "export SVGA_VGPU10=0" >> ~/.bashrc`

  `source .bashrc`

- **问题2:**[Err] [REST.cc:205] Error in REST request

  **解决:**`sudo gedit ~/.ignition/fuel/config.yaml`

  然后将`url : https://api.ignitionfuel.org`使用 # 注释

  再添加`url: https://api.ignitionrobotics.org`

- **问题3:**启动时抛出异常:`[gazebo-2] process has died [pid xxx, exit code 255, cmd.....`

  **解决:**`killall gzserver`和`killall gzclient`

**注意2:如果 gazebo没有安装，请自行安装:**

1.添加源:

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" 
>
 /etc/apt/sources.list.d/gazebo-stable.list'
Copy
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

2.安装：

```bash
sudo apt update
Copy
sudo apt install gazebo11 
sudo apt install libgazebo11-dev
```

##### 总结

三者应用中，只是创建 URDF 意义不大，一般需要结合 Gazebo 或 Rviz 使用，在 Gazebo 或 Rviz 中可以将 URDF 文件解析为图形化的机器人模型，一般的使用组合为:

- 如果非仿真环境，那么使用 URDF 结合 Rviz 直接显示感知的真实环境信息
- 如果是仿真环境，那么需要使用 URDF 结合 Gazebo 搭建仿真环境，并结合 Rviz 显示感知的虚拟环境信息

### URDF集成Rviz基本流程

前面介绍过，URDF 不能单独使用，需要结合 Rviz 或 Gazebo，URDF 只是一个文件，需要在 Rviz 或 Gazebo 中渲染成图形化的机器人模型，当前，首先演示URDF与Rviz的集成使用，因为URDF与Rviz的集成较之于URDF与Gazebo的集成更为简单，后期，基于Rviz的集成实现，我们再进一步介绍URDF语法

**需求描述:**

在 Rviz 中显示一个盒状机器人

**实现流程：**

1. 准备:新建功能包，导入依赖
2. 核心:编写 urdf 文件
3. 核心:在 launch 文件集成 URDF 与 Rviz
4. 在 Rviz 中显示机器人模型

#### 1.创建功能包，导入依赖

创建一个新的功能包，名称自定义，导入依赖包:`urdf`与`xacro`

在当前功能包下，再新建几个目录:

`urdf`: 存储 urdf 文件的目录

`meshes`:机器人模型渲染文件(暂不使用)

`config`: 配置文件

`launch`: 存储 launch 启动文件

#### 2.编写 URDF 文件

新建一个子级文件夹:`urdf`(可选)，文件夹中添加一个`.urdf`文件,复制如下内容:

```xml
<robot name="mycar">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.2 0.1" />
            </geometry>
        </visual>
    </link>
</robot>
```

#### 3.在 launch 文件中集成 URDF 与 Rviz

在`launch`目录下，新建一个 launch 文件，**该 launch 文件需要启动 Rviz，并导入 urdf 文件，Rviz 启动后可以自动载入解析`urdf`文件，并显示机器人模型**，**核心问题:如何导入 urdf 文件? 在 ROS 中，可以将 urdf 文件的路径设置到参数服务器**，使用的参数名是:`robot_description`,示例代码如下:

```xml
<launch>

    <!-- 设置参数 -->
    <param name="robot_description" textfile="$(find 包名)/urdf/urdf/urdf01_HelloWorld.urdf" />

    <!-- 启动 rviz -->
    <node pkg="rviz" type="rviz" name="rviz" />

</launch>
```

#### 4.在 Rviz 中显示机器人模型

rviz 启动后，会发现并没有盒装的机器人模型，这是因为默认情况下没有添加机器人显示组件，需要手动添加，添加方式如下:![img](http://www.autolabor.com.cn/book/ROSTutorials/assets/01_URDF%E6%96%87%E4%BB%B6%E6%89%A7%E8%A1%8Crviz%E9%85%8D%E7%BD%AE01.png)![img](http://www.autolabor.com.cn/book/ROSTutorials/assets/02_URDF%E6%96%87%E4%BB%B6%E6%89%A7%E8%A1%8Crviz%E9%85%8D%E7%BD%AE02.png)设置完毕后，可以正常显示了

#### 5.优化 rviz 启动

重复启动`launch`文件时，Rviz 之前的组件配置信息不会自动保存，需要重复执行步骤4的操作，为了方便使用，可以使用如下方式优化:

首先，将当前配置保存进`config`目录![img](http://www.autolabor.com.cn/book/ROSTutorials/assets/10_rviz%E9%85%8D%E7%BD%AE%E4%BF%9D%E5%AD%98.png)然后，`launch`文件中 Rviz 的启动配置添加参数:`args`,值设置为`-d 配置文件路径`

```xml
<launch>
    <param name="robot_description" textfile="$(find 包名)/urdf/urdf/urdf01_HelloWorld.urdf" />
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find 报名)/config/rviz/show_mycar.rviz" />
</launch>
```

再启动时，就可以包含之前的组件配置了，使用更方便快捷。

### URDF   XML语法详解

的标签用于描述机器人模型，机器人模型可能较为复杂，但是 ROS 的 URDF 中机器人的组成却是较为简单，可以主要简化为两部分:连杆(link标签) 与 关节(joint标签)，接下来我们就通过案例了解一下 URDF 中的不同标签:

- robot 根标签，类似于 launch文件中的launch标签
- link 连杆标签
- joint 关节标签
- gazebo 集成gazebo需要使用的标签

关于gazebo标签，后期在使用 gazebo 仿真时，才需要使用到，用于配置仿真环境所需参数，比如: 机器人材料属性、gazebo插件等，但是该标签不是机器人模型必须的，只有在仿真时才需设置

#### robot

urdf 中为了保证 xml 语法的完整性，使用了`robot`标签作为根标签，所有的 link 和 joint 以及其他标签都必须包含在 robot 标签内,在该标签内可以通过 name 属性设置机器人模型的名称

#### link

urdf 中的 link 标签用于描述机器人某个部件(也即刚体部分)的**外观和物理属性**，比如: 机器人底座、轮子、激光雷达、摄像头...每一个部件都对应一个 link, 在 link 标签内，可以设计该部件的形状、尺寸、颜色、惯性矩阵、碰撞参数等一系列属性

##### 1.属性

- name ---> 为连杆命名

##### 2.子标签

- visual ---> 描述外观(对应的数据是可视的)
  - geometry 设置连杆的形状
    - 标签1: box(盒状)
      - 属性:size=长(x) 宽(y) 高(z)
    - 标签2: cylinder(圆柱)
      - 属性:radius=半径 length=高度
    - 标签3: sphere(球体)
      - 属性:radius=半径
    - 标签4: mesh(为连杆添加皮肤)
      - 属性: filename=资源路径(格式:**package://<packagename>/<path>/文件**)
  - origin 设置偏移量与倾斜弧度
    - 属性1: xyz=x偏移 y便宜 z偏移
    - 属性2: rpy=x翻滚 y俯仰 z偏航 (单位是弧度)
  - metrial 设置材料属性(颜色)
    - 属性: name
    - 标签: color
      - 属性: rgba=红绿蓝权重值与透明度 (每个权重值以及透明度取值[0,1])
- collision ---> 连杆的碰撞属性
- Inertial ---> 连杆的惯性矩阵

在此，只演示`visual`使用。

##### 3.案例

**需求:**分别生成长方体、圆柱与球体的机器人部件

```xml
    <link name="base_link">
        <visual>
            <!-- 形状 -->
            <geometry>
                <!-- 长方体的长宽高 -->
                <!-- <box size="0.5 0.3 0.1" /> -->
                <!-- 圆柱，半径和长度 -->
                <!-- <cylinder radius="0.5" length="0.1" /> -->
                <!-- 球体，半径-->
                <!-- <sphere radius="0.3" /> -->

            </geometry>
            <!-- xyz坐标 rpy翻滚俯仰与偏航角度(3.14=180度 1.57=90度) -->
            <origin xyz="0 0 0" rpy="0 0 0" />
            <!-- 颜色: r=red g=green b=blue a=alpha -->
            <material name="black">
                <color rgba="0.7 0.5 0 0.5" />
            </material>
        </visual>
    </link>
```

#### joint

urdf 中的 joint 标签用于描述机器人**关节的运动学和动力学属性，还可以指定关节运动的安全极限**，机器人的两个部件(分别称之为 parent link 与 child link)以"关节"的形式相连接，不同的关节有不同的运动形式: 旋转、滑动、固定、旋转速度、旋转角度限制....,比如:安装在底座上的轮子可以360度旋转，而摄像头则可能是完全固定在底座上。

joint标签对应的数据在模型中是不可见的

##### 1.属性

- name ---> 为关节命名
- type ---> 关节运动形式
  - continuous: 旋转关节，可以绕单轴无限旋转
  - revolute: 旋转关节，类似于 continues,但是有旋转角度限制
  - prismatic: 滑动关节，沿某一轴线移动的关节，有位置极限
  - planer: 平面关节，允许在平面正交方向上平移或旋转
  - floating: 浮动关节，允许进行平移、旋转运动
  - fixed: 固定关节，不允许运动的特殊关节

##### 2.子标签

- parent(必需的)

  parent link的名字是一个强制的属性：

  - link:父级连杆的名字，是这个link在机器人结构树中的名字。

- child(必需的)

  child link的名字是一个强制的属性：

  - link:子级连杆的名字，是这个link在机器人结构树中的名字。

- origin

  - 属性: xyz=各轴线上的偏移量 rpy=各轴线上的偏移弧度。

- axis

  - 属性: xyz用于设置围绕哪个关节轴运动。

##### 3.案例

**需求:**创建机器人模型，底盘为长方体，在长方体的前面添加一摄像头，摄像头可以沿着 Z 轴 360 度旋转。

**URDF文件示例如下:**

```xml
<!-- 
    需求: 创建机器人模型，底盘为长方体，
         在长方体的前面添加一摄像头，
         摄像头可以沿着 Z 轴 360 度旋转

 -->
<robot name="mycar">
    <!-- 底盘 -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.2 0.1" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="blue">
                <color rgba="0 0 1.0 0.5" />
            </material>
        </visual>
    </link>

    <!-- 摄像头 -->
    <link name="camera">
        <visual>
            <geometry>
                <box size="0.02 0.05 0.05" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="red">
                <color rgba="1 0 0 0.5" />
            </material>
        </visual>
    </link>

    <!-- 关节 -->
    <joint name="camera2baselink" type="continuous">
        <parent link="base_link"/>
        <child link="camera" />
        <!-- 需要计算两个 link 的物理中心之间的偏移量 -->
        <origin xyz="0.2 0 0.075" rpy="0 0 0" />
        <axis xyz="0 0 1" />
    </joint>

</robot>
```

**launch文件示例如下:**

```xml
<launch>

    <param name="robot_description" textfile="$(find urdf_rviz_demo)/urdf/urdf/urdf03_joint.urdf" />
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find urdf_rviz_demo)/config/helloworld.rviz" /> 

    <!-- 添加关节状态发布节点 -->
    <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />
    <!-- 添加机器人状态发布节点 -->
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />
    <!-- 可选:用于控制关节运动的节点 -->
    <node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" />

</launch>
```

PS:

1.状态发布节点在此是必须的:

```xml
    <!-- 添加关节状态发布节点 -->
    <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />
    <!-- 添加机器人状态发布节点 -->
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />
```

2.关节运动控制节点(可选)，会生成关节控制的UI，用于测试关节运动是否正常。

```xml
    <!-- 可选:用于控制关节运动的节点 -->
    <node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" />
```

##### 4.base_footprint优化urdf

前面实现的机器人模型是半沉到地下的，因为默认情况下: 底盘的中心点位于地图原点上，所以会导致这种情况产生，可以使用的优化策略，将初始 link 设置为一个尺寸极小的 link(比如半径为 0.001m 的球体，或边长为 0.001m 的立方体)，然后再在初始 link 上添加底盘等刚体，这样实现，虽然仍然存在初始link半沉的现象，但是基本可以忽略了。这个初始 link 一般称之为 base_footprint

```xml
<!--

    使用 base_footprint 优化

-->
<robot name="mycar">
    <!-- 设置一个原点(机器人中心点的投影) -->
    <link name="base_footprint">
        <visual>
            <geometry>
                <sphere radius="0.001" />
            </geometry>
        </visual>
    </link>

    <!-- 添加底盘 -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.2 0.1" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="blue">
                <color rgba="0 0 1.0 0.5" />
            </material>
        </visual>
    </link>

    <!-- 底盘与原点连接的关节 -->
    <joint name="base_link2base_footprint" type="fixed">
        <parent link="base_footprint" />
        <child link="base_link" />
        <origin xyz="0 0 0.05" />
    </joint>

    <!-- 添加摄像头 -->
    <link name="camera">
        <visual>
            <geometry>
                <box size="0.02 0.05 0.05" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="red">
                <color rgba="1 0 0 0.5" />
            </material>
        </visual>
    </link>
    <!-- 关节 -->
    <joint name="camera2baselink" type="continuous">
        <parent link="base_link"/>
        <child link="camera" />
        <origin xyz="0.2 0 0.075" rpy="0 0 0" />
        <axis xyz="0 0 1" />
    </joint>

</robot>
```

launch 文件内容不变。

##### 5.遇到问题以及解决

**问题1:**

命令行输出如下错误提示

```
UnicodeEncodeError: 'ascii' codec can't encode characters in position 463-464: ordinal not in range(128)
[joint_state_publisher-3] process has died [pid 4443, exit code 1, cmd /opt/ros/melodic/lib/joint_state_publisher/joint_state_publisher __name:=joint_state_publisher __log:=/home/rosmelodic/.ros/log/b38967c0-0acb-11eb-aee3-0800278ee10c/joint_state_publisher-3.log].
log file: /home/rosmelodic/.ros/log/b38967c0-0acb-11eb-aee3-0800278ee10c/joint_state_publisher-3*.log
```

rviz中提示坐标变换异常，导致机器人部件显示结构异常

**原因:**编码问题导致的

**解决:**去除URDF中的中文注释

**问题2:**[ERROR] [1584370263.037038]: Could not find the GUI, install the 'joint_state_publisher_gui' package

**解决:**`sudo apt install ros-noetic-joint-state-publisher-gui`

#### URDF工具

在 ROS 中，提供了一些工具来方便 URDF 文件的编写，比如:

- `check_urdf`命令可以检查复杂的 urdf 文件是否存在语法问题
- `urdf_to_graphiz`命令可以查看 urdf 模型结构，显示不同 link 的层级关系

当然，要使用工具之前，首先需要安装，安装命令:`sudo apt install liburdfdom-tools`

##### 1.check_urdf 语法检查

进入urdf文件所属目录，调用:`check_urdf urdf文件`，如果不抛出异常，说明文件合法,否则非法

##### 2.urdf_to_graphiz 结构查看

进入urdf文件所属目录，调用:`urdf_to_graphiz urdf文件`，当前目录下会生成 pdf 文件

### URDF优化_xacro

[6.4 URDF优化_xacro · Autolabor-ROS机器人入门课程《ROS理论与实践》零基础教程](http://www.autolabor.com.cn/book/ROSTutorials/di-6-zhang-ji-qi-ren-xi-tong-fang-zhen/64-fang-zhen-urdf-rviz-yu-gazebo-zong-he-ying-yong.html)

前面 URDF 文件构建机器人模型的过程中，存在若干问题。

> 问题1:在设计关节的位置时，需要按照一定的公式计算，公式是固定的，但是在 URDF 中依赖于人工计算，存在不便，容易计算失误，且当某些参数发生改变时，还需要重新计算。
>
> 问题2:URDF 中的部分内容是高度重复的，驱动轮与支撑轮的设计实现，不同轮子只是部分参数不同，形状、颜色、翻转量都是一致的，在实际应用中，构建复杂的机器人模型时，更是易于出现高度重复的设计，按照一般的编程涉及到重复代码应该考虑封装。
>
> ......

如果在编程语言中，可以通过变量结合函数直接解决上述问题，在 ROS 中，已经给出了类似编程的优化方案，称之为:**Xacro**

#### **概念**

Xacro 是 XML Macros 的缩写，Xacro 是一种 XML 宏语言，是可编程的 XML。

#### **原理**

Xacro 可以声明变量，可以通过数学运算求解，使用流程控制控制执行顺序，还可以通过类似函数的实现，封装固定的逻辑，将逻辑中需要的可变的数据以参数的方式暴露出去，从而提高代码复用率以及程序的安全性。

#### **作用**

较之于纯粹的 URDF 实现，可以编写更安全、精简、易读性更强的机器人模型文件，且可以提高编写效率。

### Rviz中控制机器人模型运动

[6.5.1 Arbotix使用流程 · Autolabor-ROS机器人入门课程《ROS理论与实践》零基础教程](http://www.autolabor.com.cn/book/ROSTutorials/di-6-zhang-ji-qi-ren-xi-tong-fang-zhen/66-rviz-kong-zhi-ji-qi-ren-mo-xing-yun-dong/661-arbotixshi-yong-liu-cheng.html)

通过 URDF 结合 rviz 可以创建并显示机器人模型，不过，当前实现的只是静态模型，如何控制模型的运动呢？在此，可以调用 Arbotix 实现此功能。

------

#### 简介

**Arbotix:**Arbotix 是一款控制电机、舵机的控制板，**并提供相应的 ros 功能包**，这个功能包的功能不仅可以驱动真实的 Arbotix 控制板，它还提供一个差速控制器，通过接受速度控制指令更新机器人的 joint 状态，从而帮助我们实现机器人在 rviz 中的运动。

这个差速控制器在 arbotix_python 程序包中，完整的 arbotix 程序包还包括多种控制器，分别对应 dynamixel 电机、多关节机械臂以及不同形状的夹持器。

## 机器人导航(仿真)

导航是机器人系统中最重要的模块之一，比如现在较为流行的服务型室内机器人，就是依赖于机器人导航来实现室内自主移动的，本章主要就是介绍仿真环境下的导航实现，主要内容有:

- 导航相关概念
- 导航实现:机器人建图(SLAM)、地图服务、定位、路径规划....以可视化操作为主。
- 导航消息:了解地图、里程计、雷达、摄像头等相关消息格式。

### 概述

在ROS中机器人导航(Navigation)由多个功能包组合实现，ROS 中又称之为导航功能包集，关于导航模块，官方介绍如下:

```
一个二维导航堆栈，它接收来自里程计、传感器流和目标姿态的信息，并输出发送到移动底盘的安全速度命令。
```

更通俗的讲: 导航其实就是机器人自主的从 A 点移动到 B 点的过程。

#### 导航模块简介

