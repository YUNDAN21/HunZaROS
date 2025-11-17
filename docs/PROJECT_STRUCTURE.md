# 项目结构说明（面向新手）

> 本文档以通俗语言介绍本 ROS/catkin 工作区的整体架构、主要模块与常用文件路径，并给出常见的编译、运行与可视化操作说明。适合刚接触本项目或对 ROS 不熟悉的开发者快速上手。

目录

- 总体概览
- 工作区目录说明（哪些是源码，哪些是构建产物）
- 主要模块（mbot / limo / hector_slam）与关键文件路径
- 快速构建与运行（最小步骤）
- 依赖关系图与可视化
- 常见问题与排查
- 后续建议

---

## 总体概览（一句话）

这是一个基于 ROS（catkin）组织的机器人项目工作区，源代码放在 `src/`，通过 `catkin_make`/`catkin build` 构建后在 `devel/` 和 `build/` 产生相应的开发空间和中间文件。项目由若干独立的 ROS 包（package）组成，每个包实现一个清晰的功能（例如导航、SLAM、仿真、识别等）。

## 工作区目录说明（把它当成地图）

- `/home/yundan/limo_ws/src/` — 源代码根目录，也是 catkin workspace 的源码空间（Most important）
  - `/home/yundan/limo_ws/src/CMakeLists.txt` — catkin 工作区的顶层 CMake 列表，用于构建所有包
  - `src/src/` 下是具体的包目录（例如 `mbot_navigation`, `mbot_slam`, `limo` 等）
- `/home/yundan/limo_ws/build/` — 构建时产生的中间文件（不要手动修改这些文件）
- `/home/yundan/limo_ws/devel/` — 编译后生成的开发空间（源入后可直接运行本地生成的节点和脚本）

注意：开发时主要修改 `src/` 下的包；构建后要 `source devel/setup.bash` 才能使用新编译的包。

## 每个包中的关键文件（在 ROS 中常见）

- `package.xml` — 包的元信息：包名、版本、维护者与依赖（例如 `src/src/mbot_navigation/package.xml`）
- `CMakeLists.txt` — 编译规则（告诉 catkin 如何编译该包）
- `launch/` — (若存在) 一键启动若干节点的 launch 文件
- `src/` — 源代码（C++ 或 Python 节点）
- `scripts/` 或 `nodes/` — 可直接执行的脚本（通常为 Python）

## 主要模块与对应文件（按功能分组）

下面列出项目中的主要模块、用途和可打开的代表文件路径，便于你在 VS Code 中定位。

1. mbot 系列 — 感知与导航相关（机器人功能）

- 用途：实现地图构建、路径规划、探索和识别等功能
- 代表包与路径：
  - `mbot_navigation`：`/home/yundan/limo_ws/src/src/mbot_navigation/package.xml`、`.../CMakeLists.txt`
  - `mbot_explore`：`/home/yundan/limo_ws/src/src/mbot_explore/package.xml`
  - `mbot_slam`：`/home/yundan/limo_ws/src/src/mbot_slam/package.xml`（包含 hector_slam 子包集合）
  - `mbot_recognition`：`/home/yundan/limo_ws/src/src/mbot_recognition/package.xml`

2. limo 系列 — 机器人描述与 Gazebo 仿真

- 用途：包含机器人 URDF/xacro 描述、以及仿真插件和 launch
- 代表包与路径：
  - `limo_description`：`/home/yundan/limo_ws/src/src/limo/limo_description/package.xml`
  - `limo_gazebo_sim`：`/home/yundan/limo_ws/src/src/limo/limo_gazebo_sim/package.xml`

3. hector_slam 系列 — 第三方 SLAM 工具箱

- 用途：hector 提供一套 2D SLAM、地图保存、可视化与地图服务工具，项目中以该工具实现 SLAM 功能
- 常见子包（均在 `src/src/mbot_slam/hector_slam/` 下）示例文件：
  - `hector_mapping`：`.../hector_mapping/package.xml`
  - `hector_map_tools`：`.../hector_map_tools/package.xml`
  - `hector_map_server`：`.../hector_map_server/package.xml`
  - `hector_geotiff` / `hector_geotiff_plugins`：用于保存地理 tiff 图像和扩展插件
  - `hector_slam`：一个 metapackage，聚合并依赖上面多个子包

简短比喻：

- mbot = 机器人大脑（感知 + 决策）
- limo = 机器人的外壳与试验台（实体模型 + 仿真）
- hector_slam = 工具箱（为 SLAM 提供现成能力）

## 快速编译与运行（一步到位的最小流程）

1. 进入工作区根目录：

```bash
cd /home/yundan/limo_ws
```

2. 构建（常用）：

```bash
catkin_make
# 或者根据你使用的工具改为: catkin build
```

3. 构建完成后，加载开发空间环境：

```bash
source devel/setup.bash
```

4. 运行某个 launch（假设包提供了某个 launch）示例：

```bash
roslaunch mbot_navigation some_launch_file.launch
```

5. 常用调试命令：

- 列话题：`rostopic list`
- 查看节点：`rosnode list`
- 打印某话题消息：`rostopic echo /topic_name`
- 可视化运行时节点-话题关系：`rqt_graph`

## 依赖关系图（我为你生成的）

- DOT 源文件：`/home/yundan/limo_ws/src/docs/dependency_graph.dot`
- 渲染输出（PNG）：`/home/yundan/limo_ws/src/docs/dependency_graph.png`（如果你已运行 dot 命令则文件应已生成）
- 渲染命令（如果还未运行）：

```bash
dot -Tpng src/docs/dependency_graph.dot -o src/docs/dependency_graph.png
dot -Tsvg  src/docs/dependency_graph.dot -o src/docs/dependency_graph.svg
```

说明：依赖图把各个包作为节点并以有向边表示依赖（A -> B 表示 A 依赖 B）。我把外部常见依赖（如 `roscpp`, `nav_msgs`, `tf`）标记为灰色虚线节点以示上下文。

## 常见问题与排查要点

- 找不到包或运行时提示 package not found：确认已 `source devel/setup.bash`，或确认包目录下有 `package.xml`。
- catkin_make 编译失败：查看终端错误输出，通常是缺少 `package.xml` 中声明的依赖或系统库未安装（使用 apt 安装缺失的系统依赖）。
- Launch 文件启动失败：查看传入参数是否齐全或依赖节点是否在同一 ROS Master 下。

## 后续建议（我可以帮你做）

- 我可以把 `dependency_graph.png` 嵌入 `src/README.md`，让仓库首页显示结构图。
- 我可以为每个包生成一个更详细的清单（列出 `launch/`, `src/`, `msg/`, `srv/` 等）并写入 `src/docs/packagelist.md`。
- 我可以演示一次完整从构建到启动 SLAM 的命令序列并把运行日志贴出来，帮助你实际验证功能是否正常。

---

如果你希望我把这份文档直接加入到项目 README（例如 `src/README.md`）或需要我生成更详细的每包清单，请告诉我接下来要做哪一步。

作者注：本文档由项目源码自动扫描和人工整理混合生成，包含直接可打开的路径，方便在 VS Code 中逐一查看实现文件。
