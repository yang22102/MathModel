# 无人机区域划分与搜索系统

本模块提供两种使用方式：**交互式网页界面** 和 **Python 脚本**，实现无人机多区域划分与自动搜索路径规划的可视化。

---

## 功能特性

### 区域划分算法
| 算法 | 说明 |
|------|------|
| 均匀网格划分 | 将搜索区域均等分成矩形网格，每架无人机负责一块 |
| 条带划分 | 按垂直条带分配，适合狭长区域 |
| 象限/扇形划分 | 以区域中心为原点，按角度扇形分区 |
| Voronoi 划分 | 根据无人机起始位置进行 Voronoi 最近邻划分，形成不规则区域 |

### 搜索路径算法
| 算法 | 说明 |
|------|------|
| S 形覆盖搜索 | 逐行 Z 字形扫描，完全覆盖负责区域 |
| 往复扫描搜索 | 逐列往复扫描（Boustrophedon），适合竖向分区 |
| 螺旋式搜索 | 从区域中心向外螺旋扩散搜索 |
| 随机游走搜索 | 随机顺序访问区域内格点，模拟随机搜索 |

---

## 使用方式

### 方式一：网页交互界面（推荐）

直接用浏览器打开 `index.html`：

```bash
# 用任意现代浏览器打开（无需服务器）
open index.html          # macOS
xdg-open index.html      # Linux
start index.html         # Windows
```

**操作说明：**
1. 左侧面板：调整区域大小、无人机数量、划分算法、搜索算法
2. 点击「应用区域划分」查看分区结果（彩色区域）
3. 点击「开始搜索」观看动画演示
4. 可在地图上点击添加/清除障碍物
5. 点击「保存图像」导出当前画面为 PNG

### 方式二：Python 脚本

**依赖安装：**
```bash
pip install numpy matplotlib scipy
```

**运行示例：**
```bash
# 默认参数（3架无人机，30x30网格，网格划分，S形搜索）
python drone_simulation.py

# 自定义参数
python drone_simulation.py --drones 4 --partition voronoi --search spiral

# 不弹出窗口，只保存图片
python drone_simulation.py --drones 5 --width 40 --height 40 --no-show

# 查看所有参数
python drone_simulation.py --help
```

**命令行参数：**
```
--width       网格宽度（默认 30）
--height      网格高度（默认 30）
--drones      无人机数量（默认 3，最多 8）
--partition   划分算法: grid | strip | quadrant | voronoi（默认 grid）
--search      搜索算法: zigzag | boustrophedon | spiral | random（默认 zigzag）
--obstacles   障碍物数量（默认 20）
--output      图片输出目录（默认 output）
--no-show     不弹出窗口
```

---

## 截图预览

运行 Python 脚本后，图片保存在 `output/` 目录：
- `partition_<算法>_<N>drones.png` — 区域划分示意图
- `search_<搜索>_<划分>_<N>drones.png` — 搜索路径图

---

## 文件说明

```
无人机搜索界面/
├── index.html           # 交互式网页界面（浏览器直接打开）
├── drone_simulation.py  # Python 可视化脚本
└── README.md            # 本说明文档
```
