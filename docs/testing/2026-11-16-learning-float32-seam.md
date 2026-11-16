# LaserScan float32完整周期识别返修

任务 `task_9aae08045c99 / ctx_c224884bf0ce`，HEAD `593e8f13f385892f0bd62cf75c593f7b07efd2da`。本轮只修改self_learning.py及定向测试，未改变1cm位置匹配、参考点保存、风险/未知空间政策或其它工作区改动；不启服务、ROS、仿真、车辆，未暂存、提交、推送。

根因：旧预算4×ulp(2π)只覆盖binary64运算舍入，而LaserScan.angle_increment经过float32 wire量化，端点跨度中该量化误差会随束数累积。例如720束周期采样，读回step=0.008726646192371845，2π−719×step=0.0087266948642295，已比step大约4.87e−8，旧代码误判非完整扫描。

修复使用float32半ULP作为步长量化误差q，正常数由步长指数和24位有效精度计算，非正规数按2^-149间隔计算；分别向(n−1)×step及n×step传播。完整周期判据仍为原来的“(n−1)步不超过一周且n步至少覆盖一周”，只给两侧各自的乘法项加入量化传播预算和binary64算术舍入预算。angle_min在相对跨度中抵消，不计作额外间隙。非有限算术跨度不会被无穷舍入预算误判为完整周期。

未增加欧氏连接距离、物理连接角阈值、扇区过滤或其它控制条件。周期采样与inclusive重复端点两种表示继续支持；只有相应实际完整扫描的首尾有效run才会相连。

## 先失败与定向验证

新增测试使用 `struct.unpack("f", struct.pack("f", value))` 构造真实float32元数据和驱动范围值，并按这些角度/距离生成端点，不以精确Python double假替wire值。

- 周期与inclusive两种表示，各覆盖720/1080/1440束，验证学习及运行首尾过滤。
- 三类负例分别覆盖270° FOV、少一个真实样本的非完整周期、低于完整步长量化区间的相邻float32值，每类均覆盖上述三种束数；不得跨端点连接。
- 保留密集长表面、局部扫描、稳定桥断连及固定参考只读过滤的必要回归。

未修复前：4 failed、11 passed、7 deselected（0.54秒），失败为3个周期输入及1440束inclusive输入。完整证据在[evidence目录](2026-11-16-learning-float32-seam-evidence)的learning-wire-float32-before.log/.xml。

产品最终版定向命令：

```sh
.venv/bin/pytest -q tests/unit/test_learning_surface_review.py tests/unit/test_learning_frozen_reference.py --junitxml=/tmp/wire-c224884b-final.xml
.venv/bin/mypy src/senior_control/safety/self_learning.py
.venv/bin/ruff check src/senior_control/safety/self_learning.py tests/unit/test_learning_surface_review.py
.venv/bin/ruff format --check src/senior_control/safety/self_learning.py tests/unit/test_learning_surface_review.py
```

结果：**35 passed，5.08秒**；严格mypy 1个源码通过；Ruff通过；2 files already formatted。日志与JUnit已复制到evidence目录，未重跑无关套件。

本轮未运行Linux内核IPC、ROS或真实驱动；struct测试证明float32表示处理，不代替目标环境验收。此前已定位的非空core运动场景仍因原停车扫掠未知区未成功，本轮没有修改或宣称解决该可用性限制，局部精度修复不能当成整条非空学习链通过。

## 启动快照与精确增量

启动快照 `/tmp/learning-wire-float32-start` 保存本轮源码、测试及三保护文件；本轮逐文件SHA和保护校验见 `2026-11-16-learning-float32-seam-fingerprints.json`，相对本轮启动的真实增量见 `2026-11-16-learning-float32-seam-owned.diff`。指纹不自引用，diff不包含自身或指纹文件；前轮失败、验证报告和指纹均保留历史内容。
