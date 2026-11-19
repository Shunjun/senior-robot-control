# 独立云端

状态：仅架构骨架，可构建 Python wheel；未迁移实现，未安装到现有生产入口。
项目目录 `cloud/` 已确认，Python 导入名为 `cloud`，发行声明名为 `vehicle-cloud`。
本批没有 console script、服务启动函数或模拟业务返回值。根项目的安装、锁及运行入口保持现状。

## 职责与边界

拥有服务入口、认证、设备注册、WebSocket 转发与云快照缓存；只通过 contracts 与车端交换线上消息。不拥有车端连接实现、车内运行状态或 ROS 功能。
现有生产源仍在 `src/senior_control/cloud/ 及云端实际使用的认证、注册、存储适配`，由后续批次逐链迁移，避免复制实现。
只共享实际云车 WebSocket 使用的 `contracts/src/senior_contracts`；
车内 Unix IPC 与 ROS transport 独立，不能把内部状态加入共享包来绕过依赖边界。
依赖、配置安装和部署闭包在实际迁移时逐项确认，本骨架不声明尚未使用的业务依赖。

## 当前目录与后续落点

```text
cloud/
  pyproject.toml
  README.md
  src/cloud/__init__.py   # 仅导入标识，无运行功能
  tests/README.md           # 后续迁入测试的职责与来源
```

节点、算法与运行状态归 ROS 功能包。本项目的源码按本项目职责组织，
具体内部子模块名及跨进程协议调整留后续迁移清单审阅，不预造执行实现或新增公共技术项目。

## 最低验证

从仓库根执行 `uv build cloud --offline --out-dir /tmp/vehicle-architecture-skeleton-wheels`。
安装生成 wheel 后可从仓库外 `import cloud`；这只验证构建声明与导入布局，
不代表管理或云端生产链已经迁入。迁移阶段必须复用原编解码、设置、日志、监管与隔离回归，
核对原线上字段、省略和接受规则及现有安装入口。
