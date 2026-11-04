# 自托管部署

## 地址与边界

- **云主机地址**：公网 DNS 指向运行 Compose 的云主机。浏览器和机器人都使用
  `https://<cloud-domain>`，TURN 使用 `<turn-domain>`。
- **机器人宿主机地址**：运行 ROS 2、硬件驱动和 `senior-agent.service` 的 Linux 地址。
- **虚拟机地址**：仅用于仿真；它与宿主机、云主机是不同网络端点。跨机器连接时不得填写
  `localhost`，应填写目标机器可达的 DNS 或地址。

Compose 只运行一个云进程（其中包含中继与登录限流器）、Caddy 和 coturn。ROS、DDS
及机器人代理不进入 Compose，也不向公网发布。

当前机器人 agent 的进程入口尚未装配 connector、数据库和 ROS IPC；执行 unit 会以非零
状态失败。入口完成并经过 Linux 集成验证前，不得启用 `senior-agent.service`。

## 准备和 TLS

1. 将 `deploy/env.example` 复制为 `deploy/.env`，替换所有占位符。两个共享密钥分别生成
   至少 32 字节随机值；管理员密码使用项目生成的 scrypt 哈希。缺少变量时 Compose 会直接失败。
2. 将云域名的 A/AAAA 记录指向云主机。Caddy 为 HTTPS 自动申请和续期证书。
3. 为 TURN 域名准备证书，将 `fullchain.pem` 与 `privkey.pem` 只读放在
   `deploy/secrets/turn/`。TURN 保持 REST 临时凭据方案，禁止配置长期用户名密码。
4. 先运行 `docker compose -f deploy/compose.yaml config --quiet` 检查配置，再由运维人员
   在变更窗口启动。构建镜像不会下载模型权重。

公网防火墙仅开放 TCP 80/443/5349、UDP 3478/5349 和 UDP 49160–49200。不要开放 DDS
端口。HTTP 80 仅用于 ACME 与跳转；业务经 TLS 443，TURN TLS 经 5349。

## 数据、备份与恢复

`cloud-data` 保存 SQLite 数据，`caddy-data` 保存 TLS 状态，`caddy-config` 保存 Caddy
运行配置；网页来自只读 `web/dist`。每日在暂停写入或 SQLite 在线备份条件下备份
`cloud-data`，并加密备份管理员恢复资料与 TURN 证书。恢复演练应在隔离主机验证数据库、
登录、机器人列表和证书。

## 凭据撤销

在控制台删除机器人会撤销设备令牌并关闭连接。怀疑云端共享密钥泄露时，替换
`SENIOR_SECRET` 会让现有会话和令牌失效；随后重新配对设备。TURN 泄露时替换
`TURN_SHARED_SECRET` 并重建 coturn，旧的临时凭据在其短期过期前也会因新密钥失效。
管理员凭据泄露时生成新 scrypt 哈希并重建云服务。

## 升级与回退

升级前记录当前 Git 提交和固定镜像版本，备份卷，并构建新云镜像。先执行配置检查和
测试，再重建单个服务。失败时恢复上一提交及其固定镜像标签；若迁移已改变数据库，同时
恢复升级前的 `cloud-data` 备份。禁止用 `latest`，也不要在回退时复用已损坏的数据卷。

## Jenkins 本机部署

仓库根目录的 `Jenkinsfile` 在所选 Linux 节点上构建网页和云镜像，并在同一台机器的
本地 Docker daemon 上部署；不会通过 SSH 部署到其他主机。默认节点标签为
`linux-docker`，默认持久目录为 `/opt/senior-robot-control`，域名固定为
`rosrouter.doomain.cn`。节点需要 Node.js/npm、Docker Compose v2、curl、flock、
Git、Python 3 和可以访问本地 Docker 的权限；部署目录须由 Jenkins 用户持久读写。确保 DNS 指向
该节点，并开放 TCP 80/443，让 Caddy 完成证书申请。部署前先备份数据库。

在 Jenkins 中创建 Pipeline from SCM 任务，选择此仓库和要部署的分支（如 `main`），
首次运行以建立任务参数。之后 Jenkins 每约 5 分钟检查该分支的提交变化并自动部署。
如果已有 Git webhook 触发构建，可移除 Jenkinsfile 中的 `pollSCM` 触发器，改用 webhook。
配置以下三项：

1. 准备符合上述要求的 Linux 节点，并设置 `DOCKER_AGENT` 标签（默认 `linux-docker`）
   与 `DEPLOY_ROOT`（默认 `/opt/senior-robot-control`）。
2. 创建 Jenkins **Secret file** 凭据，内容至少包含 `SENIOR_SECRET` 与
   `ADMIN_PASSWORD_HASH` 两行，使用 `deploy/jenkins.env.example` 中的格式；每个值必须完整
   用单引号包裹，保留哈希中的 `$` 原样。仅允许该示例列出的变量，不允许重复项、
   占位符或多行值。将凭据 ID 填入
   `ENV_CREDENTIAL_ID`。不要将文件或密钥提交到仓库，且应限制任务及节点访问权限。
3. 将 `ACME_EMAIL` 设置为有效邮箱，并确认域名 DNS、端口和部署目录。执行任务将部署
   当前检出的提交。

默认只启动 cloud 和 Caddy。如需 TURN，勾选 `ENABLE_TURN`，在 Secret file 中另外
设置 `TURN_LISTEN_IP`、`TURN_EXTERNAL_IP`、`TURN_REALM` 和 `TURN_SHARED_SECRET`，
并将 TURN 证书放入部署目录下的 `turn/fullchain.pem` 和 `turn/privkey.pem`；按需
开放前述 TURN 端口。TURN 的配置不影响默认云服务部署。

每次构建生成以提交和构建号命名的独立 release 和镜像；三个 Compose 卷以固定项目名
持久保留。重建容器后，流水线通过 HTTPS 检查页面，并确认受保护的 API 返回 401，
成功后原子更新 `current` 标记。该检查不是完整功能测试。失败时保留 release 和容器
日志，`current` 不变，但运行中的容器可能已切换到新版本；失败时输出容器状态和最近的
cloud/Caddy 日志，并遮盖凭据文件中的值。流水线不会自动恢复旧版或
数据库。应先查看 Jenkins 日志和 `docker compose` 服务日志，确认数据库迁移及兼容性，
再由运维人员决定恢复方案。**不要盲目降级代码或自动恢复数据库**；如需恢复备份，先
停止写入并按经过验证的数据库恢复流程操作。`current` 只是最后一次健康版本的记录，
并不保证失败后正在运行的版本与它一致。

在已安装本项目 Python 依赖的环境中，可用项目函数交互式生成管理员哈希：

```sh
python -c 'from getpass import getpass; from senior_control.cloud.auth import hash_password; print(hash_password(getpass("管理员密码: ")))'
```

将输出的整行放入 Secret file 的 `ADMIN_PASSWORD_HASH='...'`，不要在日志或版本库中
保存生成的哈希。常规 `deploy/.env` 同样须使用单引号包裹完整哈希，避免 Compose 将
`$` 当作变量展开。

Jenkins 专用 Compose 为 cloud 和 Caddy 使用内部网络 `172.29.231.0/24`，仅信任
Caddy 的固定地址 `172.29.231.2`；Caddy 另连默认网络以申请证书。若该网段与节点
已有网络冲突，部署前同步修改 `deploy/compose.jenkins.yaml` 中的子网、网关、cloud 地址、Caddy 地址与
`TRUSTED_PROXY`，并保证后两者一致。
