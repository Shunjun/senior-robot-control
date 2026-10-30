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
   至少 32 字节随机值；管理员密码使用 Argon2 哈希。缺少变量时 Compose 会直接失败。
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
管理员凭据泄露时生成新 Argon2 哈希并重建云服务。

## 升级与回退

升级前记录当前 Git 提交和固定镜像版本，备份卷，并构建新云镜像。先执行配置检查和
测试，再重建单个服务。失败时恢复上一提交及其固定镜像标签；若迁移已改变数据库，同时
恢复升级前的 `cloud-data` 备份。禁止用 `latest`，也不要在回退时复用已损坏的数据卷。
