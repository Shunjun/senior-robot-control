# 2026-11-06 部署复盘与排障手册

本次通过公网域名访问机器人控制台并成功登录。未登录访问 `/api/robots` 返回 HTTP 401，验证了公网 Nginx、FRP、Caddy 和 Python 后端的请求链。Jenkins 发布状态以构建日志中的 `Deployment healthy` 和 `current` 指向为准。Certbot 已签发正式通配符证书，阿里云 DNS 插件及定时器路径已配置，模拟续期仍待通过。

## 拓扑与配置

| 位置 | 职责 |
| --- | --- |
| CentOS 7 公网中转机 | Nginx 监听公网 80/443，frps 提供控制端口及 HTTP/HTTPS 域名转发。现有 DERP 站点用精确域名匹配，后端为本机 33443。 |
| Debian 打包与部署机 | Jenkins 拉取代码、打包前端、构建 cloud 镜像并调用 Docker Compose。frpc 向 frps 注册域名。Caddy 服务静态文件，转发 API 与 WebSocket 到 cloud。 |
| 阿里云 DNS | 站点 A 记录指向公网中转机。Certbot DNS-01 通过 TXT 记录验证根域名和通配符。 |

HTTP 路径为 **公网 80 → Nginx → 中转机 frps 8080 → 部署机 frpc → 宿主机 HTTP_PORT → Caddy 80**。HTTPS 路径为 **公网 443 → Nginx 终止 TLS → 中转机 frps 8443 → 部署机 frpc → 宿主机 HTTPS_PORT → Caddy 443**。第二段 TLS 由 Nginx 验证。两台机器上的 8443 分别属于 frps 入口和 Docker 发布端口。

[统一环境模板](../../deploy/env.example) 同时适用于 Jenkins Secret file 和手动部署的 `deploy/.env`。用英文单引号包住完整值。Jenkinsfile 保留 `ENV_CREDENTIAL_ID` 和 `DEPLOY_ROOT`。部署配置字段如下。

| 字段 | 用途 |
| --- | --- |
| `DOMAIN` | 公网 DNS 名，示例 `rosrouter.doomain.cn`。Caddy、站点 Origin 和健康检查共享它。 |
| `ACME_EMAIL` | Caddy 注册 ACME 账户时使用的联系邮箱。 |
| `HTTP_PORT` / `HTTPS_PORT` | Docker 发布到宿主机的端口，本次为 8088 / 8443。容器使用 80 / 443。 |
| `SENIOR_SECRET` | 后端签发和校验凭据的密钥，至少 32 字节。重部署沿用原值。 |
| `ADMIN_PASSWORD_HASH` | 完整的 `scrypt$16384$8$1$<32 位十六进制盐>$<64 位十六进制哈希>`。登录时输入原密码。 |
| `ENABLE_TURN` | 默认为关闭。设为 `'true'` 时补齐四项 TURN 配置和证书。TURN 用于 WebRTC 中继。 |

## 踩坑记录

| 现象 | 处理与验证 |
| --- | --- |
| GitHub `Permission denied (publickey)` | 在执行 Git 的系统用户下生成专用密钥，公钥登记为仓库只读 Deploy key。用 `GIT_SSH_COMMAND` 显式指定密钥测试，再在 SSH config 或 Jenkins SCM 凭据固定身份。GitHub SSH 用户名为 `git`。 |
| 前端停在 npm 弃用警告附近 | `npm ci --include=dev` 按锁文件安装构建与测试依赖，包括 Playwright npm 包。浏览器由 `playwright install` 单独安装。用 npm debug 日志定位下载超时或安装脚本。 |
| Jenkins 缺少 `senior-control-env` | 创建任务可见的 Secret file，凭据 ID 与参数一致。SCM SSH 凭据与部署文件分别配置。 |
| `Credential file rejected` | 用统一模板和完整哈希，运行校验器读取字段或行号。诊断输出隐藏配置值。 |
| `/opt/senior-robot-control` 权限错误 | 由管理员创建项目目录并授权给 Jenkins 实际运行用户，用 `test -w` 验证。 |
| Docker Hub token 请求超时 | 单测 `auth.docker.io` 的 443 连通性和 `docker pull`。若需代理，把代理配置给 Docker daemon。 |
| 宿主机 80 已占用 | 通过 `ss` 和 `docker ps` 确认占用者。两种 Compose 部署共享可配置发布端口，FRP 指向实际端口。 |
| Caddy 日志报 `NXDOMAIN` | 补齐站点公网 DNS，指向中转机。用公网解析器查询。 |
| Caddy HTTP-01 返回 404 | 对比部署机 Caddy 8088、中转机 frps 8080 和公网 HTTP 的响应，保持 Host 和 URI。根路径 308 验证普通 HTTP，`served key authentication` 与 `certificate obtained successfully` 验证真实签发。 |
| TLS-ALPN 验证失败 | 公网 TLS 在 Nginx 终止。本次 Caddy 经 HTTP-01 成功签发。 |
| Nginx 502 与 `certificate chain too long` | 在 HTTPS 上游位置设 `proxy_ssl_verify_depth 3;`，保留 SNI、证书验证和 CA 信任配置。 `nginx -t` 通过后重载。公网 API 随后返回 401。 |
| Jenkins 健康检查失败但容器仍在运行 | Compose 在健康检查之前启动容器。先看容器日志、端口和本机 TLS，再用公网 API 验证。 `current` 记录最近一次通过检查的发布。 |

## Nginx 与 FRP 配置要点

中转机 frps 用 `bindPort` 接收 frpc 控制连接，`vhostHTTPPort = 8080` 和 `vhostHTTPSPort = 8443` 提供域名转发入口。frpc 使用独立的 `type = "http"` 和 `type = "https"` 条目，两条均注册 `customDomains = ["rosrouter.doomain.cn"]`，`localPort` 分别对应 8088 和 8443。frpc 与 Caddy 同机时使用 `localIP = "127.0.0.1"`。

Nginx 的 HTTP 站点把 Host 和原 URI 转给 `http://127.0.0.1:8080`，供普通跳转和 ACME HTTP-01 共用。HTTPS 通配符站点使用中转机证书，转发位置包含以下配置。现有 DERP 精确域名站点保留自身转发。

```nginx
proxy_pass https://127.0.0.1:8443;
proxy_ssl_server_name on;
proxy_ssl_name $host;
proxy_ssl_verify on;
proxy_ssl_verify_depth 3;
proxy_ssl_trusted_certificate /etc/pki/tls/certs/ca-bundle.crt;
proxy_set_header Host $host;
proxy_set_header X-Forwarded-Proto https;
proxy_http_version 1.1;
proxy_set_header Upgrade $http_upgrade;
proxy_set_header Connection "upgrade";
```

## 逐跳验证

在外部机器检查公网证书和后端。用实际中转机地址替换示例 IP。预期证书验证成功，未登录的 API 返回 401。

```bash
curl --noproxy '*' -v --max-time 15 \
  --resolve rosrouter.doomain.cn:443:192.0.2.10 \
  https://rosrouter.doomain.cn/api/robots
curl --noproxy '*' -i --max-time 10 http://rosrouter.doomain.cn/
```

在中转机测试 FRP HTTP 和 HTTPS。相同的 HTTPS 命令在 Debian 执行时，连接的是 Docker 发布的 Caddy 端口，借此对比两段握手。

```bash
curl --noproxy '*' -i --max-time 10 \
  -H 'Host: rosrouter.doomain.cn' http://127.0.0.1:8080/
curl --noproxy '*' -v --max-time 15 \
  --connect-to rosrouter.doomain.cn:443:127.0.0.1:8443 \
  https://rosrouter.doomain.cn/api/robots
sudo grep 'host: "rosrouter.doomain.cn"' /var/log/nginx/error.log | tail -n 5
```

在 Debian 查实际容器与证书日志。按请求时间对照日志，优先关注本轮签发结果。

```bash
sudo docker ps -a --filter name=senior-control-caddy-1
sudo docker logs --timestamps --tail 80 senior-control-caddy-1
sudo docker exec senior-control-caddy-1 printenv DOMAIN
```

## 证书续期和同步

中转机 Nginx 使用包含根域名与一级通配符的证书。部署机 Caddy 通过公网 HTTP 转发自动管理站点证书。CentOS 7 负责加载证书，Debian 负责 Certbot 签发。手动申请命令如下。

```bash
sudo certbot certonly --manual --preferred-challenges dns \
  -d doomain.cn -d '*.doomain.cn'
nslookup -type=TXT _acme-challenge.doomain.cn 223.5.5.5
nslookup -type=NS doomain.cn
```

`certonly` 保存证书和私钥。DNS-01 允许签发机器与 Nginx 分开。根域名和通配符可能要求同名的两条 TXT，两条记录同时保留到验证完成。权威 DNS 返回新值后，递归 DNS 仍可按旧 TTL 缓存。每次重跑使用当前挑战值。

自动化使用独立环境 `/opt/certbot-aliyun`，安装 Certbot 与第三方 `certbot-dns-aliyun`。专用阿里云 RAM 凭据在 Debian 的 `/etc/letsencrypt/aliyun.ini`，权限为 600。续期配置使用 `authenticator = dns-aliyun` 和 `dns_aliyun_propagation_seconds = 600`。系统级 `certbot.service` 通过 drop-in 调用独立环境的 Certbot，`certbot.timer` 定期检查是否进入续期窗口。600 秒用于 DNS 等待。当前定时器以 00:00 和 12:00 为基准，加入最多 12 小时的随机延迟，实际下次运行时间由 list-timers 显示。

```bash
sudo /opt/certbot-aliyun/bin/certbot renew --cert-name doomain.cn --dry-run
sudo systemctl show certbot.service -p DropInPaths -p ExecStart --no-pager
sudo systemctl list-timers --all certbot.timer
sudo journalctl -u certbot.service -n 30 --no-pager
```

`--dry-run` 使用 Let’s Encrypt staging，保留当前正式证书。遇到 staging TLS EOF 时先测 staging 的 `/directory`，再核对同一出网路径。Deploy hook 在成功续期后通过 SSH 同步 `fullchain.pem` 和 `privkey.pem`，在中转机执行 `nginx -t && systemctl reload nginx`。已验证 Debian 任务账号的免交互 SSH 登录。完整闭环以模拟续期通过、钩子同步成功及公网证书序列号更新为验证依据。

## CLI 速查

| 命令或参数 | 用途 |
| --- | --- |
| `openssl rand -hex 32` | 生成 32 字节随机数据并输出为 64 位十六进制文本，用于初始化后端密钥。 |
| `git ls-remote <SSH URL> HEAD` | 验证远程仓库读取权限。 |
| `ssh-keygen -t ed25519 -f <路径> -C <备注>` | 生成密钥对。 `ssh-copy-id -i <公钥> 用户@主机` 把公钥登记到远端用户的 `authorized_keys`。 |
| `ssh -i <私钥> -o IdentitiesOnly=yes -o BatchMode=yes` | 指定身份并验证免交互连接。 `StrictHostKeyChecking=yes` 要求已验证的主机密钥。 |
| `sudo install -d -o jenkins -g "$(id -gn jenkins)" -m 0750 <路径>` | 创建项目目录并设置所有者、主组和权限。 |
| `ss -ltnp 'sport = :8443'` | 看监听 TCP 端口、进程和 PID。 `docker ps --format` 看容器端口映射。 |
| `docker logs --timestamps --since 15m --tail 80 <容器>` | 按时间和行数查看容器日志。 `-f` 跟踪实时输出。 |
| `docker compose ls` / `docker compose -p <项目> -f <文件> stop` | 定位并停止整套服务，保留容器和数据卷。 |
| `curl --noproxy '*' -v --max-time 15` | 绕过代理，显示 TLS 和 HTTP 细节，并限制总耗时。 |
| `curl --resolve 域名:443:IP https://域名/` | 固定入口 IP，继续验证域名和 TLS。 |
| `curl --connect-to 域名:443:127.0.0.1:8443 https://域名/api/robots` | 直连特定入口，保留 URL、SNI 和证书验证。 |
| `curl -H 'Host: 域名' http://127.0.0.1:8080/` | 验证 frps HTTP 域名路由。 |
| `nginx -T` / `nginx -t` / `systemctl reload nginx` | 查看有效配置、校验配置、加载新配置。 |
| `systemctl cat` / `show -p ExecStart` / `daemon-reload` | 看 unit 和覆盖文件、确认实际命令、加载 unit 改动。 |
| Vim `v` / `V` / `y` / `p` / `:wq` | 字符选择、整行选择、复制、粘贴、保存退出。 |

## 旁路由排障经验

ImmortalWrt 在 OpenClash 停用时，公网 IP 可达，直查公网 DNS 可用，而 dnsmasq 因 `noresolv=1` 缺少上游返回 REFUSED。显式配置可达上游后恢复订阅更新。开启时，用 `nslookup 域名 127.0.0.1:7874` 直测 OpenClash，返回 198.18.0.x 代表 Fake-IP 正常响应。普通 53 与 7874 的结果差异指向 DNS 接管链路。用 LuCI 的 dnsmasq 转发接管设置处理启停，保留关停后的可用上游。 `ip -4 route get`、`ping`、`nslookup`、`netstat -lnp` 和 `/tmp/openclash.log` 分别验证路由、IP、DNS、监听和实际出站规则。

## 参考

- [当前云端部署手册](deploy.md)
- [FRP HTTP 与 HTTPS 转发](https://gofrp.org/en/docs/features/http-https/)
- [Nginx HTTPS 上游配置](https://nginx.org/en/docs/http/ngx_http_proxy_module.html#proxy_ssl_verify_depth)
- [Let’s Encrypt 验证方式](https://letsencrypt.org/docs/challenge-types/)
- [Certbot 续期说明](https://eff-certbot.readthedocs.io/en/stable/using.html#renewing-certificates)
