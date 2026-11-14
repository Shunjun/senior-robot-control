# 部署资产索引

| 目标 | 操作指南 | 本目录资产 |
| --- | --- | --- |
| 云端 | [云端部署](../docs/operations/deploy.md) | `Dockerfile.cloud`、`compose.yaml`、`compose.sh`、`env.example`、`Caddyfile`、`coturn/` |
| Jenkins 云端发布 | [云端部署：Jenkins](../docs/operations/deploy.md#jenkins-本机部署) | `compose.jenkins.yaml`、`jenkins-deploy.sh`、`validate-jenkins-env.py`、`redact-logs.py`；流水线入口在根目录 `Jenkinsfile` |
| 实车 native / 已有 ROS 容器 | [实车部署](../docs/operations/vehicle-deploy.md) | 新部署不套用本目录 service 模板，由 `src/senior_control/vehicle_deploy.py` 动态生成到统一目录 |
| Linux 仿真 | [仿真部署](../docs/operations/simulation.md) | 使用 ROS 仿真包和 systemd 用户服务，不使用云 Compose 或实车 system service 模板 |

`senior-agent.service` 仅保留给显式 `--profile` 的旧布局兼容（//Compatible），不是现代 `--control-dir` 默认部署入口。不要从模板推导新部署的 `/etc`、`/var/lib` 或 `/run` 目录。文件位置保持不变，现有 Compose/Jenkins 构建引用继续使用原路径；本页只分类导航，不复制各指南的操作步骤。
