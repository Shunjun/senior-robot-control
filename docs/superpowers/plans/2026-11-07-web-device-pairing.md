# 网页端设备配对 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 管理员在设备列表直接生成配对码，小车兑换后出现在列表中。

**Architecture:** 设备列表展示「添加设备」面板，复用现有管理员 HTTP 客户端。云端继续负责签发与原子兑换，车端继续通过现有 CLI 保存设备凭据。列表沿用现有 5 秒轮询和手动刷新。

**Tech Stack:** React、TypeScript、CSS Modules、Vitest、FastAPI 现有配对接口。

**Spec:** [HTTP 与 WebSocket 协议](../../protocol/http-ws.md)的配对接口，以及下方用户流程和约束。

## 用户流程与约束

1. 管理员登录后在设备列表点击「添加设备」，点击「生成配对码」。
2. 页面显示一次性配对码、剩余有效时间和车端操作说明。有效期以响应中的 `expires_in` 为准，倒计时从响应到达时估算。到期清除显示并提供重新生成入口。
3. 小车运行既有 `python -m senior_control.agent.main --pair`，在提示符输入配对码。设备兑换后由列表轮询或手动刷新发现。设备在线状态以 Agent 建连为准。
4. 配对码只保存在页面短期状态。关闭面板、登出和到期清除显示。已签发的码按云端原有一次性规则和有效期管理。
5. 同一面板的一次提交在请求完成前锁定按钮。授权过期引导重新登录，网络或限流错误提供明确提示。演示模式隐藏真实配对入口。页面沿用现有组件样式与键盘访问方式。

## Task 22A：接入配对码 API

**涉及文件：** `web/src/api/client.ts`、`web/src/features/devices/devices.test.tsx`，或现有 HTTP 客户端测试文件。

- [x] 先写失败测试，断言 `POST /api/pairing-codes` 使用同源 Cookie 和现有 CSRF 请求头，解析 `code` 与正整数 `expires_in`，区分 401、403、429 和异常响应。
- [x] 在 `web` 执行 `npm test -- --run` 的定向测试，记录失败断言。
- [x] 在 `ControlApi` 和 `HttpControlApi` 增加 `createPairingCode`，沿用现有错误类型。检查演示实现与测试替身，保持其已有行为。
- [x] 重跑定向测试与 `npm run typecheck`，复审会话和临时码边界。

## Task 22B：设备列表提供配对入口

**涉及文件：** `web/src/App.tsx`、`web/src/features/devices/DeviceList.tsx`、同目录的配对面板与 CSS Modules、`web/src/features/devices/devices.test.tsx`。

- [x] 先写失败测试，覆盖空列表入口、点击生成、等待期间重复点击、过期和登出清除、401 回到登录、429 后恢复、演示模式隐藏入口。
- [x] 实现面板和回调。用当前域名构造车端命令示例，并将配对码与命令分别显示，车端在交互提示里输入配对码。
- [x] 在 `web` 执行 `npm test -- --run`、`npm run typecheck` 和 `VITE_DEMO_DATA=0 npm run build`。
- [x] 独立审查键盘操作、窄屏布局、页面关闭时的异步响应，以及日志和存储中的凭据边界。

## Task 22C：测试环境验收

- [x] 以隔离的测试 HTTPS 地址，在浏览器验证登录、生成配对码、过期提示与设备列表刷新。隔离 HTTPS 环境采用生产前端构建与临时数据库。
- [x] 用隔离的测试 Agent 或已配置的 Linux 车端兑换一次性码，记录设备出现和 Agent 上线两项结果。记录测试域名、构建提交和日志位置，保护凭据。
- [x] 验证重复兑换与过期兑换被服务端拒绝，并确认原有设备状态保持可用。
- [x] 提交前按实际修改范围运行仓库检查，记录 Backend Actions 的 Python 3.10 / 3.12 工作流状态。独立提交配对功能和相关测试。

本任务验收边界为网页签发、车端兑换和设备在线观察。生产运动权限、定位与实车安全按各自验收任务推进。

## 验证记录

- 本地 Web 共 135 项通过，TypeScript 检查与生产构建通过。
- 本地复跑 Backend Actions 步骤，Python 3.10 和 3.12 各 1,118 项通过、4 项跳过。
- 隔离 HTTPS 验收使用 Google Chrome、真实 FastAPI 与车端 CLI、Agent WSS 及受限 IPC 夹具。离线和在线状态均观察到，重复和过期兑换返回 409，过期后页面清除配对码。构建 `index-qqKGaVYH.js`，SHA256 `f54af115c466ae2acec9bec9c4b68862636cafcaebf71147077050c08d9c1b15`。本次日志位于 `/tmp/sr22c.0bYSXp/check.log` 和同目录的 `server.log`。
- 公网环境上线由部署后的验收覆盖。实车系统部署继续在剩余工作清单中跟踪。
