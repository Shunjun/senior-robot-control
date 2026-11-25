import { expect, test } from "@playwright/test";
import { writeFile } from "node:fs/promises";
import { fileURLToPath } from "node:url";

const baseURL = process.env.PLAYWRIGHT_BASE_URL;
test.skip(
  baseURL === undefined,
  "PLAYWRIGHT_BASE_URL is required; only a static preview is used",
);
const fixture = fileURLToPath(
  new URL("./fixtures/control-fixture.js", import.meta.url),
);

for (const viewport of [
  { width: 1440, height: 1000 },
  { width: 390, height: 844 },
  { width: 844, height: 390 },
]) {
  test.describe(`${viewport.width}px fixture`, () => {
    test.use({
      viewport,
      hasTouch: viewport.width === 390,
      isMobile: viewport.width === 390,
    });
    test(`safe production-client workspace at ${viewport.width}px`, async ({
      page,
    }, testInfo) => {
      const followup: Record<string, unknown> = {};
      async function boundsAndHit(locator: ReturnType<typeof page.locator>) {
        await locator.scrollIntoViewIfNeeded();
        const result = await locator.evaluate((element) => {
          const r = element.getBoundingClientRect();
          return {
            x: r.x, y: r.y, width: r.width, height: r.height,
            hit: element.contains(document.elementFromPoint(r.x + r.width / 2, r.y + r.height / 2)),
            hitTarget: document.elementFromPoint(r.x + r.width / 2, r.y + r.height / 2)?.outerHTML.slice(0, 400),
            text: element.textContent,
          };
        });
        expect(result.hit, JSON.stringify(result)).toBe(true);
        return result;
      }
      async function typography(name: string) {
        const metrics = await page.evaluate(() => {
          const probe = document.createElement("span");
          probe.className = "text-xs";
          document.body.appendChild(probe);
          const minimum = Number.parseFloat(getComputedStyle(probe).fontSize);
          probe.remove();
          const sizes: Array<{ text: string; size: number; tag: string }> = [];
          const walker = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT);
          while (walker.nextNode()) {
            const node = walker.currentNode;
            const element = node.parentElement;
            if (!node.textContent?.trim() || !element?.checkVisibility() || element.closest(".app-sr-only, .settings-sr-only, script, style")) continue;
            if (!element.getBoundingClientRect().width) continue;
            sizes.push({ text: node.textContent.trim().slice(0, 80), size: Number.parseFloat(getComputedStyle(element).fontSize), tag: element.tagName });
          }
          return { minimum, sizes, width: innerWidth, scrollWidth: document.documentElement.scrollWidth };
        });
        expect(metrics.sizes.filter(s => s.size < metrics.minimum), name).toEqual([]);
        expect(metrics.scrollWidth, name).toBeLessThanOrEqual(viewport.width);
        followup[`typography-${name}`] = metrics;
      }
      async function expandedTypography(name: string) {
        const opened = await page.locator("details").evaluateAll(elements => elements.filter(e => e.checkVisibility() && !(e as HTMLDetailsElement).open).map(e => { (e as HTMLDetailsElement).open = true; e.setAttribute("data-typography-opened", ""); return true; }));
        await typography(`${name}-details-${opened.length}`);
        await page.locator("details[data-typography-opened]").evaluateAll(elements => elements.forEach(e => { (e as HTMLDetailsElement).open = false; e.removeAttribute("data-typography-opened"); }));
      }
      async function capture(name: string) {
        await expandedTypography(name);
        await page.screenshot({ path: testInfo.outputPath(`${name}.png`), fullPage: true });
      }
      await page.setViewportSize(viewport);
      await page.route("**/*", (route) => {
        const url = new URL(route.request().url());
        const root = new URL(baseURL!);
        return url.origin === root.origin &&
          (url.pathname === "/" ||
            url.pathname.startsWith("/assets/") ||
            url.pathname === "/control-fixture.js")
          ? route.continue()
          : route.abort();
      });
      await page.addInitScript({ path: fixture });
      // Extend only the safe socket fixture; no production or real service state.
      await page.addInitScript({ content: `
        const SafeSocket = window.WebSocket;
        window.WebSocket = class extends SafeSocket {
          emit(frame) {
            if (frame.type === 'snapshot' && window.__recoveryProbe) {
              const ui = frame.payload.vehicle_ui;
              ui.confirmed_estop_status = 'software_latched';
              ui.mode = 'estop';
              ui.allowed_actions = window.__recoveryProbe === 'allowed'
                ? ['estop_reset', 'fault_recover'] : [];
            }
            super.emit(frame);
          }
        };
      ` });
      await page.goto(baseURL!);
      await expect(
        page.getByRole("heading", { name: "还没有选择车辆" }),
      ).toBeVisible();
      await expect(
        page.getByRole("button", { name: "急停", exact: true }),
      ).toHaveCount(0);
      await page.getByRole("button", { name: "设置", exact: true }).click();
      await expect(page.getByRole("heading", { name: "车辆设置", exact: true })).toBeVisible();
      followup.emptyLogout = await boundsAndHit(page.getByRole("button", { name: "退出登录", exact: true }));
      await capture("empty-settings");
      await page.getByRole("button", { name: "返回控制 ↗", exact: true }).click();
      await expect(page.getByRole("heading", { name: "还没有选择车辆", exact: true })).toBeVisible();
      await expect(
        page.getByRole("navigation", { name: "设备列表" }),
      ).toHaveCount(0);
      await page
        .getByRole("button", { name: "选择车辆", exact: true })
        .first()
        .click();
      const refresh = page.getByRole("button", { name: "刷新设备", exact: true });
      const closeMenu = page.getByRole("button", { name: "关闭设备列表", exact: true });
      const refreshBox = await boundsAndHit(refresh);
      const closeBox = await boundsAndHit(closeMenu);
      expect(refreshBox.y).toBeGreaterThan(closeBox.y + closeBox.height);
      // Refresh belongs beside Pair, with Close in the header; their hit regions do not intersect.
      expect(refreshBox.y >= closeBox.y + closeBox.height).toBe(true);
      followup.menu = { refresh: refreshBox, close: closeBox };
      await refresh.click();
      await capture("followup-menu-empty");
      await page.mouse.click(2, 2);
      await expect(
        page.getByRole("navigation", { name: "设备列表" }),
      ).toHaveCount(0);
      await page
        .getByRole("button", { name: "选择车辆", exact: true })
        .first()
        .click();
      await page.keyboard.press("Escape");
      await expect(
        page.getByRole("navigation", { name: "设备列表" }),
      ).toHaveCount(0);
      await page
        .getByRole("button", { name: "选择车辆", exact: true })
        .first()
        .click();
      await page.getByRole("link", { name: /安全测试车辆 A/ }).click();
      await expect(
        page.getByRole("navigation", { name: "设备列表" }),
      ).toHaveCount(0);
      await page.getByRole("button", { name: "切换车辆", exact: true }).click();
      await capture("followup-menu");
      const sceneBeforePair = await page.locator(".devices-robot").evaluate(element => {
        const rect = element.getBoundingClientRect();
        return { x: rect.x + scrollX, y: rect.y + scrollY, width: rect.width, height: rect.height };
      });
      await page.getByRole("button", { name: "配对新车", exact: true }).click();
      await page.getByRole("button", { name: "生成配对码", exact: true }).click();
      await expect(page.getByLabel("配对码", { exact: true })).toHaveText("ABCD234567");
      expect(await page.locator(".devices-robot").evaluate(element => {
        const rect = element.getBoundingClientRect();
        return { x: rect.x + scrollX, y: rect.y + scrollY, width: rect.width, height: rect.height };
      })).toEqual(sceneBeforePair);
      const copyCode = page.getByRole("button", { name: "复制配对码", exact: true });
      followup.copy = await boundsAndHit(copyCode);
      await capture("followup-pair-code");
      followup.copyStyle = await copyCode.evaluate(element => {
        const style = getComputedStyle(element);
        return { background: style.backgroundColor, radius: style.borderRadius, color: style.color };
      });
      await page.getByRole("button", { name: "复制命令", exact: true }).focus();
      await page.keyboard.press("Tab");
      await expect(page.getByRole("button", { name: "关闭配对", exact: true })).toBeFocused();
      await copyCode.click();
      await expect(page.getByRole("status")).toHaveText(/已复制|无法自动复制/);
      await page.getByRole("button", { name: "关闭配对", exact: true }).click();
      const map = page.getByRole("application", { name: "地图" });
      await expect(map).toBeVisible();
      await page.getByRole("button", { name: "导航与定位", exact: true }).click();
      const navigation = page.getByRole("region", { name: "Home 与导航" });
      const navigationOrder = await navigation.locator("button").evaluateAll(buttons => buttons.filter(button => button.checkVisibility()).map(button => button.textContent?.trim()));
      expect(navigationOrder).toEqual(["在地图上设置目标", "返回 Home", "取消导航", "校准车辆位置", "编辑 Home"]);
      await expect(navigation.getByRole("radio")).toHaveCount(0);
      await expect(navigation.getByRole("textbox")).toHaveCount(0);
      await page.getByRole("button", { name: "在地图上设置目标", exact: true }).click();
      await expect(page.getByRole("region", { name: "地图位姿选取" })).toBeVisible();
      await expect(navigation).toBeHidden();
      await expect(map).toHaveAttribute("data-picking", "true");
      const navMap = await map.boundingBox();
      await expect(page.getByRole("button", { name: "确认目标", exact: true })).toHaveCount(0);
      await map.click({ position: { x: navMap!.width * .5, y: navMap!.height * .5 } });
      await expect(page.getByText("再点击地图，设置朝向", { exact: true })).toBeVisible();
      await expect(page.getByRole("button", { name: "确认目标", exact: true })).toHaveCount(0);
      await map.click({ position: { x: navMap!.width * .6, y: navMap!.height * .45 } });
      await expect(page.getByRole("button", { name: "确认目标", exact: true })).toBeVisible();
      await page.getByRole("button", { name: "重选", exact: true }).click();
      await page.getByRole("region", { name: "地图位姿选取" }).getByRole("button", { name: "取消", exact: true }).click();
      await expect(navigation).toBeVisible();
      await expect(map).toHaveAttribute("data-picking", "false");
      await page.getByRole("button", { name: "地图管理", exact:true }).click();
      const maps = page.getByRole("region", { name: "活动地图管理", exact:true });
      await expect(maps.getByText("安全测试地图", {exact:true})).toBeVisible();
      await expect(page.getByLabel("新地图名称", {exact:true})).toHaveCount(0);
      const mapOrder = await maps.locator("button").evaluateAll(buttons => buttons.filter(button => button.checkVisibility()).map(button => button.textContent?.trim()));
      expect(mapOrder).toEqual(["导出地图包", "切换", "删除", "＋ 新建地图", "导入地图包"]);
      await maps.getByRole("button", { name:"＋ 新建地图", exact:true }).click();
      await expect(page.getByRole("dialog", {name:"新建地图",exact:true})).toBeVisible();
      await page.getByRole("dialog", {name:"新建地图",exact:true}).getByRole("button",{name:"取消",exact:true}).click();
      await expect(page.getByLabel("新地图名称",{exact:true})).toHaveCount(0);
      await page.getByRole("button", { name: "手动驾驶", exact: true }).click();
      const stick = page.getByRole("group", { name: "驾驶摇杆" });
      await expect(map).toBeVisible();
      await page.evaluate(() => {
        const w = window as unknown as Record<string, unknown>;
        w.mapOwner = document.querySelector("[role=application]");
        w.stickOwner = document.querySelector("[aria-label=驾驶摇杆]");
      });
      const measurement = page.getByTestId("measured-speed");
      const standardReadout = await page.evaluate((mobile) => {
        const probe = document.createElement("span");
        probe.style.fontSize = mobile ? "var(--text-3xl)" : "var(--text-4xl)";
        document.body.appendChild(probe);
        const size = getComputedStyle(probe).fontSize;
        probe.remove();
        return size;
      }, viewport.width === 390);
      const readoutSize = await measurement.evaluate(el => getComputedStyle(el).fontSize);
      expect(readoutSize).toBe(standardReadout);
      followup.readout = { actual: readoutSize, standard: standardReadout };
      const readingBounds = await page.getByLabel("车辆测量读数", { exact: true }).evaluate(element => {
        const parent = element.getBoundingClientRect();
        return Array.from(element.children).map(child => {
          const rect = child.getBoundingClientRect();
          return { text: child.textContent, contained: rect.left >= parent.left && rect.right <= parent.right };
        });
      });
      expect(readingBounds.every(reading => reading.contained)).toBe(true);
      followup.readingBounds = readingBounds;
      await expect(
        page.getByRole("button", { name: "释放控制权", exact: true }),
      ).toHaveCount(0);
      const camera = page.getByRole("switch", { name: "摄像头" });
      await expect(camera).toHaveAttribute("aria-checked", "true");
      await camera.click();
      await expect(page.getByRole("region", { name: "实时视频" })).toHaveCount(0);
      if (viewport.width === 390) await camera.tap();
      else {
        await camera.focus();
        await page.keyboard.press("Space");
      }
      expect(
        await camera.evaluate((el) => {
          const track = el.children[1]!.getBoundingClientRect(),
            dot = el.children[1]!.firstElementChild!.getBoundingClientRect();
          return Math.abs(
            (track.top + track.bottom - dot.top - dot.bottom) / 2,
          );
        }),
      ).toBeLessThan(0.5);
      await page.getByRole("button", { name: "交换主次画面" }).click();
      async function checkScale(stage: string) {
        const scale = await page.getByLabel("地图比例尺", { exact: true }).evaluate(element => {
          const line = element.querySelector("i")!;
          const rect = line.getBoundingClientRect();
          const zoom = Number(document.querySelector('[role="application"][aria-label="地图"]')!.getAttribute("data-zoom"));
          const meters = Number(element.textContent!.replace("m", "").trim());
          const style = getComputedStyle(line);
          const dock = document.querySelector('[aria-label="视图底部工具"] > :nth-child(2)')!.getBoundingClientRect();
          const group = element.parentElement!.getBoundingClientRect();
          const map = document.querySelector('[role="application"][aria-label="地图"]')!.getBoundingClientRect();
          return { meters, zoom, inlineWidth: parseFloat(line.style.width), width: rect.width,
            boxSizing: style.boxSizing, borderLeft: style.borderLeftWidth, borderRight: style.borderRightWidth,
            scaleRight: element.getBoundingClientRect().right, groupRight: group.right, dockLeft: dock.left, mapRight: map.right };
        });
        expect(Math.abs(scale.inlineWidth - scale.meters * scale.zoom)).toBeLessThan(0.05);
        // The declared length includes the two 1px end borders (border-box).
        expect(Math.abs(scale.width - scale.inlineWidth)).toBeLessThan(0.05);
        expect(scale.boxSizing).toBe("border-box");
        // In camera-primary mode the toolbar stays inside its miniature map.
        expect(scale.scaleRight).toBeLessThanOrEqual(scale.mapRight);
        expect(scale.groupRight).toBeLessThanOrEqual(scale.mapRight);
        followup[`scale-${stage}`] = scale;
        if (viewport.width === 390) await page.screenshot({ path: testInfo.outputPath(`scale-${stage}.png`) });
      }
      await checkScale("before");
      const zoomBefore = Number(await map.getAttribute("data-zoom"));
      await page.getByRole("button", { name: "放大", exact: true }).click();
      expect(Number(await map.getAttribute("data-zoom"))).toBeGreaterThan(
        zoomBefore,
      );
      await checkScale("after");
      await page.getByRole("button", { name: "适应窗口", exact: true }).click();
      await expect(
        page
          .getByRole("button", { name: "放大", exact: true })
          .locator("..")
          .locator("span"),
      ).toBeVisible();
      await page.screenshot({
        path: testInfo.outputPath("mini-map.png"),
        fullPage: true,
      });
      // The enlarged auxiliary text wraps the mini-map toolbar; click clear map space.
      await map.click({ position: { x: 12, y: 12 } });
      expect(
        await page.evaluate(() =>
          document
            .querySelector("[data-primary]")
            ?.getAttribute("data-primary"),
        ),
      ).toBe("video");
      expect(
        await page.evaluate(() => {
          const w = window as unknown as Record<string, unknown>;
          return {
            map: w.mapOwner === document.querySelector("[role=application]"),
            stick:
              w.stickOwner === document.querySelector("[aria-label=驾驶摇杆]"),
          };
        }),
      ).toEqual({ map: true, stick: true });
      await page.getByRole("switch", { name: "摄像头" }).click();
      await expect(page.getByRole("region", { name: "实时视频" })).toHaveCount(
        0,
      );
      await page.getByRole("button", { name: "状态", exact: true }).click();
      await expect(page.getByText("Roll 横滚")).toBeVisible();
      await expect(page.getByText("Pitch 俯仰")).toBeVisible();
      await page.getByRole("button", { name: "切换速度单位" }).click();
      await expect(
        page
          .getByRole("region", { name: "手动驾驶" })
          .getByTestId("measured-speed").getByText("1.80", { exact: false }),
      ).toBeVisible();
      await page.getByRole("button", { name: "状态", exact: true }).click();
      const footer = page.getByRole("region", { name: "视图底部工具" });
      await footer.scrollIntoViewIfNeeded();
      const geometry = await page.evaluate(() => {
        const footer = document.querySelector('[aria-label="视图底部工具"]')!;
        const card = footer.closest("[data-primary]")!.getBoundingClientRect();
        const bounds = footer.getBoundingClientRect();
        const selectors = [
          '[aria-label="放大"]',
          'nav[aria-label="车辆场景"]',
          '[aria-label="地图图例"]',
        ];
        return {
          edge: card.bottom - bounds.bottom,
          operationsClear: footer.closest("[data-primary]")!.querySelector(".workspace-operations")!.getBoundingClientRect().bottom <= footer.querySelector('[aria-label="地图图例"]')!.getBoundingClientRect().top,
          toolsClear:
            footer
              .querySelector('[aria-label="放大"]')!
              .parentElement!.getBoundingClientRect().right <=
            footer
              .querySelector('[aria-label="地图图例"]')!
              .getBoundingClientRect().left,
          touchTargets: Array.from(
            footer.querySelectorAll("button[aria-label], nav button"),
          ).every((button) => {
            const box = button.getBoundingClientRect();
            return box.width >= 44 && box.height >= 44;
          }),
          contained: selectors.every((selector) => {
            const box = footer.querySelector(selector)!.getBoundingClientRect();
            return box.top >= bounds.top && box.bottom <= bounds.bottom + 1;
          }),
        };
      });
      expect(geometry.contained).toBe(true);
      expect(geometry.toolsClear).toBe(true);
      expect(geometry.operationsClear).toBe(true);
      // The approved phone dock is 140px, with 32px visible items; 44px boxes would change the layout.
      followup.visibleTargetGeometry = geometry.touchTargets;
      expect(geometry.edge).toBeCloseTo(viewport.width === 390 ? 10 : 12, 0);
      const bottoms = await footer.locator(":scope > *").evaluateAll(elements => elements.map(element => element.getBoundingClientRect().bottom));
      expect(Math.max(...bottoms) - Math.min(...bottoms)).toBeLessThanOrEqual(1);
      await expect(page.getByText("稳定结构", { exact:true })).toBeVisible();
      const moreLayers = page.getByRole("button", { name: "更多图层", exact: true });
      followup.layers = await boundsAndHit(moreLayers);
      await capture("followup-layers");
      const layerCallsBefore = await page.evaluate(() =>
        (window as unknown as { __wheelFixture: { calls: unknown[] } }).__wheelFixture.calls,
      );
      await moreLayers.click();
      await expect(page.getByText("暂未实现：独立候选层 / 空闲证据层 / 线段拟合", { exact: true })).toBeVisible();
      const layerCallsAfter = await page.evaluate(() =>
        (window as unknown as { __wheelFixture: { calls: unknown[] } }).__wheelFixture.calls,
      );
      // Only the independent periodic GET /api/robots may race with this local action.
      const withoutPolls = (calls: unknown[]) => calls.filter(call => {
        const entry = call as { http?: string; method?: string };
        return entry.http !== "/api/robots" || entry.method !== "GET";
      });
      expect(withoutPolls(layerCallsAfter)).toEqual(withoutPolls(layerCallsBefore));
      followup.layerRequests = { before: withoutPolls(layerCallsBefore).length, after: withoutPolls(layerCallsAfter).length };
      await capture("followup-layers-feedback");
      if (viewport.width === 844) {
        await page.screenshot({ path: testInfo.outputPath("followup-layers-viewport.png") });
        await page.evaluate(() => window.scrollTo(0, 0));
        await page.screenshot({ path: testInfo.outputPath("followup-layers-feedback-viewport.png") });
      }
      await page.evaluate(() => window.scrollTo(0, 0));
      const readingToastOverlap = await page.getByLabel("车辆测量读数", { exact: true }).evaluate(element => {
        const toast = document.querySelector(".workspace-toast");
        if (!toast) return false;
        const a = element.getBoundingClientRect(), b = toast.getBoundingClientRect();
        return a.left < b.right && b.left < a.right && a.top < b.bottom && b.top < a.bottom;
      });
      expect(readingToastOverlap).toBe(false);
      followup.readingToastOverlap = readingToastOverlap;
      await expandedTypography("drive");
      await page.screenshot({
        path: testInfo.outputPath("drive.png"),
        fullPage: true,
      });
      await page
        .getByRole("button", { name: "获取控制权", exact: true })
        .click();
      await expect(stick).toHaveAttribute("aria-disabled", "false");
      await expect(
        page.getByRole("button", { name: "获取控制权", exact: true }),
      ).toHaveCount(0);
      await expect(
        page.getByRole("button", { name: "释放控制权", exact: true }),
      ).toHaveCount(1);
      await stick.scrollIntoViewIfNeeded();
      if (viewport.width !== 844) await expect(page.getByRole("button", { name: "急停", exact:true })).toBeInViewport({ ratio:1 });
      else {
        // Frozen landscape scrolls its ordinary header out; record it, then prove the real action remains reachable at the top.
        followup.landscapeHeaderAfterStickScroll = await page.locator("header").first().evaluate(element => ({ y:element.getBoundingClientRect().y, scrollY:window.scrollY, position:getComputedStyle(element).position }));
        await page.evaluate(() => scrollTo(0,0));
        await expect(page.getByRole("button", { name:"急停", exact:true })).toBeInViewport({ ratio:1 });
      }
      await page.screenshot({
        path: testInfo.outputPath("estop-scrolled.png"),
      });
      await stick.focus();
      await page.keyboard.down("ArrowUp");
      await page.keyboard.up("ArrowUp");
      await page.getByRole("button", { name: "日志", exact: true }).click();
      await expect(
        page.getByRole("log", { name: "车辆逐行日志" }),
      ).toBeVisible();
      await expect(page.locator("img")).toHaveCount(0);
      await expect(page.getByText(/安全 fixture：逐行日志/)).toBeVisible();
      await capture("logs");
      await page.getByRole("button", { name: "设置", exact: true }).click();
      await expect(page.getByLabel("低速线速度上限")).toHaveValue("1.8");
      await expect(page.getByLabel("高速角速度请求上限")).toBeVisible();
      await capture("followup-settings-labels");
      await page.getByLabel("低速线速度上限").fill("2.16");
      await page
        .getByRole("button", { name: "摄像头与预览", exact: true })
        .click();
      await page
        .getByRole("switch", { name: "显示摄像头预览", exact: true })
        .click();
      await capture("settings-camera");
      await page
        .getByRole("button", { name: "状态与性能", exact: true })
        .click();
      await capture("settings-status");
      await expect(
        page.getByText("车辆诊断与固定回波校准", { exact: true }),
      ).toBeVisible();
      await page.getByRole("button", { name: "日志配置", exact: true }).click();
      await expect(page.getByLabel(/^保留时间（小时）/)).toBeVisible();
      await capture("settings-logging");
      await page.getByRole("button", { name: "驾驶与操控", exact: true }).click();
      await expect(page.getByLabel("低速线速度上限")).toHaveValue("2.16");
      await page.getByRole("button", { name: "切换速度单位" }).click();
      await expect(page.getByLabel("低速线速度上限")).toHaveValue("0.6");
      await page.getByRole("button", { name: "保存驾驶设置" }).click();
      await expect(page.getByText(/车端已确认配置生效/)).toBeVisible();
      await page.getByRole("button", { name: "控制", exact: true }).click();
      await expect(map).toBeVisible();
      await expect(camera).toHaveAttribute("aria-checked", "true");
      await page.getByRole("button", { name: "任务管理", exact: true }).click();
      const taskCard = page.getByRole("region", { name: "任务管理", exact:true });
      const listOrder = await taskCard.locator("button").evaluateAll(buttons => buttons.filter(button => button.checkVisibility()).map(button => button.getAttribute("aria-label") ?? button.textContent?.trim()));
      expect(listOrder.slice(0,2)).toEqual(["新建任务", "等待队列"]);
      expect(listOrder.indexOf("立即执行")).toBeLessThan(listOrder.indexOf("编辑 fixture-route"));
      expect(listOrder.indexOf("编辑 fixture-route")).toBeLessThan(listOrder.indexOf("查看路线 fixture-route"));
      await page
        .getByRole("button", { name: "编辑 fixture-route", exact: true })
        .click();
      await page.getByRole("button", {name:"下一步",exact:true}).click();
      await expect(page.getByLabel("任务名称")).toHaveValue("定时巡检");
      const editorGeometry = await page.locator("[data-mode=tasks]").evaluate((workspace) => {
        const card = workspace.querySelector("[data-task-stage=editor]")!.parentElement!.parentElement!;
        const r = card.getBoundingClientRect(), w = workspace.getBoundingClientRect();
        return { x: r.x, width: r.width, left: r.left - w.left, right: w.right - r.right };
      });
      followup.editor = editorGeometry;
      if (viewport.width === 390) {
        expect(editorGeometry.width).toBe(340);
        expect(editorGeometry.left).toBe(12);
        expect(editorGeometry.right).toBe(12);
      } else expect(editorGeometry.width).toBe(315);
      await capture("followup-task-editor");
      const route = page.getByRole("list", { name: "任务航点顺序" });
      await route.getByRole("button", { name: "上移 C", exact: true }).click();
      await route.getByRole("button", { name: "上移 C", exact: true }).click();
      followup.reorder = await boundsAndHit(route.getByRole("button", { name: "上移 C", exact: true }));
      const submitTask = page.getByRole("button", { name: "保存任务", exact: true });
      followup.save = await boundsAndHit(submitTask);
      const overlappingTargets = await page.locator("[data-task-stage=editor]").evaluate(editor => {
        const buttons = Array.from(editor.querySelectorAll("button")).filter(button => {
          const r = button.getBoundingClientRect();
          // Closed details descendants can retain layout boxes without being painted.
          return button.checkVisibility() && r.width > 0 && r.height > 0;
        });
        return buttons.flatMap((button, index) => buttons.slice(index + 1).flatMap(other => {
          const a = button.getBoundingClientRect(), b = other.getBoundingClientRect();
          return a.left < b.right && b.left < a.right && a.top < b.bottom && b.top < a.bottom
            ? [[button.textContent, other.textContent]] : [];
        }));
      });
      expect(overlappingTargets).toEqual([]);
      followup.editorTargetOverlaps = overlappingTargets;
      await capture("followup-task-editor-save");
      await submitTask.click();
      await expect(page.getByText("车端已确认保存任务。")).toBeVisible();
      if (viewport.width === 1440) {
        await page
          .getByRole("button", { name: "新建任务", exact: true })
          .click();
        await expect(page.getByRole("button", { name: "下一步", exact:true })).toBeDisabled();
        // Add a real fixture map point to reach the independent editor; remove it before selecting saved A below.
        await map.click({position:{x:600,y:500}});
        await page.getByRole("button", { name:"下一步", exact:true }).click();
        await page.getByRole("list",{name:"任务航点顺序"}).getByRole("button",{name:/^移除 /}).click();
        await page.getByLabel("任务名称", { exact: true }).fill("东区巡检");
        await page.getByRole("button", { name: "高级", exact: true }).click();
        await page.getByText("高级计划与任务信息", {exact:true}).click();
        const taskId = await page
          .getByLabel("内部编号", { exact: true })
          .inputValue();
        expect(taskId).toMatch(/^[0-9a-f-]{36}$/);
        await page.getByText("高级航点编辑", { exact: true }).click();
        await page
          .getByRole("combobox", { name: "可用航点", exact: true })
          .selectOption("A");
        await page
          .getByRole("button", { name: "加入路线", exact: true })
          .click();
        await page
          .getByRole("button", { name: "保存任务", exact: true })
          .click();
        await expect(page.getByText("车端已确认保存任务。")).toBeVisible();
        await page
          .getByRole("button", { name: `编辑 ${taskId}`, exact: true })
          .click();
        await page.getByRole("button", {name:"下一步",exact:true}).click();
        await expect(page.getByLabel("任务名称", { exact: true })).toHaveValue(
          "东区巡检",
        );
        await page.getByRole("button", { name: "高级", exact: true }).click();
        await page.getByText("高级计划与任务信息", { exact: true }).click();
        await expect(
          page.getByLabel("内部编号", { exact: true }),
        ).toHaveAttribute("readonly", "");
        await page.getByLabel("任务名称", { exact: true }).fill("夜班巡检");
        await page
          .getByRole("button", { name: "保存任务", exact: true })
          .click();
        await expect(page.getByText("车端已确认保存任务。")).toBeVisible();
        await expect(page.getByText("夜班巡检", { exact: true })).toBeVisible();
        await page
          .getByRole("button", { name: `查看路线 ${taskId}`, exact: true })
          .click();
        await page.getByText("点位详情 · 夜班巡检", {exact:true}).click();
        await expect(
          page.getByRole("heading", { name: "任务路线 · 夜班巡检" }),
        ).toBeVisible();
        await page.screenshot({
          path: testInfo.outputPath("names-preview.png"),
          fullPage: true,
        });
        await page
          .getByRole("button", { name: "返回当前草稿", exact: true })
          .click();
        const mutations = await page.evaluate(() =>
          (
            window as unknown as {
              __wheelFixture: {
                calls: Array<{
                  type: string;
                  payload: { body: Record<string, unknown> };
                }>;
              };
            }
          ).__wheelFixture.calls.filter(
            (call) =>
              call.type === "task.create" || call.type === "task.update",
          ),
        );
        const created = mutations.find((call) => call.type === "task.create")!;
        const renamed = mutations.find(
          (call) =>
            call.type === "task.update" && call.payload.body.task_id === taskId,
        )!;
        expect(created.payload.body).toMatchObject({
          task_id: taskId,
          name: "东区巡检",
          waypoint_ids: ["A"],
        });
        expect(renamed.payload.body).toEqual({
          ...created.payload.body,
          name: "夜班巡检",
        });
      }
      // A saved definition reaches the list on the next snapshot, followed by its metadata read.
      const taskCardAfterSave = page.getByRole("region", { name: "任务管理", exact: true });
      const revisions = await page.evaluate(() => (window as unknown as {
        __wheelFixture: { tasks: { task_id: string; revision: number }[] };
      }).__wheelFixture.tasks.map(task => ({ id: task.task_id, revision: task.revision })));
      for (const task of revisions) {
        await expect(taskCardAfterSave.getByText(`${task.id} · r${task.revision}`, { exact: true })).toBeAttached();
      }
      const taskRows = taskCardAfterSave.getByRole("article");
      await expect(taskRows.getByText(/个点 · 定时/)).toHaveCount(await taskRows.count());
      const before = await page.evaluate(
        () =>
          (
            window as unknown as { __wheelFixture: { calls: unknown[] } }
          ).__wheelFixture.calls.filter(
            (call) => !Object.hasOwn(call as object, "http"),
          ).length,
      );
      await page
        .getByRole("button", { name: "立即执行", exact: true })
        .first()
        .click();
      await expect(
        page.getByText("暂未实现：立即执行已保存任务"),
      ).toBeVisible();
      expect(
        await page.evaluate(
          (start) =>
            (
              window as unknown as { __wheelFixture: { calls: unknown[] } }
            ).__wheelFixture.calls.filter(
              (call) => !Object.hasOwn(call as object, "http"),
            ).slice(start),
          before,
        ),
      ).toEqual([]);
      expect(
        await page.evaluate(
          () => document.documentElement.scrollWidth <= innerWidth,
        ),
      ).toBe(true);
      await page.evaluate(() => window.scrollTo(0, 0));
      await expandedTypography("tasks");
      await page.screenshot({
        path: testInfo.outputPath("tasks.png"),
        fullPage: true,
      });
      await page.evaluate(() => { (window as unknown as Record<string, unknown>).__recoveryProbe = "allowed"; });
      const reset = page.getByRole("button", { name: "复位软件急停", exact: true });
      const recover = page.getByRole("button", { name: "恢复执行故障", exact: true });
      await expect(reset).toBeEnabled();
      await expect(recover).toBeEnabled();
      const recovery = [];
      for (const button of [reset, recover]) {
        const bounds = await boundsAndHit(button);
        expect(bounds.width).toBeGreaterThanOrEqual(44);
        expect(bounds.height).toBeGreaterThanOrEqual(44);
        await button.focus();
        await page.keyboard.press("Tab");
        await page.keyboard.press("Shift+Tab");
        const style = await button.evaluate(e => {
          const s = getComputedStyle(e);
          return { radius: s.borderRadius, background: s.backgroundColor, border: s.borderColor,
            focus: e.matches(":focus-visible"), outline: s.outlineStyle, outlineWidth: s.outlineWidth };
        });
        expect(style.radius).toBe("9px");
        expect(style.background).toBe("rgba(0, 0, 0, 0)");
        expect(style.focus).toBe(true);
        expect(style.outline).toBe("solid");
        recovery.push({ bounds, style });
      }
      const estop = page.getByRole("button", { name: "急停", exact: true });
      const estopBounds = await boundsAndHit(estop);
      expect(estopBounds.hit).toBe(true);
      const headerButtons = await page.locator("header button").evaluateAll(elements => elements
        .filter(e => e.checkVisibility()).map(e => {
          const r = e.getBoundingClientRect();
          return { text: e.textContent, x: r.x, y: r.y, right: r.right, bottom: r.bottom };
        }));
      for (let a = 0; a < headerButtons.length; a++) for (let b = a + 1; b < headerButtons.length; b++) {
        const x = headerButtons[a]!, y = headerButtons[b]!;
        expect(Math.min(x.right, y.right) - Math.max(x.x, y.x) > 0.5 &&
          Math.min(x.bottom, y.bottom) - Math.max(x.y, y.y) > 0.5, `${x.text} overlaps ${y.text}`).toBe(false);
      }
      followup.recovery = { buttons: recovery, estop: estopBounds, headerButtons,
        estopBackground: await estop.evaluate(e => getComputedStyle(e).backgroundColor) };
      await page.evaluate(() => window.scrollTo(0, 0));
      await page.screenshot({ path: testInfo.outputPath("recovery-actions.png") });
      const recoveryCalls = () => page.evaluate(() => (window as unknown as {
        __wheelFixture: { calls: Array<{ type?: string }> }
      }).__wheelFixture.calls.filter(c => c.type === "safety.estop.reset" || c.type === "safety.fault.recover").length);
      const beforeDisabled = await recoveryCalls();
      await page.evaluate(() => { (window as unknown as Record<string, unknown>).__recoveryProbe = "disabled"; });
      await expect(reset).toBeDisabled();
      await expect(recover).toHaveCount(0);
      // Native disabled dispatch must remain inert, including keyboard input.
      await reset.dispatchEvent("click");
      await estop.focus();
      await page.keyboard.press("Tab");
      const afterDisabled = await recoveryCalls();
      expect(afterDisabled).toBe(beforeDisabled);
      followup.disabledRecovery = { requestsBefore: beforeDisabled, requestsAfter: afterDisabled,
        ...await reset.evaluate(e => ({ disabled: (e as HTMLButtonElement).disabled,
          opacity: getComputedStyle(e).opacity, height: e.getBoundingClientRect().height })) };
      await boundsAndHit(estop);
      await page.evaluate(() => window.scrollTo(0, 0));
      await page.screenshot({ path: testInfo.outputPath("recovery-disabled.png") });
      const calls = await page.evaluate(
        () =>
          (
            window as unknown as {
              __wheelFixture: {
                calls: Array<{ type?: string; payload?: unknown }>;
              };
            }
          ).__wheelFixture.calls,
      );
      await page.getByRole("button", { name: "设置", exact: true }).click();
      const logout = page.locator(".settings-card > header").getByRole("button", { name: "退出登录", exact: true });
      await expect(logout).toBeEnabled();
      followup.logout = await boundsAndHit(logout);
      await capture("settings-logout");
      await logout.click();
      await expect(page.getByRole("heading", { name: "登录控制台", exact: true })).toBeVisible();
      const deletes = await page.evaluate(() => (window as unknown as { __wheelFixture: { calls: Array<{ http?: string; method?: string }> } }).__wheelFixture.calls.filter(c => c.http === "/api/session" && c.method === "DELETE"));
      expect(deletes).toHaveLength(1);
      await capture("logged-out");
      await writeFile(testInfo.outputPath("followup-bounds.json"), JSON.stringify(followup, null, 2));
      expect(calls.some((call) => call.type === "control.release")).toBe(true);
      expect(
        calls.find((call) => call.type === "task.update")?.payload,
      ).toMatchObject({
        body: {
          waypoint_ids: ["C", "A", "B"],
          schedule: { kind: "cron", expression: "15 9 * * 1,3" },
          timezone: "Asia/Shanghai",
        },
      });
    });
  });
}
