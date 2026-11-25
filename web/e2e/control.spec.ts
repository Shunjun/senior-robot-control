import { expect, test } from "@playwright/test";
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
      await page.goto(baseURL!);
      await expect(
        page.getByRole("heading", { name: "选择车辆，开始工作" }),
      ).toBeVisible();
      await expect(
        page.getByRole("button", { name: "急停", exact: true }),
      ).toHaveCount(0);
      await expect(
        page.getByRole("navigation", { name: "设备列表" }),
      ).toHaveCount(0);
      await page
        .getByRole("button", { name: "选择车辆", exact: true })
        .first()
        .click();
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
      const map = page.getByRole("application", { name: "地图" });
      const stick = page.getByRole("group", { name: "驾驶摇杆" });
      await expect(map).toBeVisible();
      await page.evaluate(() => {
        const w = window as unknown as Record<string, unknown>;
        w.mapOwner = document.querySelector("[role=application]");
        w.stickOwner = document.querySelector("[aria-label=驾驶摇杆]");
      });
      const measurement = page.locator("[class*=measurement]");
      expect(
        await measurement.evaluate((el) => getComputedStyle(el).fontSize),
      ).toBe(viewport.width === 390 ? "29px" : "42px");
      await expect(
        page.getByRole("button", { name: "释放控制权", exact: true }),
      ).toHaveCount(0);
      const camera = page.getByRole("switch", { name: "摄像头" });
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
      const zoomBefore = Number(await map.getAttribute("data-zoom"));
      await page.getByRole("button", { name: "放大", exact: true }).click();
      expect(Number(await map.getAttribute("data-zoom"))).toBeGreaterThan(
        zoomBefore,
      );
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
      await map.click({ position: { x: 40, y: 40 } });
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
          .getByText("1.80", { exact: false }),
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
      if (viewport.width === 390) expect(geometry.touchTargets).toBe(true);
      expect(geometry.edge).toBeCloseTo(viewport.width === 390 ? 10 : 12, 0);
      if (viewport.width === 390) {
        await page.getByText("地图图例", { exact: true }).click();
        await expect(page.getByText("稳定结构", { exact: true })).toBeVisible();
        await page.getByText("地图图例", { exact: true }).click();
      } else {
        const bottoms = await footer
          .locator(":scope > *")
          .evaluateAll((elements) =>
            elements.map((el) => el.getBoundingClientRect().bottom),
          );
        expect(Math.max(...bottoms) - Math.min(...bottoms)).toBeLessThanOrEqual(
          1,
        );
      }
      await page.evaluate(() => window.scrollTo(0, 0));
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
      await expect(
        page.getByRole("button", { name: "急停", exact: true }),
      ).toBeInViewport({ ratio: 1 });
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
      await page.getByRole("button", { name: "设置", exact: true }).click();
      await expect(page.getByLabel("低档线速度上限")).toHaveValue("1.8");
      await page.getByLabel("低档线速度上限").fill("2.16");
      await page
        .getByRole("button", { name: "摄像头预览", exact: true })
        .click();
      await page
        .getByRole("switch", { name: "显示摄像头预览", exact: true })
        .click();
      await page
        .getByRole("button", { name: "状态与性能", exact: true })
        .click();
      await expect(
        page.getByText("车辆诊断与固定回波校准", { exact: true }),
      ).toBeVisible();
      await page.getByRole("button", { name: "日志配置", exact: true }).click();
      await expect(page.getByLabel(/^保留时间（小时）/)).toBeVisible();
      await page.getByRole("button", { name: "驾驶操控", exact: true }).click();
      await expect(page.getByLabel("低档线速度上限")).toHaveValue("2.16");
      await page.getByRole("button", { name: "切换速度单位" }).click();
      await expect(page.getByLabel("低档线速度上限")).toHaveValue("0.6");
      await page.getByRole("button", { name: "保存驾驶设置" }).click();
      await expect(page.getByText(/车端已确认配置生效/)).toBeVisible();
      await page.getByRole("button", { name: "控制", exact: true }).click();
      await expect(map).toBeVisible();
      await expect(camera).toHaveAttribute("aria-checked", "true");
      await page.getByRole("button", { name: "任务管理", exact: true }).click();
      await page
        .getByRole("button", { name: "编辑 fixture-route", exact: true })
        .click();
      await expect(page.getByLabel("任务名称")).toHaveValue("定时巡检");
      const route = page.getByRole("list", { name: "任务航点顺序" });
      await route.getByRole("button", { name: "上移 C", exact: true }).click();
      await route.getByRole("button", { name: "上移 C", exact: true }).click();
      await page.getByRole("button", { name: "提交任务", exact: true }).click();
      await expect(page.getByText("车端已确认保存任务。")).toBeVisible();
      if (viewport.width === 1440) {
        await page
          .getByRole("button", { name: "新建任务", exact: true })
          .click();
        await page
          .getByRole("button", { name: "完成选点", exact: true })
          .click();
        await page.getByLabel("任务名称", { exact: true }).fill("东区巡检");
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
          .getByRole("button", { name: "提交任务", exact: true })
          .click();
        await expect(page.getByText("车端已确认保存任务。")).toBeVisible();
        await page
          .getByRole("button", { name: `编辑 ${taskId}`, exact: true })
          .click();
        await expect(page.getByLabel("任务名称", { exact: true })).toHaveValue(
          "东区巡检",
        );
        await expect(
          page.getByLabel("内部编号", { exact: true }),
        ).toHaveAttribute("readonly", "");
        await page.getByLabel("任务名称", { exact: true }).fill("夜班巡检");
        await page
          .getByRole("button", { name: "提交任务", exact: true })
          .click();
        await expect(page.getByText("车端已确认保存任务。")).toBeVisible();
        await expect(page.getByText("夜班巡检", { exact: true })).toBeVisible();
        await page
          .getByRole("button", { name: `查看路线 ${taskId}`, exact: true })
          .click();
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
          () =>
            (
              window as unknown as { __wheelFixture: { calls: unknown[] } }
            ).__wheelFixture.calls.filter(
              (call) => !Object.hasOwn(call as object, "http"),
            ).length,
        ),
      ).toBe(before);
      expect(
        await page.evaluate(
          () => document.documentElement.scrollWidth <= innerWidth,
        ),
      ).toBe(true);
      await page.evaluate(() => window.scrollTo(0, 0));
      await page.screenshot({
        path: testInfo.outputPath("tasks.png"),
        fullPage: true,
      });
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
