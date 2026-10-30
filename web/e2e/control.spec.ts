import { expect, test } from "@playwright/test";

const baseURL = process.env.PLAYWRIGHT_BASE_URL;
const password = process.env.SENIOR_CONTROL_PASSWORD;

test("test_browser_controls_simulated_robot", async ({ page }, testInfo) => {
  test.skip(baseURL === undefined, "PLAYWRIGHT_BASE_URL is required; no server is started");
  test.skip(password === undefined, "SENIOR_CONTROL_PASSWORD is required");

  await page.goto(new URL("/login", baseURL).toString());
  await page.getByLabel("管理员密码").fill(password as string);
  await page.getByRole("button", { name: "进入控制台" }).click();
  await page.getByRole("navigation", { name: "设备列表" }).waitFor();

  const robot = page.getByRole("link").filter({ hasText: "在线" }).first();
  await expect(robot).toBeVisible();
  await robot.click();
  await page.getByRole("button", { name: "获取控制权" }).click();

  const forward = page.getByRole("group", { name: "前进后退摇杆" });
  await forward.focus();
  await page.keyboard.down("ArrowUp");
  await expect(page.getByRole("button", { name: "释放控制权" })).toBeVisible();
  await page.keyboard.up("ArrowUp");
  await expect(page.getByRole("button", { name: "急停" })).toBeEnabled();

  await page.screenshot({
    path: testInfo.outputPath("desktop-drive.png"),
    fullPage: true,
  });
});

test("mobile portrait and landscape controls remain reachable", async ({ page }, testInfo) => {
  test.skip(baseURL === undefined, "PLAYWRIGHT_BASE_URL is required; no server is started");
  test.skip(password === undefined, "SENIOR_CONTROL_PASSWORD is required");

  await page.setViewportSize({ width: 390, height: 844 });
  await page.goto(new URL("/login", baseURL).toString());
  await page.getByLabel("管理员密码").fill(password as string);
  await page.getByRole("button", { name: "进入控制台" }).click();
  const robot = page.getByRole("link").filter({ hasText: "在线" }).first();
  await robot.click();
  await expect(page.getByRole("button", { name: "急停" })).toBeVisible();
  await page.screenshot({ path: testInfo.outputPath("mobile-portrait.png"), fullPage: true });

  await page.setViewportSize({ width: 844, height: 390 });
  await expect(page.getByRole("group", { name: "前进后退摇杆" })).toBeVisible();
  await expect(page.getByRole("group", { name: "左转右转摇杆" })).toBeVisible();
  await page.screenshot({ path: testInfo.outputPath("mobile-landscape.png"), fullPage: true });
});
