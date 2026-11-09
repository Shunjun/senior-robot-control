import { expect, test } from "@playwright/test";

const baseURL = process.env.PLAYWRIGHT_BASE_URL;
const password = process.env.SENIOR_CONTROL_PASSWORD;

test.skip(baseURL === undefined, "PLAYWRIGHT_BASE_URL is required; no server is started");
test.skip(password === undefined, "SENIOR_CONTROL_PASSWORD is required");

test("test_browser_controls_simulated_robot", async ({ page }, testInfo) => {
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

for (const viewport of [
  { width: 1440, height: 900 },
  { width: 390, height: 844 },
  { width: 844, height: 390 },
]) {
  test(`manual driving fits a ${viewport.width} × ${viewport.height} viewport`, async ({ page }) => {
    await page.setViewportSize(viewport);
    await page.goto(new URL("/login", baseURL).toString());
    await page.getByLabel("管理员密码").fill(password as string);
    await page.getByRole("button", { name: "进入控制台" }).click();
    await page.getByRole("link").filter({ hasText: "在线" }).first().click();

    const controls = [
      page.getByRole("region", { name: "实时视频" }),
      page.getByRole("group", { name: "前进后退摇杆" }),
      page.getByRole("group", { name: "左转右转摇杆" }),
      page.getByRole("button", { name: "停车", exact: true }),
      page.getByRole("button", { name: "急停", exact: true }),
    ];
    for (const control of controls) {
      await expect(control).toBeInViewport({ ratio: 0.99 });
    }
    const video = await controls[0]!.boundingBox();
    expect(video?.height).toBeGreaterThanOrEqual(Math.min(viewport.height / 4, 210));
    expect(await page.evaluate(() => ({
      width: document.documentElement.scrollWidth,
      height: document.documentElement.scrollHeight,
    }))).toEqual(viewport);
  });
}
