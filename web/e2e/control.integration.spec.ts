import { expect, test } from "@playwright/test";

// Opt in only against an explicitly provisioned simulated test deployment.
// No fixture, server startup, pairing redemption, or video setup is installed here.
const baseURL = process.env.PLAYWRIGHT_INTEGRATION_BASE_URL;
const password = process.env.SENIOR_CONTROL_PASSWORD;
const robotName = process.env.PLAYWRIGHT_INTEGRATION_ROBOT_NAME;
const enabled = process.env.PLAYWRIGHT_CONTROL_INTEGRATION === "1";

test.skip(
  !enabled || !baseURL || !password || !robotName,
  "Requires PLAYWRIGHT_CONTROL_INTEGRATION=1, PLAYWRIGHT_INTEGRATION_BASE_URL, SENIOR_CONTROL_PASSWORD and PLAYWRIGHT_INTEGRATION_ROBOT_NAME (simulated robot only)",
);

for (const viewport of [
  { width: 1440, height: 900 },
  { width: 390, height: 844 },
  { width: 844, height: 390 },
]) {
  test(`real HTTP/WS login and single-stick control at ${viewport.width} × ${viewport.height}`, async ({
    page,
  }, testInfo) => {
    await page.setViewportSize(viewport);
    const requests: string[] = [];
    const sockets: string[] = [];
    page.on("request", (request) => requests.push(request.url()));
    page.on("websocket", (socket) => sockets.push(socket.url()));
    await page.goto(new URL("/login", baseURL!).toString());
    await page.getByLabel("管理员密码").fill(password!);
    await page.getByRole("button", { name: "进入控制台" }).click();
    await page.getByRole("navigation", { name: "设备列表" }).waitFor();
    await page.getByRole("link").filter({ hasText: robotName! }).click();
    const stick = page.getByRole("group", { name: "驾驶摇杆" });
    await expect(stick).toBeVisible();
    await page.getByRole("button", { name: "获取控制权", exact: true }).click();
    await expect(stick).toHaveAttribute("aria-disabled", "false");
    await stick.scrollIntoViewIfNeeded();
    await expect(
      page.getByRole("button", { name: "急停", exact: true }),
    ).toBeInViewport({ ratio: 1 });
    await stick.focus();
    try {
      await page.keyboard.down("ArrowUp");
      await expect(
        page.getByRole("button", { name: "释放控制权", exact: true }),
      ).toBeVisible();
    } finally {
      await page.keyboard.up("ArrowUp");
      await page
        .getByRole("button", { name: "释放控制权", exact: true })
        .click();
    }
    await expect(
      page.getByRole("button", { name: "急停", exact: true }),
    ).toBeEnabled();
    expect(
      requests.some((url) => new URL(url).pathname === "/api/session"),
    ).toBe(true);
    expect(sockets.length).toBeGreaterThan(0);
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth <= innerWidth,
      ),
    ).toBe(true);
    await page.screenshot({
      path: testInfo.outputPath("integration-drive.png"),
      fullPage: true,
    });
  });
}
