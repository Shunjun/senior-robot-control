import react from "@vitejs/plugin-react";
import { defineConfig } from "vitest/config";

// The API and both sockets live on the same origin as the page: `__Host-session` is
// Secure and SameSite=Strict, so the console is served by (or proxied behind) the cloud.
export default defineConfig({
  plugins: [react()],
  build: { outDir: "dist", sourcemap: false },
  test: {
    environment: "jsdom",
    include: ["src/**/*.test.{ts,tsx}"],
    restoreMocks: true,
  },
});
