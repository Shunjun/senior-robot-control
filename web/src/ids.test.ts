import { afterEach, expect, it, vi } from "vitest";
import { newId } from "./ids";

afterEach(() => vi.unstubAllGlobals());

it("uses the native cryptographic UUID implementation when available", () => {
  const randomUUID = vi.fn(() => "12345678-1234-4234-8234-123456789abc");
  const getRandomValues = vi.fn();
  vi.stubGlobal("crypto", { randomUUID, getRandomValues });
  expect(newId()).toBe("12345678-1234-4234-8234-123456789abc");
  expect(randomUUID).toHaveBeenCalledTimes(1);
  expect(getRandomValues).not.toHaveBeenCalled();
});

it("creates a fresh UUIDv4 from 16 cryptographic bytes on LAN HTTP without randomUUID", () => {
  let value = 255;
  const getRandomValues = vi.fn((bytes: Uint8Array) => {
    expect(bytes).toHaveLength(16);
    return bytes.fill(value--);
  });
  vi.stubGlobal("crypto", { getRandomValues });
  expect(newId()).toBe("ffffffff-ffff-4fff-bfff-ffffffffffff");
  expect(newId()).toBe("fefefefe-fefe-4efe-befe-fefefefefefe");
  expect(getRandomValues).toHaveBeenCalledTimes(2);
});
