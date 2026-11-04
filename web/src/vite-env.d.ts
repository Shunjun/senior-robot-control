/// <reference types="vite/client" />

interface ImportMetaEnv {
  /** "1" serves labelled demo data instead of talking to the cloud. Never set in production. */
  readonly VITE_DEMO_DATA?: string;
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}
