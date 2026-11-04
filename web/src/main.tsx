import "@fontsource/chakra-petch/latin-400.css";
import "@fontsource/chakra-petch/latin-500.css";
import "@fontsource/chakra-petch/latin-600.css";
import "@fontsource/chakra-petch/latin-700.css";
import "@fontsource/ibm-plex-mono/latin-400.css";
import "@fontsource/ibm-plex-mono/latin-500.css";
import "./theme.css";

import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { BrowserRouter } from "react-router-dom";

import { App } from "./App";
import { HttpControlApi, RobotClient } from "./api/client";
import { createDemoBackend } from "./api/demo";

const demo = import.meta.env.VITE_DEMO_DATA === "1";
const backend = demo ? createDemoBackend() : { api: new HttpControlApi(), link: new RobotClient() };

const root = document.getElementById("root");
if (root === null) throw new Error("#root is missing from index.html");

createRoot(root).render(
  <StrictMode>
    <BrowserRouter>
      <App api={backend.api} link={backend.link} demo={demo} />
    </BrowserRouter>
  </StrictMode>,
);
