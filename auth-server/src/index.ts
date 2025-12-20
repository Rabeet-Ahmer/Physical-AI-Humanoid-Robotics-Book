import { Hono } from "hono";
import { serve } from "@hono/node-server";
import { cors } from "hono/cors";
import { auth } from "./auth.js";
import "dotenv/config";

const app = new Hono();

app.use(
  "*",
  cors({
    origin: ["http://localhost:3001", "https://physical-ai-humanoid-robotics-book-coral-seven.vercel.app", process.env.FRONTEND_URL || ""], // Docusaurus frontend
    allowHeaders: ["Content-Type", "Authorization"],
    allowMethods: ["POST", "GET", "OPTIONS"],
    exposeHeaders: ["Content-Length"],
    maxAge: 600,
    credentials: true,
  })
);

app.on(["POST", "GET"], "/api/auth/**", (c) => {
  return auth.handler(c.req.raw);
});

const port = 3000;
console.log(`Auth Server running on http://localhost:${port}`);

serve({
  fetch: app.fetch,
  port,
});
