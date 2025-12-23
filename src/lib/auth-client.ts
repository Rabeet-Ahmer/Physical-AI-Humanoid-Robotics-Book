import { createAuthClient } from "better-auth/react";

const isDevelopment = typeof process !== "undefined" && process.env.NODE_ENV === "development";

export const authClient = createAuthClient({
  baseURL: isDevelopment
    ? "http://localhost:3001/api/auth" 
    : "https://physical-ai-humanoid-robotics-book-coral-seven.vercel.app/api/auth",
});
