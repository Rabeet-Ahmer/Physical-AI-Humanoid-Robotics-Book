import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.NEXT_PUBLIC_AUTH_SERVER_URL || "https://physical-ai-humanoid-robotics-book-coral-seven.vercel.app/api/auth",
});
