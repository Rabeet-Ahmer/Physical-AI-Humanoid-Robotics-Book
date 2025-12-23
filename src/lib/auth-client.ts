import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: "https://physical-ai-humanoid-robotics-book-coral-seven.vercel.app/api/auth",
});
