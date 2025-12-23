import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.NODE_ENV === "development" 
    ? "http://localhost:3001/api/auth" 
    : "https://physical-ai-humanoid-robotics-book-coral-seven.vercel.app/api/auth",
});
