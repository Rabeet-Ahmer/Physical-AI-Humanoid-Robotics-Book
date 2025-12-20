import { betterAuth } from "better-auth";
import { Pool } from "pg";

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL,
  }),
  emailAndPassword: {
    enabled: true,
    async sendVerificationEmail({ user, url }: { user: { email: string }, url: string }) {
      console.log(`Verify email for ${user.email}: ${url}`);
    },
  },
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID || "",
      clientSecret: process.env.GOOGLE_CLIENT_SECRET || "",
    },
  },
  trustedOrigins: [
    "http://localhost:3000",
    "http://localhost:3001", 
    "https://physical-ai-humanoid-robotics-book-coral-seven.vercel.app",
    process.env.FRONTEND_URL || "",
  ],
});
