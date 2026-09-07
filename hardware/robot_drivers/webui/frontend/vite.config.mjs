import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
export default defineConfig({
  build: { outDir: "dist/client" },
  optimizeDeps: { include: ["react", "react-dom/client"] },
  server: {
    host: "0.0.0.0",
    allowedHosts: ["terminal.local"],
    proxy: { "/api": "http://127.0.0.1:8765" },
    warmup: { clientFiles: ["./src/main.jsx"] },
  },
  plugins: [react()],
});
