import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import path from "node:path";
export default defineConfig({
    plugins: [react()],
    resolve: {
        alias: {
            "@": path.resolve(__dirname, "./src"),
        },
    },
    server: {
        port: 5173,
        // When the FastAPI backend is wired up, point /api at it.
        // proxy: {
        //   "/api": {
        //     target: "http://localhost:8000",
        //     changeOrigin: true,
        //   },
        // },
    },
});
