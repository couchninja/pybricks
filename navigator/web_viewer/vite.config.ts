import { defineConfig } from "vite";

export default defineConfig({
  build: {
    outDir: "dist",
    emptyOutDir: true,
    rollupOptions: {
      output: {
        entryFileNames: "viewer.js",
        chunkFileNames: "viewer-[name].js",
        assetFileNames: "viewer.[ext]",
      },
    },
  },
});
