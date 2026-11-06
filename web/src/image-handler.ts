import type { Handler } from "./handler";
import { RosClient } from "./ros";
import { must } from "./dom";

import type { CompressedImage, Detections2D, Image } from "./messages";

export class ImageHandler implements Handler {
  #cameraOverlay: HTMLCanvasElement;
  #maskCanvas = document.createElement("canvas"); // reused for every mask
  #maskCtx = this.#maskCanvas.getContext("2d")!;

  constructor(ros: RosClient, root: HTMLElement) {
    const img = must<HTMLImageElement>(root, "#camera");
    this.#cameraOverlay = must<HTMLCanvasElement>(root, "#camera-overlay");

    img.onload = () => {
      const c = this.#cameraOverlay;
      if (c.width !== img.naturalWidth || c.height !== img.naturalHeight) {
        c.width = img.naturalWidth;
        c.height = img.naturalHeight;
      }
    };

    ros.watch<CompressedImage>(
      "/camera/camera/color/compressed_image",
      "sensor_msgs/msg/CompressedImage",
      (msg) => {
        if (typeof msg.data === "string") {
          img.src = `data:image/jpeg;base64,${msg.data}`;
        }
      },
    );

    ros.watch<Detections2D>(
      "/detections",
      "autobot_msgs/msg/Detections2D",
      (msg) => {
        this.onDetections(msg);
      },
    );
  }

  update(): void {}

  #decodeData(data: string | number[] | Uint8Array): Uint8Array {
    if (data instanceof Uint8Array) return data;
    if (Array.isArray(data)) return Uint8Array.from(data);
    const bin = atob(data); // rosbridge JSON sends base64
    const out = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
    return out;
  }

  #drawMask(
    ctx: CanvasRenderingContext2D,
    mask: Image,
    rgb: [number, number, number] = [0, 255, 0],
    alpha = 0.35,
  ): void {
    const { width, height, step } = mask;
    const src = this.#decodeData(mask.data);

    this.#maskCanvas.width = width; // also clears the offscreen canvas
    this.#maskCanvas.height = height;

    const imgData = this.#maskCtx.createImageData(width, height);
    const a = Math.round(alpha * 255);

    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        if (src[y * step + x] !== 0) {
          const d = (y * width + x) * 4;
          imgData.data[d] = rgb[0];
          imgData.data[d + 1] = rgb[1];
          imgData.data[d + 2] = rgb[2];
          imgData.data[d + 3] = a;
        }
      }
    }
    this.#maskCtx.putImageData(imgData, 0, 0);

    ctx.drawImage(this.#maskCanvas, 0, 0, ctx.canvas.width, ctx.canvas.height);
  }

  onDetections(detections: Detections2D): void {
    const canvas = this.#cameraOverlay;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.lineWidth = 2;
    ctx.font = "16px sans-serif";
    ctx.textBaseline = "top";

    for (const det of detections.detections) {
      const { x_offset: x, y_offset: y, width: w, height: h } = det.box;

      // mask first, so the box and label draw on top
      this.#drawMask(ctx, det.mask);

      ctx.strokeStyle = "lime";
      ctx.strokeRect(x, y, w, h);

      const text = `${det.label} ${det.score.toFixed(2)}`;
      const pad = 3;
      const tw = ctx.measureText(text).width;
      const th = 16 + pad * 2;
      const ty = y - th >= 0 ? y - th : y;

      ctx.fillStyle = "lime";
      ctx.fillRect(x, ty, tw + pad * 2, th);

      ctx.fillStyle = "black";
      ctx.fillText(text, x + pad, ty + pad);
    }
  }

  dispose(): void {}
}
