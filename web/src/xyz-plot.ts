import uPlot from "uplot";
import "uplot/dist/uPlot.min.css";

export class XYZPlot {
  #data: number[][] = [[], [], [], []]; // t, x, y, z
  #plot: uPlot;
  #dirty = false;

  constructor(
    el: HTMLElement,
    title: string,
    private windowSec = 300,
  ) {
    this.#plot = new uPlot(
      {
        title,
        width: el.clientWidth,
        height: el.clientHeight,
        scales: { x: { time: false } }, // x axis is plain seconds
        series: [
          {},
          { label: "x", stroke: "red", width: 3 },
          { label: "y", stroke: "green", width: 3 },
          { label: "z", stroke: "blue", width: 3 },
        ],
        axes: [
          // X-axis (time) - index 0
          {
            stroke: "#ccc", // tick label color - uPlot defaults to black,
            // invisible against this page's dark background
            ticks: { stroke: "#ccc" },
            grid: {
              show: true,
              stroke: "rgba(255, 255, 255, 0.3)", // Line color
              width: 1, // Line width in pixels
              //dash: [5, 5], // Optional dash pattern [dash, gap]
            },
          },
          // Y-axis (value) - index 1
          {
            stroke: "#ccc",
            ticks: { stroke: "#ccc" },
            grid: {
              show: true,
              stroke: "rgba(255, 255, 255, 0.3)",
              width: 1,
              //dash: [5, 5], // Optional dash pattern [dash, gap]
            },
          },
        ],
      },
      this.#data as uPlot.AlignedData,
      el,
    );

    new ResizeObserver(() =>
      this.#plot.setSize({ width: el.clientWidth, height: el.clientHeight }),
    ).observe(el);
  }

  push(t: number, x: number, y: number, z: number) {
    const d = this.#data;
    d[0].push(t);
    d[1].push(x);
    d[2].push(y);
    d[3].push(z);

    // drop samples older than the window
    while (d[0].length && d[0][0] < t - this.windowSec) {
      for (const col of d) col.shift();
    }
    this.#dirty = true;
  }

  redraw() {
    if (!this.#dirty) return;
    this.#dirty = false;
    this.#plot.setData(this.#data as uPlot.AlignedData);
  }
}
