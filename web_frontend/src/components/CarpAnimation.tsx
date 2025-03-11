import React, { useRef, useEffect } from "react";

const regions: Record<string, number[][]> = {
  "Region 1": [
    [90, 310],
    [130, 250],
    [160, 280],
    [120, 340],
  ],
  "Region 2": [
    [160, 200],
    [200, 170],
    [240, 220],
    [180, 260],
  ],
  "Region 3": [
    [230, 140],
    [270, 120],
    [300, 160],
    [260, 200],
  ],
  "Region 4": [
    [310, 90],
    [350, 70],
    [380, 110],
    [340, 140],
  ],
  "Region 5": [
    [380, 40],
    [420, 20],
    [440, 60],
    [400, 80],
  ],
};

type DataType = {
  Year: number[];
  Regions: Record<string, boolean[]>;
};

type CarpAnimationProps = {
  graphData: DataType;
  startAnimation: boolean;
};

const CarpAnimation: React.FC<CarpAnimationProps> = ({
  graphData,
  startAnimation,
}) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  const drawRegion = (
    ctx: CanvasRenderingContext2D,
    coords: number[][],
    color: string
  ) => {
    ctx.beginPath();
    ctx.moveTo(coords[0][0], coords[0][1]);
    coords.slice(1).forEach(([x, y]) => ctx.lineTo(x, y));
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.fill();
    ctx.strokeStyle = "black";
    ctx.stroke();
  };

  useEffect(() => {
    if (!startAnimation) return; // Only proceed when animation is started

    const canvas = canvasRef.current;
    const ctx = canvas?.getContext("2d");

    if (!canvas || !ctx) {
      console.error("Canvas or context is not available.");
      return;
    }

    const background = new Image();
    background.src = "/River.png"; // Ensure the image is in the public folder

    let frame = 0;

    const animate = () => {
      if (frame >= graphData.Year.length) {
        console.log("Animation complete.");
        return; // Stop when all frames are rendered
      }

      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // Draw the background
      if (background.complete) {
        ctx.drawImage(background, 0, 0, canvas.width, canvas.height);
      } else {
        console.warn("Background image not loaded.");
      }

      // Draw regions for the current year
      Object.keys(graphData.Regions).forEach((region) => {
        const isPresent = graphData.Regions[region][frame];
        const color = isPresent
          ? "rgba(255, 0, 0, 0.5)"
          : "rgba(255, 255, 255, 0)";
        drawRegion(ctx, regions[region], color);
      });

      // Add year label
      ctx.font = "20px Arial";
      ctx.fillStyle = "black";
      ctx.fillText(`Year: ${graphData.Year[frame]}`, 20, 30);

      frame++;
      setTimeout(animate, 1000); // 1 second per frame
    };

    background.onload = () => {
      console.log("Background image loaded. Starting animation...");
      animate();
    };

    background.onerror = () => {
      console.error("Failed to load background image.");
    };
  }, [startAnimation, graphData]);

  return <canvas ref={canvasRef} width={600} height={700} />;
};

export default CarpAnimation;
