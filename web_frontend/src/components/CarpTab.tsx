import React, { useRef, useEffect, useState } from "react";
import { Button } from "@mui/material";

const regions: Record<string, number[][]> = {
  "Region 1": [
    [120, 360],
    [150, 345],
    [150, 390],
    [132, 463],
  ],
   "Region 2": [
    [150, 345], // Bot left corner
    [180, 310], // Top left corner
    [220, 290], // Top right corner
    [170, 345], // Bot right corner. Left number is horizontal, right number is vertical.
  ],
  "Region 3": [
    [220, 290],
    [250, 200], 
    [290, 220],
    [220, 300],
  ],
  "Region 4": [
    [280, 230],
    [370, 200],
    [410, 150],
    [275, 215],
  ],
  "Region 5": [
    [400, 120],
    [440, 100],
    [460, 140],
    [420, 160],
  ],
};

const graphData = {
  Year: [2016, 2017, 2018, 2019, 2020, 2021, 2022, 2023, 2024, 2025],
  Regions: {
    "Region 1": [false, true,  true, true, true, true, true, true, true, true],
    "Region 2": [false, false, false, false, true, true, true, true, true, true],
    "Region 3": [false, false, false, false, true, true, true, true, true, true],
    "Region 4": [false, false, false, false, false, false, false, true, true, true],
    "Region 5": [false, false, false, false, false, false, false, false, false, false, ],
  },
};

const regionColors: Record<string, string> = {
  "Region 1": "red",
  "Region 2": "green",
  "Region 3": "orange",
  "Region 4": "blue",
  "Region 5": "purple",
};

const CarpAnimationGUI: React.FC = () => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const [startAnimation, setStartAnimation] = useState(false);

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
    if (!startAnimation) return;

    const canvas = canvasRef.current;
    const ctx = canvas?.getContext("2d");

    if (!canvas || !ctx) {
      console.error("Canvas or context is not available.");
      return;
    }

    
    const background = new Image();
    background.src = '/River.png'; 
    console.log("Attempting to load image:", background.src);

    let frame = 0;
    const regionKeys: (keyof typeof graphData.Regions)[] = [
      "Region 1",
      "Region 2",
      "Region 3",
      "Region 4",
      "Region 5",
    ];

    const animate = () => {
      if (frame >= graphData.Year.length) {
        console.log("Animation complete.");
        return;
      }

      ctx.clearRect(0, 0, canvas.width, canvas.height);

      
      if (background.complete) {
        console.log("Drawing background image...");
        ctx.drawImage(background, 0, 0, canvas.width, canvas.height);
      } else {
        console.warn("Background image not yet loaded.");
      }

    
      regionKeys.forEach((region) => {
        const isPresent = graphData.Regions[region][frame];
        const color = isPresent ? regionColors[region] : "rgba(255, 255, 255, 0)";
        drawRegion(ctx, regions[region], color);
      });

      
      ctx.font = "20px Arial";
      ctx.fillStyle = "black";
      ctx.fillText(`Year: ${graphData.Year[frame]}`, 20, 30);

      frame++;
      setTimeout(animate, 1000);
    };

    
    background.onload = () => {
      console.log("Image loaded successfully!");
      animate();
    };

    background.onerror = (err) => {
      console.error("Failed to load the image:", err);
    };
  }, [startAnimation]);

  return (
    <div style={{ textAlign: "center" }}>
      <Button
        variant="contained"
        color="primary"
        onClick={() => setStartAnimation(true)}
      >
        Start Animation
      </Button>
      <canvas
        ref={canvasRef}
        width={600}
        height={700}
        style={{ border: "1px solid black" }}
      />
    </div>
  );
};

export default CarpAnimationGUI;
