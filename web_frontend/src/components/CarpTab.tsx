import React, { useRef, useEffect, useState } from "react";
import { Button, Select, MenuItem } from "@mui/material";

type RegionName = "Region 1" | "Region 2" | "Region 3" | "Region 4" | "Region 5";

// Define Bézier segments per region
const regionLines: Record<RegionName, number[][]> = {
  "Region 1": [
    [110, 370, 115, 360, 120, 340],
    [120, 340, 125, 325, 130, 310],
    [130, 310, 135, 295, 140, 280],
  ],
  "Region 2": [
    [140, 280, 150, 270, 160, 260],
    [160, 260, 170, 250, 180, 240],
    [180, 240, 190, 230, 200, 220],
  ],
  "Region 3": [
    [200, 220, 210, 210, 220, 200],
    [220, 200, 230, 190, 240, 180],
    [240, 180, 250, 170, 260, 160],
  ],
  "Region 4": [
    [260, 160, 275, 150, 290, 140],
    [290, 140, 310, 130, 330, 120],
    [330, 120, 350, 110, 370, 100],
  ],
  "Region 5": [
    [370, 100, 390, 90, 410, 80],
    [410, 80, 430, 70, 450, 60],
  ],
};

// Color map
const regionColors: Record<RegionName, string> = {
  "Region 1": "red",
  "Region 2": "green",
  "Region 3": "yellow",
  "Region 4": "blue",
  "Region 5": "purple",
};

const CarpAnimationGUI: React.FC = () => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const pausedRef = useRef(false);
  const [manualData, setManualData] = useState<Record<number, Record<RegionName, boolean>>>(() => {
    const initialData: Record<number, Record<RegionName, boolean>> = {};
    for (let year = 2016; year <= 2025; year++) {
      initialData[year] = {
        "Region 1": false,
        "Region 2": false,
        "Region 3": false,
        "Region 4": false,
        "Region 5": false,
      };
    }
    return initialData;
  });

  const pathsRef = useRef<Record<RegionName, Path2D[]>>({
    "Region 1": [],
    "Region 2": [],
    "Region 3": [],
    "Region 4": [],
    "Region 5": [],
  });

  const animationRef = useRef<number | null>(null);
  const frameRef = useRef(0);
  const lastFrameTimeRef = useRef(0);

  const baseImage = new Image();
  baseImage.src = "/river.png";

  useEffect(() => {
    const ctx = canvasRef.current?.getContext("2d");
    if (!ctx) return;

    const newPaths: Record<RegionName, Path2D[]> = {
      "Region 1": [],
      "Region 2": [],
      "Region 3": [],
      "Region 4": [],
      "Region 5": [],
    };

    Object.entries(regionLines).forEach(([regionName, segments]) => {
      const region = regionName as RegionName;
      const paths: Path2D[] = [];

      if (segments.length === 0) return;

      let [startX, startY] = segments[0].slice(0, 2);

      segments.forEach(segment => {
        if (segment.length === 6) {
          const [cp1x, cp1y, cp2x, cp2y, x, y] = segment;
          const path = new Path2D();
          path.moveTo(startX, startY);
          path.bezierCurveTo(cp1x, cp1y, cp2x, cp2y, x, y);
          paths.push(path);
          startX = x;
          startY = y;
        }
      });

      newPaths[region] = paths;
    });

    pathsRef.current = newPaths;
  }, []);

  const drawAnimation = (timestamp: number, structuredData: Record<number, Record<RegionName, boolean>>) => {
    const ctx = canvasRef.current?.getContext("2d");
    if (!ctx) return;

    const years = Object.keys(structuredData).sort();

    if (timestamp - lastFrameTimeRef.current >= 1000) {
      ctx.clearRect(0, 0, 640, 480);
      ctx.drawImage(baseImage, 0, 0, 640, 480);

      const year = years[frameRef.current % years.length];
      const regionData = structuredData[parseInt(year)];

      (Object.entries(regionData) as [RegionName, boolean][]).forEach(([regionName, isActive]) => {
        const color = regionColors[regionName];
        const paths = pathsRef.current[regionName];
        if (!paths) return;

        ctx.strokeStyle = isActive ? color : "gray";
        ctx.lineWidth = 4;
        paths.forEach(path => {
          ctx.stroke(path);
        });
      });

      ctx.fillStyle = "black";
      ctx.font = "20px Arial";
      ctx.fillText(`Year: ${year}`, 20, 30);

      lastFrameTimeRef.current = timestamp;
      frameRef.current++;
    }

    if (!pausedRef.current) {
      animationRef.current = requestAnimationFrame((t) => drawAnimation(t, structuredData));
    }
  };

  const startAnimation = (structuredData: Record<number, Record<RegionName, boolean>>) => {
    cancelAnimationFrame(animationRef.current!);
    frameRef.current = 0;
    lastFrameTimeRef.current = 0;
    pausedRef.current = false;
    animationRef.current = requestAnimationFrame((timestamp) => drawAnimation(timestamp, structuredData));
  };

  const resumeAnimation = () => {
    if (pausedRef.current) {
      pausedRef.current = false;
      animationRef.current = requestAnimationFrame((timestamp) => drawAnimation(timestamp, manualData));
    }
  };

  const pauseAnimation = () => {
    pausedRef.current = true;
    cancelAnimationFrame(animationRef.current!);
  };

  return (
    <div style={{ textAlign: "center" }}>
      <h2>Carp Animation GUI</h2>
      <canvas ref={canvasRef} width="640" height="480" style={{ border: "1px solid black" }} />
      <div style={{ marginTop: "20px" }}>
        <Button
          variant="contained"
          onClick={() => {
            startAnimation(manualData);
          }}
          style={{ marginRight: "10px" }}
        >
          Run Animation with Manual Data
        </Button>
        <Button
          variant="contained"
          color="warning"
          onClick={pauseAnimation}
          style={{ marginRight: "10px" }}
        >
          Pause
        </Button>
        <Button
          variant="contained"
          color="success"
          onClick={resumeAnimation}
        >
          Resume
        </Button>
      </div>
      <div style={{ marginTop: "40px" }}>
        <h3>Manual Entry Table (2016-2025)</h3>
        <table style={{ margin: "0 auto", borderCollapse: "collapse" }}>
          <thead>
            <tr>
              <th>Year</th>
              {["Region 1", "Region 2", "Region 3", "Region 4", "Region 5"].map(region => (
                <th key={region}>{region}</th>
              ))}
            </tr>
          </thead>
          <tbody>
            {Object.entries(manualData).map(([yearStr, values]) => (
              <tr key={yearStr}>
                <td>{yearStr}</td>
                {Object.entries(values).map(([region, val]) => (
                  <td key={region}>
                    <Select
                      value={val ? "Y" : "N"}
                      onChange={(e) => {
                        const newVal = e.target.value === "Y";
                        setManualData(prev => ({
                          ...prev,
                          [yearStr]: {
                            ...prev[+yearStr],
                            [region]: newVal,
                          },
                        }));
                      }}
                      size="small"
                    >
                      <MenuItem value="Y">Y</MenuItem>
                      <MenuItem value="N">N</MenuItem>
                    </Select>
                  </td>
                ))}
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default CarpAnimationGUI;
