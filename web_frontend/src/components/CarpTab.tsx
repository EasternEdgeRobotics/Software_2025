import React, { useRef, useEffect, useState } from "react";
import { Button, Select, MenuItem } from "@mui/material";

type RegionName = "Region 1" | "Region 2" | "Region 3" | "Region 4" | "Region 5";

interface AnimationData {
  structured: Record<number, Record<RegionName, boolean>>;
  raw_ocr: string[];
  cell_count: number;
}

const CarpAnimationGUI: React.FC = () => {
  const videoRef = useRef<HTMLVideoElement | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const [imageData, setImageData] = useState<string | null>(null);
  const [animationData, setAnimationData] = useState<AnimationData | null>(null);
  const [capturing, setCapturing] = useState(false);
  const [paused, setPaused] = useState(false);

  const regionLines: Record<RegionName, number[][]> = {
    "Region 1": [[125, 320], [145, 330], [160, 300], [180, 250], [150, 230], [130, 280]],
    "Region 2": [[170, 230], [190, 210], [220, 190], [200, 220], [180, 230]],
    "Region 3": [[220, 180], [230, 160], [310, 140], [260, 190], [230, 210]],
    "Region 4": [[300, 160], [340, 150], [370, 140], [400, 120], [330, 130]],
    "Region 5": [[390, 110], [420, 90], [440, 70], [460, 90], [430, 120]],
  };

  // Pre-draw lines and store paths in a ref
  const pathsRef = useRef<Record<RegionName, Path2D | null>>({
    "Region 1": null,
    "Region 2": null,
    "Region 3": null,
    "Region 4": null,
    "Region 5": null,
  });

  useEffect(() => {
    const ctx = canvasRef.current?.getContext("2d");
    if (!ctx) return;

    // Pre-draw the lines for each region and store them
    Object.entries(regionLines).forEach(([regionName, coords]) => {
      const path = new Path2D();
      path.moveTo(coords[0][0], coords[0][1]);
      coords.forEach(([x, y]) => path.lineTo(x, y)); // Draw lines instead of filling areas
      pathsRef.current[regionName as RegionName] = path;
    });
  }, []); // Run only once when the component is mounted

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

  const baseImage = new Image();
  baseImage.src = "/river.png";

  useEffect(() => {
    navigator.mediaDevices.getUserMedia({ video: true }).then((stream) => {
      if (videoRef.current) videoRef.current.srcObject = stream;
    }).catch((err) => console.error("Camera access error:", err));
  }, []);

  const captureImage = () => {
    if (!videoRef.current || !canvasRef.current) return;
    const ctx = canvasRef.current.getContext("2d");
    if (ctx) {
      ctx.drawImage(videoRef.current, 0, 0, 640, 480);
      canvasRef.current.toBlob((blob) => {
        if (blob) {
          const imageUrl = URL.createObjectURL(blob);
          setImageData(imageUrl);
        }
      }, "image/png");
    }
  };

  const handleFileUpload = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (file) {
      const imageUrl = URL.createObjectURL(file);
      setImageData(imageUrl);
    }
  };

  const extractDataFromImage = async () => {
    if (!imageData) return;
    setCapturing(true);
    try {
      const blob = await fetch(imageData).then((res) => res.blob());
      const formData = new FormData();
      formData.append("image", blob, "image.png");

      const response = await fetch("http://localhost:5000/process-table", {
        method: "POST",
        body: formData,
      });

      const data = await response.json();
      const raw = data.raw_ocr;
      const startIndex = raw.findIndex((v: string) => /^20\d\d$/.test(v));
      const sliced = raw.slice(startIndex);
      const rowCount = Math.floor(sliced.length / 6);

      const structured: Record<number, Record<RegionName, boolean>> = {};
      for (let i = 0; i < rowCount; i++) {
        const year = parseInt(sliced[i * 6]);
        const row = sliced.slice(i * 6 + 1, i * 6 + 6);
        structured[year] = {
          "Region 1": ["Y", "YES"].includes(row[0]?.toUpperCase()),
          "Region 2": ["Y", "YES"].includes(row[1]?.toUpperCase()),
          "Region 3": ["Y", "YES"].includes(row[2]?.toUpperCase()),
          "Region 4": ["Y", "YES"].includes(row[3]?.toUpperCase()),
          "Region 5": ["Y", "YES"].includes(row[4]?.toUpperCase()),
        };
      }

      setAnimationData({ ...data, structured });
    } catch (error) {
      console.error("Error processing image:", error);
    }
    setCapturing(false);
  };

  const drawAnimation = (structuredData: Record<number, Record<RegionName, boolean>>) => {
    if (!canvasRef.current) return;

    let frame = 0;
    let animationId: number;

    const drawFrame = () => {
      const ctx = canvasRef.current?.getContext("2d");
      if (!ctx) return;

      const years = Object.keys(structuredData).sort();
      const year = years[frame % years.length];
      const regionData = structuredData[parseInt(year)];

      ctx.clearRect(0, 0, 640, 480);
      ctx.drawImage(baseImage, 0, 0, 640, 480);

      if (regionData) {
        Object.entries(regionData).forEach(([regionName, value]) => {
          const path = pathsRef.current[regionName as RegionName];
          if (path) {
            ctx.strokeStyle = "black";
            ctx.lineWidth = 2;
            ctx.stroke(path); // Draw the line for the region

            // Change the line color based on the value
            ctx.strokeStyle = value ? "rgba(76, 175, 80, 1)" : "rgba(244, 67, 54, 1)";
            ctx.lineWidth = 4;
            ctx.stroke(path);
          }
        });
      }

      ctx.fillStyle = "black";
      ctx.font = "20px Arial";
      ctx.fillText(`Year: ${year}`, 20, 30);

      if (!paused) frame++;
      animationId = window.setTimeout(() => requestAnimationFrame(drawFrame), 1000);
    };

    baseImage.onload = () => drawFrame();
    drawFrame();

    return () => clearTimeout(animationId);
  };

  return (
    <div style={{ textAlign: "center" }}>
      <h2>Carp Animation GUI</h2>

      <video ref={videoRef} width="640" height="480" autoPlay />
      <canvas ref={canvasRef} width="640" height="480" style={{ border: "1px solid black", marginTop: "10px" }} />

      <div style={{ marginTop: "20px" }}>
        <Button variant="contained" onClick={captureImage}>Capture Image</Button>
        <input type="file" accept="image/*" onChange={handleFileUpload} style={{ marginLeft: "10px" }} />
        <Button variant="contained" color="secondary" onClick={extractDataFromImage} disabled={capturing} style={{ marginLeft: "10px" }}>
          {capturing ? "Processing..." : "Extract Data"}
        </Button>
      </div>

      {animationData && (
        <div style={{ marginTop: "20px" }}>
          <h3>Extracted Table</h3>
          <p><strong>Total Cells Detected:</strong> {animationData.cell_count ?? 0}</p>
          <Button variant="contained" color="success" onClick={() => setPaused(!paused)}>{paused ? "Resume" : "Pause"}</Button>
        </div>
      )}

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
        <Button variant="contained" style={{ marginTop: 20 }} onClick={() => drawAnimation(manualData)}>Run Animation with Manual Data</Button>
      </div>
    </div>
  );
};

export default CarpAnimationGUI;
