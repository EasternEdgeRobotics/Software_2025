import React, { useRef, useEffect, useState } from "react";
import { Button } from "@mui/material";

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

  const regionShapes: Record<RegionName, number[][]> = {
    "Region 1": [[50, 60], [150, 60], [150, 160], [50, 160]],
    "Region 2": [[200, 80], [300, 80], [300, 180], [200, 180]],
    "Region 3": [[350, 100], [450, 100], [450, 200], [350, 200]],
    "Region 4": [[100, 220], [200, 220], [200, 320], [100, 320]],
    "Region 5": [[250, 250], [350, 250], [350, 350], [250, 350]],
  };

  useEffect(() => {
    navigator.mediaDevices
      .getUserMedia({ video: true })
      .then((stream) => {
        if (videoRef.current) {
          videoRef.current.srcObject = stream;
        }
      })
      .catch((err) => console.error("Camera access error:", err));
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
      console.log("Extracted Data:", data);
      setAnimationData(data);
    } catch (error) {
      console.error("Error processing image:", error);
    }
    setCapturing(false);
  };

  const baseImage = new Image();
  baseImage.src = "/river.png";

  useEffect(() => {
    if (!canvasRef.current || !animationData) return;

    let frame = 0;
    let animationId: number;

    const drawFrame = () => {
      const ctx = canvasRef.current?.getContext("2d");
      if (!ctx || !animationData.structured) return;

      const years = Object.keys(animationData.structured).sort();
      const year = years[frame % years.length];

      ctx.clearRect(0, 0, 640, 480);
      ctx.drawImage(baseImage, 0, 0, 640, 480);

      const regionData = animationData.structured[parseInt(year)];

      Object.entries(regionData).forEach(([regionName, value]) => {
        const coords = regionShapes[regionName as RegionName];
        if (!coords) return;

        ctx.beginPath();
        ctx.moveTo(coords[0][0], coords[0][1]);
        coords.slice(1).forEach(([x, y]) => ctx.lineTo(x, y));
        ctx.closePath();

        ctx.fillStyle = value ? "rgba(76, 175, 80, 0.5)" : "rgba(244, 67, 54, 0.5)";
        ctx.fill();
        ctx.strokeStyle = "black";
        ctx.stroke();
      });

      ctx.fillStyle = "black";
      ctx.font = "20px Arial";
      ctx.fillText(`Year: ${year}`, 20, 30);

      if (!paused) {
        frame++;
      }
      animationId = requestAnimationFrame(drawFrame);
    };

    baseImage.onload = () => {
      drawFrame();
    };

    return () => cancelAnimationFrame(animationId);
  }, [animationData, paused]);

  return (
    <div style={{ textAlign: "center" }}>
      <h2>Carp Animation GUI</h2>
      <video ref={videoRef} width="640" height="480" autoPlay />
      <canvas
        ref={canvasRef}
        width="640"
        height="480"
        style={{ border: "1px solid black", marginTop: "10px" }}
      />

      <div style={{ marginTop: "20px" }}>
        <Button variant="contained" color="primary" onClick={captureImage}>
          Capture Image
        </Button>
        <input
          type="file"
          accept="image/*"
          onChange={handleFileUpload}
          style={{ marginLeft: "10px" }}
        />
        <Button
          variant="contained"
          color="secondary"
          onClick={extractDataFromImage}
          disabled={capturing}
          style={{ marginLeft: "10px" }}
        >
          {capturing ? "Processing..." : "Extract Data"}
        </Button>
      </div>

      {imageData && (
        <div style={{ marginTop: "20px" }}>
          <h3>Selected Image</h3>
          <img
            src={imageData}
            alt="Uploaded Screenshot"
            width="320"
            height="240"
            style={{ border: "1px solid black" }}
          />
        </div>
      )}

      {animationData && (
        <div style={{ marginTop: "20px" }}>
          <h3>Extracted Data</h3>
          <h4>Structured Data</h4>
          <pre>{JSON.stringify(animationData.structured, null, 2)}</pre>

          <h4 style={{ marginTop: "20px" }}>Raw OCR Text (Parsed)</h4>
          {(() => {
            const raw = animationData.raw_ocr;
            const startIndex = raw.findIndex((v) => /^20\d\d$/.test(v));
            const sliced = raw.slice(startIndex);
            const rowCount = Math.floor(sliced.length / 6);
            const tableRows = [];

            for (let i = 0; i < rowCount; i++) {
              const year = sliced[i * 6];
              const row = sliced.slice(i * 6 + 1, i * 6 + 6);
              tableRows.push({ year, row });
            }

            return (
              <table style={{ margin: "0 auto", borderCollapse: "collapse" }}>
                <thead>
                  <tr>
                    <th style={{ border: "1px solid #ccc", padding: "4px" }}>Year</th>
                    {["Region 1", "Region 2", "Region 3", "Region 4", "Region 5"].map((region) => (
                      <th key={region} style={{ border: "1px solid #ccc", padding: "4px" }}>
                        {region}
                      </th>
                    ))}
                  </tr>
                </thead>
                <tbody>
                  {tableRows.map(({ year, row }) => (
                    <tr key={year}>
                      <td style={{ border: "1px solid #ccc", padding: "4px" }}>{year}</td>
                      {row.map((value, index) => (
                        <td
                          key={index}
                          style={{
                            border: "1px solid #ccc",
                            padding: "4px",
                            backgroundColor:
                              value === "Y" || value === "YES"
                                ? "#c8e6c9"
                                : value === "N" || value === "NO"
                                ? "#ffcdd2"
                                : "#fff9c4",
                          }}
                        >
                          {value}
                        </td>
                      ))}
                    </tr>
                  ))}
                </tbody>
              </table>
            );
          })()}

          <p style={{ marginTop: "10px" }}>
            <strong>Total Cells Detected:</strong> {animationData.cell_count ?? 0}
          </p>

          <Button
            variant="contained"
            color="success"
            onClick={() => setPaused(!paused)}
            style={{ marginTop: "10px" }}
          >
            {paused ? "Resume" : "Pause"}
          </Button>
        </div>
      )}
    </div>
  );
};

export default CarpAnimationGUI;
