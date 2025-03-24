import React, { useRef, useEffect, useState } from "react";
import { Button } from "@mui/material";

type RegionName = "Region 1" | "Region 2" | "Region 3" | "Region 4" | "Region 5";

const CarpAnimationGUI: React.FC = () => {
  const videoRef = useRef<HTMLVideoElement | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const [imageData, setImageData] = useState<string | null>(null);
  const [animationData, setAnimationData] = useState<Record<number, Record<RegionName, boolean>> | null>(null);
  const [capturing, setCapturing] = useState(false);
  const [paused, setPaused] = useState(false);
  const [frame, setFrame] = useState(0);

  const regions: Record<RegionName, number[][]> = {
    "Region 1": [[120, 360], [150, 345], [150, 390], [132, 463]],
    "Region 2": [[150, 345], [180, 310], [220, 290], [170, 345]],
    "Region 3": [[220, 290], [250, 200], [290, 220], [220, 300]],
    "Region 4": [[280, 230], [370, 200], [410, 150], [275, 215]],
    "Region 5": [[400, 120], [440, 100], [460, 140], [420, 160]],
  };

  const regionColors: Record<RegionName, string> = {
    "Region 1": "rgba(255, 0, 0, 0.5)",
    "Region 2": "rgba(255, 165, 0, 0.5)",
    "Region 3": "rgba(0, 128, 0, 0.5)",
    "Region 4": "rgba(0, 0, 255, 0.5)",
    "Region 5": "rgba(128, 0, 128, 0.5)",
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
    ctx?.drawImage(videoRef.current, 0, 0, 640, 480);
    const image = canvasRef.current.toDataURL("image/png");
    setImageData(image);
  };

  const extractDataFromImage = async () => {
    if (!imageData) return;
    setCapturing(true);

    try {
      // Convert base64 image to Blob
      const blob = await fetch(imageData).then((res) => res.blob());
      const formData = new FormData();
      formData.append("image", blob, "image.png");

      // Send to Flask backend
      const response = await fetch("http://localhost:5000/process-table", {
        method: "POST",
        body: formData,
      });

      const data = await response.json();
      console.log("Extracted Data from Backend:", data);
      setAnimationData(data);
    } catch (error) {
      console.error("Error processing image:", error);
    }
    setCapturing(false);
  };

  const drawRegions = (ctx: CanvasRenderingContext2D, yearIndex: number) => {
    if (!animationData) return;

    Object.entries(regions).forEach(([region, coords]) => {
      const typedRegion = region as RegionName;
      const color = animationData[2016 + yearIndex]?.[typedRegion]
        ? regionColors[typedRegion]
        : "rgba(255, 255, 255, 0)";

      ctx.beginPath();
      ctx.moveTo(coords[0][0], coords[0][1]);
      coords.forEach(([x, y]) => ctx.lineTo(x, y));
      ctx.closePath();
      ctx.fillStyle = color;
      ctx.fill();
      ctx.strokeStyle = "black";
      ctx.stroke();
    });
  };

  useEffect(() => {
    if (!animationData || paused) return;

    const canvas = canvasRef.current;
    const ctx = canvas?.getContext("2d");
    if (!canvas || !ctx) return;

    let currentFrame = 0;
    const animate = () => {
      if (!animationData || paused) return;
      if (currentFrame >= Object.keys(animationData).length) return;

      ctx.clearRect(0, 0, canvas.width, canvas.height);
      drawRegions(ctx, currentFrame);
      ctx.font = "20px Arial";
      ctx.fillStyle = "black";
      ctx.fillText(`Year: ${2016 + currentFrame}`, 20, 30);

      setFrame(currentFrame);
      currentFrame++;

      setTimeout(animate, 1000);
    };

    animate();
  }, [animationData, paused]);

  return (
    <div style={{ textAlign: "center" }}>
      <h2>Carp Animation GUI</h2>
      <video ref={videoRef} width="640" height="480" autoPlay />
      <canvas ref={canvasRef} width="640" height="480" style={{ border: "1px solid black", marginTop: "10px" }} />

      <div style={{ marginTop: "20px" }}>
        <Button variant="contained" color="primary" onClick={captureImage}>
          Capture Image
        </Button>
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

      {animationData && (
        <div style={{ marginTop: "20px" }}>
          <h3>Extracted Data</h3>
          <pre>{JSON.stringify(animationData, null, 2)}</pre>
          <Button variant="contained" color="success" onClick={() => setPaused(!paused)} style={{ marginTop: "10px" }}>
            {paused ? "Resume" : "Pause"}
          </Button>
        </div>
      )}
    </div>
  );
};

export default CarpAnimationGUI;
