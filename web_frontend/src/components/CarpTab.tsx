import React, { useRef, useEffect, useState } from "react";
import Tesseract from "tesseract.js";
import { Button } from "@mui/material";

type RegionName = "Region 1" | "Region 2" | "Region 3" | "Region 4" | "Region 5";

const CarpAnimationGUI: React.FC = () => {
  const videoRef = useRef<HTMLVideoElement | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const animationCanvasRef = useRef<HTMLCanvasElement | null>(null);
  const [imageData, setImageData] = useState<string | null>(null);
  const [animationData, setAnimationData] = useState<Record<RegionName, boolean[]> | null>(null);
  const [capturing, setCapturing] = useState(false);
  const [paused, setPaused] = useState(true);
  const [animationStarted, setAnimationStarted] = useState(false);
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
    const startCamera = async () => {
      try {
        const stream = await navigator.mediaDevices.getUserMedia({ video: true });
        if (videoRef.current) {
          videoRef.current.srcObject = stream;
        }
      } catch (error) {
        console.error("Error accessing camera:", error);
      }
    };

    startCamera();
  }, []);

  const captureImage = () => {
    if (!videoRef.current || !canvasRef.current) return;
    const ctx = canvasRef.current.getContext("2d");
    if (!ctx) return;

    ctx.drawImage(videoRef.current, 0, 0, canvasRef.current.width, canvasRef.current.height);

    const image = canvasRef.current.toDataURL("image/png");
    setImageData(image);
  };

  const extractDataFromImage = async () => {
    if (!imageData) return;
    setCapturing(true);

    try {
      const { data: { text } } = await Tesseract.recognize(imageData, "eng", {
        logger: (m) => console.log(m),
      });

      console.log("Extracted Text:", text);
      setAnimationData(parseTextToData(text));
    } catch (error) {
      console.error("OCR Error:", error);
    }
    
    setCapturing(false);
  };

  const parseTextToData = (text: string) => {
    console.log("Raw OCR Output:", text);
    const lines = text.split("\n").map(line => line.trim()).filter(line => line);
    console.log("Processed Lines:", lines);

    const data: Record<RegionName, boolean[]> = {
      "Region 1": [],
      "Region 2": [],
      "Region 3": [],
      "Region 4": [],
      "Region 5": [],
    };

    const yearData = lines.slice(1);

    yearData.forEach((line) => {
      const parts = line.split(/\s+/);
      const year = parts[0];
      const regionsData = parts.slice(1);

      regionsData.forEach((value, index) => {
        const region = `Region ${index + 1}` as RegionName;
        if (Object.prototype.hasOwnProperty.call(data, region)) {
          data[region].push(value.toUpperCase() === "Y");
        }
      });
    });

    console.log("Parsed Data:", data);
    return data;
  };

  const startAnimation = () => {
    if (!animationData) {
      console.error("No animation data available.");
      return;
    }
    setAnimationStarted(true);
    setPaused(false);
  };

  const drawAnimation = () => {
    if (!animationCanvasRef.current || !animationData || frame === 0) return;
    const ctx = animationCanvasRef.current.getContext("2d");
    if (!ctx) return;

    const year = 2016 + frame;
    ctx.clearRect(0, 0, animationCanvasRef.current.width, animationCanvasRef.current.height);
    
    const riverImg = new Image();
    riverImg.src = "/River.png";
    riverImg.onload = () => {
      if (animationCanvasRef.current && ctx) {
        ctx.drawImage(riverImg, 0, 0, animationCanvasRef.current.width, animationCanvasRef.current.height);

        Object.keys(animationData).forEach((regionKey) => {
          const region = regionKey as RegionName;
          const regionData = animationData[region];
          const regionFrameData = regionData[frame];

          if (regionFrameData) {
            const [x, y] = regions[region][0];
            ctx.beginPath();
            ctx.arc(x, y, 50, 0, 2 * Math.PI);
            ctx.fillStyle = regionColors[region];
            ctx.fill();
          }
        });
      }
    };
  };

  useEffect(() => {
    if (animationStarted && animationData && frame < 10) {
      const interval = setInterval(() => {
        setFrame((prev) => prev + 1);
      }, 1000);
      return () => clearInterval(interval);
    }
  }, [animationStarted, frame, animationData]);

  useEffect(() => {
    drawAnimation();
  }, [frame]);

  return (
    <div style={{ textAlign: "center" }}>
      <h2>Carp Animation GUI</h2>
      
      <video ref={videoRef} autoPlay playsInline width="640" height="480" style={{ border: "1px solid black" }} />
      
      <canvas ref={canvasRef} width="640" height="480" style={{ display: "none" }} />
      
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

      <canvas ref={animationCanvasRef} width="640" height="480" style={{ border: "1px solid black", marginTop: "10px" }} />
      
      {animationData && (
        <div style={{ marginTop: "20px" }}>
          <Button variant="contained" color="success" onClick={startAnimation} disabled={animationStarted}>
            Start Animation
          </Button>
          <Button variant="contained" color="success" onClick={() => setPaused(!paused)}>
            {paused ? "Resume" : "Pause"}
          </Button>
        </div>
      )}
    </div>
  );
};

export default CarpAnimationGUI;
