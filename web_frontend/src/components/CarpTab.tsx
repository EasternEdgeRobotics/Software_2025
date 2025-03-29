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
    canvasRef.current.toBlob((blob) => {
      if (blob) {
        const imageUrl = URL.createObjectURL(blob);
        setImageData(imageUrl);
      }
    }, "image/png");
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

  return (
    <div style={{ textAlign: "center" }}>
      <h2>Carp Animation GUI</h2>
      <video ref={videoRef} width="640" height="480" autoPlay />
      <canvas ref={canvasRef} width="640" height="480" style={{ border: "1px solid black", marginTop: "10px" }} />

      <div style={{ marginTop: "20px" }}>
        <Button variant="contained" color="primary" onClick={captureImage}>
          Capture Image
        </Button>
        <input type="file" accept="image/*" onChange={handleFileUpload} style={{ marginLeft: "10px" }} />
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
          <img src={imageData} alt="Uploaded Screenshot" width="320" height="240" style={{ border: "1px solid black" }} />
        </div>
      )}

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
