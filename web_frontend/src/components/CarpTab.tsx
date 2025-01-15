import React, { useState } from "react";
import CarpAnimation from "./CarpAnimation";
import { Button } from "@mui/material";

export default function CarpTab() {
  const [startAnimation, setStartAnimation] = useState(false);

  const graphData = {
    Year: [2016, 2017, 2018, 2019, 2020, 2021, 2022, 2023, 2024, 2025],
    Regions: {
      "Region 1": [false, true, true, true, true, true, true, true, true, true],
      "Region 2": [false, false, true, true, true, true, true, true, true, true],
      "Region 3": [false, false, false, true, true, true, true, true, true, true],
      "Region 4": [false, false, false, false, true, true, true, true, true, true],
      "Region 5": [false, false, false, false, false, true, true, true, true, false],
    },
  };

  const handleStart = () => {
    setStartAnimation(true);
  };

  return (
    <div>
      <Button variant="contained" color="primary" onClick={handleStart}>
        Start Animation
      </Button>
      <CarpAnimation graphData={graphData} startAnimation={startAnimation} />
    </div>
  );
}
