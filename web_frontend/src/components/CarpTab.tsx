import React, { useState } from "react";
import { Button, Box, Grid, Paper, TextField } from "@mui/material";
import CarpAnimation from "./CarpAnimation";

export default function CarpTab() {
  const [graphData, setGraphData] = useState({
    Year: [2016, 2017, 2018, 2019, 2020],
    Regions: {
      "Region 1": [true, true, true, true, true],
      "Region 2": [false, true, true, true, true],
      "Region 3": [false, false, true, true, true],
      "Region 4": [false, false, false, true, true],
      "Region 5": [false, false, false, false, true],
    },
  });
  const [startAnimation, setStartAnimation] = useState(false);
  const [inputValue, setInputValue] = useState("");

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setInputValue(event.target.value);
  };

  const handleStartAnimation = () => {
    setStartAnimation((prev) => !prev); 
  };

  return (
        <Grid item xs={12}>
          <Button variant="contained" color="primary" onClick={handleStartAnimation}>
            Start Animation
          </Button>
        </Grid>
  );
}
