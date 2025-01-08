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

  const [inputValue, setInputValue] = useState("");

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setInputValue(event.target.value);
  };

  const handleApplyData = () => {
    try {
      const parsedData = JSON.parse(inputValue); // Expecting a JSON string for the data structure
      setGraphData(parsedData);
    } catch (error) {
      alert("Invalid data format. Please enter valid JSON.");
    }
  };

  return (
    <Box>
      <Grid container spacing={2}>
        <Grid item xs={12}>
          <Paper elevation={3}>
            <CarpAnimation graphData={graphData} />
          </Paper>
        </Grid>
        <Grid item xs={12}>
          <TextField
            label="Enter Graph Data (JSON)"
            fullWidth
            multiline
            rows={6}
            value={inputValue}
            onChange={handleInputChange}
            placeholder={`{
  "Year": [2016, 2017, 2018, 2019, 2020],
  "Regions": {
    "Region 1": [true, true, true, true, true],
    "Region 2": [false, true, true, true, true],
    ...
  }
}`}
          />
        </Grid>
        <Grid item xs={12}>
          <Button variant="contained" color="primary" onClick={handleApplyData}>
            Apply Data
          </Button>
        </Grid>
      </Grid>
    </Box>
  );
}
