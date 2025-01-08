import React from "react";
import { Button, Box, Grid, Paper } from "@mui/material";
import CarpAnimation from "./CarpAnimation";

export default function CarpTab() {
  return (
    <Box>
      <Grid container spacing={2}>
        <Grid item xs={12}>
          <Paper elevation={3}>
            <CarpAnimation />
          </Paper>
        </Grid>
        <Grid item xs={12}>

          <Button variant="contained" color="primary">
            Begin Animation
          </Button>
        </Grid>
      </Grid>
    </Box>
  );
}
