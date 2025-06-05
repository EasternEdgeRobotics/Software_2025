import { Box, Grid } from "@mui/material";

export function AltCamera(props: { ip: string }) {
  return (
    <Box
      sx={{
        width: "100%",
        aspectRatio: "16 / 9",
        maxHeight: "45vh",
        backgroundImage: `url(${props.ip}), url(./nosignal.jpg)`,
        backgroundSize: "100% 100%",
        borderRadius: "12px",
      }}
    />
  );
}

function CameraTab() {
  return (
    <Box alignSelf="center">
      <Grid container spacing={2}>
        <Grid size={6}>
          <AltCamera ip="http://192.168.137.20:8080/stream" />
        </Grid>
        <Grid size={6}>
          <AltCamera ip="http://192.168.137.21:8080/stream" />
        </Grid>
        <Grid size={6}>
          <AltCamera ip="" /> {/** TODO: PUT PHOTOSPHERE CAMERA IP HERE **/}
        </Grid>
        <Grid size={6}>
          <AltCamera ip="" />
        </Grid>
      </Grid>
    </Box>
  );
}

export default CameraTab;