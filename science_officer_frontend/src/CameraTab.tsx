import { Box, Grid } from "@mui/material";

export function AltCamera(props: { ip: string }) {
  return (
    <Box width="100%" height="100%" display="flex" justifyContent="center" minHeight="45vh">
      <Box
        sx={{
          aspectRatio: "16 / 9",
          maxHeight: "45vh",
          backgroundImage: `url(${props.ip}), url(./nosignal.jpg)`,
          backgroundSize: "cover",
          borderRadius: "12px",
          backgroundPosition: "center",
          backgroundRepeat: "no-repeat",
        }}
      />
    </Box>
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