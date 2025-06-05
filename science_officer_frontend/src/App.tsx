import { Box, Tab, Tabs } from "@mui/material";
import { Camera, TestTube } from "lucide-react";
import { useState } from "react";
import CameraTab from "./CameraTab";
import ScienceTab from "./ScienceTab";

function App() {
  const [tabIndex, setTabIndex] = useState<number>(0);

  return (
    <Box height="100vh">
      <Tabs value={tabIndex} onChange={(_, index) => setTabIndex(index)} sx={{marginBottom: "8px"}} centered>
        <Tab label={<Camera />} />
        <Tab label={<TestTube />} />
      </Tabs>
      {tabIndex ? <ScienceTab /> : <CameraTab />}
    </Box>
  )
}

export default App
