import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import App from './App.tsx'
import { createTheme, CssBaseline, ThemeProvider } from '@mui/material'

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <ThemeProvider theme={createTheme({palette: {mode: "dark"}})}>
      <CssBaseline />
      <App />
    </ThemeProvider>
  </StrictMode>,
)
