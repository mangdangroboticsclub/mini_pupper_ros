import React from 'react'
import { ThemeProvider, createTheme } from '@mui/material/styles'
import red from '@mui/material/colors/red'
import './app.css'
import poster from './Media/pupper_cool.jpeg'
import { configure } from "mobx"

import { StoreProvider } from './store'
import { VideoPlayerView } from './VideoPlayer'

configure({
    enforceActions: "never"
})

const theme = createTheme({
  palette: {
    primary: red,
  },
})

function App() {
  return (
    <div className='App'>
      <StoreProvider>
        <ThemeProvider theme={theme}>
          <VideoPlayerView poster={poster} controls={false} />
        </ThemeProvider>
      </StoreProvider>
    </div>
  )
}

export default App
