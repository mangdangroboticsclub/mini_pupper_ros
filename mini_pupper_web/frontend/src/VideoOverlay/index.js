import React, { useEffect, useRef } from 'react'
import { CircularProgress, Icon } from '@mui/material'
import { PlayCircleFilled, PauseCircleFilled } from '@mui/icons-material'
import { observer } from 'mobx-react'

import './style.css'

import { useStore } from '../store'

const VideoOverlay = observer(() => {
  const { buffering, isPlaying } = useStore()
  const iconRef = useRef(null)

  useEffect(() => {
    // Avoid on mount animation
    iconRef.current.classList.add('hidden-animation')
  }, [])

  const progressIcon = () => {
    return isPlaying ? PlayCircleFilled : PauseCircleFilled
  }

  return (
    <div className='video-overlay'>
      {buffering ? (
        <CircularProgress />
      ) : (
        <Icon ref={(ref) => (iconRef.current = ref)} component={progressIcon()} className='status-animation' />
      )}
    </div>
  )
})

export default VideoOverlay
