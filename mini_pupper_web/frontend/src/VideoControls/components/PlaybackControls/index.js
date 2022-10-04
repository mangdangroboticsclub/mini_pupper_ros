import { observer } from 'mobx-react'
import { useStore } from '../../../store'
import { Grid, Icon, Tooltip } from '@mui/material'
import { Pause, PlayArrow } from '@mui/icons-material'
import React from 'react'

const PlaybackControls = observer(() => {
  const { isStreamStarted, onClick } = useStore()

  const playIcon = () => {
    return isStreamStarted ? Pause : PlayArrow
  }

  return (
    <Grid item>
      <Tooltip title={`${isStreamStarted ? 'Pause' : 'Play'}`} placement='top'>
        <Icon component={playIcon()} onClick={onClick} />
      </Tooltip>
    </Grid>
  )
})

export default PlaybackControls
