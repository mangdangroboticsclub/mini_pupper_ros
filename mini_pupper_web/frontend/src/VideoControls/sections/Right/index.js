import React from 'react'
import { observer } from 'mobx-react'
import { Grid, Tooltip, Icon } from '@mui/material'
import { Fullscreen, FullscreenExit } from '@mui/icons-material'

import { useStore } from '../../../store'

const Right = observer(() => {
  const { isFullscreen, toggleFullscreen } = useStore()

  const fullscreenIcon = () => {
    return isFullscreen ? FullscreenExit : Fullscreen
  }

  return (
    <Grid item className='right-side-icons'>
      <Tooltip title='Full screen' placement='top'>
        <Icon component={fullscreenIcon()} onClick={toggleFullscreen} />
      </Tooltip>
    </Grid>
  )
})

export default Right
