import React from 'react'
import { observer } from 'mobx-react'
import cx from 'classnames'
import './style.css'

import { Grid } from '@mui/material'
import { useStore } from '../store'

import Right from './sections/Right'
import Left from './sections/Left'

const VideoControls = observer((props) => {
  const { showControls } = useStore()

  return (
    <div className={cx('video-controls', { show: showControls })} {...props}>
      <Grid container direction='column' className='bottom-section'>
        <Grid container item direction='row' justifyContent='flex-start' alignItems='center' {...props}>
          <Left />
          <Right />
        </Grid>
      </Grid>
    </div>
  )
})

export default VideoControls
