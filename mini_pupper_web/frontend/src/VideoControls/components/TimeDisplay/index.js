import React from 'react'
import { observer } from 'mobx-react'
import { Grid } from '@mui/material'

import { useStore } from '../../../store'
import { formatTime } from '../../../util'
import './style.css'

const TimeDisplay = observer(() => {
  const { currentTime, timeFormat, duration } = useStore()

  return duration ? (
    <Grid item className='time-display'>
      <span>{formatTime(currentTime, timeFormat)}</span>
    </Grid>
  ) : null
})

export default TimeDisplay
