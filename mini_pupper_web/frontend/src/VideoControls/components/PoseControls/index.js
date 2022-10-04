import { observer } from 'mobx-react'
import { useStore } from '../../../store'
import { Grid, Icon, Tooltip } from '@mui/material'
import { ArrowCircleUp, ArrowCircleDown } from '@mui/icons-material'
import React from 'react'

const PoseControls = observer(() => {
  const { isStanding, changePose } = useStore()

  const poseIcon = () => {
    return isStanding ? ArrowCircleDown : ArrowCircleUp
  }

  const onClick = () => {
    console.log('Clicking')
    changePose()
  }

  return (
    <Grid item>
      <Tooltip title={`${isStanding ? 'Sit' : 'Stand'}`} placement='top'>
        <Icon component={poseIcon()} onClick={onClick} />
      </Tooltip>
    </Grid>
  )
})

export default PoseControls
