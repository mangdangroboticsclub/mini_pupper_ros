import { observer } from 'mobx-react'
import { useStore } from '../../../store'
import { Grid, Tooltip } from '@mui/material'
import CellWifiIcon from '@mui/icons-material/CellWifi'
import VideoCameraBackIcon from '@mui/icons-material/VideoCameraBack'
import GamepadIcon from '@mui/icons-material/Gamepad'
import React from 'react'
import { red, green } from '@mui/material/colors'

const StatusControls = observer(() => {
  const { isWSConnected, isWebRtcConnected, isTeleopReady } = useStore()
  const colorIntensity = 500

  const color = (condition) => {
    return {
      color: condition ? green[colorIntensity] : red[colorIntensity],
      paddingLeft: '5px',
    }
  }

  const label = (condition, prefix) => {
    return condition ? `${prefix} Connected` : `${prefix} Disconnected`
  }

  return (
    <Grid item>
      <Tooltip title={label(isWSConnected, 'ROS')} placement='top'>
        <CellWifiIcon sx={color(isWSConnected)} />
      </Tooltip>
      <Tooltip title={label(isWebRtcConnected, 'Camera')} placement='top'>
        <VideoCameraBackIcon sx={color(isWebRtcConnected)} />
      </Tooltip>
      <Tooltip title={label(isTeleopReady, 'Teleop')} placement='top'>
        <GamepadIcon sx={color(isTeleopReady)} />
      </Tooltip>
    </Grid>
  )
})

export default StatusControls
