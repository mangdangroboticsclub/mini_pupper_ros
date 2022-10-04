import { Stack, styled, Switch, Typography } from '@mui/material'
import { observer } from 'mobx-react'
import React, { Fragment } from 'react'
import { useStore } from '../../../store'
import { IS_SIMULATION } from '../../../constants'

const ObjectDetectionSwitch = styled(Switch)(({ theme }) => ({
  width: 28,
  height: 16,
  padding: 0,
  display: 'flex',
  '&:active': {
    '& .MuiSwitch-thumb': {
      width: 15
    },
    '& .MuiSwitch-switchBase.Mui-checked': {
      transform: 'translateX(9px)'
    }
  },
  '& .MuiSwitch-switchBase': {
    padding: 2,
    '&.Mui-checked': {
      transform: 'translateX(12px)',
      color: '#fff',
      '& + .MuiSwitch-track': {
        opacity: 1,
        backgroundColor: theme.palette.mode === 'dark' ? '#177ddc' : '#1890ff'
      }
    }
  },
  '& .MuiSwitch-thumb': {
    boxShadow: '0 2px 4px 0 rgb(0 35 11 / 20%)',
    width: 12,
    height: 12,
    borderRadius: 6,
    transition: theme.transitions.create(['width'], {
      duration: 200
    })
  },
  '& .MuiSwitch-track': {
    borderRadius: 16 / 2,
    opacity: 1,
    backgroundColor: theme.palette.mode === 'dark' ? 'rgba(255,255,255,.35)' : 'rgba(0,0,0,.25)',
    boxSizing: 'border-box'
  }
}))

const ObjectDetectionControls = observer(() => {
  const { selectedMode, useNN, setNN } = useStore()

  const handleNNChange = () => {
    setNN(!useNN)
  }

  return (
    !IS_SIMULATION ?
      <Fragment>
        <Stack direction='row' spacing={1} alignItems='center'>
          <Typography paddingLeft={2}>Detect Objects</Typography>
          <ObjectDetectionSwitch
            checked={useNN && selectedMode !== 'depth'}
            disabled={selectedMode === 'depth'}
            onChange={handleNNChange}
            inputProps={{ 'aria-label': 'ant design' }}
          />
        </Stack>
      </Fragment> : null
  )
})

export default ObjectDetectionControls
