import React, { Fragment } from 'react'
import { observer } from 'mobx-react'
import { useStore } from '../../../store'
import { lightBlue } from '@mui/material/colors'
import { Radio, RadioGroup, FormControlLabel, FormControl } from '@mui/material'
import { IS_SIMULATION } from '../../../constants'

const CameraControls = observer(() => {
  const { setMode, selectedMode, setNN } = useStore()

  const handleCameraModeChange = (event) => {
    setMode(event.target.value)
    if (selectedMode === 'depth') {
      setNN(false)
    }
  }

  const cameraLabel = (name) => {
    return (
      <FormControlLabel
        labelPlacement='start'
        value={name}
        control={
          <Radio
            size='small'
            sx={{
              color: lightBlue[800],
              '&.Mui-checked': {
                color: lightBlue[600]
              }
            }}
          />
        }
        label={name.toUpperCase()}
      />
    )
  }

  return (
    <Fragment>
      <FormControl>
        <RadioGroup
          row
          aria-labelledby='camera-radio-buttons-group-label'
          defaultValue='rgb'
          name='radio-buttons-group'
          value={selectedMode}
          onChange={handleCameraModeChange}
        >
          {!IS_SIMULATION ?
            <Fragment>
              {cameraLabel('rgb')}
              {cameraLabel('depth')}
            </Fragment> : <Fragment>{cameraLabel('simulator')}</Fragment>
          }
        </RadioGroup>
      </FormControl>
    </Fragment>
  )
})

export default CameraControls
