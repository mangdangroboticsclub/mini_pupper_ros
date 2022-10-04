import React, { Fragment } from 'react'
import { observer } from 'mobx-react'
import { useStore } from '../../../store'
import VolumeControls from '../../components/VolumeControls'
import TimeDisplay from '../../components/TimeDisplay'
import ObjectDetectionControls from '../../components/ObjectDetectionSwitch'
import CameraControls from '../../components/CameraControls'
import PlaybackControls from '../../components/PlaybackControls'
import StatusControls from '../../components/StatusControls'
import PoseControls from '../../components/PoseControls'

const Left = observer(() => {
  const { isStreamStarted } = useStore()

  return (
    <Fragment>
      <StatusControls />
      <PoseControls />
      <PlaybackControls />
      <VolumeControls />
      <TimeDisplay />
      {!isStreamStarted ? (
        <Fragment>
          <CameraControls />
          <ObjectDetectionControls />
        </Fragment>
      ) : null}
    </Fragment>
  )
})

export default Left
