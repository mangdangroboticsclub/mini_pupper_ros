import React, { useCallback, useState, useEffect, useRef } from 'react'
import { observer } from 'mobx-react'
import cx from 'classnames'
import { Icon, Slider, Grid, Tooltip } from '@mui/material'
import { VolumeUp, VolumeDown, VolumeMute } from '@mui/icons-material'
import './style.css'

import { useStore } from '../../../store'

const VolumeControls = observer(() => {
  const { volume, setVolume, muted, setMuted } = useStore()

  const anchorRef = useRef(null)
  const [volumeValue, setVolumeValue] = useState(volume)
  const [isOpen, setIsOpen] = useState(false)
  const [isMouseDown, setMouseDown] = useState(false)

  const handleChange = useCallback(
    (e, newValue) => {
      if (Number.isNaN(newValue)) return
      setVolumeValue(newValue)
      setVolume(newValue / 100)
    },
    [setVolume]
  )

  const handleClick = useCallback(
    (e) => {
      if (anchorRef.current.contains(e.target)) {
        return
      }

      setIsOpen(false)
    },
    [anchorRef, setIsOpen]
  )

  const handleOnMouseLeave = useCallback(() => {
    if (!isMouseDown) {
      setIsOpen(false)
    }
  }, [isMouseDown])

  const handleOnMouseUp = useCallback(() => {
    setMouseDown(false)
  }, [])

  const handleOnVolumeIconClick = useCallback(() => {
    setMuted(!muted)
    setMouseDown(false)
  }, [muted, setMuted])

  const volumeIcon = (volume, muted) => {
    if (muted || volume === 0) {
      return VolumeMute
    }

    if (volume < 1 && volume > 0) {
      return VolumeDown
    }

    return VolumeUp
  }

  useEffect(() => {
    document.addEventListener('mousedown', handleClick)
    document.addEventListener('mouseup', handleOnMouseUp)

    return () => {
      document.removeEventListener('mousedown', handleClick)
      document.removeEventListener('mouseup', handleOnMouseUp)
    }
  }, [handleClick, handleOnMouseUp])

  return (
    <Grid
      ref={anchorRef}
      item
      onMouseEnter={() => {
        setIsOpen(true)
      }}
      onMouseLeave={handleOnMouseLeave}
    >
      <Tooltip title='Mute' placement='top'>
        <Icon component={volumeIcon(volume, muted)} onClick={handleOnVolumeIconClick} />
      </Tooltip>
      <div
        className={cx('volume-slider', { open: isOpen })}
        onMouseDown={() => setMouseDown(true)}
        onMouseUp={handleOnMouseUp}
      >
        <Slider min={0} max={100} value={muted ? 0 : volumeValue} onChange={handleChange} />
      </div>
    </Grid>
  )
})

export default VolumeControls
