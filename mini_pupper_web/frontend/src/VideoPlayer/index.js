import React, { createRef, Fragment, useEffect } from 'react'
import { observable, computed, action, makeObservable } from 'mobx'
import { observer } from 'mobx-react'
import throttle from 'lodash.throttle'
import { BUFFERING_LEEWAY, AUTO_HIDE_CONTROLS_TIME, CameraType, IS_SIMULATION, Pose } from '../constants'
import { formatTime } from '../util'

import './style.css'

import { useStore } from '../store'
import VideoControls from '../VideoControls'
import VideoOverlay from '../VideoOverlay'
import { WebRTC } from '../WebRtc'
import RosController from '../RosController'

class VideoPlayer {
  @observable videoEl = createRef()
  @observable dataChannel = null
  @observable webrtcInstance = null
  @observable srcObject = null
  @observable displayTeleop = false
  @observable selectedMode = !IS_SIMULATION ? CameraType.RGB : CameraType.SIMULATOR
  @observable useNN = false
  @observable showControls = false
  @observable isPlaying = false
  @observable isFullscreen = false
  @observable muted = false
  @observable volume = 100
  @observable duration = null
  @observable currentTime = 0
  @observable buffering = false
  @observable loadProgress = 0
  @observable buffered = 0
  @observable timeFormat = {
    hours: true,
    minutes: true,
    seconds: true,
  }
  @observable mouseX = 0
  @observable mouseY = 0
  @observable currentAction = ''
  @observable isWSConnected = false
  @observable isWebRtcConnected = false
  @observable isTeleopReady = false
  @observable isStanding = false

  constructor() {
    makeObservable(this)
    this.rosController = new RosController(this.setIsWSConnected, this.setIsTeleopReady, this.setIsStanding)
    this.autoHideControlsTimer = null
  }

  @computed
  get formattedDuration() {
    return formatTime(this.duration, this.timeFormat)
  }

  @computed
  get isStreamStarted() {
    return this.isPlaying || this.buffering
  }

  @action
  setIsWSConnected = (value) => {
    this.isWSConnected = value
  }

  @action
  setIsTeleopReady = (value) => {
    this.isTeleopReady = value
  }

  @action
  setIsStanding = (value) => {
    console.log('Is standing', value)
    this.isStanding = value
  }

  @action
  changePose = () => {
    console.log('Changing pose')
    this.rosController.changePose(this.isStanding ? Pose.SIT : Pose.STAND)
  }

  onLoadedData = (e) => {
    const { duration } = e.target
    this.timeFormat = {
      hours: duration > 3600,
      minutes: duration > 60,
      seconds: duration > 0,
    }
    this.duration = duration
  }

  @action
  skipTo = (value) => {
    this.currentTime = value
    this.videoEl.current.currentTime = value
  }

  setIsFullscreen = () => {
    this.isFullscreen = document.fullscreenElement === this.videoEl.current.parentNode
  }

  @action
  setPlaying = () => {
    if (this.ended) {
      this.ended = false
      this.skipTo(0)
    }

    this.isPaused = false
    this.isPlaying = true
  }

  @action
  setPause = () => {
    this.isPaused = true
    this.isPlaying = false
  }

  @action
  setMuted = (value) => {
    this.videoEl.current.muted = value
  }

  @action
  onMessage = (evt) => {
    const action = JSON.parse(evt.data)
    console.log(action)
  }

  @action
  setWebRTCStatus = (value) => {
    this.isWebRtcConnected = value
  }

  @action
  start = async () => {
    const webrtc = new WebRTC(this.setWebRTCStatus)
    this.setWebrtcInstance(webrtc)

    const dataChannel = webrtc.createDataChannel(
      'pingChannel',
      () => console.log('[DC] closed'),
      () => console.log('[DC] opened'),
      this.onMessage
    )
    this.setDataChannel(dataChannel)

    webrtc.addMediaHandles((evt) => {
      if (evt.track.kind === 'video') {
        this.setVideoSource(evt.streams[0])
        this.setPlaying()
      }
    })

    try {
      await webrtc.start(this.selectedMode, this.useNN)
    } catch (error) {
      console.error(error.message)
      await this.onClick()
    }
  }

  @action
  stop = () => {
    if (this.dataChannel && this.dataChannel.readyState === 'open') {
      this.dataChannel.send(
        JSON.stringify({
          type: 'STREAM_CLOSED',
        })
      )
    }
    this.stopWebRtc()
  }

  @action
  stopWebRtc = () => {
    setTimeout(() => {
      this.webrtcInstance.stop()
      this.handleStopBuffering()
      this.setPause()
      this.handleEnded()
      this.setVideoSource(null)
      this.setDuration(null)
      this.setWebrtcInstance(null)
      this.setDataChannel(null)
    }, 100)
  }

  @action
  setDataChannel = (value) => {
    this.dataChannel = value
  }

  @action
  setWebrtcInstance = (value) => {
    this.webrtcInstance = value
  }

  @action
  setVideoSource = (value) => {
    this.videoEl.current.srcObject = value
  }

  @action
  setDuration = (value) => {
    this.duration = value
  }

  @action
  setDisplayTeleop = (value) => {
    this.displayTeleop = value
  }

  @action
  setMode = (value) => {
    this.selectedMode = value
  }

  @action
  setNN = (value) => {
    this.useNN = value
  }

  @action
  setPaused = () => {
    this.videoEl.current.paused = true
  }

  @action
  setMouseX = (value) => {
    this.mouseX = value
  }

  @action
  setMouseY = (value) => {
    this.mouseY = value
  }

  @action
  setCurrentAction = (value) => {
    this.currentAction = value
  }

  attachEvents = () => {
    const { current: videoElement } = this.videoEl

    videoElement.addEventListener('play', this.start)
    videoElement.addEventListener('pause', this.stop)
    videoElement.addEventListener('loadeddata', this.onLoadedData)
    videoElement.addEventListener('timeupdate', this.handleTimeUpdate)
    videoElement.addEventListener('volumechange', this.handleVolumeChange)
    videoElement.addEventListener('waiting', this.handleStartBuffering)
    videoElement.addEventListener('playing', this.handleStopBuffering)
    videoElement.addEventListener('ended', this.handleEnded)
    videoElement.parentNode.addEventListener('keydown', this.handleKeyboardShortcuts)
    videoElement.parentNode.addEventListener('fullscreenchange', this.setIsFullscreen)
    this.rosController.connect()
  }

  detachEvents = () => {
    const { current: videoElement } = this.videoEl

    videoElement.removeEventListener('play', this.start)
    videoElement.removeEventListener('pause', this.stop)
    videoElement.removeEventListener('timeupdate', this.handleTimeUpdate)
    videoElement.removeEventListener('volumechange', this.handleVolumeChange)
    videoElement.removeEventListener('waiting', this.handleStartBuffering)
    videoElement.removeEventListener('playing', this.handleStopBuffering)
    videoElement.removeEventListener('ended', this.handleEnded)
    videoElement.parentNode.removeEventListener('keydown', this.handleKeyboardShortcuts)
    videoElement.parentNode.removeEventListener('fullscreenchange', this.setIsFullscreen)
    this.rosController.disconnect()
  }

  handleKeyboardShortcuts = throttle((e) => {
    const keyName = e.key

    if ([' ', 'spacebar'].includes(keyName)) {
      this.onClick()
    } else {
      if (this.isTeleopReady) {
        this.rosController.publishKey(keyName)
      }
    }
  }, 50)

  @action
  handleEnded = () => {
    this.ended = true
  }

  handleStartBuffering = () => {
    clearTimeout(this.bufferingTimeout)
    this.bufferingTimeout = setTimeout(() => {
      this.buffering = true
    }, BUFFERING_LEEWAY)
  }

  handleStopBuffering = () => {
    clearTimeout(this.bufferingTimeout)
    this.buffering = false
  }

  @action
  handleVolumeChange = throttle(
    (e) => {
      this.volume = e.target.volume
      this.muted = e.target.muted
    },
    200,
    { leading: true, trailing: true }
  )

  handleTimeUpdate = throttle(
    (e) => {
      this.currentTime = Math.round(e.target.currentTime)
      return this.currentTime
    },
    200,
    { leading: true }
  )

  pause = () => {
    this.videoEl.current.pause()
  }

  @action
  setVolume = (value) => {
    this.videoEl.current.volume = value
    this.setMuted(value === 0)
  }

  @action
  handleControlsAutoHide = () => {
    clearTimeout(this.autoHideControlsTimer)

    this.showControls = true
    this.autoHideControlsTimer = setTimeout(() => {
      if (!this.videoEl.current.paused) {
        this.onMouseLeave()
      }
    }, AUTO_HIDE_CONTROLS_TIME)
  }

  onMouseMove = throttle(this.handleControlsAutoHide, 200, {
    leading: true,
    trailing: false,
  })

  @action
  onClick = async () => {
    this.handleControlsAutoHide()

    try {
      if (this.videoEl.current.paused) {
        await this.videoEl.current.play()
      } else {
        await this.videoEl.current.pause()
      }
    } catch (error) {
      console.log('Playback was interrupted')
    }
  }

  @action
  onDoubleClick = () => this.toggleFullscreen()

  @action
  toggleFullscreen = async () => {
    if (document.fullscreenElement === this.videoEl.current.parentNode) {
      await document.exitFullscreen()
    } else {
      await this.videoEl.current.parentNode.requestFullscreen()
    }
  }

  @action
  onMouseEnter = () => {
    this.handleControlsAutoHide()
  }

  @action
  onMouseOver = () => {
    this.handleControlsAutoHide()
  }

  @action
  onMouseLeave = () => {
    clearTimeout(this.autoHideControlsTimer)
    if (this.isPlaying) {
      this.showControls = false
    }
  }
}

const VideoPlayerView = observer((props) => {
  const { videoEl, onMouseEnter, onMouseLeave, onMouseOver, onMouseMove, showControls, attachEvents, detachEvents } =
    useStore()

  useEffect(() => {
    attachEvents()

    return () => {
      detachEvents()
    }
  }, [attachEvents, detachEvents])

  useEffect(() => {
    videoEl.current.parentNode.focus()
  }, [videoEl])

  return (
    <Fragment>
      <div
        tabIndex={-1}
        className='video-wrapper'
        style={{
          cursor: showControls ? 'default' : 'none',
        }}
        {...{ onMouseLeave, onMouseMove }}
      >
        <VideoOverlay />
        <video ref={videoEl} className='video' playsInline {...{ onMouseEnter, onMouseOver }} {...props} />
        <VideoControls />
      </div>
    </Fragment>
  )
})

export { VideoPlayer, VideoPlayerView }
