import {BE_URL, CameraType, NNType} from '../constants'
import rgb from './mobilenet-ssd.json'
import depth from './depth.json'
import simulator from './simulator.json'

export class WebRTC {
  constructor(setWebRTCStatus) {
    this.setWebRTCStatus = setWebRTCStatus
    this.pc = new RTCPeerConnection()

    // register some listeners to help debugging
    this.pc.addEventListener('icegatheringstatechange', this.onGather, false)
    console.log('[PC] ICE Gathering state: ', this.pc.iceGatheringState)

    this.pc.addEventListener('iceconnectionstatechange', this.onConnect, false)
    console.log('[PC] ICE Connection state: ', this.pc.iceConnectionState)

    this.pc.addEventListener('signalingstatechange', this.onSignal, false)
    console.log('[PC] Signaling state: ', this.pc.signalingState)
  }

  async negotiate(selectedMode, nn) {
    const offer = await this.pc.createOffer()
    await this.pc.setLocalDescription(offer)
    await new Promise((resolve) => {
      if (this.pc.iceGatheringState === 'complete') {
        resolve()
      } else {
        const pc = this.pc
        const checkState = () => {
          if (pc.iceGatheringState === 'complete') {
            pc.removeEventListener('icegatheringstatechange', checkState)
            resolve()
          }
        }
        this.pc.addEventListener('icegatheringstatechange', checkState)
      }
    })
    const response = await fetch(`${BE_URL}/offer`, {
      body: JSON.stringify({
        sdp: this.pc.localDescription.sdp,
        type: this.pc.localDescription.type,
        options: selectedMode === CameraType.RGB
            ? { ...rgb, nn_model: nn ? NNType.MOBILENET_SSD : NNType.NONE }
            : selectedMode === CameraType.DEPTH ? depth : simulator
      }),
      headers: {
        'Content-Type': 'application/json',
      },
      method: 'POST',
    })

    const answer = await response.json()
    if (response.ok) {
      this.setWebRTCStatus && this.setWebRTCStatus(true)
      return this.pc.setRemoteDescription(answer)
    }

    throw new Error(`Unable to start a stream: ${JSON.stringify(answer.detail)}`)
  }

  async start(selectedMode, nn) {
    return this.negotiate(selectedMode, nn)
  }

  createDataChannel(name, onClose, onOpen, onMessage) {
    const dc = this.pc.createDataChannel(name, { ordered: true })
    dc.onclose = onClose
    dc.onopen = onOpen
    dc.onmessage = onMessage
    return dc
  }

  onCheckState = () => {
    if (this.pc.iceGatheringState === 'complete') {
      this.pc.removeEventListener('icegatheringstatechange', this.onCheckState)
    }
  }

  onGather = () => {
    console.log('[PC] ICE Gathering state: ', this.pc.iceGatheringState)
  }

  onConnect = () => {
    console.log('[PC] ICE Connection state: ', this.pc.iceConnectionState)
  }

  onSignal = () => {
    console.log('[PC] Signaling state: ', this.pc.signalingState)
  }

  onTrack = (onVideo) => (evt) => {
    if (evt.track.kind === 'video' && onVideo) return onVideo(evt)
  }

  stop() {
    if (this.pc.getTransceivers) {
      this.pc.getTransceivers().forEach((transceiver) => transceiver.stop && transceiver.stop())
    }

    this.pc.getSenders().forEach((sender) => sender.track && sender.track.stop())

    this.pc.removeEventListener('icegatheringstatechange', this.onGather, false)
    this.pc.removeEventListener('iceconnectionstatechange', this.onConnect, false)
    this.pc.removeEventListener('signalingstatechange', this.onSignal, false)
    this.pc.removeEventListener('track', this.onTrack(this.onVideo))
    this.pc.close()
    this.setWebRTCStatus && this.setWebRTCStatus(false)
  }

  addMediaHandles(onVideo) {
    this.onVideo = onVideo
    if (onVideo) {
      this.pc.addTransceiver('video')
      this.pc.addEventListener('track', this.onTrack(onVideo))
    }
  }
}
