from __future__ import annotations

import uuid

from typing import Callable, Optional
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCDataChannel, RTCRtpTransceiver
from fastapi import HTTPException

from mini_pupper_webrtc.dto.offer_payload import OfferPayload
from .ros_bridge import RosBridge

from .camera_type import CameraType
from .datachannel import DataChannel
from .event_type import EventType
from .ice_connection_state import IceConnectionState
from .tranciever_kind import TrancieverKind
from .transformers import DepthAIDepthVideoTransformTrack, DepthAIVideoTransformTrack, SimulatorVideoTransformTrack, \
    VideoTransformTrack
from .uvicorn_logger import logger
from ..dto.webrtc_response import WebRTCResponse


class PeerConnection(RTCPeerConnection):
    def __init__(self, ros_bridge: RosBridge, offer_payload: OfferPayload, on_close: Callable[[PeerConnection], None]):
        super().__init__()
        self.ros_bridge = ros_bridge
        self.offer_payload = offer_payload
        self.options = self.offer_payload.options
        self.on_close = on_close
        self.channel: Optional[DataChannel] = None
        self.track: Optional[VideoTransformTrack] = None
        self.id = f"PeerConnection({uuid.uuid4()})"
        self.on(
            EventType.ICE_CONNECTION_STATE_CHANGE,
            self.on_ice_connection_state_change
        )
        self.on(EventType.TRACK, self.on_track)
        self.on(EventType.DATA_CHANNEL, self.on_data_channel)

    async def answer(self) -> WebRTCResponse:
        await self.setLocalDescription(await self.createAnswer())

        return WebRTCResponse(
            sdp=self.localDescription.sdp,
            type=self.localDescription.type
        )

    async def offer(self):
        await self.setRemoteDescription(RTCSessionDescription(sdp=self.sdp, type=self.type))

        for transceiver in self.getTransceivers():
            if transceiver.kind == TrancieverKind.VIDEO:
                await self.add_video_track()

    async def on_ice_connection_state_change(self):
        logger.info(f"ICE connection: {self.iceConnectionState}")
        if self.iceConnectionState == IceConnectionState.FAILED:
            await self.cleanup()

    @staticmethod
    def on_track(track: RTCRtpTransceiver):
        logger.info(f"{track.kind.upper()} track received")

    def on_data_channel(self, channel: RTCDataChannel):
        self.channel = DataChannel(channel)

    async def add_video_track(self):
        camera_type = self.options.camera_type

        try:
            if camera_type == CameraType.RGB:
                self.track = DepthAIVideoTransformTrack(self.options)
            elif camera_type == CameraType.DEPTH:
                self.track = DepthAIDepthVideoTransformTrack(self.options)
            elif camera_type == CameraType.SIMULATOR:
                self.track = SimulatorVideoTransformTrack(self.ros_bridge, self.options)
            else:
                logger.error(f'Unsupported camera type: {camera_type}')
        except RuntimeError as e:
            logger.error(e)
            await self.cleanup()
            raise HTTPException(status_code=500, detail="No available devices found")

        if self.track:
            self.addTrack(self.track)

    async def cleanup(self):
        if self.track:
            self.track.stop()
        await self.close()
        self.on_close(self)

    @property
    def sdp(self):
        return self.offer_payload.sdp

    @property
    def type(self):
        return self.offer_payload.type
