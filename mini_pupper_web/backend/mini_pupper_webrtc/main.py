import asyncio

from typing import Set

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware

from mini_pupper_webrtc.dto.offer_payload import OfferPayload

from .model.ros_bridge import RosBridge
from .model.camera_type import CameraType

from mini_pupper_webrtc.model.peer_connection import PeerConnection
from mini_pupper_webrtc.model.uvicorn_logger import logger
from mini_pupper_webrtc.dto.webrtc_response import WebRTCResponse

ros_bridge: RosBridge = RosBridge()
peer_connections: Set[PeerConnection] = set()
app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
    allow_credentials=True
)


@app.post('/offer', description='Establish WebRTC connection', response_model=WebRTCResponse)
async def offer(offer_payload: OfferPayload, request: Request):
    if offer_payload.options.camera_type == CameraType.SIMULATOR:
        ros_bridge.connect()

    peer_connection = PeerConnection(ros_bridge, offer_payload, on_close=on_connection_close)
    peer_connections.add(peer_connection)
    await peer_connection.offer()
    logger.info(f"Created offer for {request.client.host}")
    return await peer_connection.answer()


@app.on_event("startup")
def startup_event():
    ros_bridge.connect()


@app.on_event("shutdown")
async def on_shutdown():
    coroutines = [pc.close() for pc in peer_connections]
    await asyncio.gather(*coroutines)
    peer_connections.clear()
    ros_bridge.shutdown()


def on_connection_close(connection: PeerConnection):
    peer_connections.discard(connection)
