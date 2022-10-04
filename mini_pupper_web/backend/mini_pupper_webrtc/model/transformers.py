import base64
import cv2
import depthai as dai
import blobconverter
import numpy as np

from aiortc import VideoStreamTrack
from av import VideoFrame

from .ros_bridge import RosBridge
from .camera_resolution import CameraResolution
from .median_filter import MedianFilter
from .options import Options
from .uvicorn_logger import logger


class VideoTransformTrack(VideoStreamTrack):

    def __init__(self, options: Options):
        super().__init__()
        self.options = options
        self.dummy = False
        self.frame: np.ndarray[np.float64] = self.zero_frame
        self.detections = []
        self.labels = [
            "background",
            "aeroplane",
            "bicycle",
            "bird",
            "boat",
            "bottle",
            "bus",
            "car",
            "cat",
            "chair",
            "cow",
            "diningtable",
            "dog",
            "horse",
            "motorbike",
            "person",
            "pottedplant",
            "sheep",
            "sofa",
            "train",
            "tvmonitor"
        ]

    def get_label(self, idx):
        return self.labels[idx]

    @property
    def zero_frame(self):
        frame = np.zeros((self.options.cam_height, self.options.cam_width, 3), np.uint8)
        frame[:] = (0, 0, 0)
        return frame

    def get_frame(self):
        raise NotImplementedError('Unable to fetch frame data')

    async def return_frame(self, frame):
        pts, time_base = await self.next_timestamp()
        new_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        new_frame.pts = pts
        new_frame.time_base = time_base
        return new_frame

    def offline_frame(self):
        frame = self.zero_frame
        y, x = frame.shape[0] / 2, frame.shape[1] / 2
        left, top, right, bottom = int(x - 50), int(y - 30), int(x + 75), int(y + 75)
        cv2.rectangle(
            frame, (left, top),
            (right, bottom),
            (0, 0, 255),
            cv2.FILLED
        )
        cv2.putText(
            frame,
            "OFFLINE",
            (left, int((bottom + top) / 2 + 10)),
            cv2.FONT_HERSHEY_DUPLEX,
            1.0,
            (255, 255, 255),
            1
        )
        return frame

    async def dummy_recv(self):
        return await self.return_frame(self.offline_frame())

    async def recv(self):
        if self.dummy:
            return await self.dummy_recv()

        try:
            frame = self.get_frame()
            return await self.return_frame(frame)
        except Exception as e:
            logger.error(e)
            logger.info('Switching to dummy mode...')
            self.dummy = True
            return await self.dummy_recv()


class SimulatorVideoTransformTrack(VideoTransformTrack):

    def __init__(self, ros_bridge: RosBridge, options: Options):
        super().__init__(options)
        self.img = self.offline_frame()
        self.ros_bridge = ros_bridge
        # ToDo: think about multiple subscribers feature, move constants to env vars
        self.ros_bridge.subscribe(
            '/camera/color/image_raw/compressed',
            'sensor_msgs/CompressedImage',
            self.receive_image
        )

    def receive_image(self, msg):
        base64_bytes = msg['data'].encode('ascii')
        image_bytes = base64.b64decode(base64_bytes)
        jpg_as_np = np.frombuffer(image_bytes, dtype=np.uint8)
        self.img = cv2.imdecode(jpg_as_np, flags=1)

    def get_frame(self):
        return self.img

    def stop(self):
        super().stop()
        self.ros_bridge.unsubscribe()


class DepthAIVideoTransformTrack(VideoTransformTrack):

    def __init__(self, options: Options):
        super().__init__(options)
        self.pipeline = dai.Pipeline()
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        self.xoutRgb.setStreamName("rgb")

        # Properties
        self.camRgb.setPreviewSize(self.options.cam_width, self.options.cam_height)
        self.camRgb.setInterleaved(False)
        self.camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        # Linking
        self.camRgb.preview.link(self.xoutRgb.input)
        self.nn = None

        if options.nn_model != "":
            self.nn = self.pipeline.create(dai.node.MobileNetDetectionNetwork)
            self.nn.setConfidenceThreshold(0.5)
            self.nn.setBlobPath(blobconverter.from_zoo(options.nn_model, shaves=6))
            self.nn.setNumInferenceThreads(2)
            self.nn.input.setBlocking(False)
            self.nnOut = self.pipeline.create(dai.node.XLinkOut)
            self.nnOut.setStreamName("nn")
            self.camRgb.preview.link(self.nn.input)
            self.nn.out.link(self.nnOut.input)
        self.device = dai.Device(self.pipeline)
        self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=1, blocking=False)

        if self.nn is not None:
            self.qDet = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)

        self.device.startPipeline()

    def get_frame(self):
        frame = self.qRgb.tryGet()
        if frame:
            self.frame = frame.getCvFrame()

        if self.nn:
            inDet = self.qDet.tryGet()
            if inDet:
                self.detections = inDet.detections

        for detection in self.detections:
            bbox = self.frameNorm(
                self.frame,
                (detection.xmin, detection.ymin, detection.xmax, detection.ymax)
            )
            cv2.putText(
                self.frame,
                f"LABEL {self.labels[detection.label]}",
                (bbox[0] + 10, bbox[1] + 20),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                (255, 0, 0)
            )
            cv2.putText(
                self.frame,
                f"{int(detection.confidence * 100)}%",
                (bbox[0] + 10, bbox[1] + 40),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                (255, 0, 0)
            )
            cv2.rectangle(self.frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 0, 0), 2)
        # Show the frame
        return self.frame

    def stop(self):
        super().stop()
        if self.device:
            self.device.close()
        self.device = None

    def frameNorm(self, frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)


class DepthAIDepthVideoTransformTrack(VideoTransformTrack):

    def __init__(self, options: Options):
        super().__init__(options)
        self.device = dai.Device()

        # Check if we have stereo cameras on the device
        cams = self.device.getConnectedCameras()
        depth_enabled = dai.CameraBoardSocket.LEFT in cams and dai.CameraBoardSocket.RIGHT in cams
        if not depth_enabled:
            logger.error("You are using camera that doesn't support stereo depth!")
            super().stop()
            if self.device:
                self.device.close()
            self.device = None
            return

        self.device.startPipeline(self.create_pipeline())
        self.qDepth = self.device.getOutputQueue(name="disparity", maxSize=4, blocking=False)

    def create_pipeline(self):
        self.pipeline = dai.Pipeline()
        self.monoLeft = self.pipeline.create(dai.node.MonoCamera)
        self.monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.depth = self.pipeline.create(dai.node.StereoDepth)
        self.xoutDepth = self.pipeline.create(dai.node.XLinkOut)
        self.xoutDepth.setStreamName("disparity")

        # Properties
        mono_camera_resolution = self.options.mono_camera_resolution
        if mono_camera_resolution == CameraResolution.THE_400_P:
            self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            self.frame = np.zeros((400, 640, 3), np.uint8)
        if mono_camera_resolution == CameraResolution.THE_480_P:
            self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
            self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
            self.frame = np.zeros((480, 640, 3), np.uint8)
        elif mono_camera_resolution == CameraResolution.THE_720_P:
            self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
            self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
            self.frame = np.zeros((720, 1280, 3), np.uint8)
        elif mono_camera_resolution == CameraResolution.THE_800_P:
            self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            self.frame = np.zeros((800, 1280, 3), np.uint8)
        self.monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        self.monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
        self.depth.setConfidenceThreshold(200)
        # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
        median_filter = self.options.median_filter
        if median_filter == MedianFilter.MEDIAN_OFF:
            self.depth.setMedianFilter(dai.StereoDepthProperties.MedianFilter.MEDIAN_OFF)
        elif median_filter == MedianFilter.KERNEL_3x3:
            self.depth.setMedianFilter(dai.StereoDepthProperties.MedianFilter.KERNEL_3x3)
        elif median_filter == MedianFilter.KERNEL_5x5:
            self.depth.setMedianFilter(dai.StereoDepthProperties.MedianFilter.KERNEL_5x5)
        elif median_filter == MedianFilter.KERNEL_7x7:
            self.depth.setMedianFilter(dai.StereoDepthProperties.MedianFilter.KERNEL_7x7)
        self.depth.setExtendedDisparity(self.options.extended_disparity)
        self.depth.setSubpixel(self.options.subpixel)

        # Linking
        self.monoLeft.out.link(self.depth.left)
        self.monoRight.out.link(self.depth.right)
        self.depth.disparity.link(self.xoutDepth.input)

        return self.pipeline

    def get_frame(self):
        inDepth = self.qDepth.tryGet()
        if inDepth is not None:
            frame = inDepth.getFrame()
            frame = (frame * (255 / self.depth.getMaxDisparity())).astype(np.uint8)
            self.frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)

        return self.frame

    def stop(self):
        super().stop()
        if self.device:
            self.device.close()
        self.device = None
