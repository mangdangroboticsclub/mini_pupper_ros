from pydantic import BaseModel

from .camera_resolution import CameraResolution
from .camera_type import CameraType
from .median_filter import MedianFilter


class Options(BaseModel):
    camera_type: CameraType = CameraType.RGB
    cam_width: int = 300
    cam_height: int = 300
    nn_model: str = ''
    mono_camera_resolution: CameraResolution
    median_filter: MedianFilter = MedianFilter.KERNEL_7x7
    subpixel: bool = False
    extended_disparity: bool = False
