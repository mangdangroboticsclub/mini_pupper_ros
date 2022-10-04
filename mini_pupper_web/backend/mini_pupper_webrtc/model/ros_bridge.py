from typing import Callable, Optional
from roslibpy import Ros, Topic
from mini_pupper_webrtc.model.uvicorn_logger import logger


class RosBridge:
    ros: Optional[Ros] = None
    subscriber: Optional[Topic] = None

    def connect(self, host='localhost', port=9090):    
        try:
            if not self.is_connected:
                self.ros = Ros(host, port)
                self.ros.run()
                logger.info('Connected to ROS bridge.')
        except Exception as e:
            self.ros = None
            logger.error(f'Simulation is impossible: {e}.')

    def shutdown(self):
        if self.is_connected and self.subscriber:
            self.subscriber.unsubscribe()

        if self.ros:
            self.ros.terminate()

    @property
    def is_connected(self):
        return self.ros and self.ros.is_connected

    def subscribe(self, topic: str, message_type: str, callback: Callable):
        if self.is_connected:
            if self.subscriber:
                self.subscriber.unsubscribe()
                self.subscriber = None

            self.subscriber = Topic(self.ros, topic, message_type)
            self.subscriber.subscribe(callback)

    def unsubscribe(self):
        if self.subscriber:
            self.subscriber.unsubscribe()
