export const CameraType = {
    DEPTH: 'depth',
    RGB: 'rgb',
    SIMULATOR: 'simulator'
}

export const NNType = {
    MOBILENET_SSD: 'mobilenet-ssd',
    NONE: ''
}

export const Pose = {
    SIT: 'sit',
    STAND: 'stand'
}

export const ENV = {
    DEV: 'development',
    PROD: 'production'
}

const PREFIX = process.env.NODE_ENV === ENV.PROD ? import.meta.env : process.env
export const BUFFERING_LEEWAY = +(PREFIX.REACT_APP_BUFFERING_LEEWAY || 1000)
export const AUTO_HIDE_CONTROLS_TIME = +(PREFIX.REACT_APP_AUTO_HIDE_CONTROLS_TIME || 4000)
export const ROSBRIDGE_SERVER_IP = PREFIX.REACT_APP_ROSBRIDGE_SERVER_IP || '127.0.0.1'
export const ROSBRIDGE_SERVER_PORT = PREFIX.REACT_APP_ROSBRIDGE_SERVER_PORT || '9090'
export const ROSBRIDGE_CONNECTION_URL = `ws://${ROSBRIDGE_SERVER_IP}:${ROSBRIDGE_SERVER_PORT}`
export const BE_URL = PREFIX.REACT_APP_BE_URL || '127.0.0.1'
export const RECONNECTION_TIMER = +(PREFIX.REACT_APP_RECONNECTION_TIMER || 1000)

export const KEY_TOPIC = PREFIX.REACT_APP_KEY_TOPIC || '/key'
export const TELEOP_TOPIC = PREFIX.REACT_APP_TELEOP_TOPIC || '/teleop_status'
export const POSE_STATE_TOPIC = PREFIX.REACT_APP_POSE_TOPIC || '/pose/is_standing'
export const POSE_CHANGE_TOPIC = PREFIX.REACT_APP_POSE_TOPIC || '/pose/change'
export const TEXT_MESSAGE_TYPE = 'std_msgs/String'
export const STATUS_MESSAGE_TYPE = 'std_msgs/Bool'

export const IS_SIMULATION = PREFIX.REACT_APP_IS_SIMULATION === 'true' || false
