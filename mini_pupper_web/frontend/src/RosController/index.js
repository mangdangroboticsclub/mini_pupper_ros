import ROSLIB from 'roslib'
import { TEXT_MESSAGE_TYPE, STATUS_MESSAGE_TYPE, KEY_TOPIC, RECONNECTION_TIMER, ROSBRIDGE_CONNECTION_URL, TELEOP_TOPIC, POSE_STATE_TOPIC, POSE_CHANGE_TOPIC } from '../constants'

const Ros = ROSLIB.Ros
const RosTopic = ROSLIB.Topic
const RosMessage = ROSLIB.Message

class RosController {

  constructor(setIsWSConnected, setIsTeleopReady, setIsStanding) {
    this.setIsWSConnected = setIsWSConnected
    this.setIsTeleopReady = setIsTeleopReady
    this.setIsStanding = setIsStanding
    this.keyTopic = null
    this.teleopTopic = null
    this.ros = new Ros()
    this.ros.on('connection', () => {
      console.info('Connected to ROS bridge')
      this.setIsWSConnected && this.setIsWSConnected(true)
      this.keyTopic = new RosTopic({
        ros: this.ros,
        name: KEY_TOPIC,
        messageType: TEXT_MESSAGE_TYPE,
      })
      this.teleopTopic = new RosTopic({
        ros: this.ros,
        name: TELEOP_TOPIC,
        messageType: STATUS_MESSAGE_TYPE,
      })
      this.poseChangeTopic = new RosTopic({
        ros: this.ros,
        name: POSE_CHANGE_TOPIC,
        messageType: TEXT_MESSAGE_TYPE,
      })
      this.poseStateTopic = new RosTopic({
        ros: this.ros,
        name: POSE_STATE_TOPIC,
        messageType: STATUS_MESSAGE_TYPE,
      })
      this.teleopTopic.subscribe((msg) => this.setIsTeleopReady && this.setIsTeleopReady(msg.data))
      this.poseStateTopic.subscribe((msg) => this.setIsStanding && this.setIsStanding(msg.data))
    })
    this.ros.on('close', () => {
      console.warn('Disconnected from ROS bridge')
      this.setIsWSConnected && this.setIsWSConnected(false)
      this.setIsTeleopReady && this.setIsTeleopReady(false)
      setTimeout(() => this.connect(), RECONNECTION_TIMER)
      this.keyTopic = null
      this.teleopTopic = null
    })
    this.ros.on('error', (error) => console.log(error))
  }

  connect = () => {
    try {
      this.ros.connect(ROSBRIDGE_CONNECTION_URL)
    } catch (error) {
      console.error(`Unable to connected to ROS bridge: ${error.toString()}`)
    }
  }

  disconnect = () => {
    this.ros.close()
    this.setIsWSConnected && this.setIsWSConnected(false)
    this.setIsTeleopReady && this.setIsTeleopReady(false)
  }

  isConnected = () => {
    return this.ros.isConnected
  }

  publishKey = (key) => {
    if (this.keyTopic) {
      const keyMessage = new RosMessage({ data: key })
      this.keyTopic.publish(keyMessage)
      console.log('Published', keyMessage)
    }
  }

  changePose = (pose) => {
    if (this.poseChangeTopic) {
      const poseMessage = new RosMessage({ data: pose })
      this.poseChangeTopic.publish(poseMessage)
      console.log('Sent', poseMessage)
    }
  }
}

export default RosController
