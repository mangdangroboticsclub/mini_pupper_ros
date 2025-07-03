from flask import Flask, Response
import cv2
import time

def create_flask_app(node):
    app = Flask(__name__)

    @app.route('/')
    def index():
        return "<h2>Mini Pupper Tracking</h2><img src='/video_feed' width='1280'>"

    @app.route('/video_feed')
    def video_feed():
        def generate():
            while True:
                time.sleep(1 / 15.0)
                
                try:
                    # Non-blocking frame access with timeout
                    if node.frame_lock.acquire(timeout=0.1):
                        try:
                            frame = node.latest_frame.copy() if node.latest_frame is not None else None
                        finally:
                            node.frame_lock.release()
                    else:
                        # Skip this frame if lock can't be acquired
                        continue
                        
                    if frame is None:
                        continue
                        
                    success, buffer = cv2.imencode('.jpg', frame)
                    if not success:
                        continue
                        
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                           
                except Exception as e:
                    node.get_logger().error(f"Flask video feed error: {e}")
                    continue

        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

    return app