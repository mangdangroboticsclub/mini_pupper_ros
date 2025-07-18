# Copyright 2025 Kishan Grewal
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# !/usr/bin/env python3

import rclpy
from threading import Thread
from mini_pupper_tracking.tracking_node import TrackingNode
from mini_pupper_tracking.flask_server import create_flask_app
import webbrowser
import time


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()

    flask_config = {
        'image_display_size': node.get_parameter('flask.image_display_size').value,
        'frame_rate': node.get_parameter('flask.frame_rate').value
    }
    auto_open_browser = node.get_parameter('flask.auto_open_browser').value

    # Start Flask app
    app = create_flask_app(node, flask_config)

    # Auto-open browser
    def open_browser_delayed():
        time.sleep(1)  # Wait for Flask to be ready
        if auto_open_browser:
            try:
                webbrowser.open("http://localhost:5000")
            except Exception as e:
                print(f"Could not open browser: {e}")
        else:
            node.get_logger().info(
                "Auto-open browser is disabled. Visit http://localhost:5000 to view the stream."
            )

    Thread(target=open_browser_delayed, daemon=True).start()

    # Run Flask in background
    flask_thread = Thread(target=lambda: app.run(
        host="0.0.0.0", port=5000,
        debug=False, use_reloader=False, threaded=True), daemon=True)
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
