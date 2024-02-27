from flask import Flask, render_template, request, jsonify
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

app = Flask(__name__)

# ROS initialization
rospy.init_node('flask_teleop', anonymous=True)
twist_pub = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel', Twist, queue_size=10)


# Function to handle Twist messages based on button presses
def handle_twist(direction):
    twist_msg = Twist()

    dir_val = direction.lower()
    
    if dir_val == 'forward':
        twist_msg.linear.x = 1.0
    elif dir_val == 'backward':
        twist_msg.linear.x = -1.0
    elif dir_val == 'left':
        twist_msg.angular.z = 1.0
    elif dir_val == 'right':
        twist_msg.angular.z = -1.0

    twist_pub.publish(twist_msg)

# Function to stop the robot
def stop_robot():
    twist_msg = Twist()
    twist_pub.publish(twist_msg)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/control', methods=['POST'])
def control():
    direction = request.form.get('direction')

    if direction:
        handle_twist(direction)
        return jsonify({'status': 'success'})
    else:
        return jsonify({'status': 'error', 'message': 'Invalid direction'})

if __name__ == '__main__':
    app.run(debug=True)
