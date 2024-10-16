from flask import Flask, jsonify, request, render_template
import roslibpy

app = Flask(__name__)

# Initialize ROS connection
ros = roslibpy.Ros(host='10.0.54.84', port=9090)
ros.run()

# Define the ROS topic for sending velocity commands
cmd_vel = roslibpy.Topic(ros, '/turtle/cmd_vel', 'geometry_msgs/Twist')

@app.route('/move', methods=['POST'])
def move():
    data = request.get_json()
    direction = data.get('direction', 'stop')

    twist = roslibpy.Message({
        'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    })

    if direction == 'forward':
        twist['linear']['x'] = 0.2
    elif direction == 'backward':
        twist['linear']['x'] = -0.2
    elif direction == 'left':
        twist['angular']['z'] = 0.5
    elif direction == 'right':
        twist['angular']['z'] = -0.5

    cmd_vel.publish(twist)
    return jsonify({"status": f"Robot moved {direction}"}), 200

# Stop command
@app.route('/stop', methods=['POST'])
def stop_command():
    twist = roslibpy.Message({
        'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    })
    cmd_vel.publish(twist)
    return jsonify({"status": "Robot stopped"}), 200

@app.route('/')
def home():
    return render_template('index.html') 

@app.route('/control')
def control():
    return render_template('control.html')

@app.route('/menu')
def menu():
    return render_template('menu.html')

@app.route('/map_new_area')
def map_new_area():
    return render_template('map_new_area.html')

@app.route('/view_existing_map')
def view_existing_map():
    return render_template('view_existing_map.html')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)