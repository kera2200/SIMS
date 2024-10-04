let ros;  // Declare the ROS connection object globally
let cmdVel;  // Declare the ROS topic for robot commands globally

// Event listener for the "Connect" button
document.getElementById('connect-btn').addEventListener('click', function() {
    const robotIp = document.getElementById('robot-ip').value;
    if (robotIp) {
        console.log('Connecting to robot at IP:', robotIp);

        // Initialize ROS connection using ROSLIB
        ros = new ROSLIB.Ros({
            url: `ws://${robotIp}:9090`  // Connect to the ROSBridge WebSocket on the robot's IP
        });

        // ROS event handlers
        ros.on('connection', function() {
            console.log('Connected to ROSBridge server.');

            // Define the ROS topic for sending velocity commands
            cmdVel = new ROSLIB.Topic({
                ros: ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            });
        });

        ros.on('error', function(error) {
            console.log('Error connecting to ROSBridge server:', error);
        });

        ros.on('close', function() {
            console.log('Connection to ROSBridge server closed.');
        });
    } else {
        alert('Please enter a valid IP address.');
    }
});

// Event listener for the "Logout" button
document.getElementById('logout-btn').addEventListener('click', function() {
    // Redirect to the login page
    window.location.href = 'index.html';
});

// Commands map for controlling the robot
const commands = {
    'move-forward': 'forward',
    'move-backward': 'backward',
    'turn-left': 'left',
    'turn-right': 'right',
    'stop': 'stop'
};

// Event listeners for movement buttons
Object.keys(commands).forEach(command => {
    document.getElementById(command).addEventListener('click', function() {
        if (!ros || !ros.isConnected) {
            alert('Please connect to the robot first.');
            return;
        }

        // Create the Twist message to send to the robot
        let twist = new ROSLIB.Message({
            linear: { x: 0, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 }
        });

        // Adjust the linear and angular velocities based on the command
        switch (commands[command]) {
            case 'forward':
                twist.linear.x = 0.2;  // Move forward
                break;
            case 'backward':
                twist.linear.x = -0.2;  // Move backward
                break;
            case 'left':
                twist.angular.z = 0.5;  // Turn left
                break;
            case 'right':
                twist.angular.z = -0.5;  // Turn right
                break;
            case 'stop':
                twist.linear.x = 0;
                twist.angular.z = 0;
                break;
        }

        // Publish the twist message to the /cmd_vel topic
        cmdVel.publish(twist);
        console.log(`Command sent: ${commands[command]}`);
    });
});
