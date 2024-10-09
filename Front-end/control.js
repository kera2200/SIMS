let ros;  // Declare the ROS connection object globally
let cmdVel;  // Declare the ROS topic for robot commands globally

// Automatically connect to the robot on page load
window.addEventListener('load', function() {
    const robotIp = '10.0.54.84';  // Use the fixed IP address
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
            name: '/turtle/cmd_vel',
            messageType: 'geometry_msgs/Twist'
        });
    });

    ros.on('error', function(error) {
        console.log('Error connecting to ROSBridge server:', error);
    });

    ros.on('close', function() {
        console.log('Connection to ROSBridge server closed.');
    });
});

// Event listener for the "Logout" button
document.getElementById('logout-btn').addEventListener('click', function() {
    window.location.href = 'index.html';
});

// Commands map for controlling the robot
const commands = {
    'move-forward': { linear: { x: 0.2 }, angular: { z: 0 } },
    'move-backward': { linear: { x: -0.2 }, angular: { z: 0 } },
    'turn-left': { linear: { x: 0 }, angular: { z: 0.5 } },
    'turn-right': { linear: { x: 0 }, angular: { z: -0.5 } },
    'stop': { linear: { x: 0 }, angular: { z: 0 } }
};

// Event listeners for movement buttons
Object.keys(commands).forEach(command => {
    document.getElementById(command).addEventListener('click', function() {
        if (!ros || !ros.isConnected) {
            alert('Please connect to the robot first.');
            return;
        }

        // Create and send the Twist message based on the command
        const twist = new ROSLIB.Message(commands[command]);
        cmdVel.publish(twist);
        console.log(`Command sent: ${command}`);
    });
});
