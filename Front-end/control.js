let ros;  // Declare the ROS connection object globally
let cmdVel;  // Declare the ROS topic for robot commands globally
let commandInterval;  // Declare a variable to hold the interval ID
let keyPressed = {};  // Object to keep track of pressed keys

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

// Function to send movement commands to the robot
function sendCommand(direction) {
    let twist = new ROSLIB.Message({
        linear: { x: 0.0, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.0 }
    });

    switch (direction) {
        case 'forward':
            twist.linear.x = 0.2;
            break;
        case 'backward':
            twist.linear.x = -0.2;
            break;
        case 'left':
            twist.angular.z = 0.5;
            break;
        case 'right':
            twist.angular.z = -0.5;
            break;
        case 'stop':
            // Leave twist as zero
            break;
    }

    cmdVel.publish(twist);
}

// Function to start sending commands at intervals
function startContinuousCommand(direction) {
    sendCommand(direction);  // Send the initial command immediately
    commandInterval = setInterval(() => sendCommand(direction), 100);  // Send command every 100ms
}

// Function to stop sending commands
function stopContinuousCommand() {
    clearInterval(commandInterval);  // Stop the continuous command
    sendCommand('stop');  // Stop the robot immediately
}

// Add event listeners for mouse button presses (onmousedown/onmouseup)
document.querySelectorAll('.command-btn').forEach(button => {
    const direction = button.getAttribute('data-command');
    button.addEventListener('mousedown', () => startContinuousCommand(direction));
    button.addEventListener('mouseup', stopContinuousCommand);
    button.addEventListener('mouseleave', stopContinuousCommand);
});

// Add event listeners for keyboard presses (keydown/keyup)
document.addEventListener('keydown', function(event) {
    if (!keyPressed[event.key]) {
        keyPressed[event.key] = true;
        switch(event.key) {
            case 'ArrowUp':
                startContinuousCommand('forward');
                break;
            case 'ArrowDown':
                startContinuousCommand('backward');
                break;
            case 'ArrowLeft':
                startContinuousCommand('left');
                break;
            case 'ArrowRight':
                startContinuousCommand('right');
                break;
        }
    }
});

document.addEventListener('keyup', function(event) {
    if (keyPressed[event.key]) {
        keyPressed[event.key] = false;
        switch(event.key) {
            case 'ArrowUp':
            case 'ArrowDown':
            case 'ArrowLeft':
            case 'ArrowRight':
                stopContinuousCommand();
                break;
        }
    }
});

// Event listener for the "Logout" button
document.getElementById('logout-btn').addEventListener('click', function() {
    window.location.href = 'index.html';
});
