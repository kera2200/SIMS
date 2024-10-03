document.getElementById('connect-btn').addEventListener('click', function() {
    const robotIp = document.getElementById('robot-ip').value;
    if (robotIp) {
        console.log('Connecting to robot at IP:', robotIp);
        // Establish a WebSocket or HTTP connection to the robot here
    } else {
        alert('Please enter a valid IP address.');
    }
});

document.getElementById('logout-btn').addEventListener('click', function() {
    // Redirect to the login page
    window.location.href = 'index.html';
});


const commands = {
    'move-forward': 'forward',
    'move-backward': 'backward',
    'turn-left': 'left',
    'turn-right': 'right',
    'stop': 'stop'
};

Object.keys(commands).forEach(command => {
    document.getElementById(command).addEventListener('click', function() {
        console.log(`Sending command: ${commands[command]}`);
        // Send the command to the robot using its IP address here
    });
});
