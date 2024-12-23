let commandInterval;

// Function to send API commands to the backend
function sendAPICommand(direction) {
    fetch('/move', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ direction: direction }),
    })
    .then(response => {
        if (!response.ok) {
            throw new Error('Network response was not ok');
        }
        return response.json();
    })
    .then(data => console.log(data))
    .catch(error => console.error('Error:', error));
}

// Function to start continuous movement via API
function startContinuousCommand(direction) {
    sendAPICommand(direction);
    commandInterval = setInterval(() => sendAPICommand(direction), 100);
}

// Function to stop movement via API
function stopContinuousCommand() {
    clearInterval(commandInterval);
    fetch('/stop', { method: 'POST' })
        .then(response => {
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }
            return response.json();
        })
        .then(data => console.log(data))
        .catch(error => console.error('Error:', error));
}

document.addEventListener('DOMContentLoaded', function() {
    // Event listener för logout-knappen i menu.html
    const logoutBtn = document.getElementById('logout-btn');
    if (logoutBtn) {
        logoutBtn.addEventListener('click', function() {
            window.location.href = '/';
        });
    }

    // Event listener för go back-knappen, om den finns
    const goBackBtn = document.getElementById('GoBack-btn');
    if (goBackBtn) {
        goBackBtn.addEventListener('click', function() {
            window.location.href = '/menu';  
        });
    }
});