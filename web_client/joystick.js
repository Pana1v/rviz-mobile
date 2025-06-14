let ws;
function connectWS() {
    ws = new WebSocket(`ws://${location.hostname}:8765`);
}
connectWS();

const joystick = document.getElementById('joystick');
const size = 150;
joystick.style.width = size + 'px';
joystick.style.height = size + 'px';
joystick.style.background = '#333';
joystick.style.borderRadius = '50%';
joystick.style.position = 'relative';

// Add knob
let knob = document.createElement('div');
knob.style.width = '60px';
knob.style.height = '60px';
knob.style.background = '#888';
knob.style.borderRadius = '50%';
knob.style.position = 'absolute';
knob.style.left = (size/2 - 30) + 'px';
knob.style.top = (size/2 - 30) + 'px';
knob.style.transition = 'left 0.05s, top 0.05s';
joystick.appendChild(knob);

let dragging = false;
let center = {x: size/2, y: size/2};

function setKnob(x, y) {
    knob.style.left = (center.x + x - 30) + 'px';
    knob.style.top = (center.y + y - 30) + 'px';
}

joystick.addEventListener('touchstart', e => { dragging = true; }, false);
joystick.addEventListener('touchend', e => {
    dragging = false;
    setKnob(0, 0);
    sendTeleop(0, 0);
}, false);
joystick.addEventListener('touchmove', e => {
    if (!dragging) return;
    let touch = e.touches[0];
    let rect = joystick.getBoundingClientRect();
    let x = touch.clientX - rect.left - center.x;
    let y = touch.clientY - rect.top - center.y;
    let max = 60;
    x = Math.max(-max, Math.min(max, x));
    y = Math.max(-max, Math.min(max, y));
    setKnob(x, y);
    let linear = -y / max;
    let angular = x / max;
    sendTeleop(linear, angular);
}, false);

// Mouse support for desktop
joystick.addEventListener('mousedown', e => { dragging = true; });
document.addEventListener('mouseup', e => {
    if (dragging) {
        dragging = false;
        setKnob(0, 0);
        sendTeleop(0, 0);
    }
});
joystick.addEventListener('mousemove', e => {
    if (!dragging) return;
    let rect = joystick.getBoundingClientRect();
    let x = e.clientX - rect.left - center.x;
    let y = e.clientY - rect.top - center.y;
    let max = 70;
    x = Math.max(-max, Math.min(max, x));
    y = Math.max(-max, Math.min(max, y));
    setKnob(x, y);
    let linear = -y / max;
    let angular = x / (max*3);

    linear*= 0.25;
    angular *= 0.25; // Uncomment if you want to scale down angular speed
    sendTeleop(linear, angular);
});

function sendTeleop(linear, angular) {
    if (ws && ws.readyState === 1) {
        ws.send(JSON.stringify({type: 'teleop', linear, angular}));
    }
}
