let mapCanvas = document.getElementById('map-canvas');
let mapCtx = mapCanvas.getContext('2d');
let mapData = null;
let robotPose = {x: null, y: null}; // Placeholder for robot pose

ws.onmessage = function(event) {
    let msg = JSON.parse(event.data);
    if (msg.type === 'map') {
        mapData = msg;
        drawMap();
    }
    if (msg.type === 'video') {
        drawVideo(msg.data);
    }
    if (msg.type === 'robot_pose') {
        robotPose = msg.pose;
        drawMap(); // Redraw map with updated pose
    }
};

function drawMap() {
    if (!mapData) return;
    let w = mapData.width, h = mapData.height;
    mapCanvas.width = w;
    mapCanvas.height = h;
    let img = mapCtx.createImageData(w, h);
    for (let i = 0; i < w * h; i++) {
        let val = mapData.data[Math.floor(i / w)][i % w];
        let color = val === 100 ? 0 : val === 0 ? 255 : 127;
        img.data[i * 4 + 0] = color;
        img.data[i * 4 + 1] = color;
        img.data[i * 4 + 2] = color;
        img.data[i * 4 + 3] = 255;
    }
    mapCtx.putImageData(img, 0, 0);
    // Draw robot pose if available
    if (robotPose.x !== null && robotPose.y !== null) {
        mapCtx.save();
        mapCtx.fillStyle = 'red';
        mapCtx.beginPath();
        mapCtx.arc(robotPose.x, robotPose.y, 5, 0, 2 * Math.PI);
        mapCtx.fill();
        mapCtx.restore();
    }
}
