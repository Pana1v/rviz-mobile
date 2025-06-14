let videoCanvas = document.getElementById('video-canvas');
let videoCtx = videoCanvas.getContext('2d');

function drawVideo(b64) {
    let img = new window.Image();
    img.onload = function() {
        videoCanvas.width = img.width;
        videoCanvas.height = img.height;
        videoCtx.drawImage(img, 0, 0);
    };
    img.src = 'data:image/jpeg;base64,' + b64;
}
