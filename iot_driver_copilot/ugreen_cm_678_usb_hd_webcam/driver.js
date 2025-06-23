const express = require('express');
const { spawn } = require('child_process');
const os = require('os');
const fs = require('fs');
const path = require('path');

// Configuration from environment variables
const SERVER_HOST = process.env.SERVER_HOST || '0.0.0.0';
const SERVER_PORT = parseInt(process.env.SERVER_PORT, 10) || 8080;
const VIDEO_DEVICE = process.env.VIDEO_DEVICE || detectVideoDevice();
const VIDEO_WIDTH = process.env.VIDEO_WIDTH || '640';
const VIDEO_HEIGHT = process.env.VIDEO_HEIGHT || '480';
const VIDEO_FPS = process.env.VIDEO_FPS || '15';
const CAPTURE_IMAGE_FORMAT = process.env.CAPTURE_IMAGE_FORMAT || 'jpeg'; // jpeg or png

function detectVideoDevice() {
  // Try to find the first video device (Linux only)
  if (os.platform() === 'linux') {
    try {
      const devices = fs.readdirSync('/dev').filter(f => /^video\d+$/.test(f));
      if (devices.length > 0) {
        return '/dev/' + devices[0];
      }
    } catch (e) {}
  }
  // Default fallback
  return '';
}

function getDeviceInfo() {
  return {
    device_name: 'UGREEN CM678 USB HD Webcam',
    device_model: 'CM678',
    manufacturer: 'UGREEN',
    device_type: 'USB HD Webcam',
    video_device: VIDEO_DEVICE,
    resolution: `${VIDEO_WIDTH}x${VIDEO_HEIGHT}`,
    fps: VIDEO_FPS,
    status: VIDEO_DEVICE ? 'available' : 'not detected',
    server: {
      host: SERVER_HOST,
      port: SERVER_PORT,
    },
    endpoints: [
      { method: 'GET', path: '/probe', description: 'Device info and status' },
      { method: 'GET', path: '/video', description: 'MJPEG video stream' },
      { method: 'GET', path: '/capture', description: 'Still image capture' }
    ]
  };
}

const app = express();

// /probe endpoint
app.get('/probe', (req, res) => {
  res.json(getDeviceInfo());
});

// /video endpoint (MJPEG streaming)
app.get('/video', (req, res) => {
  if (!VIDEO_DEVICE) {
    res.status(500).json({ error: 'No video device detected' });
    return;
  }

  // Query params for resolution/quality
  const width = req.query.width || VIDEO_WIDTH;
  const height = req.query.height || VIDEO_HEIGHT;
  const fps = req.query.fps || VIDEO_FPS;

  // Start ffmpeg to capture MJPEG stream from webcam
  // -f v4l2: input from video4linux2 device
  // -vcodec mjpeg: output mjpeg
  // -r: framerate
  // -s: size
  // -q:v: quality
  // -f mjpeg: output format
  // - (stdout): pipe to HTTP
  const ffmpegArgs = [
    '-f', 'v4l2',
    '-framerate', String(fps),
    '-video_size', `${width}x${height}`,
    '-i', VIDEO_DEVICE,
    '-f', 'mjpeg',
    '-q:v', '5',
    '-'
  ];

  res.setHeader('Content-Type', 'multipart/x-mixed-replace; boundary=frame');
  res.setHeader('Cache-Control', 'no-cache');
  res.setHeader('Connection', 'close');

  const ffmpeg = spawn('ffmpeg', ffmpegArgs);

  ffmpeg.stderr.on('data', () => {}); // suppress logs

  let buffer = Buffer.alloc(0);

  ffmpeg.stdout.on('data', data => {
    buffer = Buffer.concat([buffer, data]);
    let start, end;
    while ((start = buffer.indexOf(Buffer.from([0xFF, 0xD8]))) !== -1 &&
           (end = buffer.indexOf(Buffer.from([0xFF, 0xD9]), start)) !== -1) {
      end += 2;
      const frame = buffer.slice(start, end);
      buffer = buffer.slice(end);

      res.write(
        `--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${frame.length}\r\n\r\n`
      );
      res.write(frame);
      res.write('\r\n');
    }
  });

  ffmpeg.on('close', () => { res.end(); });
  ffmpeg.on('error', () => { res.end(); });

  req.on('close', () => {
    ffmpeg.kill('SIGKILL');
  });
});

// /capture endpoint (single image capture)
app.get('/capture', (req, res) => {
  if (!VIDEO_DEVICE) {
    res.status(500).json({ error: 'No video device detected' });
    return;
  }

  const width = req.query.width || VIDEO_WIDTH;
  const height = req.query.height || VIDEO_HEIGHT;
  const format = req.query.format || CAPTURE_IMAGE_FORMAT;
  const mimeType = format === 'png' ? 'image/png' : 'image/jpeg';
  const ext = format === 'png' ? 'png' : 'jpg';

  // Use ffmpeg to capture one frame
  const tmpFile = path.join(os.tmpdir(), `webcam_capture_${Date.now()}.${ext}`);
  const ffmpegArgs = [
    '-f', 'v4l2',
    '-video_size', `${width}x${height}`,
    '-i', VIDEO_DEVICE,
    '-frames:v', '1',
    '-f', format === 'png' ? 'image2' : 'mjpeg',
    tmpFile
  ];

  const ffmpeg = spawn('ffmpeg', ffmpegArgs);

  ffmpeg.stderr.on('data', () => {}); // suppress logs

  ffmpeg.on('exit', (code) => {
    if (code === 0) {
      fs.readFile(tmpFile, (err, data) => {
        fs.unlink(tmpFile, () => {});
        if (err) {
          res.status(500).json({ error: 'Failed to read image' });
        } else {
          res.setHeader('Content-Type', mimeType);
          res.send(data);
        }
      });
    } else {
      fs.unlink(tmpFile, () => {});
      res.status(500).json({ error: 'Failed to capture image' });
    }
  });
});

app.listen(SERVER_PORT, SERVER_HOST, () => {
  // Server started
});