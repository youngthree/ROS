```javascript
// DeviceShifu for UGREEN CM678 USB HD Webcam - HTTP MJPEG Streaming Driver
// Requirements: Runs in Kubernetes, updates EdgeDevice CRD status, serves HTTP MJPEG video from USB webcam

const express = require('express');
const { spawn } = require('child_process');
const fs = require('fs');
const k8s = require('@kubernetes/client-node');
const yaml = require('js-yaml');
const path = require('path');

// --- Environment Variables ---
const EDGEDEVICE_NAME = process.env.EDGEDEVICE_NAME;
const EDGEDEVICE_NAMESPACE = process.env.EDGEDEVICE_NAMESPACE;
const SERVER_HOST = process.env.DEVICESHIFU_HTTP_SERVER_HOST || '0.0.0.0';
const SERVER_PORT = parseInt(process.env.DEVICESHIFU_HTTP_SERVER_PORT || '8080', 10);

// --- Constants ---
const INSTRUCTIONS_PATH = '/etc/edgedevice/config/instructions';
const CAM_DEVICE = process.env.DEVICESHIFU_USB_VIDEO_DEVICE || '/dev/video0'; // Allow override
const MJPEG_RESOLUTION = process.env.DEVICESHIFU_MJPEG_RESOLUTION || '640x480';
const MJPEG_FPS = process.env.DEVICESHIFU_MJPEG_FPS || '15';

// --- Status Management ---
const EdgeDevicePhase = {
    Pending: 'Pending',
    Running: 'Running',
    Failed: 'Failed',
    Unknown: 'Unknown',
};

let currentPhase = EdgeDevicePhase.Pending;

// --- Kubernetes Client Setup ---
const kc = new k8s.KubeConfig();
kc.loadFromCluster();
const k8sApiCustomObjects = kc.makeApiClient(k8s.CustomObjectsApi);

// --- Helper: Update EdgeDevice Status ---
async function updateDevicePhase(phase) {
    if (
        !EDGEDEVICE_NAME ||
        !EDGEDEVICE_NAMESPACE ||
        !phase ||
        currentPhase === phase
    ) {
        return;
    }
    try {
        // Patch only the .status.edgeDevicePhase field
        await k8sApiCustomObjects.patchNamespacedCustomObjectStatus(
            'shifu.edgenesis.io',
            'v1alpha1',
            EDGEDEVICE_NAMESPACE,
            'edgedevices',
            EDGEDEVICE_NAME,
            {
                status: {
                    edgeDevicePhase: phase,
                },
            },
            undefined,
            undefined,
            undefined,
            {
                headers: { 'Content-Type': 'application/merge-patch+json' },
            }
        );
        currentPhase = phase;
    } catch (err) {
        // Ignore update errors (e.g., resource not found)
    }
}

// --- Helper: Read EdgeDevice CRD for .spec.address ---
async function getDeviceAddress() {
    if (!EDGEDEVICE_NAME || !EDGEDEVICE_NAMESPACE) {
        return null;
    }
    try {
        const resp = await k8sApiCustomObjects.getNamespacedCustomObject(
            'shifu.edgenesis.io',
            'v1alpha1',
            EDGEDEVICE_NAMESPACE,
            'edgedevices',
            EDGEDEVICE_NAME
        );
        return resp?.body?.spec?.address || null;
    } catch (err) {
        return null;
    }
}

// --- Helper: Parse ConfigMap Instructions ---
function loadInstructions() {
    try {
        if (fs.existsSync(INSTRUCTIONS_PATH)) {
            const content = fs.readFileSync(INSTRUCTIONS_PATH, 'utf8');
            return yaml.load(content);
        }
    } catch (err) {}
    return {};
}

const instructions = loadInstructions();

// --- MJPEG Streamer (Node.js Only, No External Commands) ---
const { Readable } = require('stream');
const v4l2camera = require('v4l2camera'); // This package must be installed in the container

// Helper: Check camera device and start test capture
function testCameraDevice(device) {
    return new Promise((resolve, reject) => {
        try {
            const cam = new v4l2camera.Camera(device);
            if (!cam.configGet().formatName.includes('MJPEG')) {
                reject(new Error('Camera does not support MJPEG'));
                return;
            }
            cam.start();
            cam.capture(() => {
                cam.stop();
                resolve(true);
            });
        } catch (err) {
            reject(err);
        }
    });
}

// MJPEG Stream generator
function createMJPEGStream(device, resolution, fps) {
    const cam = new v4l2camera.Camera(device);
    let [width, height] = resolution.split('x').map(Number);
    cam.configSet({
        width: width,
        height: height,
        interval: { numerator: 1, denominator: fps },
        formatName: 'MJPEG',
    });
    cam.start();

    const stream = new Readable({
        read() {}
    });

    function captureFrame() {
        cam.capture(() => {
            const frame = cam.frameRaw();
            stream.push(Buffer.concat([
                Buffer.from(
                    `--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${frame.length}\r\n\r\n`
                ),
                frame,
                Buffer.from('\r\n')
            ]));
            setTimeout(captureFrame, 1000 / fps);
        });
    }

    captureFrame();

    stream.on('close', () => {
        cam.stop();
    });

    return stream;
}

// --- HTTP Server Setup ---
const app = express();

app.get('/capture', async (req, res) => {
    // Update status to Pending, then Running if camera works, else Failed
    await updateDevicePhase(EdgeDevicePhase.Pending);

    // Check camera device
    try {
        await testCameraDevice(CAM_DEVICE);
        await updateDevicePhase(EdgeDevicePhase.Running);
    } catch (err) {
        await updateDevicePhase(EdgeDevicePhase.Failed);
        res.status(500).send('Camera not found or not available');
        return;
    }

    // Stream MJPEG multipart HTTP
    res.writeHead(200, {
        'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
        'Cache-Control': 'no-cache',
        'Connection': 'close',
        'Pragma': 'no-cache'
    });

    const stream = createMJPEGStream(CAM_DEVICE, MJPEG_RESOLUTION, MJPEG_FPS);

    stream.pipe(res);

    req.on('close', () => {
        stream.destroy();
    });
});

// --- K8s Status: Unknown on shutdown ---
process.on('SIGTERM', async () => {
    await updateDevicePhase(EdgeDevicePhase.Unknown);
    process.exit(0);
});
process.on('SIGINT', async () => {
    await updateDevicePhase(EdgeDevicePhase.Unknown);
    process.exit(0);
});

// --- Initial Status: Unknown ---
(async () => {
    await updateDevicePhase(EdgeDevicePhase.Unknown);
})();

// --- Start Server ---
app.listen(SERVER_PORT, SERVER_HOST, () => {
    // Log start (optional)
});
```
