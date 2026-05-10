/*
 * sketch.js
 * Boundary X - Face Recognition (ESP32 Version - TX Fixed)
 * Features: Face Mesh (Black/Thin), Correct Roll/Eye Logic, Multi-language, Stop Command
 */

import { FaceLandmarker, FilesetResolver } from "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.3";

// --- Multi-language Data ---
const textData = {
  ko: {
    title: "AI 얼굴인식",
    back: "돌아가기",
    
    // Monitors
    h_monitor: "🖥️ 전송 데이터 확인",
    info_title: "📢 전송 데이터 안내",
    // [텍스트 수정] ESP32로 변경
    info_desc: "ESP32로 전송되는 <strong>19자리 숫자 데이터</strong>입니다.<br>(전송 속도: 10회/초)",
    
    // Cards with Numbers
    h_cam: "1. 카메라 설정",
    desc_cam: "카메라 버튼을 통해 화면을 설정해주세요.",
    
    h_conn: "2. 기기 연결",
    // [텍스트 수정] ESP32로 변경
    desc_conn: "블루투스 버튼을 눌러 ESP32와 연결하세요.",
    
    h_data: "3. 실시간 데이터 확인",
    desc_data: "얼굴 움직임과 표정이 아래 데이터로 변환됩니다.",
    
    h_control: "4. AI 얼굴 인식 제어",
    desc_control: "시작 버튼을 눌러 AI 인식을 시작하세요.",

    // Status & Buttons
    status_wait: "상태: 연결 대기 중",
    status_connected: "연결됨: ",
    status_fail: "연결 실패",
    status_disc: "연결 해제됨",
    
    btn_switch: "전후방 전환",
    btn_conn: "기기 연결",
    btn_disc: "연결 해제",
    btn_start_loading: "모델 로딩 중...",
    btn_start: "얼굴 인식 시작",
    btn_stop: "인식 중지",
    
    alert_loading: "모델 로딩 중입니다.",
    alert_ble: "주의: 블루투스가 연결되지 않았습니다.",
    
    // Labels (With Descriptions)
    p_x: "X (좌우)", 
    p_y: "Y (상하)", 
    p_z: "Z (거리)", 
    p_yaw: "Yaw (좌우회전)", 
    p_pitch: "Pitch (상하각도)", 
    p_roll: "Roll (기울기)", 
    p_smile: "Smile (0-9)",
    
    // Footer
    f_company: "바운더리엑스",
    f_slogan: "\"우리는 산업과 교육의 경계를 허물고, 미래 기술을 교실의 책상 위로 옮기는 사람들입니다\"",
    f_address: "경기도 화성시 동탄첨단산업1로 동탄2인큐베이팅센터 7층, 706호",
    f_product: "제품 소개",
    f_bitrun: "AI 비트런", f_bitrun_desc: "마이크로비트 기반 이족보행로봇",
    f_ponybot: "AI 포니봇", f_ponybot_desc: "마이크로비트 기반 모빌리티로봇",
    f_support: "고객 지원",
    f_contact: "문의처", f_contact_desc: "제품 문의 | 제휴 문의 | 연수 문의"
  },
  en: {
    title: "AI Face Recog",
    back: "Back",
    
    h_monitor: "🖥️ Packet Monitor",
    info_title: "📢 Data Packet Info",
    // [텍스트 수정] ESP32로 변경
    info_desc: "<strong>19-digit numeric data</strong> sent to ESP32.<br>(Rate: 10 times/sec)",
    
    h_cam: "1. Camera Settings",
    desc_cam: "Configure your camera view.",
    
    h_conn: "2. Connection",
    // [텍스트 수정] ESP32로 변경
    desc_conn: "Pair with ESP32 via Bluetooth.",
    
    h_data: "3. Real-time Data",
    desc_data: "Face movements converted to parameters.",
    
    h_control: "4. AI Control",
    desc_control: "Start or Stop the AI recognition.",

    status_wait: "Status: Waiting...",
    status_connected: "Connected: ",
    status_fail: "Connection Failed",
    status_disc: "Disconnected",
    
    btn_switch: "Switch Cam",
    btn_conn: "Connect Device",
    btn_disc: "Disconnect",
    btn_start_loading: "Loading Model...",
    btn_start: "Start Face Mesh",
    btn_stop: "Stop",
    
    alert_loading: "Model is still loading...",
    alert_ble: "Warning: Bluetooth not connected.",
    
    // Labels (With Descriptions)
    p_x: "X (Left/Right)", 
    p_y: "Y (Up/Down)", 
    p_z: "Z (Distance)", 
    p_yaw: "Yaw (Turn)", 
    p_pitch: "Pitch (Up/Down)", 
    p_roll: "Roll (Tilt)", 
    p_smile: "Smile (0-9)",
    
    f_company: "Boundary X",
    f_slogan: "\"We blur the lines between industry and education, bringing future tech to the classroom.\"",
    f_address: "706, Dongtan 2 Incubating Center, Hwaseong-si, Gyeonggi-do, Korea",
    f_product: "Products",
    f_bitrun: "AI Bit-Run", f_bitrun_desc: "Micro:bit Bipedal Robot",
    f_ponybot: "AI Pony-Bot", f_ponybot_desc: "Micro:bit Mobility Robot",
    f_support: "Support",
    f_contact: "Contact Us", f_contact_desc: "Inquiry | Partnership | Training"
  }
};

let currentLang = 'ko';

// --- Bluetooth UUIDs ---
const UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
const UART_TX_CHARACTERISTIC_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"; // 쓰기(Write) 통로
const UART_RX_CHARACTERISTIC_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"; 

// --- Variables ---
let bluetoothDevice = null;
let rxCharacteristic = null; // 실제로는 TX 권한을 부여받아 전송에 사용됩니다.
let isConnected = false;
let isSendingData = false;
let lastSentTime = 0; 
const SEND_INTERVAL = 100;

let video;
let faceLandmarker;
let lastVideoTime = -1;
let isModelLoaded = false;
let isDetecting = false;
let detectionResults = null;

let facingMode = "user";
let isFlipped = true;
let isVideoReady = false;

// Default Roll -> 5 (Center)
let params = { x: 50, y: 50, z: 50, yaw: 50, pitch: 50, roll: 5, mouth: 0, lEye: 0, rEye: 0, smile: 0, visible: 0 };

// UI Elements
let els = {};
let btnSwitch, btnConn, btnDisc, btnStart, btnStop;

// --- Init ---
async function initializeFaceLandmarker() {
  const filesetResolver = await FilesetResolver.forVisionTasks(
    "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.3/wasm"
  );
  faceLandmarker = await FaceLandmarker.createFromOptions(filesetResolver, {
    baseOptions: {
      modelAssetPath: `https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task`,
      delegate: "GPU"
    },
    outputFaceBlendshapes: true,
    runningMode: "VIDEO",
    numFaces: 1
  });
  isModelLoaded = true;
  console.log("FaceLandmarker Loaded!");
  updateLanguage(); 
}

// --- p5.js ---
function setup() {
  let canvas = createCanvas(400, 300);
  canvas.parent('p5-container');
  
  setupCamera();
  createUI();
  
  select('#lang-btn').mousePressed(() => {
      currentLang = (currentLang === 'ko') ? 'en' : 'ko';
      updateLanguage();
  });

  initializeFaceLandmarker();
}

function draw() {
  background(0);

  if (!isVideoReady || !video || video.width === 0) {
    fill(255); textAlign(CENTER); textSize(16);
    text(currentLang === 'ko' ? "카메라 로딩 중..." : "Loading Camera...", width/2, height/2);
    return;
  }

  push();
  if (isFlipped) { translate(width, 0); scale(-1, 1); }
  image(video, 0, 0, width, height);
  pop();

  if (isDetecting && detectionResults && detectionResults.faceLandmarks.length > 0) {
    drawFaceMesh(detectionResults.faceLandmarks[0]);
    calculateParameters(detectionResults.faceLandmarks[0], detectionResults.faceBlendshapes[0]);
    params.visible = 1;
  } else {
    params.visible = 0;
    params.smile = 0;
    params.mouth = 0;
  }

  updateGraphUI();
  
  if (isDetecting) {
    let currentTime = millis();
    if (currentTime - lastSentTime > SEND_INTERVAL) {
      sendPacket();
      lastSentTime = currentTime;
    }
  }
}

// --- Logic ---
function updateLanguage() {
    const t = textData[currentLang];
    
    // Update HTML text
    const langElements = document.querySelectorAll('[data-lang]');
    langElements.forEach(el => {
        const key = el.getAttribute('data-lang');
        if(t[key]) el.innerHTML = t[key];
    });

    // Update Buttons
    if(btnSwitch) btnSwitch.html(t.btn_switch);
    if(btnConn) btnConn.html(t.btn_conn);
    if(btnDisc) btnDisc.html(t.btn_disc);
    if(btnStop) btnStop.html(t.btn_stop);
    
    if(btnStart) {
        btnStart.html(isModelLoaded ? t.btn_start : t.btn_start_loading);
    }

    // Status
    const statusEl = select('#bluetoothStatus');
    if(!isConnected) {
        statusEl.html(t.status_wait);
    } else {
        statusEl.html(t.status_connected + (bluetoothDevice ? bluetoothDevice.name : ""));
    }

    select('#lang-btn').html(currentLang === 'ko' ? 'EN' : 'KO');
}

async function predictWebcam() {
  if (!faceLandmarker || !isVideoReady) return;
  let startTimeMs = performance.now();
  if (video.elt.currentTime !== lastVideoTime) {
    lastVideoTime = video.elt.currentTime;
    detectionResults = faceLandmarker.detectForVideo(video.elt, startTimeMs);
  }
  if (isDetecting) window.requestAnimationFrame(predictWebcam);
}

function drawFaceMesh(landmarks) {
  // [스타일] 검은색, 얇게
  noFill(); stroke(0); strokeWeight(3);
  let scaleX = width;
  let scaleY = height;
  beginShape(POINTS);
  for (let pt of landmarks) {
    let x = pt.x * scaleX;
    let y = pt.y * scaleY;
    if (isFlipped) x = width - x;
    vertex(x, y);
  }
  endShape();
  
  // [스타일] 코 끝 (약간 작게)
  let nose = landmarks[1]; 
  let nx = nose.x * scaleX; 
  if(isFlipped) nx = width - nx;
  
  fill(255, 0, 0); noStroke(); circle(nx, nose.y * scaleY, 10);
  noFill(); stroke(255); strokeWeight(1.5); circle(nx, nose.y * scaleY, 10);
}

function calculateParameters(landmarks, blendshapes) {
  let nose = landmarks[1];
  let rawX = isFlipped ? (1 - nose.x) : nose.x;
  params.x = constrain(Math.floor(rawX * 100), 0, 99);
  params.y = constrain(Math.floor(nose.y * 100), 0, 99);

  let widthVal = Math.abs(landmarks[234].x - landmarks[454].x);
  params.z = constrain(map(widthVal, 0.1, 0.7, 0, 99), 0, 99);
  params.z = Math.floor(params.z);

  // Yaw
  let dLeft = Math.abs(landmarks[1].x - landmarks[454].x);
  let dRight = Math.abs(landmarks[1].x - landmarks[234].x);
  let yawRatio = dRight / (dLeft + dRight); 
  if(isFlipped) yawRatio = 1 - yawRatio;
  params.yaw = constrain(Math.floor(yawRatio * 100), 0, 99);

  // Pitch
  let midEyeY = landmarks[168].y;
  let mouthY = landmarks[13].y;
  let noseY = landmarks[1].y;
  let pitchRatio = (noseY - midEyeY) / (mouthY - midEyeY); 
  params.pitch = constrain(map(pitchRatio, 0.8, 0.2, 0, 99), 0, 99);
  params.pitch = Math.floor(params.pitch);

  // [Roll] (Tilt)
  // landmarks[33]: Left Eye, landmarks[263]: Right Eye
  // dy calculation order: Right - Left
  let dy = landmarks[263].y - landmarks[33].y;
  let dx = landmarks[263].x - landmarks[33].x;
  let angle = Math.atan2(dy, dx); 
  
  if(isFlipped) angle = -angle;

  // Map -0.5 ~ 0.5 radians to 0 ~ 9
  let rollVal = map(angle, -0.5, 0.5, 0, 9);
  params.roll = constrain(Math.round(rollVal), 0, 9);

  // Blendshapes
  let shapes = {};
  if (blendshapes && blendshapes.categories) {
    blendshapes.categories.forEach(s => shapes[s.categoryName] = s.score);
  }
  let mOpen = shapes['jawOpen'] || 0;
  params.mouth = Math.floor(constrain(mOpen * 100, 0, 99));

  let lBlink = shapes['eyeBlinkLeft'] || 0;
  let rBlink = shapes['eyeBlinkRight'] || 0;
  params.lEye = Math.floor(constrain((1 - lBlink) * 100, 0, 99));
  params.rEye = Math.floor(constrain((1 - rBlink) * 100, 0, 99));

  let smileVal = ((shapes['mouthSmileLeft'] || 0) + (shapes['mouthSmileRight'] || 0)) / 2;
  params.smile = Math.floor(constrain(smileVal * 10, 0, 9)); 
}

function sendPacket() {
  if (!isConnected || !rxCharacteristic) return;
  const pad = (num) => String(num).padStart(2, '0');
  let p = params;
  let packet = "" + pad(p.x) + pad(p.y) + pad(p.z) + pad(p.yaw) + pad(p.pitch) + pad(p.mouth) + pad(p.lEye) + pad(p.rEye) + String(p.roll) + String(p.smile) + String(p.visible);
  select('#dataDisplay').html(packet);

  if (!isSendingData) {
    isSendingData = true;
    const encoder = new TextEncoder();
    rxCharacteristic.writeValue(encoder.encode(packet + "\n"))
      .catch(err => console.log(err))
      .finally(() => isSendingData = false);
  }
}

function updateGraphUI() {
  const setVal = (id, val, max) => {
    if(els[id]) {
        let percent = (val / max) * 100;
        els[id].bar.style('width', `${percent}%`);
        els[id].txt.html(val);
    }
  };
  setVal('x', params.x, 99); setVal('y', params.y, 99); setVal('z', params.z, 99);
  setVal('yaw', params.yaw, 99); setVal('pitch', params.pitch, 99);
  setVal('mouth', params.mouth, 99); 
  setVal('leye', params.lEye, 99); 
  setVal('reye', params.rEye, 99);
  setVal('roll', params.roll, 9); setVal('smile', params.smile, 9); setVal('vis', params.visible, 1);
}

function createUI() {
  const link = (key, id) => { els[key] = { bar: select(`#bar-${id}`), txt: select(`#val-${id}`) }; };
  
  link('x', 'x'); link('y', 'y'); link('z', 'z'); 
  link('yaw', 'yaw'); link('pitch', 'pitch'); link('roll', 'roll');
  link('mouth', 'mouth'); 
  link('leye', 'leye'); 
  link('reye', 'reye'); 
  link('smile', 'smile'); link('vis', 'vis');

  btnSwitch = createButton("전후방 전환");
  btnSwitch.parent('camera-control-buttons').mousePressed(switchCamera);
  
  btnConn = createButton("기기 연결");
  btnConn.parent('bluetooth-control-buttons').addClass('start-button').mousePressed(connectBluetooth);

  btnDisc = createButton("연결 해제");
  btnDisc.parent('bluetooth-control-buttons').addClass('stop-button').mousePressed(disconnectBluetooth);

  btnStart = createButton("모델 로딩 중...");
  btnStart.parent('object-control-buttons').addClass('start-button');
  btnStart.mousePressed(() => {
    const t = textData[currentLang];
    if (!isModelLoaded) return alert(t.alert_loading);
    if (!isConnected) alert(t.alert_ble);
    isDetecting = true;
    predictWebcam();
  });

  // 인식 중지 버튼: "stop" 문자열 전송
  btnStop = createButton("인식 중지");
  btnStop.parent('object-control-buttons').addClass('stop-button');
  btnStop.mousePressed(() => {
    isDetecting = false;
    params.visible = 0;
    updateGraphUI();
    
    // UI 업데이트 및 Stop 전송
    select('#dataDisplay').html("stop");
    
    if (isConnected && rxCharacteristic) {
      const encoder = new TextEncoder();
      rxCharacteristic.writeValue(encoder.encode("stop\n"))
        .catch(err => console.log(err));
    }
  });
}

function setupCamera() {
  isVideoReady = false;
  video = createCapture({ video: { facingMode: facingMode }, audio: false });
  video.hide();
  let check = setInterval(() => {
    if (video.elt.readyState >= 2 && video.elt.videoWidth > 0) {
      isVideoReady = true;
      clearInterval(check);
      if(isDetecting) predictWebcam();
    }
  }, 100);
}

function switchCamera() {
  isDetecting = false;
  if(video) { video.remove(); video = null; }
  facingMode = facingMode === "user" ? "environment" : "user";
  isFlipped = (facingMode === "user");
  setTimeout(() => { setupCamera(); isDetecting = true; }, 500);
}

async function connectBluetooth() {
  try {
    bluetoothDevice = await navigator.bluetooth.requestDevice({
      // [수정 핵심] 기기 이름 필터를 ESP로 설정
      filters: [{ namePrefix: "ESP" }],
      optionalServices: [UART_SERVICE_UUID]
    });
    const server = await bluetoothDevice.gatt.connect();
    const service = await server.getPrimaryService(UART_SERVICE_UUID);
    
    // [수정 핵심] RX가 아닌 TX 채널(0002)을 가져와서 쓰기 권한 확보
    rxCharacteristic = await service.getCharacteristic(UART_TX_CHARACTERISTIC_UUID);
    
    isConnected = true;
    const t = textData[currentLang];
    select('#bluetoothStatus').html(t.status_connected + bluetoothDevice.name).addClass('status-connected');
  } catch (e) {
    console.error(e);
    const t = textData[currentLang];
    select('#bluetoothStatus').html(t.status_fail).addClass('status-error');
  }
}

function disconnectBluetooth() {
  if (bluetoothDevice && bluetoothDevice.gatt.connected) bluetoothDevice.gatt.disconnect();
  isConnected = false;
  const t = textData[currentLang];
  select('#bluetoothStatus').html(t.status_disc).removeClass('status-connected status-error');
}

window.setup = setup;
window.draw = draw;
