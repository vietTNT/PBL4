import asyncio
import base64
import cv2
import numpy as np
import time
from collections import deque, Counter
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Body, Request, Response
from fastapi.middleware.cors import CORSMiddleware
import tensorflow as tf
import os
import uvicorn
from typing import Optional
import socket




# ================== CONFIG ==================
MODEL_PATH = "final_mobilenetv3_small _v8.keras"  # Fixed: added space before _v8
IMG_SIZE = 224  # Model v8 requires 224x224 input (changed from 180)
CLASS_NAMES = ["green", "none", "red", "stopsign", "yellow"]




# Inference / pipeline tuning
INFERENCE_INTERVAL = 0.05  # giảm xuống 50ms để phản hồi nhanh hơn
LANE_RUN_INTERVAL = 0.02   # seconds between lane runs (20ms)
MIN_CONFIDENCE = 0.5  # Chỉ chấp nhận kết quả > 50%
VOTING_WINDOW = 3  # Lưu 3 dự đoán gần nhất




# Inference / pipeline tuning
INFERENCE_INTERVAL = 0.05  # giảm xuống 50ms để phản hồi nhanh hơn
LANE_RUN_INTERVAL = 0.02   # seconds between lane runs (20ms)
MIN_CONFIDENCE = 0.5  # Chỉ chấp nhận kết quả > 50%
VOTING_WINDOW = 3  # Lưu 3 dự đoán gần nhất




# ================== GLOBAL STATE ==================


current_mode = "line_follow"  # default to line_follow mode
latest_command: Optional[str] = None
latest_result = None
last_frame = None
last_overlay = None




connected_ws: Optional[WebSocket] = None
ws_lock = asyncio.Lock()




recent_notifications = deque(maxlen=200)


# NEW: Track last sent command to avoid spamming Arduino in line_follow mode
last_sent_traffic_cmd = None




# inference flags
inference_busy = False  # tránh chạy model song song
last_inference_time = 0




# lane flags
last_lane_time = 0
latest_lane_cmd = None




# voting mechanism để ổn định kết quả
recent_predictions = deque(maxlen=VOTING_WINDOW)




# ================ LANE TUNING VARIABLES ================
# Make these easy to tune to fix drift / jitter / sensitivity
V_THRESHOLD = 140                # threshold (lowered to detect orange center line)
MORPH_KERNEL_SIZE = (3, 3)       # morphological kernel size (smaller = preserve detail)
GAUSSIAN_KERNEL_WIDTH = 7        # gaussian blur width for projection smoothing (odd)
ROI_TOP_RATIO = 0.5              # crop start as fraction of height (lower half)
DEADBAND_PCT = 0.03              # REDUCED from 0.08 to make steering more responsive
FALLBACK_CMD = "S"              # command when no peaks detected at all
MAX_CONTOURS = 4                 # how many contours to consider




# Perspective transform (bird's eye view) - inspired by Raspberry Pi code
ENABLE_PERSPECTIVE = False       # DISABLED - causing reversed steering! Need to recalibrate
# Source: trapezoid on original frame (top-left, top-right, bottom-left, bottom-right)
# Adjust these based on your camera angle and lane position
PERSPECTIVE_SRC = np.float32([
    [50, 100],    # top-left (xa camera, bên trái)
    [270, 100],   # top-right (xa camera, bên phải)
    [0, 190],     # bottom-left (gần camera, bên trái)
    [320, 190]    # bottom-right (gần camera, bên phải)
])
MAX_LANE_WIDTH_PX = 220          # Maximum lane width in pixels (increased from 160)
# Destination: rectangle for bird's eye view
PERSPECTIVE_DST = np.float32([
    [80, 0],      # top-left
    [240, 0],     # top-right
    [80, 240],    # bottom-left
    [240, 240]    # bottom-right
])
PERSPECTIVE_MATRIX = None        # Will be computed on first frame




# Smoothing / hysteresis / calibration
OFFSET_SMOOTH_ALPHA = 0.50       # EMA alpha for offset smoothing (0..1)
CMD_HYSTERESIS_FRAMES = 8        # INCREASED to prevent rapid L/R command changes
OFFSET_BIAS_PX = -10             # MUST BE 0 for symmetric lane following




# runtime smoothing state
prev_offset = 0
hysteresis_count = 0
last_sent_cmd = None




# NEW: Steering Inversion Flag
# Set this to True if your car turns RIGHT on 'L' and LEFT on 'R'
INVERT_STEERING_COMMANDS = True




# =====================================================




# ================== UTILITIES ==================




def push_notification(msg, extra=None):
    item = {"ts": time.time(), "message": msg}
    if extra:
        item.update(extra)
    recent_notifications.append(item)
    print("", msg)








def get_local_ips():
    ips = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        ips.append(ip)
        s.close()
    except:
        pass
    return ips




# ================== LOAD MODEL ==================
print("Loading model...")
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
tf.get_logger().setLevel('ERROR')




model = None
if os.path.exists(MODEL_PATH):
    try:
        model = tf.keras.models.load_model(MODEL_PATH)
        print(" Model loaded successfully!")
    except Exception as e:
        print(" Failed to load model:", e)
else:
    print(" Model file not found — running in lane-only fallback mode")




# ================== FASTAPI ==================
app = FastAPI(title=" ESP32 AI + Lane Controller")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)




# ================== AI/INFERENCE HELPERS ==================




def preprocess_image(frame):
    """Preprocess FULL frame for traffic sign detection - same as working Server.py"""
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(img_rgb, (IMG_SIZE, IMG_SIZE))
    x = resized.astype(np.float32)  # NO normalization - model expects [0-255]
    return np.expand_dims(x, axis=0)








def run_inference(img):
    if model is None:
        return "none", 1.0
    preds = model.predict(img, verbose=0)
    i = int(np.argmax(preds[0]))
    return CLASS_NAMES[i], float(preds[0][i])








def decide_command(label, mode="auto"):
    """Decide command based on traffic sign label and current mode.
   
    In LINE_FOLLOW mode: only return S (stop) or A (allow line follow)
    In AUTO mode: return full control commands
    """
    if label in ["red", "stopsign"]:
        return "S"  # Dừng ngay
    elif label == "green":
        # LINE_FOLLOW mode: gửi 'A' để cho phép line follow
        # AUTO mode: gửi 'F' để đi thẳng
        return "A" if mode == "line_follow" else "F"
    elif label == "yellow":
        return "S"  # Vàng = Dừng (an toàn)
    # Mặc định: cho phép chạy
    return "A" if mode == "line_follow" else "F"




# ================== LANE DETECTION (fast) ==================
# Camera is low, ~10cm above ground; lane = white center line + outer white edges on A1 sheet




def process_lane(frame):
    """Return command 'L','R' or 'F' and debug info (center offset in pixels).
    Fast method: crop lower region, convert to HSV, threshold bright/white, projection by column.
    """
    global prev_offset, hysteresis_count, last_sent_cmd, PERSPECTIVE_MATRIX
    h, w = frame.shape[:2]
   
    # Apply perspective transform if enabled (bird's eye view)
    if ENABLE_PERSPECTIVE:
        if PERSPECTIVE_MATRIX is None:
            PERSPECTIVE_MATRIX = cv2.getPerspectiveTransform(PERSPECTIVE_SRC, PERSPECTIVE_DST)
            print(" Perspective transform matrix computed")
        frame = cv2.warpPerspective(frame, PERSPECTIVE_MATRIX, (w, h))
   
    # crop lower area (camera low -> lane appears lower in frame)
    roi = frame[int(h * ROI_TOP_RATIO):h, :]




    # convert to grayscale for better white detection
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    # apply CLAHE to enhance contrast
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    # threshold to get white lines
    _, th = cv2.threshold(enhanced, V_THRESHOLD, 255, cv2.THRESH_BINARY)




    # morphological to clean noise
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, MORPH_KERNEL_SIZE)
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel)




    # column-wise projection
    col_sum = np.sum(th, axis=0)
    # smooth projection
    col_sum = cv2.GaussianBlur(col_sum.reshape(1, -1), (1, GAUSSIAN_KERNEL_WIDTH), 0).flatten()




    # Find all peaks with simple threshold-based method (no scipy needed)
    # A peak is where col_sum is locally maximum
    threshold = np.max(col_sum) * 0.25  # at least 25% of max
    peaks = []
    for i in range(1, len(col_sum) - 1):
        if col_sum[i] > threshold and col_sum[i] > col_sum[i-1] and col_sum[i] > col_sum[i+1]:
            # Check it's not too close to existing peaks
            too_close = False
            for p in peaks:
                if abs(i - p) < 30:  # minimum 30px separation
                    too_close = True
                    break
            if not too_close:
                peaks.append(i)
   
    # DEBUG: Print all detected peaks
    img_center = w // 2
    if peaks:
        peak_info = [f"x={p} (intensity={int(col_sum[p])}, dist_from_center={abs(p-img_center)})" for p in peaks]
        print(f"   Detected {len(peaks)} peaks: {', '.join(peak_info)}")
   
    lane_center = None
    left_x, right_x = None, None
   
    if len(peaks) >= 2:
        # QUICK PICK: prefer one peak on each side of image center (closest to center)
        left_peaks = [p for p in peaks if p < img_center]
        right_peaks = [p for p in peaks if p > img_center]
        quick_selected = False
        if left_peaks and right_peaks:
            left_choice = max(left_peaks)
            right_choice = min(right_peaks)
            width = right_choice - left_choice
            if 25 <= width <= 160:
                left_x, right_x = left_choice, right_choice
                lane_center = (left_x + right_x) // 2
                quick_selected = True
                print(f"   QUICK SELECT left/right: ({left_x},{right_x}) width={width}px")




        if not quick_selected:
            # Strategy: REQUIRE pairs that bracket image center (p1 <= center <= p2)
            # This prevents selecting two lines from the same lane side
            best_pair = None
            best_score = -1
           
            # DEBUG: Track all candidate pairs
            candidates = []
           
            # FIRST PASS: Only consider pairs that BRACKET the image center
            bracketing_pairs = []
           
            for i in range(len(peaks) - 1):
                for j in range(i + 1, len(peaks)):
                    p1, p2 = min(peaks[i], peaks[j]), max(peaks[i], peaks[j])
                    width = p2 - p1
                   
                    intensity_score = col_sum[p1] + col_sum[p2]
                    pair_center = (p1 + p2) / 2
                    center_dist = abs(pair_center - img_center)
                    proximity_bonus = max(0, w - center_dist) / w  # 0..1
                   
                    # Check if pair brackets the image center
                    brackets_center = (p1 <= img_center <= p2)
                   
                    # REVISED SCORING: Heavily prioritize pairs that bracket the center.
                    if brackets_center:
                        # HUGE bonus for bracketing. Proximity is a good tie-breaker.
                        # Width is now only for filtering, not scoring, to avoid bias against correct wide lanes.
                        score = intensity_score * (1 + proximity_bonus) * 50.0  # MASSIVE bracket bonus
                       
                        # Filter: accept lane pairs from 25px to MAX_LANE_WIDTH_PX
                        if 25 <= width <= MAX_LANE_WIDTH_PX:
                            bracketing_pairs.append((p1, p2, score, width))
                    else:
                        # Non-bracketing pairs get a much lower base score
                        score = intensity_score * (1 + proximity_bonus)
                   
                    # DEBUG: Track this candidate
                    valid = (25 <= width <= MAX_LANE_WIDTH_PX) and brackets_center
                    candidates.append({
                        'pair': (p1, p2),
                        'width': width,
                        'score': score,
                        'valid': valid,
                        'pair_center': int(pair_center),
                        'center_dist': int(center_dist),
                        'brackets': brackets_center
                    })
           
            # DEBUG: Print all candidates
            print(f"   Evaluated {len(candidates)} pairs:")
            for c in sorted(candidates, key=lambda x: x['score'], reverse=True)[:5]:  # top 5
                status = " VALID" if c['valid'] else "❌ FILTERED"
                selected = ""  # will mark later
                bracket = " BRACKETS CENTER" if c.get('brackets') else " NO BRACKET"
                print(f"     {status} {bracket} pair=({c['pair'][0]},{c['pair'][1]}) width={c['width']}px "
                      f"center={c['pair_center']} dist={c['center_dist']} score={int(c['score'])}")
           
            # SELECT: Choose best bracketing pair if any exist
            if bracketing_pairs:
                # Sort by score and pick best
                bracketing_pairs.sort(key=lambda x: x[2], reverse=True)
                best_pair = (bracketing_pairs[0][0], bracketing_pairs[0][1])
                print(f"   SELECTED BRACKETING PAIR: ({best_pair[0]}, {best_pair[1]}) width={bracketing_pairs[0][3]}px")
            else:
                print(f"   NO BRACKETING PAIRS FOUND - using fallback")
                # NEW FALLBACK: if no bracketing pairs, pick closest valid pair to center
                if candidates:
                    # Filter for pairs with valid width first
                    valid_candidates = [c for c in candidates if 25 <= c['width'] <= MAX_LANE_WIDTH_PX]
                    if valid_candidates:
                        # Sort them by distance to the center of the image
                        valid_candidates.sort(key=lambda x: x['center_dist'])
                        best_pair = valid_candidates[0]['pair']
                        print(f"   FALLBACK: Using closest valid pair ({best_pair[0]}, {best_pair[1]})")
           
            if best_pair:
                left_x, right_x = best_pair
                lane_center = (left_x + right_x) // 2
    elif len(peaks) == 1:
        # Only one peak - try contour method
        contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            contours = sorted(contours, key=cv2.contourArea, reverse=True)[:MAX_CONTOURS]
            centroids = []
            for c in contours:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    centroids.append(cx)
            if len(centroids) >= 2:
                img_center = w // 2
                # Find pair that brackets or is closest to center
                best_pair = None
                min_width = float('inf')
                for i in range(len(centroids) - 1):
                    for j in range(i + 1, len(centroids)):
                        c1, c2 = min(centroids[i], centroids[j]), max(centroids[i], centroids[j])
                        if c1 <= img_center <= c2:
                            width = c2 - c1
                            if width < min_width or best_pair is None:
                                best_pair = (c1, c2)
                                min_width = width
                if best_pair is None:
                    # Get 2 closest to center
                    cent_dists = [(abs(cx - img_center), cx) for cx in centroids]
                    cent_dists.sort()
                    best_pair = tuple(sorted([cent_dists[0][1], cent_dists[1][1]]))
                left_x, right_x = best_pair
                lane_center = (left_x + right_x) // 2




    # compute offset relative to image center and apply bias
    img_center = w // 2
    offset = 0
    cmd = FALLBACK_CMD
    if lane_center is not None:
        # LANE FOLLOWING LOGIC: offset = lane_center - img_center
        # If lane is RIGHT of center (lane_center > img_center) → offset POSITIVE → turn RIGHT to follow
        # If lane is LEFT of center (lane_center < img_center) → offset NEGATIVE → turn LEFT to follow
        offset = (lane_center - img_center) + OFFSET_BIAS_PX




        # smoothing (EMA)
        smoothed = int(OFFSET_SMOOTH_ALPHA * offset + (1 - OFFSET_SMOOTH_ALPHA) * prev_offset)
        prev_offset = smoothed




        # deadband (pixels)
        dead = int(w * DEADBAND_PCT)
        print(f"   Offset: raw={offset:+3d}px, smoothed={smoothed:+3d}px, deadband=±{dead}px")




        # desired command based on smoothed offset
        # LANE FOLLOWING LOGIC:
        # offset = lane_center - img_center
        # If lane drifts RIGHT (offset > 0) → turn RIGHT to follow lane
        # If lane drifts LEFT (offset < 0) → turn LEFT to follow lane
        if abs(smoothed) <= dead:
            wish = "F"
        elif smoothed > 0:
            wish = "R"  # offset POSITIVE → lane RIGHT of center → turn RIGHT to follow
        else:
            wish = "L"  # offset NEGATIVE → lane LEFT of center → turn LEFT to follow
       
        print(f"   Decision: smoothed={smoothed:+3d}px, deadband=±{dead}px → wish={wish}")




        # hysteresis: only change command if wish persists
        if last_sent_cmd is None or wish == last_sent_cmd:
            # no change or same as before -> reset counter and keep
            hysteresis_count = 0
            cmd = wish
            last_sent_cmd = wish
        else:
            hysteresis_count += 1
            if hysteresis_count >= CMD_HYSTERESIS_FRAMES:
                cmd = wish
                last_sent_cmd = wish
                hysteresis_count = 0
            else:
                # keep previous command until hysteresis satisfied
                cmd = last_sent_cmd or FALLBACK_CMD




        debug_offset = smoothed
    else:
        # cannot detect lane; estimate offset based on single peak if available
        if len(peaks) == 1:
            # Single peak detected - use it to estimate position
            peak_pos = peaks[0]
            if peak_pos < img_center - 30:
                # Peak far LEFT of center → lane is LEFT → turn LEFT to follow
                offset = -60  # negative offset → wish = L
                wish = "L"
                print(f"   SINGLE PEAK LEFT: x={peak_pos} < center-30 → WISH LEFT")
            elif peak_pos > img_center + 30:
                # Peak far RIGHT of center → lane is RIGHT → turn RIGHT to follow
                offset = 60  # positive offset → wish = R
                wish = "R"
                print(f"   SINGLE PEAK RIGHT: x={peak_pos} > center+30 → WISH RIGHT")
            else:
                # Peak near center → go forward
                offset = 0
                wish = "F"
                print(f"   SINGLE PEAK CENTER: x={peak_pos} ≈ center → WISH FORWARD")
           
            # Apply same hysteresis logic as normal case
            if last_sent_cmd is None or wish == last_sent_cmd:
                hysteresis_count = 0
                cmd = wish
                last_sent_cmd = wish
            else:
                hysteresis_count += 1
                if hysteresis_count >= CMD_HYSTERESIS_FRAMES:
                    cmd = wish
                    last_sent_cmd = wish
                    hysteresis_count = 0
                else:
                    cmd = last_sent_cmd or FALLBACK_CMD
           
            debug_offset = offset
        else:
            # No peaks at all - STOP for safety
            cmd = FALLBACK_CMD
            debug_offset = 0




    # NEW: Invert steering command if the flag is set (MUST be AFTER all cmd assignments)
    if INVERT_STEERING_COMMANDS:
        if cmd == "L":
            cmd = "R"
            print("   INVERTED CMD: L -> R")
        elif cmd == "R":
            cmd = "L"
            print("   INVERTED CMD: R -> L")




    debug = {
        "roi": roi,
        "thresh": th,
        "left_x": left_x,
        "right_x": right_x,
        "lane_center": lane_center,
        "offset": int(debug_offset),
        "img_center": img_center,
        "cmd": cmd,
        "wish": locals().get('wish', None),
        "last_sent_cmd": last_sent_cmd,
        "peaks": peaks,  # ADD: pass peaks for visualization
        "col_sum": col_sum,  # ADD: pass column projection for visualization
    }
    return cmd, debug








def overlay_debug(frame, debug, traffic_label=None, traffic_conf=None, final_cmd=None):
    """Draw overlay on a copy of frame; returns BGR image."""
    out = frame.copy()
    h, w = out.shape[:2]
   
    # Draw perspective transform trapezoid if enabled
    if ENABLE_PERSPECTIVE:
        pts = PERSPECTIVE_SRC.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(out, [pts], True, (255, 0, 255), 2)
        cv2.putText(out, "PERSPECTIVE", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
   
    # Draw AI detection ROI (upper 60%)
    ai_roi_bottom = int(h * 0.6)
    cv2.rectangle(out, (0, 0), (w - 1, ai_roi_bottom), (0, 255, 255), 2)
    cv2.putText(out, "AI ROI", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
   
    # Draw lane detection ROI (lower 60%)
    lane_roi_top = int(h * 0.4)
    cv2.rectangle(out, (0, lane_roi_top), (w - 1, h - 1), (255, 0, 0), 1)
    cv2.putText(out, "Lane ROI", (10, lane_roi_top + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)




    if debug is not None:
        # NEW: Draw contours around white lane regions
        if debug.get('thresh') is not None:
            th = debug['thresh']
            contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # Draw contours on the ROI area
            roi_h = th.shape[0]
            for cnt in contours:
                if cv2.contourArea(cnt) > 100:  # filter small noise
                    # Shift contours to full frame coordinates
                    cnt_shifted = cnt + np.array([0, lane_roi_top])
                    cv2.drawContours(out, [cnt_shifted], -1, (255, 255, 0), 2)  # cyan contours
       
        # NEW: Draw ALL detected peaks (not just selected pair)
        if debug.get('peaks') is not None:
            peaks = debug['peaks']
            roi_h = debug['roi'].shape[0]
            for peak_x in peaks:
                # Draw vertical line for each peak
                cv2.line(out, (peak_x, lane_roi_top), (peak_x, lane_roi_top + roi_h), (255, 0, 255), 1)  # magenta peaks
                # Draw small circle at bottom
                cv2.circle(out, (peak_x, lane_roi_top + roi_h - 10), 3, (255, 0, 255), -1)
       
        # draw SELECTED left/right markers on ROI (THICK GREEN lines)
        roi_h = debug['roi'].shape[0]
        offset_y = lane_roi_top
        if debug.get('left_x') is not None:
            cv2.line(out, (debug['left_x'], offset_y), (debug['left_x'], offset_y + roi_h), (0, 255, 0), 3)  # GREEN = selected
            cv2.putText(out, "L", (debug['left_x'] - 10, offset_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if debug.get('right_x') is not None:
            cv2.line(out, (debug['right_x'], offset_y), (debug['right_x'], offset_y + roi_h), (0, 255, 0), 3)  # GREEN = selected
            cv2.putText(out, "R", (debug['right_x'] + 5, offset_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if debug.get('lane_center') is not None:
            lc = debug['lane_center']
            cv2.line(out, (lc, offset_y), (lc, offset_y + roi_h), (0, 128, 255), 3)  # ORANGE = lane center
            cv2.putText(out, "LC", (lc - 15, offset_y + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 128, 255), 2)
            # draw center of image (RED = image center reference)
            cv2.line(out, (w//2, offset_y), (w//2, offset_y + roi_h), (0, 0, 255), 2)
            cv2.putText(out, "IMG_CENTER", (w//2 + 5, offset_y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.line(out, (w//2, offset_y), (w//2, offset_y + roi_h), (0, 0, 255), 1)
            # offset text
            cv2.putText(out, f"offset={debug['offset']}", (10, offset_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)




    # traffic info
    info_y = 30
    if traffic_label:
        cv2.putText(out, f"Traffic: {traffic_label} ({traffic_conf:.2f})", (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0) if traffic_label== 'green' else (0,0,255), 2)
        info_y += 25




    # final command
    if final_cmd:
        cv2.putText(out, f"CMD: {final_cmd}", (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,255), 3)




    return out
 
# ================== BACKGROUND INFERENCE ==================
async def handle_inference(frame):
    global inference_busy, latest_result, latest_command, last_inference_time, recent_predictions
    if inference_busy:
        return  # bỏ qua frame nếu model đang chạy
    inference_busy = True
    last_inference_time = time.time()
    loop = asyncio.get_event_loop()
    try:
        # USE FULL FRAME for traffic sign detection (not ROI)
        img = preprocess_image(frame)
        label, conf = await loop.run_in_executor(None, run_inference, img)
       
        # PRIORITY: red/stopsign → phản hồi TỨC THÌ (không cần voting)
        if label in ["red", "stopsign"] and conf >= 0.7:
            cmd = decide_command(label, current_mode)
            latest_result = {"class": label, "confidence": conf, "command": cmd}
            latest_command = cmd
            recent_predictions.clear()  # Clear voting khi có tín hiệu khẩn
            mode_prefix = " LINE_FOLLOW" if current_mode == "line_follow" else " AUTO"
            print(f" URGENT ({mode_prefix}): {label} ({conf*100:.1f}%) → {cmd}")
            return
       
        # Các màu khác cần voting để ổn định
        if conf >= MIN_CONFIDENCE:
            recent_predictions.append(label)
       
        # Kiểm tra xem có 2/3 frame gần nhất giống nhau không
        if len(recent_predictions) >= 2:
            from collections import Counter
            most_common, count = Counter(recent_predictions).most_common(1)[0]
           
            # Nếu có ít nhất 2/3 frame giống nhau → cập nhật
            if count >= 2:
                cmd = decide_command(most_common, current_mode)
                latest_result = {"class": most_common, "confidence": conf, "command": cmd}
                latest_command = cmd
                mode_prefix = " LINE_FOLLOW" if current_mode == "line_follow" else " AUTO"
                print(f"{mode_prefix}: {most_common} ({conf*100:.1f}%) → {cmd}")
            # Không in gì khi đang waiting
        else:
            # Frame đầu tiên → cho phép đi tiếp nếu là green hoặc none
            if label in ["green", "none"] and conf >= MIN_CONFIDENCE:
                cmd = decide_command(label, current_mode)
                latest_result = {"class": label, "confidence": conf, "command": cmd}
                latest_command = cmd
                mode_prefix = " LINE_FOLLOW" if current_mode == "line_follow" else " AUTO"
                print(f"{mode_prefix}: {label} ({conf*100:.1f}%) → {cmd}")
           
    finally:
        inference_busy = False




# ================== WEBSOCKET HANDLER ==================
@app.websocket("/ws/traffic_light")
async def ws_traffic_light(websocket: WebSocket):
    global connected_ws, last_frame, latest_result, latest_command, last_overlay, latest_lane_cmd, last_lane_time, last_sent_traffic_cmd
    await websocket.accept()
    connected_ws = websocket
    print(" ESP32 connected via WebSocket")




    try:
        while True:
            # receive binary (jpeg) with timeout
            try:
                data = await asyncio.wait_for(websocket.receive_bytes(), timeout=5.0)
            except asyncio.TimeoutError:
                # no frame received; keep loop
                print(" No frame for 5s, waiting...")
                continue
            except Exception as e:
                print(f" Error receiving frame: {e}")
                break




            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is None:
                print(" Invalid frame")
                continue
            last_frame = frame




            now = time.time()
           
            # ===== LINE FOLLOW MODE (Arduino controls steering, AI only traffic lights) =====
            if current_mode == 'line_follow':
                # Traffic AI runs to detect traffic lights
                if (now - last_inference_time) > INFERENCE_INTERVAL:
                    asyncio.create_task(handle_inference(frame))


                # Determine final command (ONLY S or A)
                if latest_result and latest_result.get('command') == 'S':
                    final_cmd = 'S'
                else:
                    final_cmd = 'A'  # Allow Arduino to follow line


                # ONLY send command if it changed (avoid spamming Arduino)
                if final_cmd != last_sent_traffic_cmd:
                    last_sent_traffic_cmd = final_cmd
                   
                    # Prepare message to ESP32
                    payload = {
                        'class': latest_result['class'] if latest_result else 'none',
                        'command': final_cmd,
                        'confidence': latest_result['confidence'] if latest_result else 0.0,
                        'mode': current_mode
                    }


                    # Send JSON to ESP32
                    try:
                        async with ws_lock:
                            await websocket.send_json(payload)
                        print(f" LINE_FOLLOW: Sent {final_cmd} ({'STOP' if final_cmd == 'S' else 'ALLOW LINE FOLLOW'})")
                    except Exception as e:
                        print(" Failed send to ESP:", e)


                # Produce overlay for debug (no lane debug in line_follow mode)
                try:
                    overlay = overlay_debug(frame, None,
                                          traffic_label=(latest_result['class'] if latest_result else None),
                                          traffic_conf=(latest_result['confidence'] if latest_result else 0),
                                          final_cmd=final_cmd)
                    _, jpg = cv2.imencode('.jpg', overlay)
                    last_overlay = jpg.tobytes()
                except:
                    pass




            # ===== AUTO MODE (Full AI control) =====
            elif current_mode == 'auto':
                # Lane detection at higher rate
                if (now - last_lane_time) > LANE_RUN_INTERVAL:
                    lane_cmd, lane_debug = process_lane(frame)
                    latest_lane_cmd = lane_cmd
                    last_lane_time = now
                    # DEBUG: Print lane info
                    if lane_debug:
                        offset = lane_debug.get('offset', 0)
                        lc = lane_debug.get('lane_center')
                        if lc is not None:
                            print(f" LANE: center={lc}, offset={offset:+3d}px → {lane_cmd}")
                        else:
                            print(f" LANE: NOT DETECTED → {lane_cmd}")
                else:
                    lane_debug = None




                # Traffic AI at lower rate
                if (now - last_inference_time) > INFERENCE_INTERVAL:
                    asyncio.create_task(handle_inference(frame))




                # Determine final command
                # PRIORITY: traffic AI STOP overrides lane
                if latest_result and latest_result.get('command') == 'S':
                    final_cmd = 'S'
                    print(f" FINAL CMD: {final_cmd} (traffic override)")
                else:
                    # Use lane command (L/R/F)
                    final_cmd = latest_lane_cmd or 'F'
                    if latest_lane_cmd:
                        print(f" FINAL CMD: {final_cmd} (lane following)")




                # Prepare message to ESP32
                payload = {
                    'class': latest_result['class'] if latest_result else 'lane',
                    'command': final_cmd,
                    'confidence': latest_result['confidence'] if latest_result else 1.0,
                    'mode': current_mode
                }




                # Send JSON to ESP32
                try:
                    async with ws_lock:
                        await websocket.send_json(payload)
                except Exception as e:
                    print(" Failed send to ESP:", e)




                # Produce overlay for debug
                try:
                    overlay = overlay_debug(frame, lane_debug,
                                          traffic_label=(latest_result['class'] if latest_result else None),
                                          traffic_conf=(latest_result['confidence'] if latest_result else 0),
                                          final_cmd=final_cmd)
                    _, jpg = cv2.imencode('.jpg', overlay)
                    last_overlay = jpg.tobytes()
                except:
                    pass




            # ===== MANUAL MODE =====
            else:
                if latest_command and latest_command in ["F", "B", "L", "R", "S"]:
                    async with ws_lock:
                        await websocket.send_json({
                            "class": "manual_command",
                            "command": latest_command,
                            "confidence": 1.0,
                            "mode": "manual"
                        })
                    print(f" MANUAL: Sent command '{latest_command}'")
                    # Clear lệnh sau khi gửi để tránh gửi lại
                    latest_command = None




    except WebSocketDisconnect:
        print(" ESP32 disconnected.")
    except Exception as e:
        print(" Error in WS loop:", e)
    finally:
        if connected_ws is websocket:
            connected_ws = None




# ================== DEBUG / API ENDPOINTS ==================
@app.get("/debug/frame")
def debug_frame():
    global last_frame
    if last_frame is None:
        return {"error": "No frame received yet"}
    _, jpeg = cv2.imencode('.jpg', last_frame)
    return Response(content=jpeg.tobytes(), media_type='image/jpeg')








@app.get("/debug/overlay")
def debug_overlay():
    global last_overlay
    if last_overlay is None:
        return {"error": "No overlay available yet"}
    return Response(content=last_overlay, media_type='image/jpeg')








@app.get("/api/status")
def get_status():
    return {
        "mode": current_mode,
        "latest_command": latest_command,
        "latest_result": latest_result,
        "esp32_connected": connected_ws is not None,
    }








@app.post("/api/control")
async def manual_control(request: Request, data: dict = Body(...)):
    global latest_command
    if current_mode != "manual":
        return {"error": "Đang ở chế độ Auto, không thể điều khiển thủ công"}




    cmd = data.get("command", "").upper()
    if cmd not in ["F", "B", "L", "R", "S"]:
        return {"error": "Lệnh không hợp lệ"}




    latest_command = cmd
    client_ip = request.client.host if request.client else "unknown"
    msg = f" MANUAL COMMAND: {cmd} from {client_ip}"
    push_notification(msg)
    print(msg)
    return {"status": "ok", "command": cmd}








@app.post("/api/mode")
async def set_mode(data: dict = Body(...)):
    global current_mode, last_sent_traffic_cmd, latest_result
    mode = data.get("mode", "").lower()
    if mode not in ["auto", "manual", "line_follow"]:
        return {"error": "Mode không hợp lệ. Chọn: auto, manual, hoặc line_follow"}
   
    # Reset traffic command state when switching to line_follow
    if mode == "line_follow":
        last_sent_traffic_cmd = None  # Reset để gửi lệnh mới
        # Reset latest_result để không giữ lệnh STOP cũ
        if latest_result and latest_result.get('command') == 'S':
            latest_result = None
   
    current_mode = mode
    print(f" Đổi sang chế độ: {current_mode.upper()}")
    return {"status": "ok", "mode": current_mode}








@app.get('/api/notifications')
def api_notifications(limit: int = 50):
    items = list(reversed(list(recent_notifications)))[:limit]
    return {"notifications": items}








@app.get('/api/myip')
def api_myip():
    return {"ips": get_local_ips()}








# API để nhận lệnh từ ứng dụng di động
@app.post("/api/send_command")
async def send_command(data: dict = Body(...)):
    global latest_command
    cmd = data.get("command", "").upper()
    if cmd not in ["F", "L", "R", "S"]:  # Các lệnh hợp lệ: F (Forward), L (Left), R (Right), S (Stop)
        return {"error": "Lệnh không hợp lệ. Chọn: F, L, R, S."}
    latest_command = cmd
    print(f" Command received: {cmd}")
    return {"status": "ok", "command": cmd}




# WebSocket để gửi frame liên tục
@app.websocket("/ws/frames")
async def ws_frames(websocket: WebSocket):
    global last_frame
    await websocket.accept()
    print(" Mobile app connected via WebSocket")




    try:
        while True:
            if last_frame is not None:
                # Chuyển frame thành base64 để gửi qua WebSocket
                _, buffer = cv2.imencode('.jpg', last_frame)
                frame_data = base64.b64encode(buffer).decode('utf-8')
                await websocket.send_text(frame_data)
            await asyncio.sleep(0.05)  # Gửi frame mỗi 50ms
    except Exception as e:
        print(f" WebSocket error: {e}")
    finally:
        print(" Mobile app disconnected")




# ================== RUN SERVER ==================
if __name__ == '__main__':
    uvicorn.run('server_lane_ai_full:app', host='0.0.0.0', port=8000, workers=1)

