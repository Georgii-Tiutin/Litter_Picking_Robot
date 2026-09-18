#!/usr/bin/env python3
"""
Cube arm tracker (eye-in-hand visual servoing).
Detects cubes with best.pt, picks the HIGHEST-CONFIDENCE one, and moves the arm
to keep it centered in the camera:
  - horizontal  -> joint1 (base pan)
  - vertical    -> a CHAIN of pitch joints (shoulder j2 + elbow j3, extensible)
    so when one joint saturates the next keeps lifting the "head", giving a much
    larger vertical range than a single joint.
Renders the live annotated view on the robot's monitor (cv2.imshow), publishes it
to /cube_tracker/image, and saves a jpg for remote verification.

Env knobs:
  CUBE_MODEL  path to .pt            (default best.pt)
  CUBE_CONF   confidence floor       (default 0.5)
  CUBE_MOVE   "1" enables arm motion (default "0" = dry-run)
  CUBE_SIGN   pan direction sign     (default 1)
  CUBE_VERT   "1" enable vertical    (default 1)
  CUBE_SHOW / CUBE_OUT / CUBE_TOPIC
"""
import os, time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from arm_msgs.msg import ArmJoint, ArmJoints
from ultralytics import YOLO

MODEL     = os.environ.get("CUBE_MODEL", "/home/jetson/models/cuboid_v1/best.pt")
CONF      = float(os.environ.get("CUBE_CONF", "0.5"))
MOVE      = os.environ.get("CUBE_MOVE", "0") == "1"
SIGN      = float(os.environ.get("CUBE_SIGN", "1"))
VERT      = os.environ.get("CUBE_VERT", "1") == "1"
SHOW      = os.environ.get("CUBE_SHOW", "1") == "1"
OUT       = os.environ.get("CUBE_OUT", "/tmp/cube_track_latest.jpg")
CAM_TOPIC = os.environ.get("CUBE_TOPIC", "/camera/color/image_raw")

# arm / control
HOME_J2   = float(os.environ.get("CUBE_HOME_J2", "135"))  # resting shoulder tilt; higher = head up
HOME_POSE = [90, int(HOME_J2), 0, 25, 90, 0]  # start/observation pose (user-set 2026-07-30): j4=25 wrist
MOVE_TIME = 150      # ms servo move time
CTRL_PERIOD = 0.10   # s between control ticks

# horizontal (pan) loop -> joint1
J1_HOME   = 90.0
J1_MIN, J1_MAX = 30.0, 150.0
DEADBAND  = 0.06
KP        = 4.0
MAX_STEP  = 1.0

# vertical (pitch) chain. Each entry: id, sign, home, lo, hi, weight.
# sign chosen so that a cube ABOVE center (errv<0) drives the joint to look UP.
# joint2 (shoulder) is primary; joint3 (elbow) extends the range when j2 saturates.
TILT_CHAIN = [
    {"id": 2, "sign": -1.0, "home": HOME_J2, "lo": 45.0, "hi": 155.0, "w": 1.0},
    {"id": 3, "sign": -1.0, "home": 0.0,     "lo": 0.0,  "hi": 90.0,  "w": 0.7},
]
DEADBAND_V = 0.07
KP_V      = 2.0
MAX_STEP_V = 0.6

# base-facing: after the arm settles on the cube, rotate the CHASSIS in place to
# face the cube while the arm's visual servo unwinds joint1 back toward center.
BASE_FACE   = os.environ.get("CUBE_BASE_FACE", "1") == "1"
SIGN_BASE   = float(os.environ.get("CUBE_SIGN_BASE", "1"))  # verify direction empirically
BASE_DEADBAND = 12.0   # deg of |j1-90| below which the base is "facing" -> hand off to approach steering
K_BASE      = 0.010    # rad/s per deg of joint1 offset
MAX_WZ      = 0.15     # rad/s max base spin (slow)
BASE_DWELL  = 0.5      # s the arm must stay CENTERED before the base engages
BASE_DIVERGE = 40.0    # deg |j1-90|; if exceeded while facing -> wrong sign, abort

# approach: after facing the cube, drive FORWARD (only) up to it while the tilt
# chain keeps it centered; stop when the arm has pitched down far enough (near).
APPROACH_SPEED = float(os.environ.get("CUBE_APPROACH_SPEED", "0.08"))  # m/s (slow)
APPROACH_STOP_J2 = float(os.environ.get("CUBE_STOP_J2", "70"))  # deg; j2<=this => near -> stop
APPROACH_MAX_TIME = float(os.environ.get("CUBE_APPROACH_MAXT", "10"))  # s hard cap (~speed*this m)
LOST_TIMEOUT = 2.0     # s of continuous no-detection before the state machine resets (flicker guard)
REARM_J2 = float(os.environ.get("CUBE_REARM_J2", "105"))  # arrived->re-approach when cube moves away (j2 tilts back up past this)
# continuous steering during approach: curve onto the cube (cancel parallax drift)
K_STEER = 0.012        # rad/s per deg of j1 offset, applied WHILE driving forward
MAX_STEER_WZ = 0.25    # rad/s cap on approach steering
STEER_DEADBAND = 2.0   # deg of |j1-90| below which we don't steer


class CubeArmTracker(Node):
    def __init__(self):
        super().__init__("cube_arm_tracker")
        self.bridge = CvBridge()
        self.get_logger().info("Loading model %s ..." % MODEL)
        self.model = YOLO(MODEL)
        self.names = self.model.names
        self.j1 = J1_HOME
        self.tilt = {j["id"]: j["home"] for j in TILT_CHAIN}
        self.last_ctrl = 0.0
        self.last_log = 0.0
        self.nframe = 0
        self.phase = "track"          # track -> face -> approach -> arrived
        self.centered_since = None
        self.approach_since = None
        self.lost_since = None
        self.base_wz = 0.0
        self.base_vx = 0.0
        self.base_aborted = False
        self.pub_joint  = self.create_publisher(ArmJoint,  "/arm_joint", 10)
        self.pub_joints = self.create_publisher(ArmJoints, "/arm6_joints", 10)
        self.pub_cmdvel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_img    = self.create_publisher(Image, "/cube_tracker/image", 5)
        self.create_subscription(Image, CAM_TOPIC, self.on_image, 1)
        time.sleep(1.0)
        self.send_home()
        self.get_logger().info("READY MOVE=%s SIGN=%s VERT=%s tilt=%s" %
                               (MOVE, SIGN, VERT, [j["id"] for j in TILT_CHAIN]))

    def send_home(self):
        m = ArmJoints()
        m.joint1, m.joint2, m.joint3, m.joint4, m.joint5, m.joint6 = [int(v) for v in HOME_POSE]
        m.time = 1500
        self.pub_joints.publish(m)
        self.j1 = J1_HOME
        self.tilt = {j["id"]: j["home"] for j in TILT_CHAIN}

    def send_arm(self, jid, val):
        m = ArmJoint()
        m.id = int(jid)
        m.joint = int(round(val))
        m.time = MOVE_TIME
        self.pub_joint.publish(m)

    def stop_base(self, n=1):
        for _ in range(n):
            self.pub_cmdvel.publish(Twist())

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn("cv_bridge: %s" % e)
            return
        h, w = frame.shape[:2]
        cx, cy = w / 2.0, h / 2.0
        res = self.model.predict(frame, conf=CONF, device=0, verbose=False)[0]

        boxes = []
        if res.boxes is not None:
            for b in res.boxes:
                conf = float(b.conf[0]); cls = int(b.cls[0])
                x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
                u = (x1 + x2) / 2.0; v = (y1 + y2) / 2.0
                boxes.append((conf, x1, y1, x2, y2, u, v, cls))

        target = max(boxes, key=lambda t: t[0]) if boxes else None

        state = "SEARCHING"
        err_px = 0
        errv_px = 0
        if target is not None:
            self.lost_since = None    # fresh detection -> clear the loss timer
            conf, x1, y1, x2, y2, u, v, cls = target
            err_norm = (u - cx) / (w / 2.0)
            errv_norm = (v - cy) / (h / 2.0)
            err_px = int(u - cx)
            errv_px = int(v - cy)
            h_centered = abs(err_norm) < DEADBAND
            v_centered = abs(errv_norm) < DEADBAND_V
            centered = h_centered and (v_centered or not VERT)
            state = "CENTERED" if centered else "TRACKING"
            now = time.time()
            if MOVE and (now - self.last_ctrl) >= CTRL_PERIOD:
                if not h_centered:
                    step = max(-MAX_STEP, min(MAX_STEP, KP * err_norm))
                    self.j1 = max(J1_MIN, min(J1_MAX, self.j1 + SIGN * step))
                    self.send_arm(1, self.j1)
                if VERT and not v_centered:
                    for j in TILT_CHAIN:
                        stepv = max(-MAX_STEP_V, min(MAX_STEP_V, KP_V * j["w"] * errv_norm))
                        nv = self.tilt[j["id"]] + j["sign"] * stepv
                        nv = max(j["lo"], min(j["hi"], nv))
                        self.tilt[j["id"]] = nv
                        self.send_arm(j["id"], nv)
                self.last_ctrl = now

            # ---- base phase machine: track -> face -> approach -> arrived ----
            # The arm visual servo above always keeps the cube centered; the base
            # only rotates (face) or drives forward (approach) on top of that.
            off = self.j1 - J1_HOME
            j2val = self.tilt[TILT_CHAIN[0]["id"]]   # primary pitch = proximity proxy
            vx = 0.0
            wz = 0.0
            if MOVE:
                if self.phase == "track":
                    if centered:
                        if self.centered_since is None:
                            self.centered_since = now
                        elif (now - self.centered_since) >= BASE_DWELL:
                            if BASE_FACE and abs(off) > BASE_DEADBAND and not self.base_aborted:
                                self.phase = "face"
                            else:
                                self.phase = "approach"; self.approach_since = now
                    else:
                        self.centered_since = None
                elif self.phase == "face":
                    if abs(off) <= BASE_DEADBAND:
                        self.phase = "approach"; self.approach_since = now
                    elif abs(off) > BASE_DIVERGE:
                        self.phase = "track"; self.base_aborted = True; self.centered_since = None
                        self.get_logger().warn("base-facing diverged (off=%.0f); flip CUBE_SIGN_BASE" % off)
                    else:
                        wz = max(-MAX_WZ, min(MAX_WZ, SIGN_BASE * K_BASE * off))
                elif self.phase == "approach":
                    if j2val <= APPROACH_STOP_J2:
                        self.phase = "arrived"
                        self.get_logger().info("arrived: j2=%.0f <= %.0f (cube near base)" % (j2val, APPROACH_STOP_J2))
                    elif self.approach_since and (now - self.approach_since) > APPROACH_MAX_TIME:
                        self.phase = "arrived"
                        self.get_logger().warn("approach max-time cap reached; stopping")
                    else:
                        # pursuit: steer toward the cube, and scale forward speed by how
                        # well-aligned we are -- so we rotate-in-place when far off-axis
                        # and only drive forward once pointed at the cube (no driving past).
                        align = max(0.0, 1.0 - abs(off) / 40.0)
                        vx = APPROACH_SPEED * align
                        if abs(off) > STEER_DEADBAND:
                            wz = max(-MAX_STEER_WZ, min(MAX_STEER_WZ, SIGN_BASE * K_STEER * off))
                elif self.phase == "arrived":
                    # parked at the cube; if the cube moves AWAY (arm tilts back up so
                    # j2 rises past REARM_J2), re-arm and approach it again.
                    if j2val >= REARM_J2:
                        self.phase = "track"
                        self.centered_since = None
                        self.approach_since = None
                        self.get_logger().info("re-arm: j2=%.0f >= %.0f (cube moved away)" % (j2val, REARM_J2))
                # arrived (holding) -> vx=wz=0
                self.base_vx = vx; self.base_wz = wz
                tw = Twist(); tw.linear.x = float(vx); tw.angular.z = float(wz)
                self.pub_cmdvel.publish(tw)
        else:
            # no cube THIS frame: ALWAYS stop the base immediately, but tolerate brief
            # detection flicker -- only tear down the state machine after LOST_TIMEOUT of
            # continuous loss. Without this, one dropped frame reset 'arrived' -> 'track'
            # and re-triggered the whole approach, so the robot crept forward in bursts.
            self.base_vx = 0.0
            self.base_wz = 0.0
            if MOVE:
                self.stop_base()
            now_l = time.time()
            if self.lost_since is None:
                self.lost_since = now_l
            elif (now_l - self.lost_since) > LOST_TIMEOUT:
                self.phase = "track"
                self.centered_since = None
                self.approach_since = None
                self.base_aborted = False

        now = time.time()
        if now - self.last_log >= 0.8:
            tstr = " ".join("j%d=%.0f" % (j["id"], self.tilt[j["id"]]) for j in TILT_CHAIN)
            self.get_logger().info("phase=%s state=%s err_px=%d errv_px=%d j1=%.1f [%s] vx=%.2f wz=%.2f targets=%d" %
                                   (self.phase, state, err_px, errv_px, self.j1, tstr, self.base_vx, self.base_wz, len(boxes)))
            self.last_log = now

        self.render(frame, boxes, target, cx, cy, state, err_px, errv_px)

    def render(self, frame, boxes, target, cx, cy, state, err_px, errv_px):
        h, w = frame.shape[:2]
        cv2.line(frame, (int(cx), 0), (int(cx), h), (0, 255, 255), 1)
        cv2.line(frame, (0, int(cy)), (w, int(cy)), (0, 255, 255), 1)

        for t in boxes:
            conf, x1, y1, x2, y2, u, v, cls = t
            is_t = target is not None and t is target
            color = (0, 255, 0) if is_t else (160, 160, 160)
            th = 3 if is_t else 1
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, th)
            name = self.names.get(cls, cls) if isinstance(self.names, dict) else cls
            cv2.putText(frame, "%s %.2f" % (name, conf), (int(x1), int(y1) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            if is_t:
                cv2.circle(frame, (int(u), int(v)), 5, (0, 255, 0), -1)
                cv2.line(frame, (int(cx), int(cy)), (int(u), int(v)), (255, 0, 255), 2)

        movestr = "MOVE" if MOVE else "DRY-RUN"
        tstr = " ".join("j%d:%.0f" % (j["id"], self.tilt[j["id"]]) for j in TILT_CHAIN)
        phase_txt = {"track": "TRACK (arm)", "face": "FACE (base turning)",
                     "approach": "APPROACH (driving fwd)", "arrived": "ARRIVED"}.get(self.phase, self.phase)
        hud = ["PHASE: %s" % phase_txt, "state: %s" % state,
               "err_h: %d  err_v: %d" % (err_px, errv_px),
               "pan j1: %.1f   tilt %s" % (self.j1, tstr if VERT else "off"),
               "base vx: %.2f  wz: %.2f" % (self.base_vx, self.base_wz),
               "%s  stopJ2<=%.0f" % (movestr, APPROACH_STOP_J2),
               "conf>=%.2f  targets: %d" % (CONF, len(boxes))]
        y = 22
        for line in hud:
            cv2.putText(frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
            cv2.putText(frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            y += 24

        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except Exception:
            pass
        self.nframe += 1
        if self.nframe % 5 == 0:
            cv2.imwrite(OUT, frame)
        if SHOW:
            try:
                cv2.imshow("Robot Cube Tracker", frame)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warn("imshow: %s" % e)


def main():
    rclpy.init()
    node = CubeArmTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_base(6)
        except Exception:
            pass
        try:
            node.send_home()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    main()
