cat > ~/vla_gripper_force_backup.py <<'PY'
import time, math, can

CAN_IF = "can0"
LEADER_ID = 2
FOLLOWER_ID = 3
NODES = [LEADER_ID, FOLLOWER_ID]

K = 0.025
D = 0.002
TAU_MAX = 0.08
LOOP_HZ = 100

bus = can.interface.Bus(channel=CAN_IF, interface="socketcan")

state = {
    LEADER_ID: {"pos_deg": 0.0, "vel_rpm": 0.0, "torque_nm": 0.0},
    FOLLOWER_ID: {"pos_deg": 0.0, "vel_rpm": 0.0, "torque_nm": 0.0},
}

def clamp(x, lo, hi): return max(lo, min(hi, x))
def s16(x): return x - 0x10000 if x & 0x8000 else x
def s24(x): return x - 0x1000000 if x & 0x800000 else x

def safe_send(node, data):
    try:
        msg = can.Message(
            arbitration_id=0x600 + node,
            data=bytes(data),
            is_extended_id=False
        )
        bus.send(msg, timeout=0.002)
        return True
    except can.CanOperationError as e:
        print("CAN TX drop:", e)
        return False

def parse_feedback(msg):
    arb = msg.arbitration_id
    if arb not in [0x580 + LEADER_ID, 0x580 + FOLLOWER_ID]:
        return

    node = arb - 0x580
    d = msg.data

    if len(d) != 8 or d[0] != 0x2A:
        return

    pos_raw = s24((d[1] << 16) | (d[2] << 8) | d[3])
    vel_raw = s16((d[4] << 8) | d[5])
    cur_raw = s16((d[6] << 8) | d[7])

    state[node]["pos_deg"] = pos_raw / 100.0
    state[node]["vel_rpm"] = vel_raw / 100.0
    state[node]["torque_nm"] = cur_raw / 100.0

def recv_drain(max_msgs=20):
    for _ in range(max_msgs):
        msg = bus.recv(0.0)
        if msg is None:
            break
        parse_feedback(msg)

def write_u16(node, reg, val):
    safe_send(node, [0x2B, reg >> 8, reg & 255, 0, val >> 8, val & 255, 0, 0])
    time.sleep(0.05)
    recv_drain()

def set_torque(node, tau):
    tau = clamp(tau, -TAU_MAX, TAU_MAX)
    raw = int(round(tau * 100.0))
    if raw < 0:
        raw += 65536

    safe_send(node, [0x2B, 0x00, 0x20, 0x00, raw >> 8, raw & 255, 0, 0])

def setup():
    for n in NODES:
        print("setup", n)
        write_u16(n, 0x0060, 0)   # torque mode
        write_u16(n, 0x00A2, 1)   # closed loop
        set_torque(n, 0)
        time.sleep(0.05)

def stop():
    for n in NODES:
        set_torque(n, 0)
    time.sleep(0.1)
    for n in NODES:
        write_u16(n, 0x00A0, 1)   # idle

setup()
print("VLA gripper Python backup running. leader=2 follower=3. Ctrl+C stop")

dt = 1.0 / LOOP_HZ
next_t = time.perf_counter()
last_print = 0

try:
    while True:
        recv_drain()

        qL = math.radians(state[LEADER_ID]["pos_deg"])
        qF = math.radians(state[FOLLOWER_ID]["pos_deg"])
        dqL = state[LEADER_ID]["vel_rpm"] * 2 * math.pi / 60
        dqF = state[FOLLOWER_ID]["vel_rpm"] * 2 * math.pi / 60

        err = qL - qF
        derr = dqL - dqF

        tau = -K * err - D * derr
        tau = clamp(tau, -TAU_MAX, TAU_MAX)

        set_torque(LEADER_ID, tau)
        time.sleep(0.001)
        set_torque(FOLLOWER_ID, -tau)

        now = time.perf_counter()
        if now - last_print > 0.2:
            print(
                f"q2={state[LEADER_ID]['pos_deg']:+7.2f} "
                f"q3={state[FOLLOWER_ID]['pos_deg']:+7.2f} "
                f"err={math.degrees(err):+6.2f} "
                f"tau={tau:+.3f}"
            )
            last_print = now

        next_t += dt
        sleep_t = next_t - time.perf_counter()
        if sleep_t > 0:
            time.sleep(sleep_t)
        else:
            next_t = time.perf_counter()

except KeyboardInterrupt:
    print("\nstopping")
    stop()
PY