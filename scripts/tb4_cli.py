#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys
import readline
import subprocess
import time

# ====== COLOR ======
class C:
    END    = '\033[0m'
    RED    = '\033[91m'
    GREEN  = '\033[92m'
    YELLOW = '\033[93m'
    CYAN   = '\033[96m'
    DIM    = '\033[2m'

def c(color, msg):
    return f"{color}{msg}{C.END}"

# ====== COMMAND LIST ======
COMMANDS = ["activate", "start", "deactivate", "goto", "patrol", "explore", "stop", "pause", "resume", "status", "help", "clear", "exit"]

TM_NODE = "/task_manager_lifecycle_node"
ALGOS = frozenset(['dwa', 'teb', 'pp', 'stanley'])

def lifecycle_set(transition: str) -> tuple[bool, str]:
    """Gọi ros2 lifecycle set; trả (ok, message)."""
    try:
        r = subprocess.run(
            ["ros2", "lifecycle", "set", TM_NODE, transition],
            capture_output=True, text=True, timeout=15,
        )
        out = (r.stdout + r.stderr).strip()
        return r.returncode == 0, out or f"exit {r.returncode}"
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        return False, str(e)

def lifecycle_get() -> str:
    try:
        r = subprocess.run(
            ["ros2", "lifecycle", "get", TM_NODE],
            capture_output=True, text=True, timeout=10,
        )
        out = (r.stdout + r.stderr).strip()
        if r.returncode != 0:
            return out or "lifecycle get failed"
        return out.splitlines()[0].strip() if out else "unknown"
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        return str(e)

def tm_node_present() -> bool:
    try:
        r = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True, text=True, timeout=10,
        )
        if r.returncode != 0:
            return False
        return TM_NODE in r.stdout
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False

def try_parse_goto_coords(parts: list[str]) -> dict | None:
    """Nếu parts[1] là số → goto theo tọa độ: goto x y [z] [yaw] [algo]."""
    if len(parts) < 3:
        return None
    try:
        float(parts[1])
    except ValueError:
        return None

    nums: list[float] = []
    i = 1
    while i < len(parts):
        if parts[i].lower() in ALGOS:
            break
        try:
            nums.append(float(parts[i]))
            i += 1
        except ValueError:
            return None

    if len(nums) < 2:
        return None

    x, y = nums[0], nums[1]
    z = nums[2] if len(nums) > 2 else 0.0
    yaw = nums[3] if len(nums) > 3 else 0.0
    algo = parts[i].lower() if i < len(parts) else 'dwa'
    if algo not in ALGOS:
        algo = 'dwa'
    return {'x': x, 'y': y, 'z': z, 'yaw': yaw, 'algo': algo}

# ====== AUTOCOMPLETE ======
def completer(text, state):
    buffer = readline.get_line_buffer()
    tokens = buffer.split()
    if len(tokens) <= 1:
        options = [cmd for cmd in COMMANDS if cmd.startswith(text)]
    else:
        import yaml
        try:
            with open('/ros2_ws/config/waypoints.yaml') as f:
                _data = yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError):
            _data = {}
        goals = list(_data.get('waypoints', {}).keys())
        options = [g for g in goals if g.startswith(text)]
    try:
        return options[state]
    except IndexError:
        return None

# ====== NODE ======
class TB4CLI(Node):
    def __init__(self):
        super().__init__('tb4_cli')
        self.pub = self.create_publisher(String, '/tb4/cmd', 10)
        self.sub = self.create_subscription(
            String,
            '/tb4/status',
            self.cb_status,
            10
        )
        self.last_status = None

    def cb_status(self, msg):
        self.last_status = msg.data

    def send_cmd(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)

    
    def refresh_heartbeat(self, timeout_sec: float = 1.2):
        """Đợi cập nhật heartbeat mà không gây xung đột spin_once.
           Chờ đợi cập nhật heartbeat từ /tb4/status thông qua background thread"""
        # Chỉ cần chờ đợi một khoảng thời gian để luồng background tự xử lý callback
        # Thay vì gọi spin_once ở đây, ta dùng time.sleep để đợi luồng kia nhận tin nhắn
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            time.sleep(0.1) # Nghỉ 0.1s để CPU không bị chiếm dụng
            # Không gọi spin_once ở đây nữa

def format_status(node: TB4CLI) -> str:
    lines: list[str] = []

    if not tm_node_present():
        lines.append("task_manager: node không thấy (chạy profile 'full' trước?)")
        return "\n".join(lines)

    lc = lifecycle_get()
    lines.append(f"lifecycle: {lc}")

    lc_lower = lc.lower()
    if 'unconfigured' in lc_lower:
        lines.append("→ container đang khởi động hoặc chưa configure — đợi vài giây rồi thử lại")
    elif 'inactive' in lc_lower:
        lines.append("→ đã configure, chưa activate — gõ 'activate' để điều khiển robot")
    elif 'active' in lc_lower:
        node.refresh_heartbeat()
        if node.last_status:
            lines.append(f"robot: {node.last_status}")
        else:
            lines.append("robot: (chưa nhận heartbeat — thử lại sau 1s)")

    return "\n".join(lines)

# ====== HELP ======
def print_help():
    print(c(C.CYAN, "\n=== HỆ THỐNG THỬ NGHIỆM THUẬT TOÁN TURTLEBOT4 ==="))
    cmds = [
        ("activate / start",  "Bật task_manager (lifecycle activate) — bắt buộc trước patrol/goto"),
        ("deactivate",        "Tắt task_manager (lifecycle deactivate)"),
        ("goto <wp> [algo]",  "Đi tới waypoint trong waypoints.yaml"),
        ("goto x y [z] [yaw] [algo]", "Đi tới tọa độ map (yaw rad, mặc định 0)"),
        ("patrol [algo]",     "Tuần tra tự động qua toàn bộ mission"),
        ("explore [algo]",    "Bật Frontier Exploration (Khám phá bản đồ) + thuật toán lái"),
        ("stop",              "Dừng robot ngay lập tức"),
        ("pause",             "Tạm dừng tác vụ hành trình"),
        ("resume",            "Tiếp tục hành trình đang hoãn"),
        ("status",            "Lifecycle + trạng thái robot (heartbeat)"),
        ("help",              "Hiện menu hướng dẫn này"),
        ("clear",             "Xóa màn hình dòng lệnh"),
        ("exit",              "Thoát giao diện điều khiển CLI"),
    ]
    print(c(C.DIM, "Các thuật toán (algo) hỗ trợ thử nghiệm: dwa | teb | pp | stanley\n"))
    for cmd, desc in cmds:
        print(f"  {c(C.YELLOW, cmd.ljust(28))} {c(C.DIM, desc)}")
    print(c(C.DIM, "\nVí dụ tọa độ: goto 3.5 3.5  |  goto 3.5 3.5 0 1.57 teb\n"))

# ====== CLI LOOP ======
def cli_loop(node: TB4CLI, stop_event: threading.Event):
    print(c(C.GREEN, "=== TB4 MULTI-MODULE CLI PRO ==="))
    print(c(C.DIM, "Luồng khuyến nghị: status → activate → goto/patrol/explore"))
    print(c(C.DIM, "Gõ 'help' để xem danh sách lệnh, Nhấn Tab để tự động điền (Autocomplete)\n"))

    while not stop_event.is_set():
        try:
            cmd = input("> ").strip()
            if not cmd:
                continue

            readline.add_history(cmd)
            parts = cmd.split()
            base_cmd = parts[0].lower()

            if base_cmd == "exit":
                print("Bye.")
                break

            elif base_cmd == "help":
                print_help()

            elif base_cmd == "clear":
                print("\033[2J\033[H", end="")   # ANSI clear screen
                print(c(C.GREEN, "=== TB4 MULTI-MODULE CLI PRO ===\n"))

            elif base_cmd in ("activate", "start"):
                ok, msg = lifecycle_set("activate")
                color = C.GREEN if ok else C.RED
                print(c(color, f"→ lifecycle activate: {msg}"))
                if ok:
                    node.refresh_heartbeat()

            elif base_cmd == "deactivate":
                ok, msg = lifecycle_set("deactivate")
                color = C.GREEN if ok else C.RED
                print(c(color, f"→ lifecycle deactivate: {msg}"))
                node.last_status = None

            elif base_cmd == "goto":
                coord = try_parse_goto_coords(parts)
                if coord:
                    c_ = coord
                    node.send_cmd(
                        f"goto_pos:{c_['x']}:{c_['y']}:{c_['z']}:{c_['yaw']}:{c_['algo']}"
                    )
                    print(c(C.YELLOW,
                        f"→ Sent goto pos ({c_['x']}, {c_['y']}, {c_['z']}) "
                        f"yaw={c_['yaw']} [{c_['algo'].upper()}]"))
                elif len(parts) < 2:
                    print(c(C.RED, "Dùng: goto <waypoint> [algo]  hoặc  goto x y [z] [yaw] [algo]"))
                else:
                    goal = parts[1]
                    algo = parts[2].lower() if len(parts) > 2 else "dwa"
                    if algo not in ALGOS:
                        print(c(C.RED, f"Thuật toán '{algo}' không hỗ trợ! Chạy mặc định dwa."))
                        algo = "dwa"
                    node.send_cmd(f"goto:{goal}:{algo}")
                    print(c(C.YELLOW, f"→ Sent goto: Đang đến {goal} bằng [{algo.upper()}]"))

            elif base_cmd == "patrol":
                algo = parts[1].lower() if len(parts) > 1 else "dwa"
                if algo not in ALGOS:
                    algo = "dwa"
                node.send_cmd(f"patrol:{algo}")
                print(c(C.YELLOW, f"→ Sent patrol: Bắt đầu tuần tra bằng [{algo.upper()}]"))

            elif base_cmd == "explore":
                algo = parts[1].lower() if len(parts) > 1 else "dwa"
                if algo not in ALGOS:
                    algo = "dwa"
                node.send_cmd(f"explore:{algo}")
                print(c(C.YELLOW, f"→ Sent explore: Bật Frontier Exploration bằng bộ lái [{algo.upper()}]"))

            elif base_cmd == "stop":
                node.send_cmd("stop")
                print(c(C.YELLOW, f"→ Sent stop"))
            
            elif base_cmd == "pause":
                node.send_cmd("pause")
                print(c(C.YELLOW, "→ Sent pause"))

            elif base_cmd == "resume":
                node.send_cmd("resume")
                print(c(C.YELLOW, "→ Sent resume"))

            elif base_cmd == "status":
                for line in format_status(node).splitlines():
                    print(c(C.CYAN, line))

            else:
                print(c(C.RED, f"Lệnh không hợp lệ: '{cmd}'  (gõ 'help' để xem danh sách)"))

        except KeyboardInterrupt:
            print("\nExit.")
            break
        except EOFError:
            print("\nEOF. Exit.")
            break

    stop_event.set()

# ====== MAIN ======
def main():
    rclpy.init()

    readline.parse_and_bind("tab: complete")
    readline.set_completer(completer)

    node = TB4CLI()
    stop_event = threading.Event()

    def spin_until_stopped():
        while not stop_event.is_set():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except rclpy.executors.ExternalShutdownException:
                pass

    spin_thread = threading.Thread(target=spin_until_stopped, daemon=True)
    spin_thread.start()

    try:
        cli_loop(node, stop_event)
    finally:
        stop_event.set()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
