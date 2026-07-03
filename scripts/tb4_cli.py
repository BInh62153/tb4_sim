#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys
import readline
import subprocess

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
        self.last_status = "unknown (task_manager chưa chạy?)"

    def cb_status(self, msg):
        self.last_status = msg.data

    def send_cmd(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)

    def get_status(self):
        return self.last_status

# ====== HELP ======
def print_help():
    print(c(C.CYAN, "\n=== HỆ THỐNG THỬ NGHIỆM THUẬT TOÁN TURTLEBOT4 ==="))
    cmds = [
        ("activate / start",  "Bật task_manager (lifecycle activate) — bắt buộc trước patrol/goto"),
        ("deactivate",        "Tắt task_manager (lifecycle deactivate)"),
        ("goto <đích> [algo]", "Di chuyển một lần tới waypoint (xong thì dừng, không tuần tra tiếp)"),
        ("patrol [algo]",     "Tuần tra tự động qua toàn bộ mission"),
        ("explore [algo]",    "Bật Frontier Exploration (Khám phá bản đồ) + thuật toán lái"),
        ("stop",              "Dừng robot ngay lập tức"),
        ("pause",             "Tạm dừng tác vụ hành trình"),
        ("resume",            "Tiếp tục hành trình đang hoãn"),
        ("status",            "Xem trạng thái hiện tại"),
        ("help",              "Hiện menu hướng dẫn này"),
        ("clear",             "Xóa màn hình dòng lệnh"),
        ("exit",              "Thoát giao diện điều khiển CLI"),
    ]
    print(c(C.DIM, "Các thuật toán (algo) hỗ trợ thử nghiệm: dwa | teb | pp | stanley\n"))
    for cmd, desc in cmds:
        print(f"  {c(C.YELLOW, cmd.ljust(22))} {c(C.DIM, desc)}")
    print()

# ====== CLI LOOP ======
def cli_loop(node: TB4CLI, stop_event: threading.Event):
    print(c(C.GREEN, "=== TB4 MULTI-MODULE CLI PRO ==="))
    print(c(C.DIM, "Luồng khuyến nghị: activate → goto/patrol/explore"))
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

            elif base_cmd == "deactivate":
                ok, msg = lifecycle_set("deactivate")
                color = C.GREEN if ok else C.RED
                print(c(color, f"→ lifecycle deactivate: {msg}"))

            elif base_cmd == "goto":
                if len(parts) < 2:
                    print(c(C.RED, "Dùng: goto <tên_waypoint> [dwa/teb/pp/stanley]"))
                else:
                    goal = parts[1]
                    algo = parts[2].lower() if len(parts) > 2 else "dwa"
                    if algo not in ['dwa', 'teb', 'pp', 'stanley']:
                        print(c(C.RED, f"Thuật toán '{algo}' không hỗ trợ! Chạy mặc định dwa."))
                        algo = "dwa"
                    # Gửi chuỗi định dạng cấu trúc dạng: goto:diem_A:teb
                    node.send_cmd(f"goto:{goal}:{algo}")
                    print(c(C.YELLOW, f"→ Sent goto: Đang đến {goal} bằng [{algo.upper()}]"))

            elif base_cmd == "patrol":
                algo = parts[1].lower() if len(parts) > 1 else "dwa"
                if algo not in ['dwa', 'teb', 'pp', 'stanley']:
                    algo = "dwa"
                node.send_cmd(f"patrol:{algo}")
                print(c(C.YELLOW, f"→ Sent patrol: Bắt đầu tuần tra bằng [{algo.upper()}]"))

            elif base_cmd == "explore":
                algo = parts[1].lower() if len(parts) > 1 else "dwa"
                if algo not in ['dwa', 'teb', 'pp', 'stanley']:
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
                print(c(C.CYAN, f"Status: {node.get_status()}"))

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