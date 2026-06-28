#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys
import readline

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
COMMANDS = ["goto", "patrol", "stop", "status", "help", "clear", "exit"]

# ====== AUTOCOMPLETE ======
def completer(text, state):
    buffer = readline.get_line_buffer()
    tokens = buffer.split()
    if len(tokens) <= 1:
        options = [cmd for cmd in COMMANDS if cmd.startswith(text)]
    else:
        import yaml
        with open('/ros2_ws/config/waypoints.yaml') as f:
            _data = yaml.safe_load(f)
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
    print(c(C.CYAN, "\nCác lệnh có sẵn:"))
    cmds = [
        ("goto <tên>",  "Di chuyển đến waypoint  (vd: goto diem_A)"),
        ("patrol",      "Bắt đầu tuần tra tự động"),
        ("stop",        "Dừng robot ngay lập tức"),
        ("status",      "Xem trạng thái hiện tại"),
        ("help",        "Hiện menu này"),
        ("clear",       "Xóa màn hình"),
        ("exit",        "Thoát CLI"),
    ]
    for cmd, desc in cmds:
        print(f"  {c(C.YELLOW, cmd.ljust(16))} {c(C.DIM, desc)}")
    print()

# ====== CLI LOOP ======
def cli_loop(node: TB4CLI, stop_event: threading.Event):
    print(c(C.GREEN, "=== TB4 CLI PRO ==="))
    print(c(C.DIM, "Gõ 'help' để xem danh sách lệnh, Tab để autocomplete\n"))

    while not stop_event.is_set():
        try:
            cmd = input("> ").strip()
            if not cmd:
                continue

            readline.add_history(cmd)

            if cmd == "exit":
                print("Bye.")
                break

            elif cmd == "help":
                print_help()

            elif cmd == "clear":
                print("\033[2J\033[H", end="")   # ANSI clear screen
                print(c(C.GREEN, "=== TB4 CLI PRO ===\n"))

            elif cmd.startswith("goto "):
                parts = cmd.split(" ", 1)
                if len(parts) < 2 or not parts[1].strip():
                    print(c(C.RED, "Dùng: goto <tên_waypoint>"))
                else:
                    goal = parts[1].strip()
                    node.send_cmd(f"goto:{goal}")
                    print(c(C.YELLOW, f"→ Sent goto: {goal}"))

            elif cmd == "patrol":
                node.send_cmd("patrol")
                print(c(C.YELLOW, "→ Sent patrol"))

            elif cmd == "stop":
                node.send_cmd("stop")
                print(c(C.YELLOW, "→ Sent stop"))
            
            elif cmd == "pause":
                node.send_cmd("pause")
                print(c(C.YELLOW, "→ Sent pause"))

            elif cmd == "resume":
                node.send_cmd("resume")
                print(c(C.YELLOW, "→ Sent resume"))

            elif cmd == "status":
                print(c(C.CYAN, f"Status: {node.get_status()}"))

            else:
                print(c(C.RED, f"Lệnh không hợp lệ: '{cmd}'  (gõ 'help' để xem danh sách)"))

        except KeyboardInterrupt:
            print("\nExit.")
            break
        except EOFError:
            # Ctrl+D
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

    # Spin thread: dừng khi stop_event được set
    def spin_until_stopped():
        while not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)

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