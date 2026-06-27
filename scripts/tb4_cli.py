#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys
import readline

# ====== COLOR ======
class C:
    END = '\033[0m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    CYAN = '\033[96m'

def c(color, msg):
    return f"{color}{msg}{C.END}"


# ====== COMMAND LIST ======
COMMANDS = ["goto", "patrol", "stop", "status", "exit"]

# ====== AUTOCOMPLETE ======
def completer(text, state):
    buffer = readline.get_line_buffer()
    tokens = buffer.split()

    # autocomplete command
    if len(tokens) <= 1:
        options = [cmd for cmd in COMMANDS if cmd.startswith(text)]
    else:
        # autocomplete goal (demo)
        goals = ["diem_A", "diem_B", "diem_C"]
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

        self.last_status = "unknown"

    def cb_status(self, msg):
        self.last_status = msg.data

    def send_cmd(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)

    def get_status(self):
        return self.last_status


# ====== CLI LOOP ======
def cli_loop(node: TB4CLI):
    print(c(C.GREEN, "=== TB4 CLI PRO ==="))

    while True:
        try:
            cmd = input("> ").strip()

            if not cmd:
                continue

            readline.add_history(cmd)

            if cmd == "exit":
                print("Bye.")
                rclpy.shutdown()
                sys.exit(0)

            elif cmd.startswith("goto "):
                goal = cmd.split(" ", 1)[1]
                node.send_cmd(f"goto:{goal}")
                print(c(C.YELLOW, f"Sent goto → {goal}"))

            elif cmd == "patrol":
                node.send_cmd("patrol")
                print(c(C.YELLOW, "Sent patrol"))

            elif cmd == "stop":
                node.send_cmd("stop")
                print(c(C.YELLOW, "Sent stop"))

            elif cmd == "status":
                print(c(C.CYAN, f"{node.get_status()}"))

            else:
                print(c(C.RED, "Unknown command"))

        except KeyboardInterrupt:
            print("\nExit.")
            rclpy.shutdown()
            sys.exit(0)


# ====== MAIN ======
def main():
    rclpy.init()

    # setup readline
    readline.parse_and_bind("tab: complete")
    readline.set_completer(completer)

    node = TB4CLI()

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )
    spin_thread.start()

    cli_loop(node)


if __name__ == "__main__":
    main()