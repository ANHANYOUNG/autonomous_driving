#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ------------------------------------------------------------
# 빠른 모드 전환을 위해 만든 파일
# (1) 허용 명령 목록
#     - 새 명령을 추가/삭제하려면 아래 집합만 수정하세요.
# ------------------------------------------------------------
VALID_COMMANDS = {
    # 상위 상태 전환
    "RUN",
    "KEY",
    "CAL",
    "E_STOP",
    "ALIGN",
    "STOP",
    "AUTO_START",
    "path_A",   # ← 여기에 새 명령을 추가하세요 (예: "path_C")
    "path_B",
}
# ------------------------------------------------------------


HELP_TEXT = (
    "Allowed commands:\n"
    "  " + ", ".join(sorted(list(VALID_COMMANDS))) + "\n\n"
    "Examples:\n"
    "  RUN\n"
    "  path_A\n"
    "  AUTO_START\n"
    "  E_STOP\n"
)


class StateCommandPublisher(Node):
    """상시 실행되며 /state_command 에 문자열 명령을 퍼블리시하는 노드."""

    def __init__(self, topic_name: str = "/state_command"):
        super().__init__("state_command_publisher")
        self.pub = self.create_publisher(String, topic_name, 10)
        self.get_logger().info(f"[CMD_SEND] State command publisher ready → {topic_name}")

    def publish_command(self, text: str, repeat: int = 1, rate_hz: float = 0.0):
        """한 명령을 여러 번(옵션) 발행. rate_hz>0면 해당 주기로 반복 발행."""
        if text not in VALID_COMMANDS:
            self.get_logger().warn(
                f"[CMD_SEND] Unknown command: '{text}'. Allowed: {sorted(list(VALID_COMMANDS))}"
            )
            return

        sleep_dt = (1.0 / rate_hz) if rate_hz and rate_hz > 0.0 else 0.0
        msg = String()
        msg.data = text

        repeat = max(1, int(repeat))
        for i in range(repeat):
            self.pub.publish(msg)
            self.get_logger().info(f"[CMD_SEND] published[{i+1}/{repeat}]: '{text}'")
            if sleep_dt > 0.0 and i + 1 < repeat:
                time.sleep(sleep_dt)

def interactive_shell(repeat: int, rate_hz: float):
    """대화형 모드: 프롬프트에서 명령 입력 → 즉시 발행."""
    rclpy.init()
    node = StateCommandPublisher()
    node.get_logger().info(
        "[CMD_SEND] Interactive mode. Type a command and press Enter.\n"
        "Type 'help' for a list, 'exit' to quit."
    )

    try:
        while rclpy.ok():
            # DDS 이벤트 처리(필요시). 블로킹 없이 0초로 호출.
            rclpy.spin_once(node, timeout_sec=0.0)

            sys.stdout.write("> ")
            sys.stdout.flush()
            line = sys.stdin.readline()
            if not line:  # EOF
                break

            raw = line.strip()
            if not raw:
                continue
            if raw.lower() in ("exit", "quit", "q"):
                break
            if raw.lower() in ("help", "h", "?"):
                print(HELP_TEXT, flush=True)
                continue

            node.publish_command(raw, repeat=repeat, rate_hz=rate_hz)

    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("[CMD_SEND] Bye.")
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description="Fast persistent publisher for /state_command.\n"
                    "Run without args for interactive shell."
    )
    parser.add_argument(
        "command",
        nargs="*",
        help="Command(s) to publish (e.g., RUN, path_A, AUTO_START, E_STOP).",
    )
    parser.add_argument(
        "--times", "-t", type=int, default=1,
        help="Publish each command multiple times (default: 1)."
    )
    parser.add_argument(
        "--rate", "-r", type=float, default=0.0,
        help="Frequency (Hz) when publishing multiple times."
    )
    parser.add_argument(
        "--list", action="store_true",
        help="Print allowed commands and exit."
    )
    args = parser.parse_args()

    if args.list:
        print(HELP_TEXT)
        return

    interactive_shell(repeat=args.times, rate_hz=args.rate)


if __name__ == "__main__":
    main()
