#!/usr/bin/env python3
# bt_receiver_robot_driver.py — Bluetooth RFCOMM server on the Pi.
# Receives JSON (newline-delimited) with "steps": [{"direction","duration_ms","speed_percent"}...]
# Then calls your movement layer. Requires python3-bluez.
#
# Run:
#   sudo python3 bt_receiver_robot_driver.py
#
import json
import os
import socket
import sys
import threading
import time

# Prefer your existing movement module(s)
MOVE = STOP = ESTOP = None
try:
    from robot_movement import move as RM_move, stop as RM_stop, emergency_stop as RM_estop
    MOVE, STOP, ESTOP = RM_move, RM_stop, RM_estop
except Exception:
    try:
        from enhanced_robot_movement import move as RM_move, stop as RM_stop, emergency_stop as RM_estop
        MOVE, STOP, ESTOP = RM_move, RM_stop, RM_estop
    except Exception as e:
        print("Could not import robot movement module. Make sure robot_movement.py is on PYTHONPATH.")
        raise

# PyBluez
try:
    from bluetooth import (
        BluetoothSocket, RFCOMM, PORT_ANY,
        SERIAL_PORT_CLASS, SERIAL_PORT_PROFILE,
        advertise_service
    )
except Exception as e:
    print("PyBluez not available. Install with: sudo apt-get install python3-bluez")
    raise

UUID_SPP = "00001101-0000-1000-8000-00805F9B34FB"
DEFAULT_TEST_SPEED = int(os.environ.get("ROBOT_TEST_SPEED", "18"))  # % PWM for testing

def handle_client(sock):
    sock_file = sock.makefile("rwb", buffering=0)
    buf = b""
    try:
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="ignore").strip()
                if not text:
                    continue
                try:
                    msg = json.loads(text)
                except json.JSONDecodeError:
                    continue
                process_message(msg, sock)
    except (OSError, socket.error):
        pass
    finally:
        try:
            sock.close()
        except Exception:
            pass
        print("Client disconnected.")


def process_message(msg, sock):
    mtype = msg.get("type")
    if mtype == "emergency_stop":
        try:
            ESTOP()
        except Exception:
            STOP()
        _reply(sock, {"ok": True, "action": "emergency_stop"})
        return

    if mtype == "plan_steps":
        steps = msg.get("steps", [])
        slow_pct = int(msg.get("meta", {}).get("speed_percent", DEFAULT_TEST_SPEED))
        executed = 0
        print(f"Received plan with {len(steps)} steps. Using default speed%={slow_pct}.")
        for step in steps:
            direction = (step.get("direction") or "").lower()
            duration_ms = int(step.get("duration_ms", 0))
            speed_pct = int(step.get("speed_percent", slow_pct))

            # Normalize a few aliases
            alias = {
                "left": "turnleft", "right": "turnright",
                "forward": "forward", "back": "backward",
                "backward": "backward", "stop": "stop",
                "turn_left": "turnleft", "turn_right": "turnright",
            }
            direction = alias.get(direction, direction)

            if direction not in ("forward", "backward", "turnleft", "turnright", "moveleft", "moveright", "stop"):
                print(f"Skipping unknown direction: {direction}")
                continue

            try:
                ok = True
                if direction == "stop":
                    ok = STOP()
                else:
                    ok = MOVE(direction, speed=speed_pct, duration_ms=duration_ms)
                executed += 1 if ok else 0
            except Exception as e:
                print(f"Movement error: {e}")
                try:
                    STOP()
                except Exception:
                    pass
                break

        _reply(sock, {"ok": True, "executed": executed})
        return

    _reply(sock, {"ok": False, "error": "unknown_message_type"})


def _reply(sock, obj):
    try:
        sock.sendall((json.dumps(obj) + "\n").encode("utf-8"))
    except Exception:
        pass


def main():
    server_sock = BluetoothSocket(RFCOMM)
    server_sock.bind(("", PORT_ANY))
    server_sock.listen(1)
    port = server_sock.getsockname()[1]

    advertise_service(
        server_sock,
        "RobotNavSPP",
        service_id=UUID_SPP,
        service_classes=[UUID_SPP, SERIAL_PORT_CLASS],
        profiles=[SERIAL_PORT_PROFILE],
    )

    print(f"Bluetooth RFCOMM server listening on channel {port} (SPP). Waiting for connection...")
    try:
        while True:
            client_sock, client_info = server_sock.accept()
            print(f"Client connected: {client_info}")
            t = threading.Thread(target=handle_client, args=(client_sock,), daemon=True)
            t.start()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            server_sock.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
