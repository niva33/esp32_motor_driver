#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import threading
import queue
import time
from collections import deque
import sys
import signal

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime

SERVER_IP = "192.168.1.102"
SERVER_PORT = 5010

# Cấu hình hiển thị
WINDOW_SECONDS = 5
RECONNECT_DELAY = 2.0
EXPECTED_PREFIX = "$"
CSV_LOG = None  # ví dụ: "data_log.csv" nếu muốn ghi file CSV

data_queue = queue.Queue()
stop_event = threading.Event()


def log_csv_init(path):
    if path is None:
        return None
    try:
        f = open(path, "w", encoding="utf-8")
        f.write("timestamp,value1,value2,value3\n")
        f.flush()
        return f
    except Exception as e:
        print(f"[WARN] Không thể mở file CSV '{path}': {e}", file=sys.stderr)
        return None


def log_csv_row(f, ts, v1, v2, v3):
    if f is None:
        return
    try:
        f.write(f"{ts:.3f},{v1},{v2},{v3}\n")
        f.flush()
    except Exception as e:
        print(f"[WARN] Ghi CSV lỗi: {e}", file=sys.stderr)


def socket_reader(ip, port, q, stop_evt):
    """
    Thread kết nối TCP, gửi chuỗi '111202020' và nhận dữ liệu.
    """
    csv_fp = log_csv_init(CSV_LOG)
    while not stop_evt.is_set():
        sock = None
        try:
            print(f"[INFO] Đang kết nối tới {ip}:{port} ...")
            sock = socket.create_connection((ip, port), timeout=10)
            sock.settimeout(5.0)
            print("[INFO] Đã kết nối.")

            # Gửi chuỗi 111202020 ngay khi kết nối thành công
            init_msg = "011202020"
            try:
                sock.sendall(init_msg.encode("utf-8"))
                print(f"[INFO] Đã gửi chuỗi khởi tạo: {init_msg}")
            except Exception as e:
                print(f"[WARN] Không gửi được chuỗi khởi tạo: {e}")

            fobj = sock.makefile("r", encoding="utf-8", newline="\n")
            while not stop_evt.is_set():
                line = fobj.readline()
                if not line:
                    raise ConnectionError("Mất kết nối (EOF).")
                line = line.strip()
                if not line.startswith(EXPECTED_PREFIX):
                    continue

                payload = line[len(EXPECTED_PREFIX):]
                parts = [p.strip() for p in payload.split(",") if p.strip() != ""]
                if len(parts) < 3:
                    continue
                try:
                    v1 = float(parts[0])
                    v2 = float(parts[1])
                    v3 = float(parts[2])
                except ValueError:
                    continue

                ts = time.time()
                q.put((ts, v1, v2, v3))
                log_csv_row(csv_fp, ts, v1, v2, v3)

        except (socket.timeout, ConnectionError, OSError) as e:
            if stop_evt.is_set():
                break
            print(f"[WARN] Lỗi kết nối/nhận dữ liệu: {e}. Sẽ thử lại sau {RECONNECT_DELAY}s.")
            time.sleep(RECONNECT_DELAY)
        finally:
            try:
                if sock:
                    sock.close()
            except Exception:
                pass

    if csv_fp:
        try:
            csv_fp.close()
        except Exception:
            pass
    print("[INFO] Socket reader dừng.")


def graceful_exit(*_):
    stop_event.set()
    plt.close('all')


def main():
    signal.signal(signal.SIGINT, graceful_exit)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, graceful_exit)

    th = threading.Thread(target=socket_reader, args=(SERVER_IP, SERVER_PORT, data_queue, stop_event), daemon=True)
    th.start()

    times = deque(maxlen=10000)
    ch1, ch2, ch3 = deque(maxlen=10000), deque(maxlen=10000), deque(maxlen=10000)

    plt.figure(figsize=(10, 5))
    ax = plt.gca()
    line1, = ax.plot([], [], label="Kênh 1 (v1)")
    line2, = ax.plot([], [], label="Kênh 2 (v2)")
    line3, = ax.plot([], [], label="Kênh 3 (v3)")
    ax.set_title("Realtime Plot từ TCP $v1,v2,v3")
    ax.set_xlabel("Thời gian (giây)")
    ax.set_ylabel("Giá trị")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="upper left")

    def update_plot(_):
        drained = 0
        try:
            while True:
                ts, v1, v2, v3 = data_queue.get_nowait()
                times.append(ts)
                ch1.append(v1)
                ch2.append(v2)
                ch3.append(v3)
                drained += 1
        except queue.Empty:
            pass

        if len(times) == 0:
            return line1, line2, line3

        now = time.time()
        tmin = now - WINDOW_SECONDS
        xs, y1, y2, y3 = [], [], [], []
        for i, t in enumerate(times):
            if t >= tmin:
                xs.append(t - now)
                y1.append(ch1[i])
                y2.append(ch2[i])
                y3.append(ch3[i])

        if not xs:
            return line1, line2, line3

        line1.set_data(xs, y1)
        line2.set_data(xs, y2)
        line3.set_data(xs, y3)
        ax.set_xlim(min(xs), 0.0)
        ymin, ymax = min(min(y1), min(y2), min(y3)), max(max(y1), max(y2), max(y3))
        if ymin == ymax:
            ymin -= 1
            ymax += 1
        pad = 0.05 * (ymax - ymin)
        ax.set_ylim(ymin - pad, ymax + pad)
        ax.set_title(f"Realtime Plot TCP | {datetime.now().strftime('%H:%M:%S')} | {drained} mẫu mới")
        return line1, line2, line3

    ani = animation.FuncAnimation(plt.gcf(), update_plot, interval=100, blit=True)
    print("[INFO] Đang hiển thị đồ thị realtime. Nhấn Ctrl+C để thoát.")
    plt.show()

    stop_event.set()
    th.join(timeout=2.0)
    print("[INFO] Thoát.")


if __name__ == "__main__":
    main()
