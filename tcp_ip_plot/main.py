#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import threading
import queue
import time
from collections import deque
import sys
import signal
from datetime import datetime

import matplotlib.pyplot as plt
import matplotlib.animation as animation

SERVER_IP = "192.168.1.119"
SERVER_PORT = 5010

# Cấu hình
WINDOW_SECONDS = 60
RECONNECT_DELAY = 2.0
EXPECTED_PREFIX = "$"
CSV_LOG = None  # hoặc "data_log.csv" để ghi log
SEND_SUFFIX = ""  # đổi thành "\n" nếu server yêu cầu newline khi nhận lệnh

# Truyền thông giữa các thread
data_queue = queue.Queue()        # (ts, [v1..v6]) cho thread plot
send_queue = queue.Queue()        # chuỗi cần gửi tới server từ thread nhập bàn phím
stop_event = threading.Event()

# init_msg mặc định; có thể thay đổi khi người dùng nhập
init_msg_lock = threading.Lock()
init_msg = "111202020"


def log_csv_init(path):
    if path is None:
        return None
    try:
        f = open(path, "w", encoding="utf-8")
        f.write("timestamp,v1,v2,v3,v4,v5,v6\n")
        f.flush()
        return f
    except Exception as e:
        print(f"[WARN] Không thể mở file CSV '{path}': {e}", file=sys.stderr)
        return None


def log_csv_row(f, ts, values):
    if f is None:
        return
    try:
        f.write(f"{ts:.3f}," + ",".join(map(str, values)) + "\n")
        f.flush()
    except Exception as e:
        print(f"[WARN] Ghi CSV lỗi: {e}", file=sys.stderr)


def socket_reader(ip, port, q_data, q_send, stop_evt):
    """
    Kết nối TCP, vừa nhận dữ liệu dòng theo '\n', vừa xử lý hàng đợi gửi lệnh.
    Không tự động gửi init_msg; chỉ gửi khi người dùng nhấn Enter ở console.
    """
    csv_fp = log_csv_init(CSV_LOG)
    while not stop_evt.is_set():
        sock = None
        try:
            print(f"[INFO] Đang kết nối tới {ip}:{port} ...")
            sock = socket.create_connection((ip, port), timeout=10)
            sock.settimeout(0.1)  # vòng lặp ngắn để vừa nhận vừa kiểm tra hàng đợi gửi
            print("[INFO] Đã kết nối. Bạn có thể nhấn Enter để gửi lệnh (mặc định '111202020').")

            recv_buf = ""
            while not stop_evt.is_set():
                # 1) Gửi mọi lệnh đang chờ
                try:
                    while True:
                        msg = q_send.get_nowait()
                        if not isinstance(msg, str):
                            msg = str(msg)
                        to_send = (msg + SEND_SUFFIX).encode("utf-8", errors="ignore")
                        sock.sendall(to_send)
                        print(f"[INFO] Đã gửi: {msg!r}")
                except queue.Empty:
                    pass
                except (OSError, ConnectionError) as e:
                    raise ConnectionError(f"Gửi lỗi: {e}")

                # 2) Nhận dữ liệu (không chặn lâu)
                try:
                    chunk = sock.recv(4096)
                    if not chunk:
                        raise ConnectionError("Mất kết nối (EOF).")
                    recv_buf += chunk.decode("utf-8", errors="ignore")
                except socket.timeout:
                    pass

                # 3) Xử lý các dòng đầy đủ
                if "\n" in recv_buf:
                    lines = recv_buf.split("\n")
                    recv_buf = lines[-1]  # giữ lại phần dở
                    for line in lines[:-1]:
                        line = line.strip()
                        if not line.startswith(EXPECTED_PREFIX):
                            continue
                        payload = line[len(EXPECTED_PREFIX):]
                        parts = [p.strip() for p in payload.split(",") if p.strip() != ""]
                        if len(parts) < 6:
                            continue
                        try:
                            values = [float(p) for p in parts[:6]]
                        except ValueError:
                            continue
                        ts = time.time()
                        q_data.put((ts, values))
                        log_csv_row(csv_fp, ts, values)

        except (socket.timeout, ConnectionError, OSError) as e:
            if stop_evt.is_set():
                break
            print(f"[WARN] Lỗi kết nối/nhận dữ liệu: {e}. Sẽ thử lại sau {RECONNECT_DELAY}s.")
            time.sleep(RECONNECT_DELAY)
        finally:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass

    if csv_fp:
        try:
            csv_fp.close()
        except Exception:
            pass
    print("[INFO] Socket reader dừng.")


def input_sender(q_send, stop_evt):
    """
    Đọc bàn phím: mỗi lần người dùng nhấn Enter sẽ gửi lệnh.
    - Dòng trống: gửi init_msg hiện tại.
    - Dòng có nội dung: cập nhật init_msg = dòng đó, và gửi luôn.
    """
    global init_msg
    print("[CTRL] Gõ lệnh rồi Enter để gửi. Enter trống = gửi lệnh hiện tại.")
    print(f"[CTRL] Lệnh hiện tại (init_msg) mặc định: {init_msg!r}")
    try:
        while not stop_evt.is_set():
            try:
                line = input()
            except EOFError:
                break
            text = line.strip()
            with init_msg_lock:
                if text:
                    init_msg = text
                msg_to_send = init_msg
            q_send.put(msg_to_send)
    except KeyboardInterrupt:
        pass
    print("[CTRL] Bộ nhập lệnh dừng.")


def graceful_exit(*_):
    stop_event.set()
    plt.close('all')


def main():
    signal.signal(signal.SIGINT, graceful_exit)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, graceful_exit)

    th_net = threading.Thread(target=socket_reader, args=(SERVER_IP, SERVER_PORT, data_queue, send_queue, stop_event), daemon=True)
    th_inp = threading.Thread(target=input_sender, args=(send_queue, stop_event), daemon=True)
    th_net.start()
    th_inp.start()

    # Bộ đệm thời gian & 6 kênh
    times = deque(maxlen=10000)
    ch = [deque(maxlen=10000) for _ in range(6)]

    # Figure: 3 subplot, mỗi subplot 2 kênh
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.subplots_adjust(hspace=0.25)

    lines = []
    pairs = [(0, 1), (2, 3), (4, 5)]
    titles = ["Kênh 1 & 2", "Kênh 3 & 4", "Kênh 5 & 6"]

    for ax, (i, j), ttl in zip(axes, pairs, titles):
        ln1, = ax.plot([], [], label=f"Kênh {i+1}")
        ln2, = ax.plot([], [], label=f"Kênh {j+1}")
        ax.set_ylabel("Giá trị")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.legend(loc="upper left")
        ax.set_title(ttl)
        lines.extend([ln1, ln2])

    axes[-1].set_xlabel("Thời gian (giây, 0 là hiện tại)")

    def update_plot(_):
        drained = 0
        try:
            while True:
                ts, values = data_queue.get_nowait()
                times.append(ts)
                for i in range(6):
                    ch[i].append(values[i])
                drained += 1
        except queue.Empty:
            pass

        if not times:
            return tuple(lines)

        now = time.time()
        tmin = now - WINDOW_SECONDS

        # tìm idx bắt đầu cửa sổ
        idx_start = 0
        for k, t in enumerate(times):
            if t >= tmin:
                idx_start = k
                break
        t_list = list(times)[idx_start:]
        xs = [t - now for t in t_list]  # âm -> 0

        if not xs:
            return tuple(lines)

        for ax, (i, j), li, lj in zip(axes, pairs, lines[0::2], lines[1::2]):
            yi = list(ch[i])[idx_start:]
            yj = list(ch[j])[idx_start:]

            li.set_data(xs, yi)
            lj.set_data(xs, yj)
            ax.set_xlim(min(xs), 0.0)

            local_all = yi + yj
            ymin, ymax = min(local_all), max(local_all)
            if ymin == ymax:
                ymin -= 1.0
                ymax += 1.0
            pad = 0.05 * (ymax - ymin)
            ax.set_ylim(ymin - pad, ymax + pad)

        fig.suptitle(
            f"Realtime TCP Plot ($v1..v6)  |  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}  |  {drained} mẫu mới",
            y=0.98, fontsize=12
        )
        return tuple(lines)

    ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=True)
    print("[INFO] Đang hiển thị đồ thị realtime (3 subplot). Nhấn Ctrl+C để thoát.")
    plt.show()

    stop_event.set()
    th_net.join(timeout=2.0)
    th_inp.join(timeout=2.0)
    print("[INFO] Thoát.")


if __name__ == "__main__":
    main()
