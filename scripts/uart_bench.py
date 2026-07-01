"""
uart_bench.py — UART 接收性能基准测试

用法:
    python uart_bench.py <串口> [波特率]

示例:
    python uart_bench.py COM5         # Windows，默认 115200
    python uart_bench.py /dev/ttyUSB0 115200
"""

import serial
import serial.tools.list_ports
import sys
import time
import re
import itertools


# ── 测试参数 ──────────────────────────────────────────────────────────

BAUD = 921600          # 默认波特率
REPORT_TIMEOUT = 6     # 等待固件上报超时（秒）
IDLE_GUARD = 0.8       # 两次测试间的静默间隔

# ── 辅助函数 ──────────────────────────────────────────────────────────


def list_ports():
    """列出可用串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("! 未检测到串口设备")
        return
    print("可用串口：")
    for p in sorted(ports, key=lambda x: x.device):
        print(f"  {p.device}  —  {p.description}")


def open_serial(port: str, baud: int = BAUD) -> serial.Serial:
    ser = serial.Serial(port, baud, timeout=3)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def wait_for_boot(ser: serial.Serial, timeout: float = 5):
    """等待固件启动完成（检测 uart3 ready）"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if "uart3 ready" in line:
            return True
    return False


def drain(ser: serial.Serial, duration: float = 0.3):
    """清空串口缓冲，让固件进入空闲状态"""
    time.sleep(duration)
    ser.reset_input_buffer()


def send_pattern(ser: serial.Serial, size: int, count: int,
                 interval: float = 0):
    """发送 count 个 size 字节的数据包"""
    for i in range(count):
        # 每个包首字节为序列号（低 8 位），其余填充固定模式
        pkt = bytearray([i & 0xFF] * size)
        ser.write(bytes(pkt))
        if interval > 0:
            time.sleep(interval)


def send_stream(ser: serial.Serial, total: int, chunk: int = 1024):
    """连续发送 total 字节数据"""
    pattern = bytes(range(256)) * (chunk // 256 + 1)
    sent = 0
    while sent < total:
        n = min(chunk, total - sent)
        ser.write(pattern[:n])
        sent += n


def wait_report(ser: serial.Serial, timeout: float = REPORT_TIMEOUT
                ) -> dict | None:
    """等待 [BENCH] 报告并解析"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            line = ser.readline().decode(errors="ignore").strip()
        except serial.SerialException:
            break
        if "[BENCH]" in line:
            return parse_report(line)
    return None


RE_BENCH = re.compile(
    r"bytes=(?P<bytes>\d+)\s+"
    r"reads=(?P<reads>\d+)\s+"
    r"min=(?P<min>\d+)\s+"
    r"max=(?P<max>\d+)\s+"
    r"avg=(?P<avg>\d+)\s+"
    r"rd_cyc=(?P<rd_cyc>\d+)\s+"
    r"avg_cyc=(?P<avg_cyc>\d+)\s+"
    r"time=(?P<time_ms>-?\d+)ms\s+"
    r"bw=(?P<bw>\d+)B/s"
)


def parse_report(line: str) -> dict | None:
    m = RE_BENCH.search(line)
    if not m:
        return None
    d = {k: int(v) for k, v in m.groupdict().items()}
    d["bw_kbps"] = d["bw"] / 1024
    d["time_s"]  = d["time_ms"] / 1000
    return d


# ── 测试用例 ──────────────────────────────────────────────────────────


def run_case(ser: serial.Serial, name: str, fn):
    """执行单个测试用例"""
    print(f"\n--- {name} ---", end=" ", flush=True)
    drain(ser)
    fn(ser)
    report = wait_report(ser)
    if report:
        print("OK")
    else:
        print("NO REPORT")
    return (name, report)


def case_small_packets(ser):
    """16 字节小包 × 500"""
    send_pattern(ser, 16, 500)


def case_medium_packets(ser):
    """64 字节中包 × 500"""
    send_pattern(ser, 64, 500)


def case_large_packets(ser):
    """256 字节大包 × 500"""
    send_pattern(ser, 256, 500)


def case_max_packets(ser):
    """512 字节最大包 × 200"""
    send_pattern(ser, 512, 200)


def case_throughput(ser):
    """连续流 50KB"""
    send_stream(ser, 50 * 1024)


def case_heavy(ser):
    """重负载 200KB"""
    send_stream(ser, 200 * 1024)


def case_spaced_burst(ser):
    """256 字节 × 200，间隔 5ms"""
    send_pattern(ser, 256, 200, interval=0.005)


def case_mixed_sizes(ser):
    """混合包长 1~256 字节 × 500"""
    for i in range(500):
        size = (i % 256) + 1
        pkt = bytes([i & 0xFF] * size)
        ser.write(bytes(pkt))


def case_read_latency(ser):
    """逐字节发送 × 200，测 Read() 延迟"""
    for i in range(200):
        ser.write(bytes([i & 0xFF]))


# ── 主流程 ────────────────────────────────────────────────────────────


def main():
    if len(sys.argv) >= 2 and sys.argv[1] in ("-l", "--list"):
        list_ports()
        sys.exit(0)

    port = sys.argv[1] if len(sys.argv) >= 2 else "COM21"
    baud = int(sys.argv[2]) if len(sys.argv) >= 3 else BAUD

    print(f"串口: {port}  @  {baud} baud\n")
    ser = open_serial(port, baud)

    # 等待固件就绪
    print("等待固件启动...", end=" ", flush=True)
    if wait_for_boot(ser):
        print("OK")
    else:
        print("TIMEOUT — 继续尝试...")
    drain(ser, 1.0)

    # 执行所有测试
    cases = [
        ("16B ×500 小包",      case_small_packets),
        ("64B ×500 中包",      case_medium_packets),
        ("256B×500 大包",      case_large_packets),
        ("512B×200 最大包",    case_max_packets),
        ("50KB 连续流",        case_throughput),
        ("200KB 重负载",       case_heavy),
        ("256B×200 5ms间隔",   case_spaced_burst),
        ("1~256B×500 混合",    case_mixed_sizes),
        ("1B×200 逐字节延迟",  case_read_latency),
    ]

    results = []
    for name, fn in cases:
        r = run_case(ser, name, fn)
        results.append(r)

    ser.close()

    # ── 结果汇总 ──────────────────────────────────────────────────
    print("\n\n" + "=" * 90)
    print(f"{'测试项':30s} {'字节':>8s} {'Reads':>6s} {'min':>4s} "
          f"{'max':>4s} {'avg':>4s} {'读延迟':>8s} {'耗时':>8s} "
          f"{'吞吐':>10s}")
    print("-" * 90)

    for name, r in results:
        if r is None:
            print(f"{name:30s}  {'— 无报告':15s}")
            continue
        print(f"{name:30s} {r['bytes']:>8d} {r['reads']:>6d} "
              f"{r['min']:>4d} {r['max']:>4d} {r['avg']:>4d} "
              f"{r['avg_cyc']:>6d}cyc "
              f"{r['time_s']:>7.2f}s "
              f"{r['bw_kbps']:>8.1f}KB/s")

    print("=" * 90)
    print("\n* 读延迟 = avg_cyc: 单次 Read() 平均 CPU 周期数")
    print("* 吞吐   = bw:      平均每秒接收字节数 (KB/s)")
    print("* min/max/avg = 单次 Read() 返回的字节数统计")


if __name__ == "__main__":
    main()
