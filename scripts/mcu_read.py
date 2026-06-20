#!/usr/bin/env python3
"""
MCU Variable Reader
连 OpenOCD GDB 端口 → halt 读变量地址 → resume → tkinter 显示
依赖: Python 标准库 (无需 pip install)
用法:  python mcu_read.py
"""

import socket, struct, threading, time, subprocess, re, os, sys
import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass
from typing import Optional

# ───────────────────────────── 配置 ─────────────────────────────
ELF   = "build/zephyr/zephyr.elf"          # elf 路径
NM    = "riscv64-zephyr-elf-nm"            # nm 工具路径
GDB_HOST = "127.0.0.1"                      # OpenOCD GDB server 地址
GDB_PORT = 3333                             # OpenOCD GDB server 端口
# ───────────────────────────────────────────────────────────────

@dataclass
class Var:
    name: str
    addr: int
    size: int = 1


class Monitor:
    def __init__(self, elf, nm_tool):
        self.elf = elf
        self.nm = nm_tool
        self.vars: dict[str, Var] = {}
        self.sock: Optional[socket.socket] = None
        self.running = False
        self.lock = threading.Lock()

        # GUI
        self.root = tk.Tk()
        self.root.title("MCU Read")
        self.root.geometry("640x320")

        ctrl = ttk.Frame(self.root, padding=4)
        ctrl.pack(fill=tk.X)

        ttk.Label(ctrl, text="变量:").pack(side=tk.LEFT)
        self.entry = ttk.Entry(ctrl, width=24)
        self.entry.pack(side=tk.LEFT, padx=4)
        self.entry.bind("<Return>", lambda e: self.add_var())

        ttk.Button(ctrl, text="添加", command=self.add_var).pack(side=tk.LEFT, padx=2)
        ttk.Button(ctrl, text="搜索", command=self.search_var).pack(side=tk.LEFT, padx=2)
        ttk.Button(ctrl, text="删除选中", command=self.del_sel).pack(side=tk.LEFT, padx=8)

        ttk.Button(ctrl, text="连接", command=self.do_connect).pack(side=tk.RIGHT, padx=2)
        ttk.Button(ctrl, text="开始", command=self.do_start).pack(side=tk.RIGHT, padx=2)
        ttk.Button(ctrl, text="停止", command=self.do_stop).pack(side=tk.RIGHT, padx=2)

        self.status = ttk.Label(ctrl, text="未连接", foreground="red")
        self.status.pack(side=tk.RIGHT, padx=6)

        # 表格
        f = ttk.Frame(self.root, padding=4)
        f.pack(fill=tk.BOTH, expand=True)
        cols = ("name", "addr", "value", "hex", "type")
        self.tree = ttk.Treeview(f, columns=cols, show="headings", height=16)
        for c, w in [("name",200), ("addr",90), ("value",80), ("hex",80), ("type",60)]:
            self.tree.heading(c, text=c)
            self.tree.column(c, width=w)
        sb = ttk.Scrollbar(f, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.bind("<Delete>", lambda e: self.del_sel())

    # ── nm 查符号 ──
    def get_addr(self, name: str) -> Optional[int]:
        try:
            r = subprocess.run([self.nm, "-n", self.elf], capture_output=True, text=True, timeout=10)
            for line in r.stdout.splitlines():
                p = line.strip().split()
                if len(p) >= 3:
                    if p[2] == name:
                        return int(p[0], 16)
                if len(p) >= 3 and name in p[2]:
                    return int(p[0], 16)
        except: pass
        return None

    def search_syms(self, pat: str):
        try:
            r = subprocess.run([self.nm, "-n", self.elf], capture_output=True, text=True, timeout=10)
            return [(p[2], int(p[0],16)) for line in r.stdout.splitlines()
                    if (p:=line.strip().split()) and len(p)>=3 and pat.lower() in p[2].lower()]
        except: return []

    # ── GDB remote protocol ──
    def _cs(self, data: bytes) -> str:
        return f"{(sum(data)&0xff):02x}"

    def _pkt(self, data: bytes) -> Optional[bytes]:
        """发 GDB 包, 收响应."""
        if not self.sock: return None
        with self.lock:
            try:
                self.sock.sendall(b"$" + data + b"#" + self._cs(data).encode())
                resp = b""
                while True:
                    b = self.sock.recv(1)
                    if b == b'$': resp = b""
                    elif b == b'#': self.sock.recv(2); break
                    elif b: resp += b
                    else: break
                return resp
            except: return None

    def _halt(self) -> bool:
        if not self.sock: return False
        with self.lock:
            try:
                self.sock.sendall(b"\x03")
                time.sleep(0.05)
                resp = b""
                while True:
                    try:
                        b = self.sock.recv(1)
                        if b == b'$': resp = b""
                        elif b == b'#': self.sock.recv(2); break
                        elif b: resp += b
                        else: break
                    except: break
                return resp.startswith(b'T') or resp.startswith(b'S')
            except: return False

    def _resume(self):
        if not self.sock: return
        with self.lock:
            try: self.sock.sendall(b"$c#63")
            except: pass

    def read_mem(self, addr: int, size: int) -> Optional[bytes]:
        """halt → 读 → resume"""
        if not self._halt(): return None
        d = self._pkt(f"m{addr:x},{size:x}".encode())
        self._resume()
        if d and not d.startswith(b'E'):
            try: return bytes.fromhex(d.decode())
            except: return None
        return None

    def do_connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)
            self.sock.connect((GDB_HOST, GDB_PORT))
            self.sock.recv(4096)  # 扔掉欢迎包
            self.status.config(text="已连接", foreground="green")
        except Exception as e:
            self.status.config(text=f"连接失败: {e}", foreground="red")
            self.sock = None

    # ── GUI 操作 ──
    def add_var(self):
        name = self.entry.get().strip()
        if not name or name in self.vars: return
        addr = self.get_addr(name)
        if addr is None:
            self.status.config(text=f"找不到符号: {name}", foreground="red")
            return
        self.vars[name] = Var(name, addr)
        self.tree.insert("", tk.END, iid=name, values=(name, f"0x{addr:08x}", "-", "-", "uint8"))
        self.entry.delete(0, tk.END)

    def del_sel(self):
        for i in self.tree.selection():
            self.vars.pop(i, None)
            self.tree.delete(i)

    def search_var(self):
        pat = self.entry.get().strip()
        if not pat: return
        syms = self.search_syms(pat)
        if not syms: return
        w = tk.Toplevel(self.root)
        w.title("选择")
        w.geometry("420x300")
        lb = tk.Listbox(w, font=("Consolas",10))
        lb.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        for nm, a in syms: lb.insert(tk.END, f"0x{a:08x}  {nm}")
        def sel():
            s = lb.curselection()
            if s:
                self.entry.delete(0, tk.END)
                self.entry.insert(0, lb.get(s[0]).split("  ",1)[1])
            w.destroy()
        ttk.Button(w, text="选择", command=sel).pack(pady=4)

    def do_start(self):
        if not self.vars: return
        self.running = True
        self._poll()

    def do_stop(self):
        self.running = False

    def _poll(self):
        if not self.running: return
        for v in self.vars.values():
            d = self.read_mem(v.addr, v.size)
            if d:
                val = int.from_bytes(d, 'little')
                self.tree.item(v.name, values=(v.name, f"0x{v.addr:08x}", str(val), d.hex(), "uint8"))
            else:
                self.tree.item(v.name, values=(v.name, f"0x{v.addr:08x}", "ERR", "-", "uint8"))
        self.root.after(100, self._poll)  # 10 Hz

    def run(self):
        self.root.mainloop()
        self.running = False
        if self.sock: self.sock.close()


if __name__ == "__main__":
    # 支持命令行参数: python mcu_read.py <elf路径> [nm路径]
    elf = sys.argv[1] if len(sys.argv) > 1 else ELF
    nm  = sys.argv[2] if len(sys.argv) > 2 else NM
    if not os.path.exists(elf):
        print(f"ELF 文件不存在: {elf}")
        sys.exit(1)
    Monitor(elf, nm).run()
