#!/usr/bin/env python3
"""
根轨迹可视化 — 增益 K 变化时闭环极点如何在 s 平面移动
用法:
    python root_locus.py                          # 默认二阶系统
    python root_locus.py --poles 0,-2,-5          # 指定开环极点
    python root_locus.py --num 1 --den 1,6,11,6   # 分子/分母系数
    python root_locus.py --tf "s+1/s^2+2s+2"      # 传递函数字符串
依赖: numpy, matplotlib (pip install numpy matplotlib)
"""

import numpy as np
import matplotlib.pyplot as plt
from argparse import ArgumentParser


def tf_str_to_coeffs(s: str):
    """解析 's+1/s^2+2s+2' → (num, den) 系数列表"""
    num_s, den_s = s.replace(" ", "").split("/")

    def parse_poly(t: str):
        t = t.replace("s^", "s**")
        # 替换形如 '3s' → '3*s'
        import re
        t = re.sub(r'(\d)s', r'\1*s', t)
        t = re.sub(r'(\d)s\*\*', r'\1*s**', t)
        # 独立 s → 1*s
        t = re.sub(r'(?<!\*)s', r'1*s', t)
        t = re.sub(r'\*\*s', '**1', t)  # no-op
        coeffs = []
        max_power = max(int(m.group(1)) for m in re.finditer(r'\*\*(\d+)', t)) if '**' in t else 1
        for n in range(max_power, -1, -1):
            term = f"s**{n}" if n > 1 else ("s" if n == 1 else "1")
            # 找系数
            pat = rf'([+-]?\d*\.?\d*)\*?{term.replace("*", r"\*")}'
            m = re.search(pat, t)
            if m:
                v = m.group(1)
                if v in ("+", "-", ""):
                    v = v + "1"
                coeffs.append(float(v))
            else:
                coeffs.append(0.0)
        return coeffs

    return parse_poly(num_s), parse_poly(den_s)


def rlocus(num, den, k_min=0, k_max=50, n_k=2000):
    """
    手动计算根轨迹：对每个 K 求解 1 + K*num/den = 0 → den + K*num = 0 的根
    返回: (k_vec, poles_list) — 每列对应一个极点随 K 的轨迹
    """
    num, den = np.array(num, dtype=float), np.array(den, dtype=float)
    n = max(len(den) - 1, len(num) - 1)
    # 补齐到相同长度
    num_pad = np.concatenate([np.zeros(n - len(num) + 1), num]) if len(num) <= n else num
    den_pad = np.concatenate([np.zeros(n - len(den) + 1), den]) if len(den) <= n else den

    k_vals = np.linspace(k_min, k_max, n_k)
    poles = np.zeros((n_k, n + 1), dtype=complex)

    for i, k in enumerate(k_vals):
        coeff = den_pad + k * num_pad
        # 去除前导零
        coeff = np.trim_zeros(coeff, 'f')
        if len(coeff) <= 1:
            poles[i, :] = np.nan
            continue
        r = np.roots(coeff)
        # 按实部排序保证轨迹连续性
        r = r[np.argsort(r.real)]
        # 补齐
        for j in range(min(len(r), n + 1)):
            poles[i, j] = r[j]

    return k_vals, poles.T  # (n_poles, n_k)


def plot_root_locus(ax, k_vals, pole_traj, poles_open, zeros_open, title):
    """绘制根轨迹图"""
    colors = plt.cm.tab10(np.linspace(0, 1, len(pole_traj)))

    # 画轨迹
    for i, traj in enumerate(pole_traj):
        ax.plot(traj.real, traj.imag, color=colors[i], linewidth=1.0, alpha=0.8)

    # 开环极点 X
    for p in poles_open:
        ax.plot(p.real, p.imag, 'rx', markersize=10, markeredgewidth=2, label='开环极点' if p == poles_open[0] else "")
    # 开环零点 O
    for z in zeros_open:
        ax.plot(z.real, z.imag, 'bo', markersize=8, markerfacecolor='none', markeredgewidth=2, label='开环零点' if z == zeros_open[0] else "")

    # 渐进线（仅 n>m 时有）
    n, m = len(poles_open), len(zeros_open)
    if n > m:
        sigma = (sum(poles_open.real) - sum(zeros_open.real)) / (n - m)
        ax.axvline(x=sigma, color='gray', linestyle='--', linewidth=0.7, alpha=0.5)

    ax.axhline(y=0, color='gray', linewidth=0.5, alpha=0.4)
    ax.axvline(x=0, color='gray', linewidth=0.5, alpha=0.4)
    ax.set_xlabel('实轴 Re(s)')
    ax.set_ylabel('虚轴 Im(s) (Hz)')
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend(loc='upper right', fontsize=8)


def plot_gain_vs_pole(ax, k_vals, pole_traj, title):
    """绘制增益 K → 闭环极点的分叉图"""
    for i, traj in enumerate(pole_traj):
        mask = ~np.isnan(traj.real)
        ax.plot(k_vals[mask], traj.real[mask], linewidth=1.0, label=f'极点 {i+1} 实部')
        ax.plot(k_vals[mask], traj.imag[mask], linewidth=1.0, linestyle='--', label=f'极点 {i+1} 虚部')

    ax.set_xlabel('增益 K')
    ax.set_ylabel('极点位置')
    ax.set_title(title)
    ax.axhline(y=0, color='gray', linewidth=0.5, alpha=0.4)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, loc='upper right')


def main():
    parser = ArgumentParser(description="根轨迹可视化")
    parser.add_argument("--poles", type=str, default="",
                        help="开环极点，逗号分隔 (如 0,-2,-5)")
    parser.add_argument("--zeros", type=str, default="",
                        help="开环零点，逗号分隔 (如 -1)")
    parser.add_argument("--num", type=str, default="",
                        help="分子多项式系数 (如 1 或 1,2)")
    parser.add_argument("--den", type=str, default="",
                        help="分母多项式系数 (如 1,6,11,6)")
    parser.add_argument("--tf", type=str, default="",
                        help="传递函数字符串 (如 's+1/s^2+2s+2')")
    parser.add_argument("--k-max", type=float, default=50,
                        help="K 最大值 (默认 50)")
    parser.add_argument("--n-k", type=int, default=2000,
                        help="K 采样点数 (默认 2000)")
    parser.add_argument("--no-gui", action="store_true",
                        help="不显示窗口，直接保存图片")
    parser.add_argument("-o", "--output", type=str, default="",
                        help="输出图片路径 (如 root_locus.png)")
    args = parser.parse_args()

    # ── 确定传递函数 ──
    if args.tf:
        num, den = tf_str_to_coeffs(args.tf)
    elif args.num and args.den:
        num = [float(x) for x in args.num.split(",")]
        den = [float(x) for x in args.den.split(",")]
    elif args.poles:
        pole_list = [complex(p) for p in args.poles.split(",")]
        zeros_list = [complex(z) for z in args.zeros.split(",")] if args.zeros else []
        den = np.poly([float(p.real) for p in pole_list])
        num = np.poly([float(z.real) for z in zeros_list]) if zeros_list else [1]
        # 虚部处理：目前简化，有共轭复数极点的直接用 --num/--den
    else:
        # 默认: G(s) = K / s(s+2)(s+5)
        num = [1]
        den = [1, 7, 10, 0]
        print("使用默认系统: G(s) = K / [s(s+2)(s+5)]")

    print(f"开环传递函数: G(s)H(s) = K * ({np.poly1d(num)}) / ({np.poly1d(den)})")

    # 开环零极点
    if len(num) > 1 or num[0] != 1:
        zeros_open = np.roots(num)
    else:
        zeros_open = np.array([])
    poles_open = np.roots(den)

    print(f"开环极点: {poles_open}")
    print(f"开环零点: {zeros_open}")

    # ── 计算根轨迹 ──
    k_vals, pole_traj = rlocus(num, den, k_max=args.k_max, n_k=args.n_k)

    # ── 绘图 ──
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(f"根轨迹分析 — K ∈ [0, {args.k_max}]", fontsize=13, fontweight='bold')

    plot_root_locus(ax1, k_vals, pole_traj, poles_open, zeros_open,
                    f"根轨迹 (s 平面)\nG(s)H(s) = K · ({np.poly1d(num)}) / ({np.poly1d(den)})")

    plot_gain_vs_pole(ax2, k_vals, pole_traj,
                      f"增益 K → 闭环极点\n(实线=实部, 虚线=虚部)")

    plt.tight_layout()

    if args.output:
        plt.savefig(args.output, dpi=150, bbox_inches='tight')
        print(f"已保存: {args.output}")

    if not args.no_gui:
        plt.show()


if __name__ == "__main__":
    main()
