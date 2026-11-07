# -*- coding: utf-8 -*-
"""
力控阶跃响应绘图脚本
====================

用途
    把串口原始日志画成给导师看的一张图。

输入
    fc_rigid_*.txt  —— 直接把串口输出另存为 txt，不用改格式。
    脚本只认两种行：
        [ADM] 锚点 pos_ref = xxxxxx counts   （分组起点）
        [FC]  t_ms cmd fdb err travel F_cmd F_meas e_F ok bad

输出
    <同名>.png   2x2 组图
    控制台打印每组的稳态统计，方便核对

用法
    python plot_fc.py fc_rigid_20261107.txt

改数据只需改下面两个地方
    STEADY  稳态取数区间（ms）。要避开扰动段，见文件末尾说明。
    NAMES   每条曲线的图例名称

作者注
    travel = cmd - pos_ref，是积分器把指令推出了多远。
    "真顶住"的判据不是 err（稳态下 err≈0 是正常的，速度环积分在维持转矩），
    而是 travel 在稳态段是否停止增长。
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------- 中文字体 ----------------
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DengXian"]
matplotlib.rcParams["axes.unicode_minus"] = False

# ---------------- 可改配置 ----------------

# 稳态取数区间 [ms]，按曲线顺序。
# 组 2 在 18.1 s 处有一次外力扰动（F_meas 从 0.49 跳到 0.75），统计时避开。
STEADY = [
    (9000, 21600),    # 组 1：1.0 N·m
    (13000, 17900),   # 组 2：0.5 N·m（扰动前）
    (7600, 15700),    # 组 3：1.0 N·m
]

NAMES = [
    "1.0 N·m（第 1 次）",
    "0.5 N·m",
    "1.0 N·m（第 2 次）",
]

COLORS = ["#1f6fb4", "#e07b39", "#2e8b57"]

COUNTS_PER_DEG = 524288.0 / 360.0   # 1456.36

# 统计时剔除的时间窗 [ms]，按曲线顺序。用于外部扰动（手碰、线缆拉动）造成的瞬时尖峰。
# 剔除只影响统计，不影响曲线绘制——图上仍画出来，并加标注说明闭环自己恢复了。
EXCLUDE = [
    [(14050, 14200)],   # 组 1：14.1 s 处 F_meas 瞬跌到 0.910
    [],                 # 组 2：区间已截到扰动前，无需剔除
    [],
]

# 扰动标注（组号, 时刻 ms, 文字）。没有就留空列表。
EVENTS = [
    (0, 14114, "短时扰动"),
    (1, 18112, "外力扰动\n闭环 0.4 s 恢复"),
]


# ---------------- 解析 ----------------

def parse(path):
    """返回 [{'name':..., 'pos_ref':..., 't':[], 'cmd':[], 'fdb':[], 'err':[],
              'travel':[], 'fcmd':[], 'fmeas':[]}, ...]"""
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.read().splitlines()

    groups = []
    cur = None

    for line in lines:
        s = line.strip()

        if "力控模式启动" in s:
            if cur is not None and len(cur["t"]) > 0:
                groups.append(cur)
            cur = {"pos_ref": None, "t": [], "cmd": [], "fdb": [], "err": [],
                   "travel": [], "fcmd": [], "fmeas": []}
            continue

        if "pos_ref" in s and "counts" in s and cur is not None:
            try:
                cur["pos_ref"] = int(s.split("pos_ref")[1].split("=")[1].split()[0])
            except Exception:
                pass
            continue

        if not s.startswith("[FC]"):
            continue
        if cur is None:
            continue

        p = s.split()
        if len(p) < 9:
            continue
        try:
            t = int(p[1])
            cmd = int(p[2])
            fdb = int(p[3])
            err = int(p[4])
            travel = int(p[5])
            fcmd = float(p[6])
            fmeas = float(p[7])
        except ValueError:
            continue

        cur["t"].append(t)
        cur["cmd"].append(cmd)
        cur["fdb"].append(fdb)
        cur["err"].append(err)
        cur["travel"].append(travel)
        cur["fcmd"].append(fcmd)
        cur["fmeas"].append(fmeas)

    if cur is not None and len(cur["t"]) > 0:
        groups.append(cur)

    for g in groups:
        for k in ("t", "cmd", "fdb", "err", "travel"):
            g[k] = np.asarray(g[k], dtype=float)
        for k in ("fcmd", "fmeas"):
            g[k] = np.asarray(g[k], dtype=float)

    return groups


def steady_stats(g, t0, t1, exclude=None):
    m = (g["t"] >= t0) & (g["t"] <= t1)
    if exclude:
        for e0, e1 in exclude:
            m &= ~((g["t"] >= e0) & (g["t"] <= e1))
    f = g["fmeas"][m]
    tr = g["travel"][m]
    tt = g["t"][m]
    target = float(np.max(g["fcmd"]))

    mean = float(np.mean(f))
    std = float(np.std(f, ddof=1))
    pk = float(np.max(f) - np.min(f))
    err_pct = (mean - target) / target * 100.0
    tr_drift = float(tr[-1] - tr[0])
    tr_rate = tr_drift / max((tt[-1] - tt[0]) / 1000.0, 1e-9)

    return {
        "target": target, "mean": mean, "std": std, "pk": pk,
        "err_pct": err_pct, "n": int(np.sum(m)),
        "tr_drift": tr_drift, "tr_rate": tr_rate,
        "t_start": float(tt[0]) if len(tt) else 0.0,
        "t_span": (tt[-1] - tt[0]) / 1000.0,
    }


# ---------------- 绘图 ----------------

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "fc_rigid_20261107.txt"
    if not os.path.isabs(path):
        cand = os.path.join(os.path.dirname(os.path.abspath(__file__)), path)
        if os.path.exists(cand):
            path = cand

    groups = parse(path)
    if not groups:
        print("没解析到 [FC] 数据，检查文件格式")
        return

    print("解析到 %d 组，行数 %s" % (len(groups), [len(g["t"]) for g in groups]))

    stats = []
    for i, g in enumerate(groups):
        if i < len(STEADY):
            t0, t1 = STEADY[i]
        else:
            t0, t1 = g["t"][int(len(g["t"]) * 0.6)], g["t"][-1]
        exc = EXCLUDE[i] if i < len(EXCLUDE) else None
        stats.append(steady_stats(g, t0, t1, exc))

    # ---- 版式 ----
    fig, axes = plt.subplots(2, 2, figsize=(13.4, 9.2))
    fig.suptitle("力控阶跃响应（位置型导纳 · 纯积分 · 顶住刚性挡块）",
                 fontsize=15, fontweight="bold", y=0.975)

    grid_kw = dict(color="#d9d9d9", linewidth=0.6, linestyle="--", alpha=0.9)

    def style(ax):
        ax.grid(True, **grid_kw)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.tick_params(labelsize=9.5, colors="#333333")

    # ================= 左上：全景 =================
    ax = axes[0][0]
    for i, (g, st) in enumerate(zip(groups, stats)):
        ax.plot(g["t"] / 1000.0, g["fmeas"], color=COLORS[i % 3],
                lw=1.5, label=NAMES[i] if i < len(NAMES) else "组%d" % (i + 1))
        ax.axhline(st["target"], color=COLORS[i % 3], ls=":", lw=1.0, alpha=0.65)
    for gi, tt, txt in EVENTS:
        if gi < len(groups):
            ax.annotate(txt, xy=(tt / 1000.0, groups[gi]["fmeas"][np.argmin(np.abs(groups[gi]["t"] - tt))]),
                        xytext=(tt / 1000.0 + 1.4, 0.80), fontsize=8.2, color="#b03030",
                        arrowprops=dict(arrowstyle="->", color="#b03060",
                                        lw=0.9, connectionstyle="arc3,rad=0.2"))
    ax.set_xlabel("时间 [s]", fontsize=10.5)
    ax.set_ylabel("关节输出力矩 $F_{meas}$ [N·m]", fontsize=10.5)
    ax.set_title("(a) 力跟踪全过程：指令阶跃 → 建立 → 稳态", fontsize=11.5, loc="left")
    ax.legend(fontsize=9, loc="upper left", framealpha=0.9, edgecolor="#cccccc")
    style(ax)

    # ================= 右上：精度误差棒 =================
    ax = axes[0][1]
    ax.axhspan(0.98, 1.02, color="#8fbf8f", alpha=0.20, zorder=0,
               label="±2% 带")
    ax.axhline(1.0, color="#555555", lw=1.0, ls="--", zorder=1, label="目标值")

    xs = np.arange(len(stats))
    for i, st in enumerate(stats):
        yv = st["mean"] / st["target"]
        ev = 2.0 * st["std"] / st["target"]
        ax.errorbar(xs[i], yv, yerr=ev, fmt="o", color=COLORS[i % 3],
                    ms=8.5, capsize=7, capthick=1.6, elinewidth=1.9, zorder=3)
        ax.annotate("误差 %+.2f%%\n峰峰 %.3f" % (st["err_pct"], st["pk"]),
                    xy=(xs[i], yv), xytext=(0, -58), textcoords="offset points",
                    ha="center", fontsize=8.6, color="#333333", linespacing=1.4)

    ax.set_xticks(xs)
    ax.set_xticklabels([n.replace("（", "\n（") for n in NAMES[:len(stats)]], fontsize=9.2)
    ax.set_xlim(-0.55, len(stats) - 0.45)
    ax.set_ylim(0.915, 1.065)
    ax.set_ylabel("$F_{meas}\\,/\\,F_{cmd}$   (误差棒 = ±2σ)", fontsize=10.5)
    ax.set_title("(b) 稳态精度：三档实测值与目标值的偏差", fontsize=11.5, loc="left")
    ax.legend(fontsize=8.8, loc="upper left", framealpha=0.92, edgecolor="#cccccc")
    style(ax)

    # ================= 左下：travel =================
    ax = axes[1][0]
    for i, (g, st) in enumerate(zip(groups, stats)):
        ax.plot(g["t"] / 1000.0, g["travel"] / COUNTS_PER_DEG,
                color=COLORS[i % 3], lw=1.5, label=NAMES[i] if i < len(NAMES) else "组%d" % (i + 1))
        ax.axvspan(STEADY[i][0] / 1000.0, STEADY[i][1] / 1000.0,
                   color=COLORS[i % 3], alpha=0.05, lw=0)
    ax.set_xlabel("时间 [s]", fontsize=10.5)
    ax.set_ylabel("指令位移 travel [deg]", fontsize=10.5)
    ax.set_title("(c) 积分器输出位移：稳态段停止增长 = 真顶住挡块",
                 fontsize=11.5, loc="left")
    ax.legend(fontsize=9, loc="upper left", framealpha=0.9, edgecolor="#cccccc")
    style(ax)

    # ================= 右下：统计表 =================
    ax = axes[1][1]
    ax.axis("off")
    ax.set_title("(d) 稳态统计（取数区间已避开扰动段）",
                 fontsize=11.5, loc="left")

    col_labels = ["工况", "目标\n[N·m]", "实测均值\n[N·m]", "相对误差",
                  "标准差\n[N·m]", "峰峰值\n[N·m]", "travel 漂移"]
    rows = []
    for i, st in enumerate(stats):
        rows.append([
            NAMES[i] if i < len(NAMES) else "组%d" % (i + 1),
            "%.2f" % st["target"],
            "%.4f" % st["mean"],
            "%+.2f%%" % st["err_pct"],
            "%.4f" % st["std"],
            "%.3f" % st["pk"],
            "%+.3f°" % (st["tr_drift"] / COUNTS_PER_DEG),
        ])

    tbl = ax.table(cellText=rows, colLabels=col_labels, loc="center",
                   cellLoc="center", colLoc="center",
                   colWidths=[0.255, 0.10, 0.145, 0.115, 0.125, 0.105, 0.125])
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(9.2)
    tbl.scale(1.0, 2.15)

    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#c8c8c8")
        cell.set_linewidth(0.7)
        if r == 0:
            cell.set_facecolor("#eef3f8")
            cell.set_text_props(fontweight="bold", color="#1a1a1a")
        else:
            if r % 2 == 0:
                cell.set_facecolor("#fafafa")
            if c == 3:
                cell.set_text_props(fontweight="bold", color="#1a6b32")

    note = ("注：实测力矩已扣除重力项；travel = 指令位置 − 启动锚点。\n"
            "稳态段 travel 漂移均 < 0.09°（三组挡块位置不同，travel 绝对值不可横向比较）。\n"
            "闭环参数：K=0 纯积分，Ki=2000 counts/(N·m·s)，PP 位置流 40 ms 换点。")
    ax.text(0.0, 0.02, note, transform=ax.transAxes, fontsize=8.4,
            color="#555555", va="bottom", ha="left", linespacing=1.6)

    fig.tight_layout(rect=[0, 0, 1, 0.955])

    out = os.path.splitext(path)[0] + ".png"
    fig.savefig(out, dpi=200, facecolor="white", bbox_inches="tight")
    print("图已保存: %s" % out)

    # ---- 控制台报告 ----
    print("\n%-18s %8s %10s %9s %9s %8s %10s" %
          ("工况", "目标", "均值", "误差", "标准差", "峰峰", "travel漂移"))
    for i, st in enumerate(stats):
        print("%-18s %8.2f %10.4f %8.2f%% %9.4f %8.3f %+9.3f°" %
              (NAMES[i] if i < len(NAMES) else "组%d" % (i + 1),
               st["target"], st["mean"], st["err_pct"],
               st["std"], st["pk"], st["tr_drift"] / COUNTS_PER_DEG))

    print("\n稳态取数区间 (ms):", STEADY[:len(groups)])
    print("若某组在区间内有外力扰动，把该组区间切到扰动前后两段分别统计。")


if __name__ == "__main__":
    main()
