# -*- coding: utf-8 -*-
import math

data = [
    (116.48, -0.2232),
    (126.48, -0.1623),
    (136.48, -0.0899),
    (146.48, -0.0164),
    (156.48,  0.0357),
    (166.48,  0.1034),
    (176.48,  0.1720),
    (186.48,  0.2405),
]
n = len(data)
th = [math.radians(d) for d, _ in data]
y = [v for _, v in data]


def solve3(X, y):
    m = len(X[0])
    M = [[sum(X[k][i] * X[k][j] for k in range(n)) for j in range(m)]
         + [sum(X[k][i] * y[k] for k in range(n))] for i in range(m)]
    for i in range(m):
        p = max(range(i, m), key=lambda r: abs(M[r][i]))
        M[i], M[p] = M[p], M[i]
        pv = M[i][i]
        for j in range(i, m + 1):
            M[i][j] /= pv
        for r in range(m):
            if r != i:
                f = M[r][i]
                for j in range(i, m + 1):
                    M[r][j] -= f * M[i][j]
    return [M[i][m] for i in range(m)], M


def r2(y, yh):
    ym = sum(y) / n
    ssr = sum((a - b) ** 2 for a, b in zip(y, yh))
    sst = sum((a - ym) ** 2 for a in y)
    return 1 - ssr / sst, math.sqrt(ssr / (n - 3))


# ---------- 模型 A: a*cos + b*sin + c ----------
X3 = [[math.cos(t), math.sin(t), 1.0] for t in th]
(a, b, c), _ = solve3(X3, y)
yh = [a * math.cos(t) + b * math.sin(t) + c for t in th]
R2, rms = r2(y, yh)
A = math.hypot(a, b)
phi = math.degrees(math.atan2(b, a))

print("=== 模型 A: tau = a*cos + b*sin + c (3 参数) ===")
print(f"  a = {a:+.6f} N.m")
print(f"  b = {b:+.6f} N.m")
print(f"  c = {c:+.6f} N.m")
print(f"  A = sqrt(a^2+b^2) = {A:.6f} N.m   (m*g*L)")
print(f"  phi = atan2(b,a) = {phi:+.2f} deg")
print(f"  R^2 = {R2:.6f}   RMS 残差 = {rms*1000:.4f} mN.m")
print()
print("  点   实测      拟合      残差(mN.m)")
for i, (d, v) in enumerate(data):
    print(f"  {i}  {v:+.4f}  {yh[i]:+.4f}  {(v-yh[i])*1000:+8.3f}")

# ---------- 模型 B: b*sin + c ----------
X2 = [[math.sin(t), 1.0] for t in th]
# 2x2
S11 = sum(r[0]**2 for r in X2); S12 = sum(r[0] for r in X2); S22 = n
T1 = sum(X2[k][0]*y[k] for k in range(n)); T2 = sum(y)
det = S11*S22 - S12*S12
bb = (T1*S22 - T2*S12)/det
cc = (T2*S11 - T1*S12)/det
yh2 = [bb*math.sin(t) + cc for t in th]
R2b, rmsb = r2(y, yh2)
print(f"\n=== 模型 B: tau = b*sin + c (2 参数) ===")
print(f"  b = {bb:+.6f}  c = {cc:+.6f}   R^2 = {R2b:.6f}  RMS = {rmsb*1000:.4f} mN.m")

# ---------- 模型 C: 纯线性 ----------
X1 = [[t, 1.0] for t in th]
S11 = sum(r[0]**2 for r in X1); S12 = sum(r[0] for r in X1); S22 = n
T1 = sum(X1[k][0]*y[k] for k in range(n)); T2 = sum(y)
det = S11*S22 - S12*S12
kk = (T1*S22 - T2*S12)/det
cc1 = (T2*S11 - T1*S12)/det
yh3 = [kk*t + cc1 for t in th]
R2c, rmsc = r2(y, yh3)
print(f"=== 模型 C: tau = k*theta + c (线性) ===")
print(f"  k = {kk:+.6f} N.m/rad   R^2 = {R2c:.6f}  RMS = {rmsc*1000:.4f} mN.m")

# ---------- 参数标准差 ----------
print()
print("=== 参数估计标准差（sigma = RMS，自由度 5）===")
print(f"  噪声 sigma = {rms*1000:.3f} mN.m")
print(f"  零点处 tau(0) = c = {c:+.4f} N.m")

# ---------- 差分检查 ----------
print()
print("=== 相邻两点差分（每 10 deg）===")
prev = None
for d, v in data:
    if prev is not None:
        print(f"  {d:6.2f} deg  dtau = {(v-prev)*1000:+7.2f} mN.m")
    prev = v
