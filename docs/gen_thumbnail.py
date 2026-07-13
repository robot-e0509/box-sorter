#!/usr/bin/env python3
"""프로젝트 썸네일 (깃헙 소셜 프리뷰 규격 1280x640)."""
from PIL import Image, ImageDraw, ImageFont

W, H = 1280, 640
NOTO = "/usr/share/fonts/opentype/noto/NotoSansCJK-%s.ttc"


def font(weight, size):
    return ImageFont.truetype(NOTO % weight, size, index=1)


BG = "#0B1220"
CARD = "#111C2E"
LINE = "#1E2E47"
WHITE = "#F1F5F9"
MUTED = "#64748B"
SUB = "#94A3B8"
SKY = "#38BDF8"
GREEN = "#4ADE80"
AMBER = "#FBBF24"
RED = "#F87171"

img = Image.new("RGB", (W, H), BG)
d = ImageDraw.Draw(img)

# 배경 격자 (아주 은은하게)
for x in range(0, W, 40):
    d.line([(x, 0), (x, H)], fill="#0E1729", width=1)
for y in range(0, H, 40):
    d.line([(0, y), (W, y)], fill="#0E1729", width=1)

# 좌우 분리선
d.line([(640, 60), (640, 580)], fill=LINE, width=1)

# ─────────────────────────────────────────────────────
#  왼쪽 — 타이틀
# ─────────────────────────────────────────────────────
X = 72
d.text((X, 92), "DOOSAN  E0509   ·   RH-P12-RN   ·   ROS 2  JAZZY",
       font=font("Medium", 15), fill=MUTED)

d.text((X, 138), "적응형 파지", font=font("Bold", 62), fill=WHITE)
d.text((X, 214), "물류 분류 로봇", font=font("Bold", 62), fill=SKY)

d.line([(X, 320), (X + 56, 320)], fill=AMBER, width=4)

d.text((X, 352), "라벨을 읽고,", font=font("Medium", 25), fill=SUB)
d.text((X, 390), "상자에 맞춰 잡는 힘을 조절합니다.", font=font("Medium", 25), fill=SUB)

# 목적지 칩
chips = [("서울", True), ("수원", False), ("경기", False)]
cx = X
for name, active in chips:
    tw = d.textlength(name, font=font("Bold", 19))
    w = int(tw) + 40
    fill = "#0E2A3C" if active else CARD
    edge = SKY if active else LINE
    txt = SKY if active else MUTED
    d.rounded_rectangle([cx, 470, cx + w, 514], radius=22, fill=fill, outline=edge, width=2)
    d.text((cx + 20, 480), name, font=font("Bold", 19), fill=txt)
    cx += w + 12

# ─────────────────────────────────────────────────────
#  오른쪽 — 로봇팔 + 그리퍼가 라벨 붙은 상자를 잡고 있고,
#           카메라가 그 라벨을 읽는다
# ─────────────────────────────────────────────────────
CX = 990          # 그리퍼 중심

# 로봇팔 (오른쪽 위에서 내려옴)
ARM = "#334155"
d.line([(1268, 74), (1150, 132)], fill=ARM, width=22)          # 상완
d.line([(1150, 132), (CX, 176)], fill=ARM, width=18)           # 전완
for j in [(1150, 132), (CX, 176)]:                              # 관절
    d.ellipse([j[0] - 13, j[1] - 13, j[0] + 13, j[1] + 13], fill="#475569", outline=BG, width=2)
d.line([(CX, 176), (CX, 208)], fill=ARM, width=14)             # 플랜지

# 그리퍼 본체
d.rounded_rectangle([CX - 66, 204, CX + 66, 240], radius=8, fill=CARD, outline=SUB, width=2)

# 상자 (라벨 붙은) — 손가락보다 먼저 그려서 손가락이 위에 오게
d.rounded_rectangle([CX - 84, 262, CX + 84, 372], radius=8, fill="#1A2437", outline=AMBER, width=3)
d.rounded_rectangle([CX - 56, 292, CX + 56, 344], radius=6, fill="#F1F5F9")
tw = d.textlength("서울", font=font("Bold", 30))
d.text((CX - tw / 2, 300), "서울", font=font("Bold", 30), fill="#0B1220")

# 그리퍼 손가락 — 상자 옆면을 물고 있음
d.rounded_rectangle([CX - 100, 238, CX - 78, 350], radius=6, fill="#233047", outline=SUB, width=2)
d.rounded_rectangle([CX + 78, 238, CX + 100, 350], radius=6, fill="#233047", outline=SUB, width=2)

# 조이는 힘
d.line([(CX - 122, 300), (CX - 106, 300)], fill=RED, width=3)
d.polygon([(CX - 106, 292), (CX - 106, 308), (CX - 94, 300)], fill=RED)
d.line([(CX + 122, 300), (CX + 106, 300)], fill=RED, width=3)
d.polygon([(CX + 106, 292), (CX + 106, 308), (CX + 94, 300)], fill=RED)

# 카메라 — 왼쪽 위에서 상자 라벨을 내려다본다
KX, KY = 730, 178
d.rounded_rectangle([KX - 46, KY - 30, KX + 46, KY + 30], radius=10, fill=CARD, outline=LINE, width=2)
d.rounded_rectangle([KX - 32, KY - 17, KX - 4, KY + 11], radius=6, fill=BG, outline=SKY, width=2)
d.ellipse([KX - 26, KY - 11, KX - 10, KY + 5], fill=SKY)
d.text((KX + 6, KY - 12), "CAM", font=font("Bold", 14), fill=MUTED)

# 카메라 → 상자 윗면 (점선). 빨간 조임 화살표(y≈300)를 피해 위로 지나간다.
import math
x0, y0 = KX + 52, KY + 14
x1, y1 = CX - 48, 254
dx, dy = x1 - x0, y1 - y0
dist = math.hypot(dx, dy)
ux, uy = dx / dist, dy / dist
step, gap = 10, 8
t = 0.0
while t < dist - 22:                     # 화살촉 자리를 남겨둔다
    d.line([(x0 + ux * t, y0 + uy * t),
            (x0 + ux * (t + step), y0 + uy * (t + step))], fill=SKY, width=2)
    t += step + gap
# 화살촉
px, py = -uy, ux
tipx, tipy = x1, y1
d.polygon([(tipx, tipy),
           (tipx - ux * 16 + px * 8, tipy - uy * 16 + py * 8),
           (tipx - ux * 16 - px * 8, tipy - uy * 16 - py * 8)], fill=SKY)

# OCR 결과 태그
d.rounded_rectangle([KX - 64, KY + 76, KX + 64, KY + 112], radius=10,
                    fill="#0E2A3C", outline=SKY, width=2)
d.text((KX - 50, KY + 84), 'OCR  "서울"', font=font("Bold", 17), fill=SKY)

# ── 힘 게이지 ────────────────────────────────────────
GX0, GX1, GY = CX - 190, CX + 190, 452
d.text((GX0, GY - 30), "파지 강도  current", font=font("Medium", 15), fill=MUTED)

# 구간: 놓침 / 안전 / 손상
d.rounded_rectangle([GX0, GY, GX0 + 120, GY + 22], radius=6, fill="#2A1620", outline="#5B2733", width=1)
d.rectangle([GX0 + 120, GY, GX1 - 120, GY + 22], fill="#14301F", outline="#245C38", width=1)
d.rounded_rectangle([GX1 - 120, GY, GX1, GY + 22], radius=6, fill="#2A1620", outline="#5B2733", width=1)

d.text((GX0 + 22, GY + 30), "놓침", font=font("Medium", 14), fill=RED)
tw = d.textlength("안전 구간", font=font("Bold", 14))
d.text(((GX0 + GX1) / 2 - tw / 2, GY + 30), "안전 구간", font=font("Bold", 14), fill=GREEN)
tw = d.textlength("손상", font=font("Medium", 14))
d.text((GX1 - 22 - tw, GY + 30), "손상", font=font("Medium", 14), fill=RED)

# 현재값 마커
MX = (GX0 + GX1) // 2 + 10
d.line([(MX, GY - 8), (MX, GY + 30)], fill=GREEN, width=3)
d.polygon([(MX - 7, GY - 14), (MX + 7, GY - 14), (MX, GY - 5)], fill=GREEN)

# 하단 캡션
cap = "미끄러지지도, 찌그러지지도 않는 좁은 구간을 찾습니다"
tw = d.textlength(cap, font=font("Medium", 17))
d.text((CX - tw / 2, GY + 68), cap, font=font("Medium", 17), fill=MUTED)

img.save("/home/sooya/doosan_ws/docs/thumbnail.png")
print("docs/thumbnail.png  (%dx%d)" % (W, H))
