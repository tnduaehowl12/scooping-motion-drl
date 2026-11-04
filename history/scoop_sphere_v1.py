# -*- coding: utf-8 -*-
# =====================================================================
# 아이스크림 스쿠핑 - 회전 성형(sphere) 방식 테스트 스크립트 (v1)
# 구조: Z축 진입(X,Z 힘 동시 감지, 델타 기반 표면감지)
#       -> 표면 기준 추가진입(2~3cm)
#       -> J5 보정으로 스쿱을 바닥과 수직으로 세움
#       -> J6 360도 회전으로 재료를 구체로 성형 (벽 감지/push 로직 전면 삭제)
#       -> 후퇴/상승 -> 붓기 위치 이동 -> 배출 -> J6 -360도 복구 -> 홈 복귀
#
# *** TODO 표시된 값들은 기존 스크립트의 실측값으로 반드시 교체 후 실행할 것 ***
# 아직 실기 테스트 전 초안이므로, 처음 실행 시에는 저속(VEL/ACC 낮게) +
# 사람이 비상정지 버튼에 손 올린 상태로 1사이클만 검증 권장.
# =====================================================================

# ---------------------------------------------------------------------
# 0. 파라미터
# ---------------------------------------------------------------------

# --- 홈 자세 (실측값, 기존 go_home()에서 그대로 가져옴) ---
HOME_POSX = posx(367.48, 3.07, 213.43, 89.68, 179.98, 90.19)

# --- 진입 관련 (전부 실측값 - 이전 세션에서 검증된 값) ---
# 스쿱이 기울어진 채(약 143도 자세) Z축으로 내려가며 진입하는 자세.
ENTRY_TILT_RX = 0.07
ENTRY_TILT_RY = -143.31
ENTRY_TILT_RZ = 1.13

PROBE_Z_FREEFALL_END = 140.0  # 실측값 (여러 번 튜닝 끝에 정착한 최종값)
PROBE_Z_STEP = 1.0            # mm, 실측 최종값
PROBE_Z_SETTLE_WAIT = 0.4     # sec, 실측 최종값
PROBE_Z_MAX_DEPTH = 12.0      # mm, 실측 최종값 (좌표 캡 대신 상대 깊이 캡으로 안전장치)

F_ENTRY_DELTA = 0.5    # N, 실측 최종값 (X,Z 합성 기준이라 그대로 쓰되 실측 후 미세조정 권장)
F_HARD_LIMIT = 12.0    # N, 실측 최종값

ENTRY_EXTRA_DEPTH = 25.0  # mm, 표면 감지 후 추가 진입 깊이 (2~3cm 중간값 -> 실측 후 20~30 사이 튜닝, 여긴 새 설계라 미확정)

# --- 수직화(J5 보정) ---
# 표면+추가깊이 지점의 X,Y,Z는 유지한 채, 방향만 "그리퍼가 바닥을 보는" 자세로 전환.
# movej로 J5만 돌리면 TCP가 호를 그리며 옆으로 밀리므로, movel(posx)로 위치 고정 + 방향만 전환.
# 완전한 수직(180deg)은 실측 검증된 적 없어서, 커얼 궤적에서 실제로 도달이 확인된
# 거의-수직 자세(177.56도)를 기준으로 사용 - 미검증 값보다 이게 안전함.
VERTICAL_RX = -0.96
VERTICAL_RY = 177.56
VERTICAL_RZ = 0.11

VEL_STRAIGHTEN = 30
ACC_STRAIGHTEN = 30

# --- J6 회전 성형 ---
J6_SHAPE_DEGREE = 360   # 단방향 360도 (왕복 180/180 대신 선택)
VEL_J6_SHAPE = 60       # deg/s, 기존보다 빠르게 -> 실기에서 60부터 시작, 문제 없으면 상향
ACC_J6_SHAPE = 60
F_HARD_LIMIT_SHAPE = 15.0  # 회전 중에도 최소한의 비상정지는 유지 (관성으로 인한 순간값 튐 감안, 델타감시는 끔)

# --- 후퇴/상승 ---
LIFT_Z_TARGET_REL = 150.0  # mm, 상승량
RETREAT_X_REL = 20.0       # mm, 통 벽에서 살짝 물러남 (여유 확보용)

# --- 붓기 위치 (실측값) ---
POUR_TARGET_POSX = posx(680.09, -238.02, 322.16, 0.42, 104.43, 1.08)

# --- 레인(진입 지점) 목록 - 통 규격만으로 자동 계산, 개수 무관 ---
# X는 고정(397.89) - 지금까지 모든 스쿱이 이 X에서 진입했음.
# Y만 레인 중심(-131.04) 기준 ±35mm로 3곳 - 기존 사각 통 레인 로직 값 그대로.
ENTRY_X = 397.89
LANE_Y_CENTER = -131.04
LANE_DY_LIST = [-35.0, 0.0, 35.0]   # 짧은 변(14cm) 방향 - 기존 검증된 3레인 값

# --- 긴 변(18cm) 방향 격자 - 통 규격 기반, 6cm 간격 3칸으로 18cm 커버 ---
TUB_LENGTH_LONG = 180.0     # mm, 18cm (긴 변)
COLUMN_SPACING = 60.0       # mm, 6cm 간격 (요청하신 값)
N_COLUMNS = int(round(TUB_LENGTH_LONG / COLUMN_SPACING))  # 180/60 = 3


def get_entry_points(count):
    """통 규격(TUB_LENGTH_LONG, COLUMN_SPACING)과 레인 목록(LANE_DY_LIST)만으로
    필요한 개수(count)만큼의 진입 지점(x, y)을 자동 생성한다.
    개별 위치를 티칭할 필요 없이, 이 두 값만 조정하면 스쿱 개수/배치가 그대로 따라온다.

    순서: 레인 우선 - 한 레인 안에서 긴 변 방향으로 3칸(6cm씩)을 먼저 돌고,
    그 다음 레인으로 넘어간다. (요청하신 "레인1 3칸 -> 레인2 3칸 -> ..." 순서와 동일)

    격자(레인 수 x 3칸)를 다 쓰면 처음부터 다시 순환한다 - Z축은 매번 실시간
    표면 탐색을 하기 때문에, 같은 칸을 여러 번 재사용해도 재료가 줄어든 만큼
    자동으로 더 깊이 들어가 표면을 찾으므로 안전하다."""

    grid = []
    for lane_idx in range(len(LANE_DY_LIST)):
        y = LANE_Y_CENTER + LANE_DY_LIST[lane_idx]
        # 레인이 바뀔 때마다 진행 방향을 왕복(지그재그)시켜서
        # 매 레인 전환마다 로봇이 긴 변 반대쪽 끝까지 되돌아가는 큰 이동을 없앤다.
        col_order = range(N_COLUMNS) if lane_idx % 2 == 0 else reversed(range(N_COLUMNS))
        for col in col_order:
            x = ENTRY_X + col * COLUMN_SPACING
            grid.append((x, y))

    points = []
    i = 0
    while len(points) < count:
        points.append(grid[i % len(grid)])
        i = i + 1
    return points


LANE_ENTRY_POINTS = get_entry_points(3)  # 시연용 3스쿱 - 12/20으로 바꾸려면 숫자만 교체


# ---------------------------------------------------------------------
# 1. 유틸 함수
# ---------------------------------------------------------------------

def force_magnitude_xz(f):
    """X,Z 힘을 합성한 크기. 스쿱이 기울어진 채 진입하므로 접촉이 X,Z 어느 쪽으로도
    나타날 수 있어 두 축을 함께 본다. (ROS2 이식 시에는 sqrt 대신 제곱값 비교로 변경할 것)"""
    return sqrt(f[0] * f[0] + f[2] * f[2])


def emergency_retreat():
    """어떤 실패/중단 지점에서도 다음 동작으로 넘어가기 전 반드시 먼저 호출.
    현재 위치 기준으로 -X 후퇴 + Z 상승."""
    cur = get_current_posx()[0]
    tp_log("!! 비상후퇴 실행 중 - 현재위치 기준 -X, +Z")
    retreat_pos = posx(cur[0] - RETREAT_X_REL, cur[1], cur[2] + 30.0, cur[3], cur[4], cur[5])
    movel(retreat_pos, vel=30, acc=30, ref=DR_BASE)
    lift_pos = get_current_posx()[0]
    lift_pos2 = posx(lift_pos[0], lift_pos[1], lift_pos[2] + LIFT_Z_TARGET_REL, lift_pos[3], lift_pos[4], lift_pos[5])
    movel(lift_pos2, vel=30, acc=30, ref=DR_BASE)


def emergency_check(f):
    """합성 힘이 하드리밋을 넘으면 1 반환."""
    mag = force_magnitude_xz(f)
    if mag >= F_HARD_LIMIT:
        tp_log("!! 비상: 힘 한계(" + str(F_HARD_LIMIT) + "N) 초과, 현재=" + str(mag))
        return 1
    return 0


# ---------------------------------------------------------------------
# 2. 표면 감지 진입 (Z축, X/Z 힘 동시 감시, 델타 기반)
# ---------------------------------------------------------------------

def probe_surface_entry(entry_x, entry_y):
    """자유낙하 후 작은 스텝으로 내려가며 표면 접촉을 델타 기반으로 감지.
    반환: 성공 시 표면 감지 지점의 posx, 실패 시 -1"""

    # 자유낙하 (진입 자세로 안전 높이까지 빠르게 하강)
    freefall_pos = posx(entry_x, entry_y, PROBE_Z_FREEFALL_END,
                         ENTRY_TILT_RX, ENTRY_TILT_RY, ENTRY_TILT_RZ)
    movel(freefall_pos, vel=60, acc=60, ref=DR_BASE)
    wait(0.3)

    # 기준값(baseline) 측정 - 이 자세에서는 접촉 없이도 편향값이 실리므로 반드시 여기서 기준 잡음
    f_base = get_tool_force()
    baseline_mag = force_magnitude_xz(f_base)
    tp_log("진입 기준값 baseline=" + str(baseline_mag))

    cur_pos = get_current_posx()[0]

    for step in range(PROBE_Z_MAX_STEPS):
        next_pos = posx(cur_pos[0], cur_pos[1], cur_pos[2] - PROBE_Z_STEP,
                         ENTRY_TILT_RX, ENTRY_TILT_RY, ENTRY_TILT_RZ)
        movel(next_pos, vel=15, acc=15, ref=DR_BASE)
        wait(PROBE_Z_SETTLE_WAIT)

        f = get_tool_force()

        if emergency_check(f) == 1:
            emergency_retreat()
            return -1

        mag = force_magnitude_xz(f)
        delta = abs(mag - baseline_mag)
        tp_log("PROBE_STEP" + str(step + 1) + " mag=" + str(mag) + " delta=" + str(delta))

        if delta >= F_ENTRY_DELTA:
            tp_log(">> 표면 감지됨 (step=" + str(step + 1) + ")")
            return get_current_posx()[0]

        cur_pos = get_current_posx()[0]

    # 최대 스텝까지 감지 못하면 실패로 처리 (임의로 값을 만들어 진행하지 않음)
    tp_log("!! 표면 감지 실패 - 최대 스텝 도달")
    emergency_retreat()
    return -1


# ---------------------------------------------------------------------
# 3. 추가 진입 + 수직화
# ---------------------------------------------------------------------

def enter_additional_depth():
    """표면 감지 지점에서 같은 방향(진입 자세 유지)으로 추가 진입."""
    cur = get_current_posx()[0]
    target = posx(cur[0], cur[1], cur[2] - ENTRY_EXTRA_DEPTH,
                   ENTRY_TILT_RX, ENTRY_TILT_RY, ENTRY_TILT_RZ)

    f = get_tool_force()
    if emergency_check(f) == 1:
        emergency_retreat()
        return -1

    movel(target, vel=20, acc=20, ref=DR_BASE)
    wait(0.3)

    f2 = get_tool_force()
    if emergency_check(f2) == 1:
        emergency_retreat()
        return -1

    return get_current_posx()[0]


# -- 진입자세->수직자세 툴 프레임 기준 상대회전 (계산으로 검증된 값) --
# R_rel = R_entry^-1 * R_vertical 을 ZYZ 오일러로 변환한 값.
# DR_TOOL 기준으로 순수 회전만 상대이동시키면, 회전 중심이 TCP(가정: 스쿱 날 끝)가 되어
# 베이스 기준 이동처럼 날이 옆으로 쓸리는 현상 없이 제자리에서 세워질 것으로 기대.
# *** DR_TOOL이 이 프로젝트에서 실제 확인된 상수가 아니므로 저속 테스트 먼저 필요 ***
TOOL_REL_ROTATE_TO_VERTICAL = posx(0, 0, 0, 178.94, 39.13, -178.92)


def straighten_to_vertical():
    """진입자세에서 수직자세로, 툴 프레임 기준 순수 회전만 상대이동.
    TCP가 실제 스쿱 접촉점과 일치한다는 전제 하에 그 지점을 축으로 회전한다."""
    f = get_tool_force()
    if force_magnitude_xz(f) >= F_HARD_LIMIT_SHAPE:
        tp_log("!! 수직화 시작 전 이미 힘 초과 - 중단")
        emergency_retreat()
        return -1

    movel(TOOL_REL_ROTATE_TO_VERTICAL, vel=VEL_STRAIGHTEN, acc=ACC_STRAIGHTEN,
          ref=DR_TOOL, mod=DR_MV_MOD_REL)
    wait(0.2)

    f2 = get_tool_force()
    if force_magnitude_xz(f2) >= F_HARD_LIMIT_SHAPE:
        emergency_retreat()
        return -1

    return get_current_posx()[0]


# ---------------------------------------------------------------------
# 4. J6 회전 성형 (구체 만들기)
# ---------------------------------------------------------------------

def shape_sphere_j6():
    """J6를 360도 단방향 회전시켜 재료를 구체 형태로 성형.
    회전 중에는 관성으로 힘값이 흔들릴 수 있어 델타 감시는 끄고,
    최소한의 비상 하드리밋 체크만 유지."""

    f_before = get_tool_force()
    if force_magnitude_xz(f_before) >= F_HARD_LIMIT_SHAPE:
        tp_log("!! 회전 시작 전 이미 힘 초과 - 중단")
        emergency_retreat()
        return -1

    tp_log(">> J6 360도 성형 회전 시작")
    movej(posj(0, 0, 0, 0, 0, J6_SHAPE_DEGREE), vel=VEL_J6_SHAPE, acc=ACC_J6_SHAPE,
          ref=DR_BASE, mod=DR_MV_MOD_REL)
    wait(0.2)

    f_after = get_tool_force()
    if force_magnitude_xz(f_after) >= F_HARD_LIMIT_SHAPE:
        tp_log("!! 회전 중/후 힘 초과 감지 - 비상후퇴")
        emergency_retreat()
        return -1

    return 1


def unwind_j6():
    """붓기(배출) 완료 직후, 스쿱이 빈 상태에서 J6를 원위치로 복구.
    - 성형 직후가 아니라 '배출 후'에 하는 이유: 만들어진 구체를 흐트러뜨릴 위험이 없고,
      다음 사이클 시작 전 J6를 항상 기준각으로 리셋해서 케이블 누적 감김을 방지하기 위함."""
    tp_log(">> J6 복구 회전 (-360도)")
    movej(posj(0, 0, 0, 0, 0, -J6_SHAPE_DEGREE), vel=VEL_J6_SHAPE, acc=ACC_J6_SHAPE,
          ref=DR_BASE, mod=DR_MV_MOD_REL)
    wait(0.2)


# ---------------------------------------------------------------------
# 5. 상승/이동/배출
# ---------------------------------------------------------------------

def lift_after_shaping():
    cur = get_current_posx()[0]
    target = posx(cur[0], cur[1], cur[2] + LIFT_Z_TARGET_REL, cur[3], cur[4], cur[5])
    movel(target, vel=40, acc=40, ref=DR_BASE)


def move_to_pour_and_release():
    """붓기 위치로 이동 후 배출. 배출 메커니즘(그리퍼 동작 등)은 기존 로직 그대로 사용."""
    movel(POUR_TARGET_POSX, vel=50, acc=50, ref=DR_BASE)  # TODO: 기존 pour_target 이동 방식으로 교체
    wait(0.3)
    # TODO: 기존 배출 동작 (예: RG2 오픈, 혹은 스쿱을 다시 기울이는 등 기존 메커니즘) 호출
    tp_log(">> 배출 완료 (TODO: 실제 배출 로직 연결)")


def go_home():
    movejx(HOME_POSX, vel=31, acc=47, sol=255)  # 기존 VEL_J_SLOW/ACC_J_SLOW 실측값


# ---------------------------------------------------------------------
# 6. 사이클 오케스트레이션
# ---------------------------------------------------------------------

def scoop_cycle_sphere(entry_x, entry_y):
    tp_log("===== 스쿱 사이클 시작 =====")

    surf = probe_surface_entry(entry_x, entry_y)
    if surf == -1:
        return 0

    r1 = enter_additional_depth()
    if r1 == -1:
        return 0

    r2 = straighten_to_vertical()
    if r2 == -1:
        return 0

    r3 = shape_sphere_j6()
    if r3 == -1:
        return 0

    lift_after_shaping()
    move_to_pour_and_release()
    unwind_j6()

    tp_log("===== 스쿱 사이클 완료 =====")
    return 1


def main_test(scoop_count=3):
    """시연용 - scoop_count로 원하는 스쿱 수 지정 (예: 3, 12, 20).
    개별 위치를 따로 입력할 필요 없이 통 규격 기반 격자에서 자동 생성된다."""
    go_home()

    entry_points = get_entry_points(scoop_count)

    for i in range(scoop_count):
        ex, ey = entry_points[i]
        result = scoop_cycle_sphere(ex, ey)
        if result == 0:
            tp_log("!! " + str(i + 1) + "번째 스쿱 실패 - 테스트 중단")
            break
        go_home()

    tp_log("테스트 종료")


# main_test(3)   # 시연용 3스쿱
# main_test(12)  # 확장 시 - 통 규격 기반 격자에서 자동 생성됨
# main_test(20)  # 확장 시 - 격자를 다 쓰면 처음부터 순환 (Z는 매번 실시간 재탐색)
# 실행 전: move_to_pour_and_release() 안의 배출 로직 연결, get_current_posx()/sqrt() 검증 필요
