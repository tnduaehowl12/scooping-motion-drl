# -*- coding: utf-8 -*-
# =====================================================================
# 아이스크림 스쿠핑 - 회전 성형(sphere) 방식 (v6)
#
# v5 -> v6 변경점 (2026-08-26)
#   [P] 노이즈 자동 상향 로직 완전 삭제  *** 바닥까지 내려간 직접 원인 ***
#       noise_max x 2.0 이 임계값을 4~6N으로 밀어올려서, 실제 표면 접촉
#       (2~3N)이 영원히 감지되지 않고 PROBE_MIN_Z까지 내려갔다.
#       -> F_ENTRY_DELTA = 2.0 고정.
#   [Q] 2단 탐색 -> 1단 복귀
#       시작점을 125로 내려 표면 바로 위에서 출발하므로 조대/정밀을 나눌
//      이유가 없어졌다. 2mm 스텝 x 0.25s, 최대 15스텝 = 8초 이내.
#       v5에서 보이던 "Z축으로 한번 올라왔다가 대기" 동작(backoff)도 사라짐.
#   [R] 퍼올리기: Ry와 Z 동시 보간
#       실측 지시 - Rx, Rz는 그대로 두고 Ry만 -170 -> -110 (+60도 방향).
#       v3의 방향이 맞았고 v5에서 뒤집은 것이 잘못이었다. v3가 -124.96에서
#       [2.1206]으로 멈춘 것은 방향 문제가 아니라 그 자세의 도달 한계였고,
#       Z를 함께 올리면 해소된다.
#   [S] 높이 재설정 - 통 80mm / 내용물 35mm 기준
#       PROBE_START_Z 175 -> 125,  PROBE_MIN_Z 118 -> 95
#
# 이전 이력 요약
#   [A~D] 힘 판정 전면 델타 기반화, baseline 다중 샘플
#   [C]   분할 상승 + movejx 복귀 (2.1206 NOT REACHABLE 대응)
#   [E]   퍼올리기 movej(J5) -> movel(자세 전용) (통 벽 충돌 원인)
#   [G]   추가진입 분할 + 힘 감시
#   [M]   F_HARD_DELTA 5.0 -> 12.0 (바닥 접촉 5~9N보다 위)
#
# *** 실행 전 확인 ***
#   - 스쿱 장착 상태로 실행할 것 (TCP가 스쿱 끝 기준)
#   - PROBE_MIN_Z(95)는 통 바닥 추정치. 실측 바닥 Z를 알면 그 값 +5로 교체.
# =====================================================================

# ---------------------------------------------------------------------
# 0. 파라미터
# ---------------------------------------------------------------------

HOME_POSX = posx(367.48, 3.07, 213.43, 89.68, 179.98, 90.19)

# --- 진입 자세 (실측값) ---
ENTRY_TILT_RX = 0.07
ENTRY_TILT_RY = -143.31
ENTRY_TILT_RZ = 1.13

# --- 높이 (통 80mm, 내용물 35mm 기준)  [S] ---
PROBE_START_Z = 125.0   # 표면 바로 위에서 출발 - 여기서 바로 감지 시작
PROBE_MIN_Z   = 95.0    # 통 바닥 추정 90 + 마진 5. 절대 하한.

# --- 탐색  [Q] ---
PROBE_STEP      = 2.0
PROBE_WAIT      = 0.25
PROBE_VEL       = 15
PROBE_MAX_STEPS = 15     # 2mm x 15 = 30mm (125 -> 95)
CONSEC_HIT_NEEDED = 2

# --- baseline ---
BASELINE_SETTLE_WAIT = 0.6
BASELINE_SAMPLES     = 5
BASELINE_SAMPLE_GAP  = 0.06

FORCE_READ_SAMPLES = 2
FORCE_READ_GAP     = 0.03

# --- 힘 임계값 (전부 baseline 대비 델타)  [P] ---
F_ENTRY_DELTA      = 2.0    # 표면 접촉 (실측: X/Z 2~3N). 자동 상향 없음.
F_HARD_DELTA       = 12.0   # 진입 중 비상정지 (바닥 접촉 5~9N보다 위)
F_HARD_DELTA_SHAPE = 12.0
F_EXTRA_STOP_DELTA = 5.0    # 추가진입 정상 종료 저항

# --- 추가 진입 ---
ENTRY_EXTRA_DEPTH = 12.0
ENTRY_EXTRA_STEP  = 3.0

# --- 수직화 ---
VEL_STRAIGHTEN = 30
ACC_STRAIGHTEN = 30

# --- J6 회전 성형 ---
J6_SHAPE_DEGREE = 360
VEL_J6_SHAPE    = 60
ACC_J6_SHAPE    = 60

# --- 퍼올리기: Ry 절대 목표 + Z 동시 상승  [R] ---
SCOOP_UP_RY_TARGET = -110.0   # 절대 목표 자세 (델타 아님). Rx, Rz는 불변.
SCOOP_UP_Z_RISE    = 60.0     # 회전과 동시에 올릴 총 높이
SCOOP_UP_STEPS     = 6        # 10도 + 10mm 씩 6회
VEL_SCOOP_UP = 20
ACC_SCOOP_UP = 20

# --- 후퇴/상승 ---
RETREAT_X_REL    = 20.0
RETREAT_LIFT_REL = 80.0
LIFT_AFTER_SHAPE = 60.0    # 퍼올리기에서 이미 60mm 올라오므로 축소
LIFT_STEP        = 25.0
VEL_LIFT = 40
ACC_LIFT = 40

# --- 붓기 위치 (실측값) ---
POUR_TARGET_POSX = posx(680.09, -238.02, 322.16, 0.42, 104.43, 1.08)

# --- 레인(진입 지점) ---
ENTRY_X       = 397.89
LANE_Y_CENTER = -131.04
LANE_DY_LIST  = [-35.0, 0.0, 35.0]

TUB_LENGTH_LONG = 180.0
COLUMN_SPACING  = 60.0
N_COLUMNS = int(round(TUB_LENGTH_LONG / COLUMN_SPACING))

VERTICAL_ORIENT_BY_LANE = {
    0: (0.0, -170.0, 0.0),   # 실측 확인됨 (Y=-166.02)
    1: (0.0, -170.0, 0.0),   # TODO: 레인1(Y=-131.04) 실측 필요
    2: (0.0, -170.0, 0.0),   # TODO: 레인2(Y=-96.04)  실측 필요
}
VERTICAL_ORIENT_DEFAULT = (0.0, -170.0, 0.0)

BASELINE = [0.0]


# ---------------------------------------------------------------------
# 1. 유틸
# ---------------------------------------------------------------------

def get_entry_points(count):
    """통 규격과 레인 목록만으로 진입 지점(x, y)을 자동 생성.
    레인 우선 + 레인 전환 시 지그재그. 격자를 다 쓰면 순환."""
    grid = []
    for lane_idx in range(len(LANE_DY_LIST)):
        y = LANE_Y_CENTER + LANE_DY_LIST[lane_idx]
        if lane_idx % 2 == 0:
            col_order = range(N_COLUMNS)
        else:
            col_order = reversed(range(N_COLUMNS))
        for col in col_order:
            grid.append((ENTRY_X + col * COLUMN_SPACING, y))

    points = []
    i = 0
    while len(points) < count:
        points.append(grid[i % len(grid)])
        i = i + 1
    return points


def force_magnitude_xz(f):
    """X,Z 힘 합성 크기. (ROS2 이식 시 sqrt 대신 제곱값 비교로 변경할 것)"""
    return sqrt(f[0] * f[0] + f[2] * f[2])


def read_force_avg():
    s = 0.0
    k = 0
    while k < FORCE_READ_SAMPLES:
        s = s + force_magnitude_xz(get_tool_force())
        wait(FORCE_READ_GAP)
        k = k + 1
    return s / FORCE_READ_SAMPLES


def capture_baseline(tag):
    """현재 자세의 무접촉 기준 힘을 평균해 BASELINE[0]에 저장."""
    wait(BASELINE_SETTLE_WAIT)
    s = 0.0
    k = 0
    while k < BASELINE_SAMPLES:
        s = s + force_magnitude_xz(get_tool_force())
        wait(BASELINE_SAMPLE_GAP)
        k = k + 1
    BASELINE[0] = s / BASELINE_SAMPLES
    tp_log("[baseline:" + tag + "] " + str(BASELINE[0]) + "N")
    return BASELINE[0]


def force_delta():
    return abs(read_force_avg() - BASELINE[0])


def over_limit(limit, tag):
    d = force_delta()
    if d >= limit:
        tp_log("!! 힘 초과 [" + tag + "] delta=" + str(d)
               + " limit=" + str(limit) + " base=" + str(BASELINE[0]))
        return 1
    return 0


def move_z_to(z, v):
    """현재 X,Y,자세를 유지한 채 Z만 절대 좌표로 이동."""
    cur = get_current_posx()[0]
    movel(posx(cur[0], cur[1], z, cur[3], cur[4], cur[5]),
          vel=v, acc=v, ref=DR_BASE)


# ---------------------------------------------------------------------
# 2. 안전 상승 / 비상후퇴
# ---------------------------------------------------------------------

def safe_lift(total_dz):
    """틸트 자세로 한 번에 크게 올리면 [2.1206]이 난다. 25mm씩 분할."""
    done = 0.0
    n = 0
    while done < total_dz:
        dz = LIFT_STEP
        if total_dz - done < LIFT_STEP:
            dz = total_dz - done
        cur = get_current_posx()[0]
        movel(posx(cur[0], cur[1], cur[2] + dz, cur[3], cur[4], cur[5]),
              vel=VEL_LIFT, acc=ACC_LIFT, ref=DR_BASE)
        done = done + dz
        n = n + 1
        tp_log("  상승 " + str(n) + "스텝 (+" + str(done) + "/" + str(total_dz) + "mm)")


def emergency_retreat():
    """통 밖 탈출 최소량만 Cartesian, 복귀는 movejx(sol 자동)."""
    tp_log("!! 비상후퇴 실행 - 이후 상승 스텝은 정상 동작")
    cur = get_current_posx()[0]
    movel(posx(cur[0] - RETREAT_X_REL, cur[1], cur[2] + 30.0,
               cur[3], cur[4], cur[5]),
          vel=VEL_LIFT, acc=ACC_LIFT, ref=DR_BASE)
    safe_lift(RETREAT_LIFT_REL)
    movejx(HOME_POSX, vel=20, acc=20, sol=255)
    tp_log("!! 비상후퇴 완료 - 홈 복귀")


# ---------------------------------------------------------------------
# 3. 표면 감지 (단일 단계)  [Q]
# ---------------------------------------------------------------------

def probe_surface_entry(entry_x, entry_y):
    """표면 바로 위(PROBE_START_Z)에서 출발해 2mm씩 내려가며 접촉 감지.
    임계값은 F_ENTRY_DELTA 고정 - 자동 상향 없음.
    반환: 성공 시 표면 지점 posx, 실패 시 -1"""

    movel(posx(entry_x, entry_y, PROBE_START_Z,
               ENTRY_TILT_RX, ENTRY_TILT_RY, ENTRY_TILT_RZ),
          vel=50, acc=50, ref=DR_BASE)

    capture_baseline("entry")

    z = PROBE_START_Z
    hit = 0
    i = 0

    while i < PROBE_MAX_STEPS:
        if z - PROBE_STEP < PROBE_MIN_Z:
            tp_log("!! 하한(" + str(PROBE_MIN_Z) + ") 도달 - 표면 감지 실패")
            emergency_retreat()
            return -1

        z = z - PROBE_STEP
        move_z_to(z, PROBE_VEL)
        wait(PROBE_WAIT)
        i = i + 1

        d = force_delta()
        tp_log("PROBE z=" + str(z) + " delta=" + str(d)
               + " th=" + str(F_ENTRY_DELTA) + " hit=" + str(hit))

        if d >= F_HARD_DELTA:
            tp_log("!! 탐색 중 하드리밋 초과")
            emergency_retreat()
            return -1

        if d >= F_ENTRY_DELTA:
            hit = hit + 1
        else:
            hit = 0

        if hit >= CONSEC_HIT_NEEDED:
            tp_log(">> 표면 확정 z=" + str(z) + " (step=" + str(i) + ")")
            return get_current_posx()[0]

    tp_log("!! 최대 스텝 도달 - 표면 감지 실패")
    emergency_retreat()
    return -1


# ---------------------------------------------------------------------
# 4. 추가 진입
# ---------------------------------------------------------------------

def enter_additional_depth():
    """3mm씩 내려가며 저항이 F_EXTRA_STOP_DELTA를 넘으면 정상 종료."""
    done = 0.0
    while done < ENTRY_EXTRA_DEPTH:
        dz = ENTRY_EXTRA_STEP
        if ENTRY_EXTRA_DEPTH - done < ENTRY_EXTRA_STEP:
            dz = ENTRY_EXTRA_DEPTH - done

        cur = get_current_posx()[0]
        if cur[2] - dz < PROBE_MIN_Z:
            tp_log(">> 추가진입 중단 - 하한 도달")
            break

        movel(posx(cur[0], cur[1], cur[2] - dz,
                   ENTRY_TILT_RX, ENTRY_TILT_RY, ENTRY_TILT_RZ),
              vel=PROBE_VEL, acc=PROBE_VEL, ref=DR_BASE)
        wait(0.2)
        done = done + dz

        d = force_delta()
        tp_log("EXTRA depth=" + str(done) + " delta=" + str(d))

        if d >= F_HARD_DELTA:
            tp_log("!! 추가진입 중 하드리밋 초과")
            emergency_retreat()
            return -1

        if d >= F_EXTRA_STOP_DELTA:
            tp_log(">> 추가진입 조기 종료 (저항 " + str(d)
                   + "N, 깊이 " + str(done) + "mm)")
            break

    return get_current_posx()[0]


# ---------------------------------------------------------------------
# 5. 수직화
# ---------------------------------------------------------------------

def get_vertical_orient_for_y(y):
    lane_idx = 0
    best_diff = None
    i = 0
    while i < len(LANE_DY_LIST):
        candidate_y = LANE_Y_CENTER + LANE_DY_LIST[i]
        diff = abs(candidate_y - y)
        if best_diff is None or diff < best_diff:
            best_diff = diff
            lane_idx = i
        i = i + 1

    if lane_idx in VERTICAL_ORIENT_BY_LANE:
        return VERTICAL_ORIENT_BY_LANE[lane_idx]
    return VERTICAL_ORIENT_DEFAULT


def straighten_to_vertical():
    """X,Y,Z는 유지하고 방향만 수직으로 전환. TCP 고정이라 벽 방향 이동 없음."""
    cur = get_current_posx()[0]
    vrx, vry, vrz = get_vertical_orient_for_y(cur[1])
    tp_log("수직화: Y=" + str(cur[1]) + " -> Ry " + str(vry))

    movel(posx(cur[0], cur[1], cur[2], vrx, vry, vrz),
          vel=VEL_STRAIGHTEN, acc=ACC_STRAIGHTEN, ref=DR_BASE)
    wait(0.2)

    capture_baseline("vertical")
    return get_current_posx()[0]


# ---------------------------------------------------------------------
# 6. J6 회전 성형
# ---------------------------------------------------------------------

def shape_sphere_j6():
    """J6 360도 회전으로 재료를 구체로 성형. J6는 툴 축 회전이라 TCP 불변."""
    tp_log(">> J6 360도 성형 회전 시작")
    movej(posj(0, 0, 0, 0, 0, J6_SHAPE_DEGREE),
          vel=VEL_J6_SHAPE, acc=ACC_J6_SHAPE, mod=DR_MV_MOD_REL)
    wait(0.2)

    if over_limit(F_HARD_DELTA_SHAPE, "post-shape") == 1:
        emergency_retreat()
        return -1

    tp_log(">> 성형 회전 완료")
    return 1


def unwind_j6():
    """배출 완료 후(스쿱이 빈 상태에서) J6 원위치 복구."""
    tp_log(">> J6 복구 회전 (-360도)")
    movej(posj(0, 0, 0, 0, 0, -J6_SHAPE_DEGREE),
          vel=VEL_J6_SHAPE, acc=ACC_J6_SHAPE, mod=DR_MV_MOD_REL)
    wait(0.2)


# ---------------------------------------------------------------------
# 7. 퍼올리기 - Ry 절대 목표 + Z 동시 상승  [R]
# ---------------------------------------------------------------------

def scoop_up_motion():
    """Rx, Rz는 그대로 두고 Ry만 SCOOP_UP_RY_TARGET(-110)까지 회전하면서
    동시에 Z를 SCOOP_UP_Z_RISE(60mm) 올린다.

    Z를 함께 올리는 이유: 수직 자세에서 Ry만 단독으로 돌리면 그 높이에서
    도달 가능 영역을 벗어나 [2.1206]이 난다(v3에서 -124.96에서 정지).
    상승과 함께 돌리면 해소되고, 통에서 빠져나오는 동작을 겸한다.

    X, Y는 고정 - 통 벽 방향 이동이 발생하지 않는다."""

    capture_baseline("scoopup-start")

    start = get_current_posx()[0]
    ry0 = start[4]
    d_ry = (SCOOP_UP_RY_TARGET - ry0) / SCOOP_UP_STEPS
    d_z  = SCOOP_UP_Z_RISE / SCOOP_UP_STEPS

    tp_log("퍼올리기: Ry " + str(ry0) + " -> " + str(SCOOP_UP_RY_TARGET)
           + " / Z +" + str(SCOOP_UP_Z_RISE) + "mm")

    i = 0
    while i < SCOOP_UP_STEPS:
        i = i + 1
        ry_t = ry0 + d_ry * i
        z_t  = start[2] + d_z * i

        movel(posx(start[0], start[1], z_t, start[3], ry_t, start[5]),
              vel=VEL_SCOOP_UP, acc=ACC_SCOOP_UP, ref=DR_BASE)
        wait(0.1)
        tp_log("SCOOPUP " + str(i) + "/" + str(SCOOP_UP_STEPS)
               + " Ry=" + str(ry_t) + " Z=" + str(z_t))

    end = get_current_posx()[0]
    tp_log("퍼올리기 완료 Ry=" + str(end[4]) + " Z=" + str(end[2])
           + " / XY이동량=" + str(abs(end[0] - start[0]))
           + ", " + str(abs(end[1] - start[1])))
    return end


# ---------------------------------------------------------------------
# 8. 상승 / 이동 / 배출
# ---------------------------------------------------------------------

def lift_after_shaping():
    safe_lift(LIFT_AFTER_SHAPE)


def move_to_pour_and_release():
    movejx(POUR_TARGET_POSX, vel=50, acc=50, sol=255)
    wait(0.3)
    # TODO: 실제 배출 동작 연결 (RG2 오픈 / 스쿱 기울이기 등 기존 메커니즘)
    tp_log(">> 배출 완료 (TODO: 실제 배출 로직 연결)")


def go_home():
    movejx(HOME_POSX, vel=40, acc=50, sol=255)


# ---------------------------------------------------------------------
# 9. 사이클
# ---------------------------------------------------------------------

def scoop_cycle_sphere(entry_x, entry_y):
    tp_log("===== 스쿱 사이클 시작 (x=" + str(entry_x) + ", y=" + str(entry_y) + ") =====")

    if probe_surface_entry(entry_x, entry_y) == -1:
        return 0
    if enter_additional_depth() == -1:
        return 0
    if straighten_to_vertical() == -1:
        return 0
    if shape_sphere_j6() == -1:
        return 0
    if scoop_up_motion() == -1:
        return 0

    lift_after_shaping()
    move_to_pour_and_release()
    unwind_j6()

    tp_log("===== 스쿱 사이클 완료 =====")
    return 1


def main_test(scoop_count=1):
    go_home()

    entry_points = get_entry_points(scoop_count)

    for i in range(scoop_count):
        ex, ey = entry_points[i]
        if scoop_cycle_sphere(ex, ey) == 0:
            tp_log("!! " + str(i + 1) + "번째 스쿱 실패 - 테스트 중단")
            break
        go_home()

    tp_log("테스트 종료")


main_test(1)
# main_test(3)
