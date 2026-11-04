# -*- coding: utf-8 -*-
# =====================================================================
# 아이스크림 스쿠핑 - 회전 성형(sphere) 방식 (v3)
#
# v2 -> v3 변경점 (2026-08-26, [5.7060] 충돌 감지 / 과도 관입 대응)
#   [E] scoop_up_motion() 을 movej(J5 상대회전) -> movel(자세 전용 회전) 으로 교체
#       J5는 손목 관절이고 TCP가 210mm 떨어져 있어, 90도 회전 시 스쿱 끝이
#       반경 200mm+ 호를 그리며 통 벽으로 돌진했음 (충돌의 직접 원인).
#       movel 로 X,Y,Z 를 고정한 채 방향만 바꾸면 IK가 조인트를 알아서 푼다.
#   [F] PROBE_Z_FREEFALL_END 140 -> 175
#       140은 이미 표면에 닿는 높이여서 탐색 여유가 2mm뿐이었고,
#       그 상태에서 뜬 baseline이 접촉값으로 오염돼 델타 비교가 무의미해졌음.
#   [G] ENTRY_EXTRA_DEPTH 25 -> 12, 그리고 맹목적 1회 movel -> 3mm 분할 + 힘 감시
#   [H] F_ENTRY_DELTA 0.5 -> 1.2, CONSEC_HIT_NEEDED 2 -> 3 (step2 오감지 방지)
#
# v1 -> v2 변경점
#   [A] 힘 판정 전면 델타 기반화 (틸트 자세의 -10~-11N 편향 때문)
#   [B] baseline 정착 대기 + 다중 샘플 평균, 연속 히트 조건
#   [C] Cartesian 대량 상승 제거 -> 분할 상승 + movejx 복귀 (2.1206 대응)
#   [D] 성형/수직화/퍼올리기 힘 비교도 델타로 통일
# =====================================================================

# ---------------------------------------------------------------------
# 0. 파라미터
# ---------------------------------------------------------------------

# --- 홈 자세 (실측값) ---
HOME_POSX = posx(367.48, 3.07, 213.43, 89.68, 179.98, 90.19)

# --- 진입 자세 (실측값) ---
ENTRY_TILT_RX = 0.07
ENTRY_TILT_RY = -143.31
ENTRY_TILT_RZ = 1.13

# --- 표면 탐색 ---
PROBE_Z_FREEFALL_END = 175.0   # [F] 140 -> 175. 무접촉 상태에서 baseline을 떠야 함.
PROBE_Z_STEP         = 1.0     # mm
PROBE_Z_SETTLE_WAIT  = 0.8     # sec
PROBE_Z_MAX_DEPTH    = 60.0    # mm, 시작점이 높아진 만큼 여유도 확대

BASELINE_SETTLE_WAIT = 1.2     # sec
BASELINE_SAMPLES     = 5
BASELINE_SAMPLE_GAP  = 0.1     # sec

F_ENTRY_DELTA     = 1.2        # [H] 0.5 -> 1.2 (step2 오감지 방지)
CONSEC_HIT_NEEDED = 3          # [H] 2 -> 3

# --- 비상 하드리밋 (전부 델타 기준) ---
F_HARD_DELTA       = 5.0       # 진입/추가진입 중 비상정지
F_HARD_DELTA_SHAPE = 11.0      # 회전/수직화/퍼올리기 중 (관성 튐 감안)

# --- 추가 진입  [G] ---
ENTRY_EXTRA_DEPTH   = 12.0     # mm, 25 -> 12
ENTRY_EXTRA_STEP    = 3.0      # mm, 분할 단위
F_EXTRA_STOP_DELTA  = 3.5      # N, 이 이상 저항이 오르면 그 깊이에서 정지(정상 종료)

# --- 수직화 ---
VEL_STRAIGHTEN = 30
ACC_STRAIGHTEN = 30

# --- J6 회전 성형 ---
J6_SHAPE_DEGREE = 360
VEL_J6_SHAPE    = 60
ACC_J6_SHAPE    = 60

# --- 퍼올리기 (자세 전용 회전)  [E] ---
# 수직 자세는 Ry=-170 (툴 Z축이 거의 바닥을 향함).
# 여기서 Ry를 +90도 움직여 -80 으로 만들면 툴 축이 수평(-X 방향)으로 눕는다.
# 스쿱 컵이 천장이 아니라 바닥을 보면 SCOOP_UP_RY_DELTA 부호만 뒤집을 것.
# -80 방향은 통 벽(+X)에서 멀어지는 쪽이라 첫 시도로 더 안전하다.
SCOOP_UP_RY_DELTA = 90.0       # deg, -170 -> -80
SCOOP_UP_STEP_DEG = 15.0       # deg, 분할 단위 (도달 불가/충돌 조기 포착)
VEL_SCOOP_UP = 15
ACC_SCOOP_UP = 15

# --- 후퇴/상승 ---
RETREAT_X_REL    = 20.0
RETREAT_LIFT_REL = 80.0
LIFT_AFTER_SHAPE = 120.0
LIFT_STEP        = 25.0
VEL_LIFT = 30
ACC_LIFT = 30

# --- 붓기 위치 (실측값) ---
POUR_TARGET_POSX = posx(680.09, -238.02, 322.16, 0.42, 104.43, 1.08)

# --- 레인(진입 지점) ---
ENTRY_X       = 397.89
LANE_Y_CENTER = -131.04
LANE_DY_LIST  = [-35.0, 0.0, 35.0]

TUB_LENGTH_LONG = 180.0
COLUMN_SPACING  = 60.0
N_COLUMNS = int(round(TUB_LENGTH_LONG / COLUMN_SPACING))

# --- 레인별 수직 자세 (BASE 기준) ---
VERTICAL_ORIENT_BY_LANE = {
    0: (0.0, -170.0, 0.0),   # 실측 확인됨 (Y=-166.02)
    1: (0.0, -170.0, 0.0),   # TODO: 레인1(Y=-131.04) 실측 필요
    2: (0.0, -170.0, 0.0),   # TODO: 레인2(Y=-96.04)  실측 필요
}
VERTICAL_ORIENT_DEFAULT = (0.0, -170.0, 0.0)

# --- baseline 저장소 ---
# DRL에서 global 선언 없이 값을 공유하기 위해 1칸 리스트를 컨테이너로 사용.
BASELINE = [0.0]


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
            x = ENTRY_X + col * COLUMN_SPACING
            grid.append((x, y))

    points = []
    i = 0
    while len(points) < count:
        points.append(grid[i % len(grid)])
        i = i + 1
    return points


# ---------------------------------------------------------------------
# 1. 힘 유틸
# ---------------------------------------------------------------------

def force_magnitude_xz(f):
    """X,Z 힘 합성 크기. (ROS2 이식 시 sqrt 대신 제곱값 비교로 변경할 것)"""
    return sqrt(f[0] * f[0] + f[2] * f[2])


def capture_baseline(tag):
    """현재 자세에서 무접촉 기준 힘을 평균해 BASELINE[0]에 저장.
    반드시 접촉이 없는 상태에서 호출해야 한다."""
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
    """현재 힘의 baseline 대비 증가분."""
    return abs(force_magnitude_xz(get_tool_force()) - BASELINE[0])


def over_limit(limit, tag):
    """델타가 limit을 넘으면 1. (절대값 비교 금지)"""
    d = force_delta()
    if d >= limit:
        tp_log("!! 힘 초과 [" + tag + "] delta=" + str(d) + " limit=" + str(limit)
               + " base=" + str(BASELINE[0]))
        return 1
    return 0


# ---------------------------------------------------------------------
# 2. 안전 상승 / 비상후퇴
# ---------------------------------------------------------------------

def safe_lift(total_dz):
    """틸트 자세로 한 번에 크게 올리면 solution space 제약 때문에
    [2.1206] NOT REACHABLE이 난다. 작게 나눠 올린다."""
    done = 0.0
    while done < total_dz:
        dz = LIFT_STEP
        if total_dz - done < LIFT_STEP:
            dz = total_dz - done
        cur = get_current_posx()[0]
        movel(posx(cur[0], cur[1], cur[2] + dz, cur[3], cur[4], cur[5]),
              vel=VEL_LIFT, acc=ACC_LIFT, ref=DR_BASE)
        done = done + dz
    tp_log("상승 완료 +" + str(total_dz) + "mm")


def emergency_retreat():
    """통 밖 탈출 최소량만 Cartesian으로 처리하고,
    복귀는 movejx(sol 자동)에 맡겨 IK 실패를 회피."""
    tp_log("!! 비상후퇴 실행")
    cur = get_current_posx()[0]
    movel(posx(cur[0] - RETREAT_X_REL, cur[1], cur[2] + 30.0,
               cur[3], cur[4], cur[5]),
          vel=VEL_LIFT, acc=ACC_LIFT, ref=DR_BASE)
    safe_lift(RETREAT_LIFT_REL)
    movejx(HOME_POSX, vel=20, acc=20, sol=255)
    tp_log("!! 비상후퇴 완료 - 홈 복귀")


# ---------------------------------------------------------------------
# 3. 표면 감지 진입
# ---------------------------------------------------------------------

def probe_surface_entry(entry_x, entry_y):
    """자유낙하 후 1mm 스텝으로 내려가며 표면 접촉을 델타로 감지.
    반환: 성공 시 표면 감지 지점 posx, 실패 시 -1"""

    movel(posx(entry_x, entry_y, PROBE_Z_FREEFALL_END,
               ENTRY_TILT_RX, ENTRY_TILT_RY, ENTRY_TILT_RZ),
          vel=40, acc=40, ref=DR_BASE)

    capture_baseline("entry")   # [F] 이제 확실히 무접촉 높이

    cur_pos = get_current_posx()[0]
    found_pos = -1
    depth = 0.0
    step = 0
    hit = 0

    while depth < PROBE_Z_MAX_DEPTH:
        movel(posx(cur_pos[0], cur_pos[1], cur_pos[2] - PROBE_Z_STEP,
                   ENTRY_TILT_RX, ENTRY_TILT_RY, ENTRY_TILT_RZ),
              vel=15, acc=15, ref=DR_BASE)
        wait(PROBE_Z_SETTLE_WAIT)
        depth = depth + PROBE_Z_STEP
        step = step + 1

        if over_limit(F_HARD_DELTA, "probe") == 1:
            emergency_retreat()
            return -1

        d = force_delta()
        tp_log("PROBE_STEP" + str(step) + " delta=" + str(d)
               + " depth=" + str(depth) + " hit=" + str(hit))

        if d >= F_ENTRY_DELTA:
            hit = hit + 1
        else:
            hit = 0                      # 노이즈 1회는 리셋

        if hit >= CONSEC_HIT_NEEDED:
            tp_log(">> 표면 감지 (step=" + str(step) + ", depth=" + str(depth) + ")")
            found_pos = get_current_posx()[0]
            break

        cur_pos = get_current_posx()[0]

    if found_pos != -1:
        return found_pos

    tp_log("!! 표면 감지 실패 - 최대 깊이(" + str(PROBE_Z_MAX_DEPTH) + "mm) 도달")
    emergency_retreat()
    return -1


# ---------------------------------------------------------------------
# 4. 추가 진입 (분할 + 힘 감시)  [G]
# ---------------------------------------------------------------------

def enter_additional_depth():
    """표면 감지 지점에서 진입 자세를 유지한 채 추가 진입.
    v2처럼 25mm를 맹목적으로 밀어넣지 않고, 3mm씩 내려가며
    저항이 F_EXTRA_STOP_DELTA를 넘으면 그 깊이에서 '정상 종료'한다.
    (재료가 단단하면 얕게, 무르면 설정 깊이까지 - 자동 적응)"""
    if over_limit(F_HARD_DELTA, "pre-extra") == 1:
        emergency_retreat()
        return -1

    done = 0.0
    while done < ENTRY_EXTRA_DEPTH:
        dz = ENTRY_EXTRA_STEP
        if ENTRY_EXTRA_DEPTH - done < ENTRY_EXTRA_STEP:
            dz = ENTRY_EXTRA_DEPTH - done

        cur = get_current_posx()[0]
        movel(posx(cur[0], cur[1], cur[2] - dz,
                   ENTRY_TILT_RX, ENTRY_TILT_RY, ENTRY_TILT_RZ),
              vel=15, acc=15, ref=DR_BASE)
        wait(0.3)
        done = done + dz

        if over_limit(F_HARD_DELTA, "extra") == 1:
            emergency_retreat()
            return -1

        d = force_delta()
        tp_log("EXTRA depth=" + str(done) + " delta=" + str(d))

        if d >= F_EXTRA_STOP_DELTA:
            tp_log(">> 추가진입 조기 종료 (저항 " + str(d) + "N, 깊이 " + str(done) + "mm)")
            break

    return get_current_posx()[0]


# ---------------------------------------------------------------------
# 5. 수직화
# ---------------------------------------------------------------------

def get_vertical_orient_for_y(y):
    """현재 Y가 어느 레인인지 찾아 해당 레인의 수직 자세를 반환."""
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
    """X,Y,Z는 유지하고 방향만 수직으로 전환 (BASE 기준).
    TCP 위치가 고정되므로 통 벽 방향 이동이 발생하지 않는다."""
    if over_limit(F_HARD_DELTA_SHAPE, "pre-straighten") == 1:
        emergency_retreat()
        return -1

    cur = get_current_posx()[0]
    vrx, vry, vrz = get_vertical_orient_for_y(cur[1])
    tp_log("수직화: Y=" + str(cur[1]) + " -> ("
           + str(vrx) + ", " + str(vry) + ", " + str(vrz) + ")")

    movel(posx(cur[0], cur[1], cur[2], vrx, vry, vrz),
          vel=VEL_STRAIGHTEN, acc=ACC_STRAIGHTEN, ref=DR_BASE)
    wait(0.2)

    # 자세가 바뀌면 중력 편향도 바뀌므로 baseline 재취득
    capture_baseline("vertical")

    return get_current_posx()[0]


# ---------------------------------------------------------------------
# 6. J6 회전 성형
# ---------------------------------------------------------------------

def shape_sphere_j6():
    """J6 360도 단방향 회전으로 재료를 구체로 성형.
    J6는 툴 축 자체의 회전이라 TCP 위치가 바뀌지 않는다."""
    if over_limit(F_HARD_DELTA_SHAPE, "pre-shape") == 1:
        emergency_retreat()
        return -1

    tp_log(">> J6 360도 성형 회전 시작")
    movej(posj(0, 0, 0, 0, 0, J6_SHAPE_DEGREE),
          vel=VEL_J6_SHAPE, acc=ACC_J6_SHAPE, mod=DR_MV_MOD_REL)
    wait(0.2)

    if over_limit(F_HARD_DELTA_SHAPE, "post-shape") == 1:
        emergency_retreat()
        return -1

    return 1


def unwind_j6():
    """배출 완료 후(스쿱이 빈 상태에서) J6 원위치 복구."""
    tp_log(">> J6 복구 회전 (-360도)")
    movej(posj(0, 0, 0, 0, 0, -J6_SHAPE_DEGREE),
          vel=VEL_J6_SHAPE, acc=ACC_J6_SHAPE, mod=DR_MV_MOD_REL)
    wait(0.2)


# ---------------------------------------------------------------------
# 7. 퍼올리기 - 자세 전용 회전  [E]
# ---------------------------------------------------------------------

def scoop_up_motion():
    """스쿱이 천장을 보도록 방향만 바꾼다.

    *** v2에서 movej(J5 상대회전)를 쓴 것이 통 벽 충돌의 원인이었다. ***
    J5는 손목 관절이고 TCP는 210mm 떨어져 있어, J5를 90도 돌리면
    스쿱 끝이 반경 200mm+ 호를 그리며 X+ 방향으로 돌진한다.

    대신 movel로 X,Y,Z를 고정하고 Ry만 바꾸면 IK가 조인트를 알아서
    풀어주면서 TCP는 제자리를 지킨다. 15도씩 나눠 돌려서
    도달 불가나 저항 상승을 조기에 잡는다."""

    if over_limit(F_HARD_DELTA_SHAPE, "pre-scoopup") == 1:
        emergency_retreat()
        return -1

    start = get_current_posx()[0]
    tp_log("퍼올리기 시작: Ry=" + str(start[4])
           + " -> " + str(start[4] + SCOOP_UP_RY_DELTA)
           + " (X,Y,Z 고정: " + str(start[0]) + ", " + str(start[1]) + ", " + str(start[2]) + ")")

    done = 0.0
    total = abs(SCOOP_UP_RY_DELTA)
    if SCOOP_UP_RY_DELTA >= 0:
        sign = 1.0
    else:
        sign = -1.0

    while done < total:
        d_deg = SCOOP_UP_STEP_DEG
        if total - done < SCOOP_UP_STEP_DEG:
            d_deg = total - done

        cur = get_current_posx()[0]
        # X,Y,Z와 Rx,Rz는 그대로, Ry만 증분
        movel(posx(start[0], start[1], start[2],
                   cur[3], cur[4] + sign * d_deg, cur[5]),
              vel=VEL_SCOOP_UP, acc=ACC_SCOOP_UP, ref=DR_BASE)
        wait(0.15)
        done = done + d_deg

        if over_limit(F_HARD_DELTA_SHAPE, "scoopup") == 1:
            emergency_retreat()
            return -1

    end = get_current_posx()[0]
    tp_log("퍼올리기 완료: Ry=" + str(end[4])
           + " / XY이동량=" + str(abs(end[0] - start[0])) + ", " + str(abs(end[1] - start[1])))

    return end


# ---------------------------------------------------------------------
# 8. 상승 / 이동 / 배출
# ---------------------------------------------------------------------

def lift_after_shaping():
    safe_lift(LIFT_AFTER_SHAPE)


def move_to_pour_and_release():
    """붓기 위치로 이동 후 배출."""
    movejx(POUR_TARGET_POSX, vel=40, acc=40, sol=255)
    wait(0.3)
    # TODO: 실제 배출 동작 연결 (RG2 오픈 / 스쿱 기울이기 등 기존 메커니즘)
    tp_log(">> 배출 완료 (TODO: 실제 배출 로직 연결)")


def go_home():
    movejx(HOME_POSX, vel=31, acc=47, sol=255)


# ---------------------------------------------------------------------
# 9. 사이클 오케스트레이션
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
    """시연용 - scoop_count로 스쿱 수 지정. 진입 지점은 통 규격 기반 자동 생성."""
    go_home()

    entry_points = get_entry_points(scoop_count)

    for i in range(scoop_count):
        ex, ey = entry_points[i]
        if scoop_cycle_sphere(ex, ey) == 0:
            tp_log("!! " + str(i + 1) + "번째 스쿱 실패 - 테스트 중단")
            break
        go_home()

    tp_log("테스트 종료")


main_test(1)     # 검증용 1스쿱
# main_test(3)
# main_test(12)
