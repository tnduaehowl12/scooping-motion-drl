# -*- coding: utf-8 -*-
# =====================================================================
# 아이스크림 스쿠핑 - 회전 성형(sphere) 방식 (v4)
#
# v3 -> v4 변경점 (2026-08-26)
#   [I] 퍼올리기 회전 방향 반전 + 각도 래핑 처리
#       v3는 Ry를 +방향(-170 -> -125)으로 돌려 종이컵 반대편(-X)으로 누웠고,
#       그 방향은 팔이 몸통으로 접히는 쪽이라 [2.1206] NOT REACHABLE 발생.
#       올바른 방향은 -170 -> -180(=+180) -> +170 -> ... -> +100 (종이컵 쪽, +X).
#       posx의 Ry는 +-180 범위라 매 스텝 norm_deg()로 정규화해야 한다.
#   [J] 표면 감지 임계값 자동 캘리브레이션
#       고정 임계값(0.5 -> 1.2)이 계속 노이즈에 걸려 2~4스텝에서 오감지했다.
#       자유낙하 직후 확실한 무접촉 구간(NOISE_PROBE_STEPS)에서 노이즈 바닥을
#       직접 측정하고, 그 최대값 x NOISE_MARGIN 을 실제 임계값으로 채택한다.
#   [K] 매 스텝 힘을 3회 평균해서 읽음 (단발 읽기의 랜덤 튐 제거)
#
# *** 실행 전 필수 확인 ***
#   1) 스쿱을 실제로 장착할 것. TCP는 플랜지 +210mm(스쿱 끝)에 정의되어 있어
#      스쿱이 없으면 접촉 자체가 물리적으로 불가능하고 모든 감지는 노이즈다.
#   2) Workcell Manager의 툴 무게/무게중심(payload)이 스쿱 장착 상태와
#      일치하는지 확인할 것. 어긋나면 중력보상이 틀어져 가짜 힘이 실린다.
#
# 이전 변경 이력
#   [A] 힘 판정 전면 델타 기반화 (틸트 자세의 -10~-11N 편향)
#   [B] baseline 정착 대기 + 다중 샘플 평균
#   [C] Cartesian 대량 상승 제거 -> 분할 상승 + movejx 복귀 (2.1206)
#   [D] 성형/수직화/퍼올리기 힘 비교 델타 통일
#   [E] 퍼올리기 movej(J5) -> movel(자세 전용) 교체 (통 벽 충돌 원인)
#   [F] 자유낙하 종료 140 -> 175 (무접촉 baseline 확보)
#   [G] 추가진입 25mm 맹목 -> 12mm 분할 + 힘 감시
# =====================================================================

# ---------------------------------------------------------------------
# 0. 파라미터
# ---------------------------------------------------------------------

HOME_POSX = posx(367.48, 3.07, 213.43, 89.68, 179.98, 90.19)

# --- 진입 자세 (실측값) ---
ENTRY_TILT_RX = 0.07
ENTRY_TILT_RY = -143.31
ENTRY_TILT_RZ = 1.13

# --- 표면 탐색 ---
PROBE_Z_FREEFALL_END = 175.0
PROBE_Z_STEP         = 1.0
PROBE_Z_SETTLE_WAIT  = 0.8
PROBE_Z_MAX_DEPTH    = 60.0

BASELINE_SETTLE_WAIT = 1.2
BASELINE_SAMPLES     = 5
BASELINE_SAMPLE_GAP  = 0.1

FORCE_READ_SAMPLES = 3       # [K] 매 스텝 힘 읽기 평균 횟수
FORCE_READ_GAP     = 0.05

# --- 임계값 자동 캘리브레이션  [J] ---
NOISE_PROBE_STEPS = 6        # 자유낙하 직후 감지를 끄고 노이즈만 재는 스텝 수
NOISE_MARGIN      = 2.5      # 임계값 = 관측 노이즈 최대치 x 이 배수
F_ENTRY_DELTA_MIN = 1.5      # 임계값 하한 (노이즈가 아주 작게 나와도 이 밑으로는 안 감)
F_ENTRY_DELTA_MAX = 6.0      # 임계값 상한 (이보다 크면 센서/payload 설정 의심)
CONSEC_HIT_NEEDED = 3

# --- 비상 하드리밋 (전부 델타 기준) ---
F_HARD_DELTA       = 5.0
F_HARD_DELTA_SHAPE = 11.0

# --- 추가 진입 ---
ENTRY_EXTRA_DEPTH   = 12.0
ENTRY_EXTRA_STEP    = 3.0
F_EXTRA_STOP_DELTA  = 3.5

# --- 수직화 ---
VEL_STRAIGHTEN = 30
ACC_STRAIGHTEN = 30

# --- J6 회전 성형 ---
J6_SHAPE_DEGREE = 360
VEL_J6_SHAPE    = 60
ACC_J6_SHAPE    = 60

# --- 퍼올리기 (자세 전용 회전)  [I] ---
# 수직 자세 Ry=-170 에서 종이컵이 놓인 +X 방향으로 눕혀야 한다.
# 부호가 음수 = -170 -> -180(=+180) -> +170 -> ... -> +100
# (v3의 +90은 정반대인 -X 방향이었고 도달 불가로 멈췄음)
SCOOP_UP_RY_DELTA = -90.0
SCOOP_UP_STEP_DEG = 15.0
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

VERTICAL_ORIENT_BY_LANE = {
    0: (0.0, -170.0, 0.0),   # 실측 확인됨 (Y=-166.02)
    1: (0.0, -170.0, 0.0),   # TODO: 레인1(Y=-131.04) 실측 필요
    2: (0.0, -170.0, 0.0),   # TODO: 레인2(Y=-96.04)  실측 필요
}
VERTICAL_ORIENT_DEFAULT = (0.0, -170.0, 0.0)

# --- 런타임 공유 값 (DRL에서 global 없이 쓰기 위해 1칸 리스트 사용) ---
BASELINE      = [0.0]    # 현재 자세의 무접촉 기준 힘
ENTRY_THRESH  = [0.0]    # [J] 이번 사이클에서 자동 결정된 표면 감지 임계값


# ---------------------------------------------------------------------
# 1. 각도 / 진입지점 유틸
# ---------------------------------------------------------------------

def norm_deg(a):
    """각도를 -180 ~ +180 범위로 정규화.  [I]
    Ry를 -170에서 -90도 더 돌리면 -260이 되는데, posx는 이 값을 받지 못한다.
    -260 -> +100 으로 접어줘야 한다."""
    while a > 180.0:
        a = a - 360.0
    while a < -180.0:
        a = a + 360.0
    return a


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


# ---------------------------------------------------------------------
# 2. 힘 유틸
# ---------------------------------------------------------------------

def force_magnitude_xz(f):
    """X,Z 힘 합성 크기. (ROS2 이식 시 sqrt 대신 제곱값 비교로 변경할 것)"""
    return sqrt(f[0] * f[0] + f[2] * f[2])


def read_force_avg():
    """[K] 단발 읽기는 랜덤하게 튄다. 여러 번 평균해서 반환."""
    s = 0.0
    k = 0
    while k < FORCE_READ_SAMPLES:
        s = s + force_magnitude_xz(get_tool_force())
        wait(FORCE_READ_GAP)
        k = k + 1
    return s / FORCE_READ_SAMPLES


def capture_baseline(tag):
    """현재 자세의 무접촉 기준 힘을 평균해 BASELINE[0]에 저장.
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
    """현재 힘의 baseline 대비 증가분 (평균 읽기 기반)."""
    return abs(read_force_avg() - BASELINE[0])


def over_limit(limit, tag):
    """델타가 limit을 넘으면 1. (절대값 비교 금지)"""
    d = force_delta()
    if d >= limit:
        tp_log("!! 힘 초과 [" + tag + "] delta=" + str(d) + " limit=" + str(limit)
               + " base=" + str(BASELINE[0]))
        return 1
    return 0


# ---------------------------------------------------------------------
# 3. 안전 상승 / 비상후퇴
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
# 4. 표면 감지 진입 (임계값 자동 캘리브레이션)  [J]
# ---------------------------------------------------------------------

def probe_surface_entry(entry_x, entry_y):
    """자유낙하 후 1mm 스텝으로 내려가며 표면 접촉을 감지.

    처음 NOISE_PROBE_STEPS 구간은 확실한 무접촉이므로 감지를 끄고
    노이즈 바닥만 측정한다. 그 최대치 x NOISE_MARGIN 을 임계값으로 채택해
    고정 임계값이 노이즈에 걸리던 문제를 없앤다.

    반환: 성공 시 표면 감지 지점 posx, 실패 시 -1"""

    movel(posx(entry_x, entry_y, PROBE_Z_FREEFALL_END,
               ENTRY_TILT_RX, ENTRY_TILT_RY, ENTRY_TILT_RZ),
          vel=40, acc=40, ref=DR_BASE)

    capture_baseline("entry")

    cur_pos = get_current_posx()[0]
    found_pos = -1
    depth = 0.0
    step = 0
    hit = 0
    noise_max = 0.0
    ENTRY_THRESH[0] = 0.0

    while depth < PROBE_Z_MAX_DEPTH:
        movel(posx(cur_pos[0], cur_pos[1], cur_pos[2] - PROBE_Z_STEP,
                   ENTRY_TILT_RX, ENTRY_TILT_RY, ENTRY_TILT_RZ),
              vel=15, acc=15, ref=DR_BASE)
        wait(PROBE_Z_SETTLE_WAIT)
        depth = depth + PROBE_Z_STEP
        step = step + 1

        d = force_delta()

        # --- 노이즈 측정 구간: 감지하지 않고 바닥값만 갱신 ---
        if step <= NOISE_PROBE_STEPS:
            if d > noise_max:
                noise_max = d
            tp_log("NOISE_STEP" + str(step) + " delta=" + str(d)
                   + " max=" + str(noise_max))

            if step == NOISE_PROBE_STEPS:
                th = noise_max * NOISE_MARGIN
                if th < F_ENTRY_DELTA_MIN:
                    th = F_ENTRY_DELTA_MIN
                if th > F_ENTRY_DELTA_MAX:
                    tp_log("!! 노이즈가 비정상적으로 큼(" + str(noise_max)
                           + "N) - payload/TCP 설정 확인 필요. 상한으로 클램프.")
                    th = F_ENTRY_DELTA_MAX
                ENTRY_THRESH[0] = th
                tp_log(">> 감지 임계값 자동 결정: " + str(th)
                       + "N (노이즈최대 " + str(noise_max) + "N x " + str(NOISE_MARGIN) + ")")

            cur_pos = get_current_posx()[0]
            continue

        # --- 실제 감지 구간 ---
        if over_limit(F_HARD_DELTA, "probe") == 1:
            emergency_retreat()
            return -1

        tp_log("PROBE_STEP" + str(step) + " delta=" + str(d)
               + " th=" + str(ENTRY_THRESH[0])
               + " depth=" + str(depth) + " hit=" + str(hit))

        if d >= ENTRY_THRESH[0]:
            hit = hit + 1
        else:
            hit = 0

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
# 5. 추가 진입 (분할 + 힘 감시)
# ---------------------------------------------------------------------

def enter_additional_depth():
    """3mm씩 내려가며 저항이 F_EXTRA_STOP_DELTA를 넘으면 정상 종료.
    재료가 단단하면 얕게, 무르면 설정 깊이까지 자동 적응."""
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

        d = force_delta()
        tp_log("EXTRA depth=" + str(done) + " delta=" + str(d))

        if d >= F_HARD_DELTA:
            tp_log("!! 추가진입 중 힘 초과 delta=" + str(d))
            emergency_retreat()
            return -1

        if d >= F_EXTRA_STOP_DELTA:
            tp_log(">> 추가진입 조기 종료 (저항 " + str(d) + "N, 깊이 " + str(done) + "mm)")
            break

    return get_current_posx()[0]


# ---------------------------------------------------------------------
# 6. 수직화
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
    """X,Y,Z는 유지하고 방향만 수직으로 전환. TCP 고정이라 벽 방향 이동 없음."""
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

    capture_baseline("vertical")   # 자세가 바뀌면 중력 편향도 바뀜
    return get_current_posx()[0]


# ---------------------------------------------------------------------
# 7. J6 회전 성형
# ---------------------------------------------------------------------

def shape_sphere_j6():
    """J6 360도 회전으로 재료를 구체로 성형. J6는 툴 축 회전이라 TCP 불변."""
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
# 8. 퍼올리기 - 자세 전용 회전 (종이컵 방향)  [I]
# ---------------------------------------------------------------------

def scoop_up_motion():
    """스쿱이 천장을 보도록 X,Y,Z 고정한 채 Ry만 회전.

    방향: 수직(-170)에서 종이컵이 놓인 +X 쪽으로 눕힌다.
      -170 -> -185(=+175) -> ... -> +100
    v3는 반대편(-X)으로 돌아 도달 불가로 멈췄다.
    각도가 +-180을 넘어가므로 매 스텝 norm_deg()로 정규화한다."""

    if over_limit(F_HARD_DELTA_SHAPE, "pre-scoopup") == 1:
        emergency_retreat()
        return -1

    start = get_current_posx()[0]
    start_ry = start[4]
    tp_log("퍼올리기 시작: Ry=" + str(start_ry)
           + " 목표=" + str(norm_deg(start_ry + SCOOP_UP_RY_DELTA))
           + " (X,Y,Z 고정: " + str(start[0]) + ", " + str(start[1]) + ", " + str(start[2]) + ")")

    total = abs(SCOOP_UP_RY_DELTA)
    if SCOOP_UP_RY_DELTA >= 0:
        sign = 1.0
    else:
        sign = -1.0

    done = 0.0
    while done < total:
        d_deg = SCOOP_UP_STEP_DEG
        if total - done < SCOOP_UP_STEP_DEG:
            d_deg = total - done
        done = done + d_deg

        # 누적 각도를 시작값 기준으로 계산 후 정규화 (오차 누적 방지)
        ry_target = norm_deg(start_ry + sign * done)

        cur = get_current_posx()[0]
        movel(posx(start[0], start[1], start[2], cur[3], ry_target, cur[5]),
              vel=VEL_SCOOP_UP, acc=ACC_SCOOP_UP, ref=DR_BASE)
        wait(0.15)
        tp_log("SCOOPUP " + str(done) + "/" + str(total) + "deg  Ry=" + str(ry_target))

        if over_limit(F_HARD_DELTA_SHAPE, "scoopup") == 1:
            emergency_retreat()
            return -1

    end = get_current_posx()[0]
    tp_log("퍼올리기 완료: Ry=" + str(end[4])
           + " / XY이동량=" + str(abs(end[0] - start[0]))
           + ", " + str(abs(end[1] - start[1])))

    return end


# ---------------------------------------------------------------------
# 9. 상승 / 이동 / 배출
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
# 10. 사이클 오케스트레이션
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
    """시연용 - scoop_count로 스쿱 수 지정."""
    go_home()

    entry_points = get_entry_points(scoop_count)

    for i in range(scoop_count):
        ex, ey = entry_points[i]
        if scoop_cycle_sphere(ex, ey) == 0:
            tp_log("!! " + str(i + 1) + "번째 스쿱 실패 - 테스트 중단")
            break
        go_home()

    tp_log("테스트 종료")


main_test(1)     # 검증용 1스쿱 - 반드시 스쿱 장착 상태로 실행할 것
# main_test(3)
# main_test(12)
