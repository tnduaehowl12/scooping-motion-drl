# -*- coding: utf-8 -*-
# =====================================================================
# 아이스크림 스쿠핑 - 회전 성형(sphere) 방식 (v5)
#
# v4 -> v5 변경점 (2026-08-26)
#   [L] 2단 탐색 도입 - 탐색 시간 2분 -> 약 10초
#       v4는 175에서 1mm씩 0.8초 대기로 내려갔다. 표면이 130~140 부근이라
#       40스텝 x 0.95초 = 2분이 걸렸다. 전체 사이클 목표가 2분인데 탐색만
#       2분이면 성립하지 않는다.
#       -> 조대(5mm/0.2s)로 접근 후 5mm 되올라가서 정밀(1mm/0.35s) 확정.
#   [M] 임계값을 실측 기준으로 재설정
#       F_HARD_DELTA 5.0 -> 12.0
#         *** v4가 "고장난 것처럼" 보인 직접 원인 ***
#         바닥 접촉이 5~7N인데 하드리밋이 5.0이라, 표면을 감지하기 전에
#         비상후퇴가 먼저 터졌다. 그 뒤 safe_lift(80)이 25mm씩 4스텝
#         올라간 것이 "4번째 Z축 상승 후 정지"의 정체.
#       F_ENTRY_DELTA 자동 -> 2.0 고정 (실측: X/Z 2~3N에서 표면 접촉)
#       F_EXTRA_STOP_DELTA 3.5 -> 5.0 (통 바닥 9N보다 충분히 아래)
#   [N] 자동 캘리브레이션은 '하한 보정'으로만 축소
#       조대 탐색 첫 3스텝(확실한 허공)에서 노이즈를 재고,
#       노이즈가 크면 임계값을 그만큼 올린다. 작으면 2.0N 고정.
#   [O] scoop_up_motion() 회전 직전 baseline 재취득 (자세 변경 시 원칙 통일)
#
# 이전 이력 요약
#   [A~D] 힘 판정 전면 델타 기반화, baseline 다중 샘플
#   [C]   분할 상승 + movejx 복귀 (2.1206 NOT REACHABLE 대응)
#   [E]   퍼올리기 movej(J5) -> movel(자세 전용) (통 벽 충돌 원인)
#   [G]   추가진입 분할 + 힘 감시
#   [I]   퍼올리기 회전 방향 반전(+X, 종이컵 쪽) + norm_deg 래핑
#
# *** 실행 전 확인 ***
#   - 스쿱 장착 상태로 실행할 것 (TCP가 스쿱 끝 기준)
#   - 워크셀 툴 무게는 디폴트 유지. 스쿱 130g은 델타 방식에서 무시 가능.
# =====================================================================

# ---------------------------------------------------------------------
# 0. 파라미터
# ---------------------------------------------------------------------

HOME_POSX = posx(367.48, 3.07, 213.43, 89.68, 179.98, 90.19)

# --- 진입 자세 (실측값) ---
ENTRY_TILT_RX = 0.07
ENTRY_TILT_RY = -143.31
ENTRY_TILT_RZ = 1.13

# --- 탐색 시작/한계 (실측 근거) -------------------------------------
# 이전 실측: P_EN의 Z = 125.00 (재료 있을 때 진입 깊이)
#            통 중심 (374.51, -153.52), 통 바닥 접촉 = 9N
# 표면은 대략 130~145 사이. 175에서 시작하면 30~45mm 위 = 조대 6~9스텝.
PROBE_START_Z   = 175.0
PROBE_MIN_Z     = 118.0    # 이 아래로는 절대 내려가지 않음 (통 바닥 보호)

# --- 조대 탐색  [L] ---
COARSE_STEP        = 5.0
COARSE_WAIT        = 0.2
COARSE_VEL         = 30
NOISE_COARSE_STEPS = 3     # 이 구간은 확실한 허공 - 노이즈만 측정

# --- 정밀 탐색  [L] ---
FINE_BACKOFF      = 5.0    # 조대 감지 후 되올라갈 거리 (= COARSE_STEP)
FINE_STEP         = 1.0
FINE_WAIT         = 0.35
FINE_VEL          = 12
FINE_MAX_STEPS    = 10     # backoff 5mm + 여유
CONSEC_HIT_NEEDED = 2

# --- baseline ---
BASELINE_SETTLE_WAIT = 0.8
BASELINE_SAMPLES     = 5
BASELINE_SAMPLE_GAP  = 0.06

FORCE_READ_SAMPLES = 2     # 속도를 위해 3 -> 2
FORCE_READ_GAP     = 0.03

# --- 힘 임계값 (전부 baseline 대비 델타)  [M] ---
F_ENTRY_DELTA      = 2.0   # 표면 접촉 (실측: X/Z 2~3N)
F_HARD_DELTA       = 12.0  # 진입 중 비상정지 (통 바닥 9N 위)
F_HARD_DELTA_SHAPE = 12.0  # 회전/수직화/퍼올리기 중
F_EXTRA_STOP_DELTA = 5.0   # 추가진입 정상 종료 저항

NOISE_FLOOR_WARN   = 1.0   # [N] 노이즈가 이보다 크면 임계값을 노이즈x2로 상향
NOISE_MARGIN       = 2.0

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

# --- 퍼올리기 (자세 전용 회전, 종이컵 쪽 +X) ---
SCOOP_UP_RY_DELTA = -90.0   # -170 -> +100 (norm_deg로 래핑)
SCOOP_UP_STEP_DEG = 15.0
VEL_SCOOP_UP = 20
ACC_SCOOP_UP = 20

# --- 후퇴/상승 ---
RETREAT_X_REL    = 20.0
RETREAT_LIFT_REL = 80.0
LIFT_AFTER_SHAPE = 120.0
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

# --- 런타임 공유 값 (DRL에서 global 없이 쓰기 위해 1칸 리스트) ---
BASELINE     = [0.0]
ENTRY_THRESH = [F_ENTRY_DELTA]


# ---------------------------------------------------------------------
# 1. 유틸
# ---------------------------------------------------------------------

def norm_deg(a):
    """각도를 -180 ~ +180 범위로 정규화.
    Ry를 -170에서 -90도 더 돌리면 -260이 되는데 posx는 이 값을 못 받는다."""
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


def force_magnitude_xz(f):
    """X,Z 힘 합성 크기. (ROS2 이식 시 sqrt 대신 제곱값 비교로 변경할 것)"""
    return sqrt(f[0] * f[0] + f[2] * f[2])


def read_force_avg():
    """단발 읽기는 랜덤하게 튄다. 여러 번 평균."""
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
    """현재 힘의 baseline 대비 증가분."""
    return abs(read_force_avg() - BASELINE[0])


def over_limit(limit, tag):
    """델타가 limit을 넘으면 1."""
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
    """틸트 자세로 한 번에 크게 올리면 solution space 제약 때문에
    [2.1206] NOT REACHABLE이 난다. 25mm씩 나눠 올린다.
    (80mm면 4스텝 - 로그에 스텝 번호를 찍어 오해 없도록 함)"""
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
    tp_log("!! 비상후퇴 실행 - 이후 4스텝 상승은 정상 동작임")
    cur = get_current_posx()[0]
    movel(posx(cur[0] - RETREAT_X_REL, cur[1], cur[2] + 30.0,
               cur[3], cur[4], cur[5]),
          vel=VEL_LIFT, acc=ACC_LIFT, ref=DR_BASE)
    safe_lift(RETREAT_LIFT_REL)
    movejx(HOME_POSX, vel=20, acc=20, sol=255)
    tp_log("!! 비상후퇴 완료 - 홈 복귀")


# ---------------------------------------------------------------------
# 3. 표면 감지 - 2단 탐색  [L]
# ---------------------------------------------------------------------

def probe_surface_entry(entry_x, entry_y):
    """조대(5mm) 탐색으로 표면 근처까지 빠르게 내려간 뒤,
    5mm 되올라가서 정밀(1mm) 탐색으로 표면을 확정한다.

    v4는 175에서 1mm씩 0.8초 대기로만 내려가 2분이 걸렸다.
    이 구조로 약 10초로 줄어들면서 1mm 정밀도는 유지된다.

    반환: 성공 시 표면 지점 posx, 실패 시 -1"""

    movel(posx(entry_x, entry_y, PROBE_START_Z,
               ENTRY_TILT_RX, ENTRY_TILT_RY, ENTRY_TILT_RZ),
          vel=50, acc=50, ref=DR_BASE)

    capture_baseline("entry")
    ENTRY_THRESH[0] = F_ENTRY_DELTA

    # ---------- 1단: 조대 탐색 ----------
    z = PROBE_START_Z
    step = 0
    noise_max = 0.0
    coarse_hit_z = -1.0

    while z - COARSE_STEP >= PROBE_MIN_Z:
        z = z - COARSE_STEP
        move_z_to(z, COARSE_VEL)
        wait(COARSE_WAIT)
        step = step + 1

        d = force_delta()

        # 첫 몇 스텝은 확실한 허공 - 노이즈 바닥만 측정  [N]
        if step <= NOISE_COARSE_STEPS:
            if d > noise_max:
                noise_max = d
            tp_log("NOISE" + str(step) + " delta=" + str(d) + " z=" + str(z))

            if step == NOISE_COARSE_STEPS:
                if noise_max > NOISE_FLOOR_WARN:
                    th = noise_max * NOISE_MARGIN
                    if th > F_ENTRY_DELTA:
                        ENTRY_THRESH[0] = th
                        tp_log(">> 노이즈 큼(" + str(noise_max)
                               + "N) - 임계값 상향: " + str(th) + "N")
                tp_log(">> 감지 임계값 = " + str(ENTRY_THRESH[0])
                       + "N (노이즈최대 " + str(noise_max) + "N)")
            continue

        tp_log("COARSE z=" + str(z) + " delta=" + str(d)
               + " th=" + str(ENTRY_THRESH[0]))

        if d >= F_HARD_DELTA:
            tp_log("!! 조대 탐색 중 하드리밋 초과")
            emergency_retreat()
            return -1

        if d >= ENTRY_THRESH[0]:
            coarse_hit_z = z
            tp_log(">> 조대 접촉 신호 z=" + str(z))
            break

    if coarse_hit_z < 0:
        tp_log("!! 조대 탐색 실패 - 하한(" + str(PROBE_MIN_Z) + ") 도달")
        emergency_retreat()
        return -1

    # ---------- 되올라가기 ----------
    z = coarse_hit_z + FINE_BACKOFF
    move_z_to(z, COARSE_VEL)
    wait(0.3)
    tp_log(">> 정밀 탐색 시작 z=" + str(z))

    # ---------- 2단: 정밀 탐색 ----------
    hit = 0
    i = 0
    while i < FINE_MAX_STEPS:
        if z - FINE_STEP < PROBE_MIN_Z:
            tp_log("!! 정밀 탐색 중 하한 도달")
            emergency_retreat()
            return -1

        z = z - FINE_STEP
        move_z_to(z, FINE_VEL)
        wait(FINE_WAIT)
        i = i + 1

        d = force_delta()
        tp_log("FINE z=" + str(z) + " delta=" + str(d) + " hit=" + str(hit))

        if d >= F_HARD_DELTA:
            tp_log("!! 정밀 탐색 중 하드리밋 초과")
            emergency_retreat()
            return -1

        if d >= ENTRY_THRESH[0]:
            hit = hit + 1
        else:
            hit = 0

        if hit >= CONSEC_HIT_NEEDED:
            tp_log(">> 표면 확정 z=" + str(z))
            return get_current_posx()[0]

    # 정밀 구간에서 확정 못하면 조대 신호 지점을 표면으로 채택
    tp_log(">> 정밀 미확정 - 조대 지점(z=" + str(coarse_hit_z) + ")을 표면으로 채택")
    move_z_to(coarse_hit_z, FINE_VEL)
    wait(0.2)
    return get_current_posx()[0]


# ---------------------------------------------------------------------
# 4. 추가 진입
# ---------------------------------------------------------------------

def enter_additional_depth():
    """3mm씩 내려가며 저항이 F_EXTRA_STOP_DELTA를 넘으면 정상 종료.
    재료가 단단하면 얕게, 무르면 설정 깊이까지 자동 적응."""
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
              vel=FINE_VEL, acc=FINE_VEL, ref=DR_BASE)
        wait(0.25)
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
# 7. 퍼올리기 - 자세 전용 회전 (종이컵 쪽)
# ---------------------------------------------------------------------

def scoop_up_motion():
    """스쿱이 천장을 보도록 X,Y,Z 고정한 채 Ry만 회전.
    J5 movej는 TCP가 반경 210mm 호를 그려 통 벽에 충돌하므로 쓰지 않는다.
    방향: 수직(-170) -> +100 (종이컵이 놓인 +X 쪽)."""

    capture_baseline("scoopup-start")   # [O] 자세 변경 직전 기준 재취득

    start = get_current_posx()[0]
    start_ry = start[4]
    tp_log("퍼올리기: Ry=" + str(start_ry)
           + " -> " + str(norm_deg(start_ry + SCOOP_UP_RY_DELTA)))

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

        ry_target = norm_deg(start_ry + sign * done)
        cur = get_current_posx()[0]
        movel(posx(start[0], start[1], start[2], cur[3], ry_target, cur[5]),
              vel=VEL_SCOOP_UP, acc=ACC_SCOOP_UP, ref=DR_BASE)
        wait(0.1)
        tp_log("SCOOPUP " + str(done) + "/" + str(total) + " Ry=" + str(ry_target))

        if over_limit(F_HARD_DELTA_SHAPE, "scoopup") == 1:
            emergency_retreat()
            return -1

    end = get_current_posx()[0]
    tp_log("퍼올리기 완료 Ry=" + str(end[4])
           + " / XY이동량=" + str(abs(end[0] - start[0]))
           + ", " + str(abs(end[1] - start[1])))
    return end


# ---------------------------------------------------------------------
# 8. 상승 / 이동 / 배출
# ---------------------------------------------------------------------

def lift_after_shaping():
    safe_lift(LIFT_AFTER_SHAPE)


def move_to_pour_and_release():
    """붓기 위치로 이동 후 배출."""
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
