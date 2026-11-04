# -*- coding: utf-8 -*-
# =====================================================================
# 아이스크림 스쿠핑 - 회전 성형(sphere) 방식 (v2)
#
# v1 -> v2 변경점 (2026-08-26, [2.1206] NOT REACHABLE 에러 대응)
#   [A] emergency_check() 를 절대값 -> baseline 델타 기반으로 전면 교체
#       (틸트 자세에서 접촉 없이도 Fz -10~-11N 편향이 상시로 실려서
#        절대 12N 하드리밋은 시작부터 여유가 1~2N밖에 없었음)
#   [B] baseline 안정화: 자유낙하 감속 + 정착 대기 1.2s + 5회 평균
#       표면 감지에 연속 히트(CONSEC_HIT_NEEDED) 조건 추가 -> 1스텝 오감지 방지
#       PROBE_Z_MAX_DEPTH 12 -> 50mm (통 표면 편차보다 작았음)
#   [C] Cartesian 대량 상승 제거. 틸트 자세를 유지한 채 movel 로 +150mm 올리면
#       solution space 유지 제약 때문에 IK 해가 없어 NOT REACHABLE 발생.
#       -> 통 탈출 최소량만 movel(분할), 나머지는 movejx(sol 자동)로 홈 복귀
#   [D] 모든 힘 비교(성형/수직화/퍼올리기)도 델타 기준으로 통일
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
PROBE_Z_FREEFALL_END = 140.0   # 자유낙하 종료 Z (실측값)
PROBE_Z_STEP         = 1.0     # mm
PROBE_Z_SETTLE_WAIT  = 0.8     # sec  [B] 0.4 -> 0.8 (진동 잔류 제거)
PROBE_Z_MAX_DEPTH    = 50.0    # mm   [B] 12 -> 50 (표면 편차 커버)

BASELINE_SETTLE_WAIT = 1.2     # sec  [B] 자유낙하 직후 정착 대기
BASELINE_SAMPLES     = 5       #      [B] 평균 샘플 수
BASELINE_SAMPLE_GAP  = 0.1     # sec

F_ENTRY_DELTA     = 0.5        # N, baseline 대비 증가분 = 표면 접촉 판정
CONSEC_HIT_NEEDED = 2          # [B] 연속 N스텝 초과해야 진짜 표면으로 인정

# --- 비상 하드리밋 (전부 델타 기준) ---
F_HARD_DELTA       = 8.0       # [A] 진입/추가진입 중 비상정지 (절대 12N 폐기)
F_HARD_DELTA_SHAPE = 11.0      # [D] 회전/수직화 중 (관성 튐 감안해 여유)

ENTRY_EXTRA_DEPTH = 25.0       # mm, 표면 감지 후 추가 진입 깊이

# --- 수직화 ---
VEL_STRAIGHTEN = 30
ACC_STRAIGHTEN = 30

# --- J6 회전 성형 ---
J6_SHAPE_DEGREE = 360
VEL_J6_SHAPE    = 60
ACC_J6_SHAPE    = 60

# --- 퍼올리기 (J5 추가 회전으로 스쿱이 천장을 보게) ---
SCOOP_UP_J5_DELTA_DEG = -90.0   # 방향 반대면 부호만 뒤집을 것
VEL_SCOOP_UP = 20
ACC_SCOOP_UP = 30

# --- 후퇴/상승 ---  [C]
RETREAT_X_REL       = 20.0     # mm, 통 벽에서 물러남
RETREAT_LIFT_REL    = 80.0     # mm, 통 탈출에 필요한 최소 상승량
LIFT_AFTER_SHAPE    = 120.0    # mm, 성형 후 상승량
LIFT_STEP           = 25.0     # mm, 분할 상승 1스텝 (IK 실패 지점 조기 포착용)
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
    0: (0.0, -170.0, 0.0),   # 실측 확인됨 (Y=-166.04)
    1: (0.0, -170.0, 0.0),   # TODO: 레인1(Y=-131.04) 실측 필요
    2: (0.0, -170.0, 0.0),   # TODO: 레인2(Y=-96.04)  실측 필요
}
VERTICAL_ORIENT_DEFAULT = (0.0, -170.0, 0.0)


# --- baseline 저장소 ---------------------------------------------------
# DRL에서 global 선언 없이 안전하게 값을 공유하기 위해 1칸 리스트를 컨테이너로 사용.
# BASELINE[0] = 현재 사이클의 무접촉 기준 힘 크기
BASELINE = [0.0]


def get_entry_points(count):
    """통 규격(TUB_LENGTH_LONG, COLUMN_SPACING)과 레인 목록(LANE_DY_LIST)만으로
    필요한 개수(count)만큼의 진입 지점(x, y)을 자동 생성.
    레인 우선 + 레인 전환 시 지그재그(왕복)로 큰 이동 제거.
    격자를 다 쓰면 처음부터 순환 (Z는 매번 실시간 재탐색이라 안전)."""
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
    """X,Z 힘 합성 크기. 스쿱이 기울어져 진입하므로 접촉이 두 축 어디로도 나타남.
    (ROS2 이식 시 sqrt 대신 제곱값 비교로 변경할 것)"""
    return sqrt(f[0] * f[0] + f[2] * f[2])


def capture_baseline(tag):
    """현재 자세에서 무접촉 기준 힘을 여러 번 평균해 BASELINE[0]에 저장.
    이 자세는 접촉이 없어도 -10~-11N 편향이 실리므로, 반드시 실제
    작업 자세에서 잡은 뒤 델타로만 비교해야 한다."""
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
    """델타가 limit을 넘으면 1. (절대값 비교 금지 - v1 실패 원인)"""
    d = force_delta()
    if d >= limit:
        tp_log("!! 힘 초과 [" + tag + "] delta=" + str(d) + " limit=" + str(limit)
               + " base=" + str(BASELINE[0]))
        return 1
    return 0


# ---------------------------------------------------------------------
# 2. 안전 상승 / 비상후퇴   [C]
# ---------------------------------------------------------------------

def safe_lift(total_dz):
    """틸트 자세를 유지한 채 Cartesian으로 한 번에 크게 올리면
    solution space 유지 제약 때문에 [2.1206] NOT REACHABLE이 난다.
    작게 나눠서 올려 실패 지점을 최소화한다."""
    done = 0.0
    while done < total_dz:
        dz = LIFT_STEP
        if total_dz - done < LIFT_STEP:
            dz = total_dz - done
        cur = get_current_posx()[0]
        tgt = posx(cur[0], cur[1], cur[2] + dz, cur[3], cur[4], cur[5])
        movel(tgt, vel=VEL_LIFT, acc=ACC_LIFT, ref=DR_BASE)
        done = done + dz
    tp_log("상승 완료 +" + str(total_dz) + "mm")


def emergency_retreat():
    """어떤 실패 지점에서도 다음 동작 전 반드시 호출.
    통 밖으로 나올 최소량만 Cartesian으로 처리하고,
    나머지 복귀는 movejx(sol 자동)에 맡겨 IK 실패를 회피한다."""
    tp_log("!! 비상후퇴 실행")
    cur = get_current_posx()[0]
    movel(posx(cur[0] - RETREAT_X_REL, cur[1], cur[2] + 30.0,
               cur[3], cur[4], cur[5]),
          vel=VEL_LIFT, acc=ACC_LIFT, ref=DR_BASE)
    safe_lift(RETREAT_LIFT_REL)
    movejx(HOME_POSX, vel=20, acc=20, sol=255)
    tp_log("!! 비상후퇴 완료 - 홈 복귀")


# ---------------------------------------------------------------------
# 3. 표면 감지 진입 (Z축, X/Z 합성, 델타 기반)
# ---------------------------------------------------------------------

def probe_surface_entry(entry_x, entry_y):
    """자유낙하 후 1mm 스텝으로 내려가며 표면 접촉을 델타로 감지.
    반환: 성공 시 표면 감지 지점 posx, 실패 시 -1"""

    freefall_pos = posx(entry_x, entry_y, PROBE_Z_FREEFALL_END,
                        ENTRY_TILT_RX, ENTRY_TILT_RY, ENTRY_TILT_RZ)
    movel(freefall_pos, vel=40, acc=40, ref=DR_BASE)   # [B] 60 -> 40 (진동 감소)

    capture_baseline("entry")   # [B] 정착 대기 + 다중 샘플 평균

    cur_pos = get_current_posx()[0]
    found_pos = -1
    depth = 0.0
    step = 0
    hit = 0

    while depth < PROBE_Z_MAX_DEPTH:
        next_pos = posx(cur_pos[0], cur_pos[1], cur_pos[2] - PROBE_Z_STEP,
                        ENTRY_TILT_RX, ENTRY_TILT_RY, ENTRY_TILT_RZ)
        movel(next_pos, vel=15, acc=15, ref=DR_BASE)
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
            hit = 0                      # [B] 노이즈 1회는 리셋

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
# 4. 추가 진입 + 수직화
# ---------------------------------------------------------------------

def enter_additional_depth():
    """표면 감지 지점에서 진입 자세를 유지한 채 추가 진입."""
    if over_limit(F_HARD_DELTA, "pre-extra") == 1:
        emergency_retreat()
        return -1

    cur = get_current_posx()[0]
    target = posx(cur[0], cur[1], cur[2] - ENTRY_EXTRA_DEPTH,
                  ENTRY_TILT_RX, ENTRY_TILT_RY, ENTRY_TILT_RZ)
    movel(target, vel=20, acc=20, ref=DR_BASE)
    wait(0.3)

    if over_limit(F_HARD_DELTA, "post-extra") == 1:
        emergency_retreat()
        return -1

    return get_current_posx()[0]


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
    """X,Y,Z는 유지하고 방향만 수직으로 전환 (BASE 기준)."""
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

    # 자세가 바뀌면 중력 편향도 바뀌므로 baseline 재취득  [A]
    capture_baseline("vertical")

    return get_current_posx()[0]


# ---------------------------------------------------------------------
# 5. J6 회전 성형 (구체 만들기)
# ---------------------------------------------------------------------

def shape_sphere_j6():
    """J6 360도 단방향 회전으로 재료를 구체로 성형.
    회전 중에는 관성으로 힘값이 흔들리므로 델타 감시는 끄고
    회전 전/후 하드리밋 체크만 유지."""
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


def scoop_up_motion():
    """성형 직후 J5를 추가로 돌려 스쿱이 천장을 보게 만드는 '퍼올리기'.
    이 상태로 상승해야 구체가 스쿱에 담긴 채 올라온다."""
    if over_limit(F_HARD_DELTA_SHAPE, "pre-scoopup") == 1:
        emergency_retreat()
        return -1

    j_before = get_current_posj()
    tp_log("퍼올리기 전 J5=" + str(j_before[4]))

    movej(posj(0, 0, 0, 0, SCOOP_UP_J5_DELTA_DEG, 0),
          vel=VEL_SCOOP_UP, acc=ACC_SCOOP_UP, mod=DR_MV_MOD_REL)
    wait(0.2)

    j_after = get_current_posj()
    tp_log("퍼올리기 후 J5=" + str(j_after[4])
           + " (변화량=" + str(j_after[4] - j_before[4]) + ")")

    if over_limit(F_HARD_DELTA_SHAPE, "post-scoopup") == 1:
        emergency_retreat()
        return -1

    return get_current_posx()[0]


def unwind_j6():
    """배출 완료 후(스쿱이 빈 상태에서) J6 원위치 복구.
    구체를 흐트러뜨리지 않고 케이블 누적 감김도 방지."""
    tp_log(">> J6 복구 회전 (-360도)")
    movej(posj(0, 0, 0, 0, 0, -J6_SHAPE_DEGREE),
          vel=VEL_J6_SHAPE, acc=ACC_J6_SHAPE, mod=DR_MV_MOD_REL)
    wait(0.2)


# ---------------------------------------------------------------------
# 6. 상승 / 이동 / 배출
# ---------------------------------------------------------------------

def lift_after_shaping():
    """[C] 분할 상승. 한 번에 크게 올리지 않는다."""
    safe_lift(LIFT_AFTER_SHAPE)


def move_to_pour_and_release():
    """붓기 위치로 이동 후 배출."""
    movejx(POUR_TARGET_POSX, vel=40, acc=40, sol=255)   # [C] movel -> movejx
    wait(0.3)
    # TODO: 실제 배출 동작 연결 (RG2 오픈 / 스쿱 기울이기 등 기존 메커니즘)
    tp_log(">> 배출 완료 (TODO: 실제 배출 로직 연결)")


def go_home():
    movejx(HOME_POSX, vel=31, acc=47, sol=255)


# ---------------------------------------------------------------------
# 7. 사이클 오케스트레이션
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


def main_test(scoop_count=3):
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


main_test(3)
# main_test(12)
# main_test(20)
