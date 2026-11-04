set_ref_coord(DR_BASE)
set_tcp("GripperDA_v1_A3")

VEL_X_NORMAL = [72, 36]
ACC_X_NORMAL = [120, 60]
VEL_X_SLOW = [27, 13]
ACC_X_SLOW = [55, 27]
VEL_J_NORMAL = 47
ACC_J_NORMAL = 70
VEL_J_SLOW = 31
ACC_J_SLOW = 47

DO_SCOOP_GRIP = 1
DO_GRIPPER_OPEN = 2
DO_CUP_GRIP = 3

SOL_SCOOP = 2

P_HOME = posx(367.48, 3.07, 213.43, 89.68, 179.98, 90.19)

P_SC_APP = posx(607.86, 115.34, 265.02, 62.45, 178.05, 63.29)
P_SC = posx(607.86, 115.34, 145.02, 62.45, 178.05, 63.29)
P_SC_UP = posx(607.86, 115.34, 245.02, 62.45, 178.05, 63.29)

P_EN_APP = posx(397.89, -131.04, 250.00, 0.07, -143.31, 1.13)

PROBE_Z_START = 160.00
PROBE_Z_FREEFALL_END = 140.00
PROBE_Z_FLOOR = 95.00
PROBE_Z_STEP = 1.0
PROBE_Z_SETTLE_WAIT = 0.4
PROBE_Z_MAX_DEPTH = 12.0
PROBE_SAFE_Z = 250.00
F_ENTRY_DELTA = 0.5

CURL_X_OFFSET = [19.49, 25.06, 30.63, 36.20, 41.76, 45.88, 50.00]
CURL_ORIENT = [
    (0.07, -143.31, 1.13),
    (-179.62, 173.74, -178.55),
    (-0.96, 177.56, 0.11),
    (-0.21, 168.87, 0.87),
    (-0.12, 160.17, 0.96),
    (-0.08, 151.48, 1.00),
    (-0.06, 145.06, 1.02),
    (-0.05, 138.65, 1.04),
]

STRAIGHT_EXTRA_STEP = 2.0
STRAIGHT_EXTRA_MAX = 30.0
STRAIGHT_SETTLE_WAIT = 0.6
F_STRAIGHT_DELTA = 5.0
F_STRAIGHT_EARLY_WARN = 10.0

SPEED_TRANSITION_FIRST = 5
SPEED_TRANSITION_FIRST_ACC = 3
SPEED_STRAIGHT = 10
SPEED_STRAIGHT_ACC = 6

LIFT_Z_TARGET_REL = 150.0
LIFT_X_RETREAT = 30.0

# -- 통 상단 안전 이동 높이: 예전에 320으로 벽을 넘어가는 것 검증됨, 그대로 유지 --
SAFE_TRAVEL_Z = 320.00

# -- 블렌딩 반경(mm): 이 지점에서 완전히 멈추지 않고 다음 이동으로 이어짐 --
# 실측 후 튜닝 필요 - 너무 크면 경로가 계획대로 안 갈 수 있고(총 이동거리의 절반 넘으면
# 자동 축소됨), 너무 작으면 블렌딩 효과가 거의 안 느껴짐. 30mm부터 테스트 권장.
BLEND_RADIUS_TRANSPORT = 30.0

P_POUR = posx(680.09, -238.02, 322.16, 0.42, 104.43, 1.08)

WAIT_SETTLE = 2.0
WAIT_DROP = 1.5
WAIT_TRANSITION_SETTLE = 1.5

F_HARD_LIMIT = 12.0
POUR_ANGLE = 120

SPEED_PUSH = 20
SPEED_PUSH_ACC = 15
SPEED_PROBE = 8
SPEED_PROBE_ACC = 6
SPEED_RETREAT = 60
SPEED_TRANSPORT_L = 94
SPEED_TRANSPORT_J = 62
SPEED_POUR_APPROACH = 66
SPEED_POUR_APPROACH_ACC = 44

CUP_FLIP_VELJ = 29
CUP_FLIP_ACCJ = 57


def grip_open_all():
    set_digital_output(DO_SCOOP_GRIP, OFF)
    set_digital_output(DO_CUP_GRIP, OFF)
    set_digital_output(DO_GRIPPER_OPEN, ON)
    wait(1.5)


def grip_scoop_close():
    set_digital_output(DO_GRIPPER_OPEN, OFF)
    wait(0.3)
    set_digital_output(DO_SCOOP_GRIP, ON)
    wait(1.5)


def force_base():
    return get_tool_force(ref=DR_BASE)


def emergency_check(f):
    if (f[0] <= -F_HARD_LIMIT or f[0] >= F_HARD_LIMIT or
            f[1] <= -F_HARD_LIMIT or f[1] >= F_HARD_LIMIT or
            f[2] <= -F_HARD_LIMIT or f[2] >= F_HARD_LIMIT):
        tp_log("!! 비상: 힘 한계(12N) 초과 " + str(f))
        return 1
    return 0


def emergency_retreat():
    cur = get_current_posx()[0]
    tp_log("긴급 후퇴 시작, 현재위치 X=" + str(cur[0]) + " Z=" + str(cur[2]))
    retreat_point = posx(cur[0] - LIFT_X_RETREAT, cur[1], cur[2] + LIFT_Z_TARGET_REL,
                         cur[3], cur[4], cur[5])
    set_velx(15, 10)
    movel(retreat_point)
    wait(0.3)
    tp_log("긴급 후퇴 완료")


def normalize_j6():
    j = get_current_posj()
    j6 = j[5]
    tp_log("정규화 전 J6=" + str(j6))

    while j6 > 180.0:
        set_velj(CUP_FLIP_VELJ)
        set_accj(CUP_FLIP_ACCJ)
        movej(posj(0, 0, 0, 0, 0, -360), mod=DR_MV_MOD_REL)
        wait(0.3)
        j = get_current_posj()
        j6 = j[5]

    while j6 < -180.0:
        set_velj(CUP_FLIP_VELJ)
        set_accj(CUP_FLIP_ACCJ)
        movej(posj(0, 0, 0, 0, 0, 360), mod=DR_MV_MOD_REL)
        wait(0.3)
        j = get_current_posj()
        j6 = j[5]

    tp_log("정규화 후 J6=" + str(j6))


def go_home():
    movejx(P_HOME, vel=VEL_J_SLOW, acc=ACC_J_SLOW, sol=255)
    wait(0.5)


def pick_scoop():
    set_velx(SPEED_TRANSPORT_L, SPEED_TRANSPORT_J)
    set_accx(100, 60)
    grip_open_all()

    movejx(P_SC_APP, sol=SOL_SCOOP)

    set_velx(27, 17)
    movel(P_SC)

    grip_scoop_close()
    if get_digital_input(1) == OFF:
        tp_log("!! 스쿱 파지 확인 신호 없음")

    movel(P_SC_UP)
    wait(0.3)


def return_scoop():
    set_velx(SPEED_TRANSPORT_L, SPEED_TRANSPORT_J)
    movejx(P_SC_APP, sol=SOL_SCOOP)
    set_velx(27, 17)
    movel(P_SC)
    grip_open_all()
    movel(P_SC_APP)


def probe_entry_z(x, y, a, b, c):
    # -- 수정된 부분: 예전에는 아래 2단계였음 --
    #   stage1 = posx(x, y, SAFE_TRAVEL_Z, P_RISE자세)   <- 중간 자세로 재자세
    #   stage2 = posx(x, y, PROBE_SAFE_Z, 진입자세)       <- 여기서 다시 진입자세로 재자세
    # 지금은 처음부터 진입자세(a,b,c)로 한 번에 향하는 단일 경유점 + 블렌딩으로 통합.
    via = posx(x, y, SAFE_TRAVEL_Z, a, b, c)
    app = posx(x, y, PROBE_Z_START, a, b, c)
    freefall_end = posx(x, y, PROBE_Z_FREEFALL_END, a, b, c)

    set_velx(SPEED_TRANSPORT_L, SPEED_TRANSPORT_J)
    movejx(via, sol=SOL_SCOOP, radius=BLEND_RADIUS_TRANSPORT)
    wait(0.1)

    set_velx(27, 17)
    movel(app, radius=BLEND_RADIUS_TRANSPORT / 2.0)
    wait(0.1)

    # -- 여기서부터는 힘을 측정해야 하니 블렌딩 없이 완전 정지 --
    set_velx(30, 20)
    movel(freefall_end)
    wait(0.15)

    f_baseline = force_base()
    baseline_fz = f_baseline[2]
    tp_log("FREEFALL_END 기준 Fz=" + str(baseline_fz))

    set_velx(SPEED_PROBE, SPEED_PROBE_ACC)

    found_z = -1
    z = PROBE_Z_FREEFALL_END
    depth = 0.0
    while depth < PROBE_Z_MAX_DEPTH:
        z = z - PROBE_Z_STEP
        depth = depth + PROBE_Z_STEP
        target = posx(x, y, z, a, b, c)
        movel(target)
        wait(PROBE_Z_SETTLE_WAIT)

        f = force_base()
        delta = f[2] - baseline_fz
        if delta < 0:
            delta = -delta
        tp_log("PROBE_Z z=" + str(z) + " Fz=" + str(f[2]) + " delta=" + str(delta) + " depth=" + str(depth))

        if emergency_check(f) == 1:
            emergency_retreat()
            return -1

        if delta >= F_ENTRY_DELTA:
            found_z = z
            break

    if found_z == -1:
        tp_log("!! Z 진입 탐색 실패 - 최대 깊이(12mm)까지 델타 미도달")
        return -1

    tp_log("진입 깊이 확정: z=" + str(found_z))
    return found_z


def scoop_one_cycle(pour_target):
    x = P_EN_APP[0]
    y = P_EN_APP[1]
    a0, b0, c0 = CURL_ORIENT[0]

    found_z = probe_entry_z(x, y, a0, b0, c0)
    if found_z == -1:
        return -1

    set_velx(SPEED_PUSH, SPEED_PUSH_ACC)

    last_x = x
    last_orient = CURL_ORIENT[0]

    k = 0
    while k < 7:
        off = CURL_X_OFFSET[k]
        a, b, c = CURL_ORIENT[k + 1]
        target = posx(x + off, y, found_z, a, b, c)
        movel(target)
        wait(0.1)

        f = force_base()
        tp_log("CURL_STEP" + str(k + 1) + " (감시안함) Fx=" + str(f[0]))

        if emergency_check(f) == 1:
            emergency_retreat()
            return 0

        last_x = x + off
        last_orient = (a, b, c)
        k = k + 1

    wait(WAIT_TRANSITION_SETTLE)
    f_transition = force_base()
    tp_log("전환 안정화 후 Fx=" + str(f_transition[0]) + " Fz=" + str(f_transition[2]))

    if emergency_check(f_transition) == 1:
        tp_log("!! 전환 시점 이미 힘 초과 - 중단")
        emergency_retreat()
        return 0

    baseline_fx = f_transition[0]
    a, b, c = last_orient

    set_velx(SPEED_TRANSITION_FIRST, SPEED_TRANSITION_FIRST_ACC)
    first_straight_x = last_x + STRAIGHT_EXTRA_STEP
    target = posx(first_straight_x, y, found_z, a, b, c)
    movel(target)
    wait(STRAIGHT_SETTLE_WAIT)

    f_first = force_base()
    tp_log("직선 첫스텝 도달 Fx=" + str(f_first[0]) + " Fz=" + str(f_first[2]))

    if emergency_check(f_first) == 1:
        tp_log("!! 직선 첫 스텝에서 힘 초과 - 즉시 중단")
        emergency_retreat()
        return 0

    last_x = first_straight_x
    wall_hit = 0

    delta0 = f_first[0] - baseline_fx
    if delta0 < 0:
        delta0 = -delta0
    if delta0 >= F_STRAIGHT_DELTA:
        wall_hit = 1
        tp_log("첫 스텝에서 벽 접촉 감지")

    if wall_hit == 0:
        set_velx(SPEED_STRAIGHT, SPEED_STRAIGHT_ACC)
        ex = STRAIGHT_EXTRA_STEP
        while ex < STRAIGHT_EXTRA_MAX:
            ex = ex + STRAIGHT_EXTRA_STEP
            target = posx(x + CURL_X_OFFSET[6] + ex, y, found_z, a, b, c)
            movel(target)
            wait(STRAIGHT_SETTLE_WAIT)

            f = force_base()
            delta = f[0] - baseline_fx
            if delta < 0:
                delta = -delta

            fx_abs = f[0]
            if fx_abs < 0:
                fx_abs = -fx_abs

            tp_log("STRAIGHT_STEP Fx=" + str(f[0]) + " delta=" + str(delta))

            if emergency_check(f) == 1:
                emergency_retreat()
                return 0

            if fx_abs >= F_STRAIGHT_EARLY_WARN:
                wall_hit = 1
                tp_log("!! 조기 경보(10N) 발동 - 즉시 정지")
                break

            last_x = x + CURL_X_OFFSET[6] + ex

            if delta >= F_STRAIGHT_DELTA:
                wall_hit = 1
                tp_log("벽 접촉 감지 (delta 기준)")
                break

    if wall_hit == 0:
        tp_log("!! 직선 구간에서 벽 미감지 - 연장 한계 도달")

    retreat_point = posx(last_x - LIFT_X_RETREAT, y, found_z, a, b, c)
    set_velx(25, 15)
    movel(retreat_point)
    wait(0.2)

    lift_point = posx(last_x - LIFT_X_RETREAT, y, found_z + LIFT_Z_TARGET_REL, a, b, c)
    set_velx(25, 15)
    movel(lift_point)
    wait(0.3)

    wait(WAIT_SETTLE)

    set_velx(SPEED_POUR_APPROACH, SPEED_POUR_APPROACH_ACC)
    movel(pour_target)
    wait(0.3)

    set_velj(90)
    set_accj(120)
    movej(posj(0, 0, 0, 0, 0, -POUR_ANGLE), mod=DR_MV_MOD_REL)
    wait(WAIT_DROP)

    set_velj(30)
    set_accj(60)
    movej(posj(0, 0, 0, 0, 0, POUR_ANGLE), mod=DR_MV_MOD_REL)

    return 1


wait(3.0)

go_home()
normalize_j6()

pick_scoop()

result = scoop_one_cycle(P_POUR)
if result == -1:
    tp_log("!! 진입/잔량 문제 - 중단")
elif result == 0:
    tp_log("!! 스쿠핑 실패 - 중단")

return_scoop()

go_home()
normalize_j6()

tp_log("=== 전체 사이클 테스트 종료 ===")
