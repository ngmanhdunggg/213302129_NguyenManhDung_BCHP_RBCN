clear; clc; close all;
% 1. KHAI BÁO ROBOT
[my_Robot, geom, q_home] = create_robot();
% 2. CẤU HÌNH CHUNG
FPS_PLOT = 60;
dt       = 1 / FPS_PLOT;
% Cấu hình mặc định
cfg.mode_default   = 'J';    % nếu waypoint không ghi mode thì dùng cái này
cfg.v_lin_default  = 300;    % mm/s cho MoveL
cfg.a_lin_default  = 1200;   % mm/s^2 cho MoveL
cfg.vJ_default     = 0.8;    % rad/s cho MoveJ
cfg.aJ_default     = 2.0;    % rad/s^2 cho MoveJ
cfg.T_wait         = 0.5;    % dừng 0.5 s ở mỗi điểm
cfg.T_min_L        = 0.3;    % Tmin cho MoveL
cfg.T_min_J        = 0.3;    % Tmin cho MoveJ
% 3. TẠO DANH SÁCH CÁC ĐIỂM
center_x = 300; 
center_y = 0; 
z_draw   = 300;       
side     = 150;

P0 = [350, 0, 400];          
P1 = [center_x - side/2, center_y - side, z_draw];
P2 = [center_x + side/2, center_y - side, z_draw];
P3 = [center_x + side/2, center_y + side, z_draw];
P4 = [center_x - side/2, center_y + side, z_draw];                   
% GOTO có thể nhận:
%   - 'HOME', [],   [],   ''  -> về q_home
%   - [x y z], v, a, mode -> đi tới XYZ
%   mode: 'L' (MoveL mm/s), 'J' (MoveJ rad/s)
GOTO = {
    'HOME', [],   [],   ''; 
    P0,     2,    0.8,  'J';  
    P1,     300,  800,  'L'; 
    P2,     300,  800,  'L';
    P3,     300,  800,  'L'; 
    P4,     300,  800,  'L';   
    P0,     300,  800,  'L'; 
    'HOME', [],   [],   '';  
};
% ====== CHUYỂN GOTO -> WP (hỗ trợ 'HOME') ======
n_cmd   = size(GOTO, 1);
WP_cell = {};
for i = 1:n_cmd
    dest = GOTO{i,1};
    v    = GOTO{i,2};
    a    = GOTO{i,3};
    mode = GOTO{i,4};
    if ischar(dest) || isstring(dest)
        % Dạng 'HOME'
        wp.pos  = [];          % không dùng pos trong HOME
        wp.v    = v;
        wp.a    = a;
        wp.mode = 'HOME';      % đánh dấu đặc biệt
        WP_cell{end+1} = wp;
    else
        % Dạng [x y z]
        WP_cell{end+1} = make_wp(dest(1), dest(2), dest(3), v, a, mode); 
    end
end
if isempty(WP_cell)
    error('Khong co waypoint nao hop le trong GOTO.');
end
WP = [WP_cell{:}];
% 4. LẬP QUỸ ĐẠO CHO CẢ ĐƯỜNG ĐI (HỖ TRỢ HOME) + CHECK TCP
[q_traj, qd_traj, qdd_traj] = plan_path_with_home_wp(WP, q_home, cfg, geom, dt, my_Robot);
% 5. MÔ PHỎNG
if size(q_traj,1) <= 1
    disp('[THONG BAO] Khong co quy dao hop le de mo phong.');
else
    figure(1); clf;
    view(45, 20);
    w = [-1000 1000 -1000 1000 0 1300]; 
    my_Robot.plot(q_traj, 'workspace', w, 'trail', 'r-', 'fps', FPS_PLOT, 'noshadow');
end
% =====================================================================
% TẠO ROBOT VÀ THAM SỐ HÌNH HỌC
% =====================================================================
function [my_Robot, geom, q_home] = create_robot()
    LIMITS_DEG = [-170, 170; -170, 170; -170, 170; -170, 170; -170, 170; -170, 170];
    LIMITS_RAD = deg2rad(LIMITS_DEG); 
    a1 = 0;   a2 = 250; a3 = 200; a4 = 150; a5 = 0; a6 = 0;
    d1 = 300; d2 = 0;   d3 = 0;   d4 = 0;   d5 = 0; d6 = 220;
    L(1) = Link([0, d1, a1,  pi/2, 0,     0   ],  'qlim', LIMITS_RAD(1,:));
    L(2) = Link([0, d2, a2,  0,    0,     pi/2],  'qlim', LIMITS_RAD(2,:));
    L(3) = Link([0, d3, a3,  0,    0,     0   ],  'qlim', LIMITS_RAD(3,:));
    L(4) = Link([0, d4, a4,  0,    0,     0   ],  'qlim', LIMITS_RAD(4,:));
    L(5) = Link([0, d5, a5, -pi/2, 0,    -pi/2],  'qlim', LIMITS_RAD(5,:)); 
    L(6) = Link([0, d6, a6,  0,    0,     0   ],  'qlim', LIMITS_RAD(6,:));
    my_Robot = SerialLink(L, 'name', 'MyRobot');
    q_home   = [0 0 0 0 0 0];
    % struct tham số hình học để truyền cho các hàm khác
    geom.a2 = a2;
    geom.a3 = a3;
    geom.a4 = a4;
    geom.d1 = d1;
    geom.d6 = d6;
end
% =====================================================================
% TẠO 1 WAYPOINT: pos + vận tốc + gia tốc + mode
% =====================================================================
function wp = make_wp(px, py, pz, v, a, mode)
    if nargin < 6, mode = ''; end
    if nargin < 5, a = [];   end
    if nargin < 4, v = [];   end
    wp.pos  = [px, py, pz];
    wp.v    = v;
    wp.a    = a;
    wp.mode = upper(mode);
end
% =====================================================================
% TÁCH GIÁ TRỊ TỪ 1 WAYPOINT (CHO CẢ HOME)
% =====================================================================
function [P, v, a, mode] = unpack_wp(wp, cfg)
    % mode
    if isfield(wp, 'mode') && ~isempty(wp.mode)
        mode = upper(wp.mode);
    else
        mode = upper(cfg.mode_default);
    end
    % pos (cho phép rỗng nếu là HOME)
    if isfield(wp, 'pos') && ~isempty(wp.pos)
        P = wp.pos;
    else
        P = [];
    end
    % vận tốc
    if isfield(wp, 'v') && ~isempty(wp.v)
        v = wp.v;
    else
        if mode == 'L'
            v = cfg.v_lin_default;
        else
            v = cfg.vJ_default;
        end
    end
    % gia tốc
    if isfield(wp, 'a') && ~isempty(wp.a)
        a = wp.a;
    else
        if mode == 'L'
            a = cfg.a_lin_default;
        else
            a = cfg.aJ_default;
        end
    end
end
% =====================================================================
% LẬP QUỸ ĐẠO QUA LIST WAYPOINT (CÓ HOME) + CHECK TCP
% =====================================================================
function [q_traj, qd_traj, qdd_traj] = plan_path_with_home_wp(WP, q_home, cfg, geom, dt, my_Robot)
    n_wp = numel(WP);
    if n_wp < 2
        error('Can it nhat 2 waypoint.');
    end
    TOL_POS = 5;   % mm cho sai số TCP
    q_task = [];
    for i = 1:(n_wp - 1)
        wp_start = WP(i);
        wp_end   = WP(i+1);
        [P_start, ~,      ~,      mode_start] = unpack_wp(wp_start, cfg);
        [P_end,   v_end,  a_end,  mode_end]   = unpack_wp(wp_end,   cfg);
        mode_start = upper(mode_start);
        mode_end   = upper(mode_end);
        is_home_start = strcmp(mode_start, 'HOME');
        is_home_end   = strcmp(mode_end,   'HOME');
        ok_seg  = true;
        pos_err = 0;
        q_seg   = [];
        qd_seg  = [];
        qdd_seg = [];
        % ===== CÁC TRƯỜNG HỢP ĐẶC BIỆT CÓ HOME =====
        if is_home_start && is_home_end
            % HOME -> HOME: đứng im tại q_home
            N_WAIT = round(cfg.T_wait / dt);
            q_seg  = repmat(q_home, N_WAIT, 1);
        elseif ~is_home_start && is_home_end
            % P_start -> HOME (joint-space)
            q_start = ik_point(P_start, geom);
            if any(isnan(q_start)) || ~isreal(q_start)
                ok_seg  = false;
                pos_err = 1e6;
            else
                q_seg = scurve_joint(q_start, q_home, v_end, a_end, dt, cfg.T_min_J);
                [ok_seg, pos_err] = check_forbidden_qtraj(q_seg, my_Robot);
            end
        elseif is_home_start && ~is_home_end
            % HOME -> P_end (joint-space)
            q_start = q_home;
            q_end   = ik_point(P_end, geom);
            if any(isnan(q_end)) || ~isreal(q_end)
                ok_seg  = false;
                pos_err = 1e6;
            else
                q_seg = scurve_joint(q_start, q_end, v_end, a_end, dt, cfg.T_min_J);
                [ok_seg, pos_err] = check_forbidden_qtraj(q_seg, my_Robot);
            end
        else
            % ===== CẢ HAI ĐỀU LÀ ĐIỂM THƯỜNG =====
            mode_seg = upper(mode_end);
            if mode_seg == 'L'
                [q_seg, qd_seg, qdd_seg, ok_seg, pos_err] = move_segment(P_start, P_end, 'L', ...
                                             v_end, a_end, cfg.T_min_L, ...
                                             geom, dt, my_Robot, TOL_POS);
            else
                [q_seg, qd_seg, qdd_seg, ok_seg, pos_err] = move_segment(P_start, P_end, 'J', ...
                                             v_end, a_end, cfg.T_min_J, ...
                                             geom, dt, my_Robot, TOL_POS);
            end
        end
        % ===== XỬ LÝ NẾU ĐOẠN NÀY LỖI =====
        if ~ok_seg
            if pos_err < 0
                fprintf('[THONG BAO] Khong the di chuyen toi diem P%d (TCP nam trong vung cam hoac z<0).\n', i+1);
            else
                fprintf('[THONG BAO] Khong the di chuyen toi diem P%d (sai so hoac workspace, pos_err = %.2f).\n', ...
                        i+1, pos_err);
            end
            if isempty(q_task)
                q_traj  = q_home;
                qd_traj = zeros(1, numel(q_home));
                qdd_traj= zeros(1, numel(q_home));
            else
                q_traj = q_task;
                [qd_traj, qdd_traj] = compute_vel_acc(q_traj, dt);
            end
            return;
        end
        % Đoạn OK -> ghép vào quỹ đạo tổng
        q_task = [q_task; q_seg];
        % Dừng tại điểm đích
        N_WAIT = round(cfg.T_wait / dt);
        q_end_pose = q_seg(end,:);
        q_task = [q_task; repmat(q_end_pose, N_WAIT, 1)]; %#ok<AGROW>
    end
    if isempty(q_task)
        q_traj  = q_home;
        qd_traj = zeros(1, numel(q_home));
        qdd_traj= zeros(1, numel(q_home));
        return;
    end
    q_traj = q_task;
    [qd_traj, qdd_traj] = compute_vel_acc(q_traj, dt);
end
% =====================================================================
% DI CHUYỂN 1 ĐOẠN TỪ P_start -> P_end + CHECK TCP + CHECK ĐOẠN THẲNG XY
% =====================================================================
function [q_traj, qd_traj, qdd_traj, ok, pos_err] = move_segment(P_start, P_end, mode, ...
                                                   v_max, a_max, ...
                                                   T_min, geom, dt, ...
                                                   my_Robot, TOL_POS)
    mode    = upper(mode);
    ok      = true;
    pos_err = 0;
    qd_traj  = [];
    qdd_traj = [];
    if mode == 'L'
        % ========== MoveL ==========
        dP   = P_end - P_start;
        Lseg = norm(dP);
        % Chỉ check đường thẳng nếu z thấp
        need_line_check = (min(P_start(3), P_end(3)) < 350);
        if need_line_check
            if segment_hits_xy_box(P_start, P_end, -50, 50, -50, 50)
                ok      = false;
                pos_err = -1;    % vùng cấm XY
                q_traj  = [];
                return;
            end
        end
        if Lseg < 1e-9
            % Đoạn rất ngắn -> 1 điểm
            q_traj = ik_point(P_start, geom);
            if any(isnan(q_traj)) || ~isreal(q_traj)
                ok      = false;
                pos_err = 1e6;
                q_traj  = [];
                return;
            end
            % Check TCP
            T_now = my_Robot.fkine(q_traj);
            P_now = transl(T_now);
            if is_forbidden_tcp(P_now(1:3).')
                ok      = false;
                pos_err = -1;
                q_traj  = [];
                return;
            end
            [qd_traj, qdd_traj] = compute_vel_acc(q_traj, dt);
        else
            % Profile S-curve dùng chung
            s = scurve_quintic(Lseg, v_max, a_max, dt, T_min);
            q_traj = [];
            for k = 1:length(s)
                u = s(k);
                P = P_start + u * dP;
                q_temp = ik_point(P, geom);
                if any(isnan(q_temp)) || ~isreal(q_temp)
                    ok      = false;
                    pos_err = 1e6;
                    q_traj  = [];
                    return;
                end
                T_now = my_Robot.fkine(q_temp);
                P_now = transl(T_now);
                if is_forbidden_tcp(P_now(1:3).')
                    ok      = false;
                    pos_err = -1;
                    q_traj  = [];
                    return;
                end
                q_traj = [q_traj; q_temp]; %#ok<AGROW>
            end
            [qd_traj, qdd_traj] = compute_vel_acc(q_traj, dt);
        end
    else
        % ========== MoveJ ==========
        q_start = ik_point(P_start, geom);
        q_end   = ik_point(P_end,   geom);
        if any(isnan(q_start)) || any(isnan(q_end)) || ~isreal(q_start)   || ~isreal(q_end)
            ok      = false;
            pos_err = 1e6;
            q_traj  = [];
            return;
        end
        q_traj = scurve_joint(q_start, q_end, v_max, a_max, dt, T_min);
        [ok, pos_err] = check_forbidden_qtraj(q_traj, my_Robot);
        [qd_traj, qdd_traj] = compute_vel_acc(q_traj, dt);
    end
    % ===== CHECK ĐỘ LỆCH CUỐI =====
    if ~isempty(q_traj)
        q_goal = q_traj(end,:);
        T_goal = my_Robot.fkine(q_goal);
        P_goal = transl(T_goal);
        P_xyz  = P_goal(1:3).';
        pos_err = norm(P_xyz - P_end(:));
        if pos_err > TOL_POS
            ok = false;
        end
    end
end
% =====================================================================
% CHECK VÙNG CẤM TRÊN QUỸ ĐẠO q_traj (DÙNG CHUNG)
% =====================================================================
function [ok, pos_err] = check_forbidden_qtraj(q_traj, my_Robot)
    ok      = true;
    pos_err = 0;

    for k = 1:size(q_traj,1)
        T_now = my_Robot.fkine(q_traj(k,:));
        P_now = transl(T_now);
        if is_forbidden_tcp(P_now(1:3).')
            ok      = false;
            pos_err = -1;
            return;
        end
    end
end
% =====================================================================
% IK CHO MỘT ĐIỂM P = [px py pz]
% =====================================================================
function q = ik_point(P, geom)
    px = P(1); py = P(2); pz = P(3);
    a2 = geom.a2;
    a3 = geom.a3;
    a4 = geom.a4;
    d1 = geom.d1;
    d6 = geom.d6;
    % 1. Xoay đế
    theta1_val = atan2(py, px);
    % 2. Tâm cổ tay
    wx = px;
    wy = py;
    wz = pz + d6;
    r_wrist = sqrt(wx^2 + wy^2);
    z_wrist = wz - d1;
    % 3. Góc phi (khâu 4) theo độ cao
    phi_up = atan2(z_wrist, r_wrist);
    if z_wrist < 350
        phi = 0;
    elseif z_wrist > 650
        phi = phi_up;
    else
        k = (z_wrist - 350)/(650 - 350);
        k = 3*k^2 - 2*k^3;  % easing
        phi = k * phi_up;
    end
    % 4. Giải khớp 2,3
    r_3 = r_wrist - a4 * cos(phi); 
    z_3 = z_wrist - a4 * sin(phi);
    c3 = (r_3^2 + z_3^2 - a2^2 - a3^2) / (2 * a2 * a3);
    c3 = max(min(c3, 1), -1);
    theta3_val = -acos(c3);
    alpha = atan2(z_3, r_3);
    beta  = atan2(a3*sin(theta3_val), a2 + a3*cos(theta3_val));
    theta2_val = alpha - beta;
    theta4_val = phi - (theta2_val + theta3_val);
    % 5. Khớp 5 giữ TCP hướng xuống
    theta5_val = pi - phi;
    % 6. Khớp 6 quay cùng khớp 1
    theta6_val = theta1_val;
    q = [theta1_val, theta2_val - pi/2, theta3_val, theta4_val, theta5_val + pi/2, theta6_val];
    q = wrapToPi(q);
end
% =====================================================================
% PROFILE S-CURVE QUINTIC CHUẨN HÓA s(t) ∈ [0,1] (DÙNG CHUNG)
% =====================================================================
function s = scurve_quintic(L, v_max, a_max, dt, T_min)
    if L < 1e-9
        s = 0;
        return;
    end
    if v_max <= 0
        v_max = 100;
    end
    if a_max <= 0
        a_max = 500;
    end
    if T_min <= 0
        T_min = 0.3;
    end
    C_v = 1.875;      % max(s')
    C_a = 5.7735;     % max(s'')
    T_v = C_v * L / v_max;
    T_a = sqrt(C_a * L / a_max);
    T   = max([T_v, T_a, T_min]);
    t = 0:dt:T;
    if t(end) < T
        t = [t, T];
    end
    tau = t / T;
    s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
    s(1)   = 0;
    s(end) = 1;
end
% =====================================================================
% S-CURVE JOINT-SPACE (MOVE J) – DÙNG CHUNG PROFILE s
% =====================================================================
function q_list = scurve_joint(q0, q1, v_max, a_max, dt, T_min)
    q0 = wrapToPi(q0);
    q1 = wrapToPi(q1);
    dq = wrapToPi(q1 - q0);

    L = max(abs(dq));
    if L < 1e-9
        q_list = q0;
        return;
    end
    if v_max <= 0
        v_max = 0.5;
    end
    if a_max <= 0
        a_max = 1.0;
    end
    if T_min <= 0
        T_min = 0.3;
    end

    s = scurve_quintic(L, v_max, a_max, dt, T_min);  % dùng chung
    q_list = q0 + s'.*dq;                           % N x 6
end
% =====================================================================
% TÍNH qd, qdd TỪ q_traj (SAI PHÂN)
% =====================================================================
function [qd_traj, qdd_traj] = compute_vel_acc(q_traj, dt)
    [n_steps, n_joints] = size(q_traj);
    qd_traj  = zeros(n_steps, n_joints);
    qdd_traj = zeros(n_steps, n_joints);
    if n_steps < 3
        return;
    end
    for i = 2:(n_steps-1)
        qd_traj(i,:)  = (q_traj(i+1,:) - q_traj(i-1,:) ) / (2*dt);
        qdd_traj(i,:) = (q_traj(i+1,:) - 2*q_traj(i,:) + q_traj(i-1,:)) / (dt^2);
    end
    qd_traj(1,:)   = (q_traj(2,:) - q_traj(1,:) ) / dt;
    qdd_traj(1,:)  = (q_traj(3,:) - 2*q_traj(2,:) + q_traj(1,:)) / (dt^2);
    qd_traj(end,:) = (q_traj(end,:) - q_traj(end-1,:) ) / dt;
    qdd_traj(end,:)= (q_traj(end,:) - 2*q_traj(end-1,:) + q_traj(end-2,:)) / (dt^2);
end
% =====================================================================
% HÀM wrapToPi (nếu không có Mapping Toolbox)
% =====================================================================
function ang = wrapToPi(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end
% =====================================================================
% HÀM CHECK ĐOẠN THẲNG P_start -> P_end CẮT HỘP XY
% =====================================================================
function hit = segment_hits_xy_box(P_start, P_end, x_min, x_max, y_min, y_max)
    x0 = P_start(1);  y0 = P_start(2);
    dx = P_end(1) - x0;
    dy = P_end(2) - y0;
    u0 = 0; 
    u1 = 1;
    % ---- Theo trục X ----
    if abs(dx) < 1e-9
        if x0 <= x_min || x0 >= x_max
            hit = false;
            return;
        end
    else
        ux1 = (x_min - x0) / dx;
        ux2 = (x_max - x0) / dx;
        u_x_min = min(ux1, ux2);
        u_x_max = max(ux1, ux2);

        u0 = max(u0, u_x_min);
        u1 = min(u1, u_x_max);
        if u0 > u1
            hit = false;
            return;
        end
    end
    % ---- Theo trục Y ----
    if abs(dy) < 1e-9
        if y0 <= y_min || y0 >= y_max
            hit = false;
            return;
        end
    else
        uy1 = (y_min - y0) / dy;
        uy2 = (y_max - y0) / dy;
        u_y_min = min(uy1, uy2);
        u_y_max = max(uy1, uy2);
        u0 = max(u0, u_y_min);
        u1 = min(u1, u_y_max);
        if u0 > u1
            hit = false;
            return;
        end
    end
    hit = (u1 >= 0) && (u0 <= 1);
end
% =====================================================================
% HÀM CHUNG: 1 ĐIỂM TCP CÓ NẰM VÙNG CẤM KHÔNG
% =====================================================================
function bad = is_forbidden_tcp(P_xyz)
    x = P_xyz(1);
    y = P_xyz(2);
    z = P_xyz(3);
    in_xy_box = (x > -50 && x < 50 && y > -50 && y < 50);
    low_z  = (z < 350);  % vùng thấp
    below0 = (z < 0);    % luôn cấm

    bad = (in_xy_box && low_z) || below0;
end
