function export_log_v1(out, scenario, csvfile)
%EXPORT_LOG_V1  Write a svan_simulate result as a "svan_log v1" CSV (see COMPARISON_PROTOCOL.md).
%   export_log_v1(out, 'trot', 'results/svan_trot_matlab.csv')
%   Columns: t | base xyz | roll pitch yaw | cmd vx vy wz | Fn FR FL RR RL | q(12) | dq(12) | qref(12) | tau(12)
%   tau = torque APPLIED during the tick that starts at t (svan_simulate already stores tau_applied, 1 tick latency).
legs = {'FR', 'FL', 'RR', 'RL'};  jn = {'hip', 'thigh', 'calf'};
names = cell(1, 12);  k = 0;
for i = 1:4
    for j = 1:3
        k = k + 1;  names{k} = [legs{i} '_' jn{j}];
    end
end
hdr = {'t', 'base_x', 'base_y', 'base_z', 'roll', 'pitch', 'yaw', 'cmd_vx', 'cmd_vy', 'cmd_wz', ...
       'Fn_FR', 'Fn_FL', 'Fn_RR', 'Fn_RL'};
pre = {'q_', 'dq_', 'qref_', 'tau_'};
for a = 1:4
    for k = 1:12
        hdr{end + 1} = [pre{a} names{k}];   %#ok<AGROW>
    end
end
A = [out.t; out.p; out.rpy; out.cmd; out.Fn; out.q; out.qd; out.q_ref; out.tau]';
if size(A, 2) ~= numel(hdr), error('export_log_v1: %d data columns but %d header names', size(A, 2), numel(hdr)); end
fid = fopen(csvfile, 'w');
if fid < 0, error('export_log_v1: cannot open %s', csvfile); end
o = out.opts;
fprintf(fid, '# svan_log v1\n# source=matlab scenario=%s log_dt=0.01\n', scenario);
fprintf(fid, '# T=%g vx=%g t_walk=%g ctrl_hz=500 latency_ticks=%d physics_dt=%g\n', o.T, o.vx, o.t_walk, o.latency, o.dt);
fprintf(fid, '%s', hdr{1});
for k = 2:numel(hdr), fprintf(fid, ',%s', hdr{k}); end
fprintf(fid, '\n');
fmt = [repmat('%.6g,', 1, size(A, 2) - 1) '%.6g\n'];
fprintf(fid, fmt, A');
fclose(fid);
fprintf('wrote %d rows x %d columns to %s\n', size(A, 1), size(A, 2), csvfile);
end
