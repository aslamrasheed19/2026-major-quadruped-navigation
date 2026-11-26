function out = run_scenario_v1(scenario, csvfile)
%RUN_SCENARIO_V1  Run one of the two comparison scenarios and write the svan_log v1 CSV.
%   out = run_scenario_v1('stand')    % 6 s: stand-up ramp, then hold            -> results/svan_stand_matlab.csv
%   out = run_scenario_v1('trot')     % 12 s: stand, then 0.20 m/s straight from 4 s -> results/svan_trot_matlab.csv
%   Needs the svan_matlab folder on the path (run startup_svan first).  Same settings as the Gazebo scenario.
if nargin < 1, scenario = 'trot'; end
if nargin < 2, csvfile = fullfile('results', ['svan_' scenario '_matlab.csv']); end
switch lower(scenario)
    case 'stand'
        out = svan_simulate('stand', struct('T', 6));
    case 'trot'
        out = svan_simulate('trot', struct('T', 12, 'vx', 0.20, 't_walk', 4.0));
    otherwise
        error('scenario must be stand | trot');
end
export_log_v1(out, lower(scenario), csvfile);
end
