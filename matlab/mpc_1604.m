% try vq = interp1(...
%     x,v,...
%     xq); catch; end


function [meta, data] = mpc_1604()
%% Set up meta
    meta.date = '20181107/';
    meta.run = '001'; % testing face feedback
    meta.dataroot = '/home/benjamin/ros/data/';
    
%% Load data
    trial_path = [meta.dataroot meta.date meta.run];
    files = what(trial_path);
    for i= 1:length(files.m)
        eval_str = ['[data.' files.m{i}(1:end-6) '] = loaddatadotm(files.m{i}(1:end-6), meta);'];
        disp(eval_str)
        try 
            eval(eval_str); 
            catch; disp(['    ** Issue loading ' files.m{i}(1:end-6) ' data']); end
    end
    
    clear i files trial_path
%% Sync Timers
    timers.StartTime = [];
    timers.EndTime = [0];
    timers.SimpleCells = {...
        'data.mpc_log.uav_pose.time'; ...
        'data.mpc_log.cmd_vel.time'; ...
        'data.mpc_log.uav_des_pose.time'; ...
            };

    for n = 1:length(timers.SimpleCells)
        try
            timers.StartTime = min([ timers.StartTime min(eval(timers.SimpleCells{n})) ] );
        catch
            disp(['   ** does ' timers.SimpleCells{n} ' exist?'])
        end
    end
    for n = 1:length(timers.SimpleCells)
        try
            eval([timers.SimpleCells{n} ' = ' timers.SimpleCells{n} ' - timers.StartTime;']);
            timers.EndTime = max([timers.EndTime max( eval(timers.SimpleCells{n}) ) ]);
        catch
        end
    end

    data.timers = timers;

%% plot data
[meta, data] = plotmpc(meta, data);
end

function [out] = loaddatadotm(in, meta)
%%
    here = pwd;
    root = [meta.dataroot meta.date meta.run '/'];
    mat_file = [in '_' meta.run '.mat'];
    m_file = [in '_' meta.run '.m'];
    cd(root);
    
    if exist(mat_file)
        disp(['    Loading ' mat_file ' ...'])
        load(mat_file);
    elseif exist(m_file, 'file')
        
        disp(['    Loading ' root m_file ' ...'])
        try
            eval([in '_prealloc_' meta.run])
        catch
%             debugprint(['     ' in ': no preallocation found'])
        end
        eval([in '_' meta.run])
        if exist(in, 'var')
            vprint(['     Saving ' root mat_file ' ...'])
            save([root mat_file], in)
        end
    end
    out = eval(in);
    cd(here);
end
function [meta, data] = plotmpc(meta, data)
%% Turn plotting on
%     set(0, 'DefaultFigureVisible', 'on');
%     figHandles = findall(0, 'Type', 'figure');
%     set(figHandles(:), 'visible', 'on');
%     clear figHandles
%% figure(1); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(1); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    title('vrpn position, x global')
    hold on
        try plot(data.mpc_log.uav_pose.time, data.mpc_log.uav_pose.position(:,1), 'k.', 'displayname', 'uav actual x'); catch; end
        try plot(data.mpc_log.uav_des_pose.time, data.mpc_log.uav_des_pose.position(:,1), 'b.', 'displayname', 'uav desired x'); catch; end
        try plot(data.mpc_log.cmd_vel.time, data.mpc_log.cmd_vel.linear(:,1), 'r.', 'displayname', 'uav actual'); catch; end
    hold off
    grid on
    legend('toggle')
    clear current_fig
%% figure(2); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(2); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    title('vrpn position, x global')
    hold on
        try plot(data.mpc_log.uav_pose.time, data.mpc_log.uav_pose.position(:,2), 'k.', 'displayname', 'uav actual x'); catch; end
        try plot(data.mpc_log.uav_des_pose.time, data.mpc_log.uav_des_pose.position(:,2), 'b.', 'displayname', 'uav desired x'); catch; end
        try plot(data.mpc_log.cmd_vel.time, data.mpc_log.cmd_vel.linear(:,2), 'r.', 'displayname', 'uav actual'); catch; end
    hold off
    grid on
    legend('toggle')
    clear current_fig
%% figure(3); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(3); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    title('vrpn position, x global')
    hold on
        try plot(data.mpc_log.uav_pose.time, data.mpc_log.uav_pose.position(:,3), 'k.', 'displayname', 'uav actual x'); catch; end
        try plot(data.mpc_log.uav_des_pose.time, data.mpc_log.uav_des_pose.position(:,3), 'b.', 'displayname', 'uav desired x'); catch; end
        try plot(data.mpc_log.cmd_vel.time, data.mpc_log.cmd_vel.linear(:,3), 'r.', 'displayname', 'uav actual'); catch; end
    hold off
    grid on
    legend('toggle')
    clear current_fig

    
end
function [] = shortcut()

clc
clear all
close all

warning('off','MATLAB:legend:PlotEmpty')

meta.root = '/home/benjamin/ros/src/usma_ardrone/matlab/';
cd([meta.root])
addpath(genpath([meta.root]));
[meta, data] = uft_1604();


end