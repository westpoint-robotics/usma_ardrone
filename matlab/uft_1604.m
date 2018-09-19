function [meta, data] = uft_1604()
%% Set up meta
    meta.date = '20180919/';
%     meta.run = '002'; %optitrack control works, but does not settle, it just oscillates
    meta.run = '003'; %testing face tracking and feedback, but uav did not seem to follow my face
    
    meta.dataroot = '/home/benjamin/ros/data/';
    
%% Load data
    try [data.fta] = loaddatadotm('fta', meta); catch; disp(['    ** Issue loading fta data']); end
    try [data.optAutopilot] = loaddatadotm('optAutopilot', meta); catch; disp(['    ** Issue loading optAutopilot data']); end
    
%% Sync Timers
    timers.StartTime = [];
    timers.EndTime = [0];
    timers.SimpleCells = {...
        'data.fta.mocap.vel_msg.time'; ...
        'data.optAutopilot.cmd.time'; ...
        'data.optAutopilot.mocap_pose.time'; ...
        'data.fta.face.centroid_msg.time'; ...
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

%% plot data
[meta, data] = plotuft(meta, data);

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

function [meta, data] = plotuft(meta, data)
%% Turn plotting on
%     set(0, 'DefaultFigureVisible', 'on');
%     figHandles = findall(0, 'Type', 'figure');
%     set(figHandles(:), 'visible', 'on');
%     clear figHandles

%% figure(1); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(1); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav position, x global')
    hold on
        try plot(data.optAutopilot.mocap_pose.time, data.optAutopilot.mocap_pose.p(:,1), 'k.'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.desired_pose_global.p(:,1), 'r.'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.error_pose_global.p(:,1), 'b.'); catch; end
    hold off
    grid on
%% figure(2); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(2); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav position, y global')
    hold on
        try plot(data.optAutopilot.mocap_pose.time, data.optAutopilot.mocap_pose.p(:,2), 'k.'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.desired_pose_global.p(:,2), 'r.'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.error_pose_global.p(:,2), 'b.'); catch; end
    hold off
    grid on
%% figure(3); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(3); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav position, z global')
    hold on
        try plot(data.optAutopilot.mocap_pose.time, data.optAutopilot.mocap_pose.p(:,3), 'k.'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.desired_pose_global.p(:,3), 'r.'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.error_pose_global.p(:,3), 'b.'); catch; end
    hold off
    grid on
%% figure(4); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(4); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav position, yaw global')
    hold on
        try plot(data.optAutopilot.mocap_pose.time, data.optAutopilot.mocap_pose.yaw, 'k.'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.desired_pose_global.yaw, 'r.'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.msg_angular(:,3), 'm.'); catch; end
        
    hold off
    grid on
%% figure(5); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(5); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav error, x body')
    hold on
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.error_pose_body.p(:,1), 'r.', 'displayname', 'error bosd body'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.msg_linear(:,1), 'b.', 'displayname', 'cmd msd linear'); catch; end
        try plot(data.fta.mocap.vel_msg.time, data.fta.mocap.vel_msg.linear(:,1), 'kx', 'displayname', 'cmd msd linear'); catch; end
        
    hold off
    grid on
    legend('toggle')
%% figure(6); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(6); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav error, y body')
    hold on
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.error_pose_body.p(:,2), 'r.'); catch; end
    hold off
    grid on
%% figure(7); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(7); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav error, z body')
    hold on
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.error_pose_body.p(:,3), 'r.'); catch; end
    hold off
    grid on
%% figure(8); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(8); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav position, yaw body')
    hold on
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.error_pose_body.yaw, 'r.'); catch; end
    hold off
    grid on
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