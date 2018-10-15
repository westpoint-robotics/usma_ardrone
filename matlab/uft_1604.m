% try vq = interp1(...
%     x,v,...
%     xq); catch; end


function [meta, data] = uft_1604()
%% Set up meta
%     meta.date = '20180919/';
%     meta.run = '002'; %optitrack control works, but does not settle, it just oscillates
%     meta.run = '003'; %testing face tracking and feedback, but uav did not seem to follow my face
%     meta.run = '008'; %testing face tracking and feedback, testing in the office
    meta.date = '20181010/';
    meta.run = '003'; % testing face feedback
%     meta.run = '006'; % testing face feedback
    meta.dataroot = '/home/benjamin/ros/data/';
    
%% Load data
    try [data.fta] = loaddatadotm('fta', meta); catch; disp(['    ** Issue loading fta data']); end
    try [data.facetoCmd] = loaddatadotm('facetoCmd', meta); catch; disp(['    ** Issue loading facetoCmd data']); end
    try [data.optAutopilot] = loaddatadotm('optAutopilot', meta); catch; disp(['    ** Issue loading optAutopilot data']); end
    try [data.vrpn_ardrone] = loaddatadotm('vrpn_ardrone', meta); catch; disp(['    ** Issue loading vrpn_ardrone data']); end
    try [data.vrpn_blockhead] = loaddatadotm('vrpn_blockhead', meta); catch; disp(['    ** Issue loading vrpn_blockhead data']); end
%% Sync Timers
    timers.StartTime = [];
    timers.EndTime = [0];
    timers.SimpleCells = {...
        'data.fta.face.switched_cmd_vel_msg.time'; ...
        'data.fta.mocap.vel_msg.time'; ...
        'data.fta.face.cmd_msg.time'; ...
        'data.facetoCmd.face.centroid_msg.time'; ...
        'data.facetoCmd.face.feedback.time'; ...
        'data.facetoCmd.face.face_desired_pose.time'; ...
        'data.optAutopilot.mocap_pose.time'; ...
        'data.optAutopilot.cmd.time'; ...
        'data.optAutopilot.face.time'; ...
        'data.vrpn_ardrone.pose.time'; ...
        'data.vrpn_blockhead.pose.time'; ...
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
%% spline data
for i = 1:3
    try data.facetoCmd.face.pose(:,i) = interp1(...
        data.vrpn_ardrone.pose.time, data.vrpn_ardrone.pose.linear(:,i),...
        data.facetoCmd.face.centroid_msg.time, ...
        'linear', 'extrap'); catch; end
end; clear i

data.sync_vrpn.time = data.vrpn_ardrone.pose.time;
if 0 % bug fixed 20181015
    data.vrpn_ardrone.pose.yaw = data.vrpn_ardrone.pose.angular(:,1);
end
for i = 1:3
    try data.sync_vrpn.uav.pose(:,i) = interp1(...
        data.vrpn_ardrone.pose.time, data.vrpn_ardrone.pose.linear(:,i),...
        data.sync_vrpn.time, ...
        'linear', 'extrap'); catch; end
    try 
        data.sync_vrpn.face.pose(:,i) = interp1(...
        data.vrpn_blockhead.pose.time, data.vrpn_blockhead.pose.linear(:,i),...
        data.sync_vrpn.time, ...
        'linear', 'extrap'); 
    catch; end
    try 
        data.sync_vrpn.diff.pose_global(:,i) = data.sync_vrpn.face.pose(:,i) - data.sync_vrpn.uav.pose(:,i); 
    catch; end
end; clear i

try data.sync_vrpn.uav.yaw = interp1(...
    data.optAutopilot.mocap_pose.time, data.optAutopilot.mocap_pose.yaw,...
    data.sync_vrpn.time, ...
    'linear', 'extrap'); catch; end


% get relative angle between face and UAV
try data.sync_vrpn.angle_to_face = atan2(data.sync_vrpn.diff.pose_global(:,2), data.sync_vrpn.diff.pose_global(:,1)) - data.vrpn_ardrone.pose.yaw; catch; end


%% spline data to image time
data.sync_camtime.time = data.facetoCmd.face.centroid_msg.time;
for i = 1:3
    try data.sync_camtime.uav.pose(:,i) = interp1(...
        data.vrpn_ardrone.pose.time, data.vrpn_ardrone.pose.linear(:,i),...
        data.sync_camtime.time, ...
        'linear', 'extrap'); catch; end

    try data.sync_camtime.face.pose(:,i) = interp1(...
        data.vrpn_blockhead.pose.time, data.vrpn_blockhead.pose.linear(:,i),...
        data.sync_camtime.time, ...
        'linear', 'extrap'); catch; end
    try data.sync_camtime.diff.pose_global(:,i) = data.sync_camtime.face.pose(:,i) - data.sync_camtime.uav.pose(:,i); catch; end
end, clear i
try data.sync_camtime.uav.yaw = interp1(...
    data.optAutopilot.mocap_pose.time, data.optAutopilot.mocap_pose.yaw,...
    data.sync_camtime.time, ...
    'linear', 'extrap'); catch; end

% get relative angle between face and UAV
try data.sync_camtime.angle_to_face = atan2(data.sync_camtime.diff.pose_global(:,2), data.sync_camtime.diff.pose_global(:,1)) - data.sync_camtime.uav.yaw; catch; end




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
%% figure(1); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(1); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    title('vrpn position, x global')
    hold on
        try plot(data.optAutopilot.mocap_pose.time, data.optAutopilot.mocap_pose.p(:,1), 'k.', 'displayname', 'uav actual'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.desired_pose_global.p(:,1), 'r.', 'displayname', 'uav desired'); catch; end
        try plot(data.vrpn_blockhead.pose.time, data.vrpn_blockhead.pose.linear(:,1), 'b.', 'displayname', 'face actual'); catch; end
    hold off
    grid on
    legend('toggle')
    clear current_fig
%% figure(1); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(1); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    title('vrpn position, x global')
    hold on
        try plot(data.optAutopilot.mocap_pose.time, data.optAutopilot.mocap_pose.p(:,1), 'k.', 'displayname', 'uav actual'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.desired_pose_global.p(:,1), 'r.', 'displayname', 'uav desired'); catch; end
        try plot(data.vrpn_blockhead.pose.time, data.vrpn_blockhead.pose.linear(:,1), 'b.', 'displayname', 'face actual'); catch; end
    hold off
    grid on
    legend('toggle')
    clear current_fig
%% figure(2); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(2); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    title('vrpn position, y global')
    hold on
        try plot(data.optAutopilot.mocap_pose.time, data.optAutopilot.mocap_pose.p(:,2), 'k.', 'displayname', 'uav actual'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.desired_pose_global.p(:,2), 'r.', 'displayname', 'uav desired'); catch; end
        try plot(data.vrpn_blockhead.pose.time, data.vrpn_blockhead.pose.linear(:,2), 'b.', 'displayname', 'face actual'); catch; end
    hold off
    grid on
    legend('toggle')
    clear current_fig
%% figure(3); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(3); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    title('vrpn position, z global')
    hold on
        try plot(data.optAutopilot.mocap_pose.time, data.optAutopilot.mocap_pose.p(:,3), 'k.', 'displayname', 'uav actual'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.desired_pose_global.p(:,3), 'r.', 'displayname', 'uav desired'); catch; end
        try plot(data.vrpn_blockhead.pose.time, data.vrpn_blockhead.pose.linear(:,3), 'b.', 'displayname', 'face actual'); catch; end
    hold off
    grid on
    legend('toggle')
    clear current_fig
%% figure(4); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(4); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    title([meta.date meta.run ' ' 'uav position, yaw global'])
    hold on
        try plot(data.optAutopilot.mocap_pose.time, data.optAutopilot.mocap_pose.yaw, 'k.', 'displayname', 'mocap pose'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.desired_pose_global.yaw, 'r.', 'displayname', 'desired pose (global)'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.msg_angular(:,3), 'm.', 'displayname', 'cmd msg yaw'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.error_pose_global.yaw, 'b.', 'displayname', 'yaw error'); catch; end
    hold off
    grid on
    legend('toggle')
    try saveas(gcf, [meta.dataroot meta.date meta.run '/figure' num2str(current_fig.Number) '.png']); catch; end
    clear current_fig
%% figure(5); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(5); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    title('uav error, x body')
    hold on
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.error_pose_body.p(:,1), 'r.', 'displayname', 'error body'); catch; end
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.msg_linear(:,1), 'b.', 'displayname', 'cmd msg linear'); catch; end
        try plot(data.fta.mocap.vel_msg.time, data.fta.mocap.vel_msg.linear(:,1), 'kx', 'displayname', 'cmd msg linear'); catch; end
        
    hold off
    grid on
    legend('toggle')
    clear current_fig
%% figure(6); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(6); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..'])
    title('uav error, y body')
    hold on
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.error_pose_body.p(:,2), 'r.'); catch; end
    hold off
    grid on
    clear current_fig
%% figure(7); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(7); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); 
    title('uav error, z body')
    hold on
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.error_pose_body.p(:,3), 'r.'); catch; end
    hold off
    grid on
    clear current_fig
%% figure(8); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); 
figure(8); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); 
    title('uav position, yaw body')
    hold on
        try plot(data.optAutopilot.cmd.time, data.optAutopilot.cmd.error_pose_body.yaw, 'r.'); catch; end
    hold off
    grid on
    clear current_fig
%% figure(9); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(9); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    try [AX1 Hleft1 Hright1] = plotyy(...
            data.facetoCmd.face.centroid_msg.time, data.facetoCmd.face.centroid_msg.dxdy(:,1),...
            data.optAutopilot.cmd.time, data.optAutopilot.cmd.msg_angular(:,3));
        hold(AX1(1)); % hold the left axis for more plots
        hold(AX1(2)); % hold the right axis for more plots
        set(Hleft1(1), 'Color', 'b', 'Marker', 'x', 'LineStyle', 'none', 'displayname', 'Centroid pixel');
        set(Hright1(1), 'Color', 'r', 'Marker', '.', 'LineStyle', 'none', 'displayname', 'cmd msg yaw');
        h1 = plot(AX1(2), data.facetoCmd.face.feedback.time, data.facetoCmd.face.feedback.yaw, 'ko', 'displayname', 'face yaw cmd');
    catch
    end
    grid on
    legend('toggle')
    title([meta.date meta.run ' ' 'yaw cmd vs pixel centroid'])
    try saveas(gcf, [meta.dataroot meta.date meta.run '/figure' num2str(current_fig.Number) '.png']); catch; end
    clear current_fig
%% figure(10); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(10); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    title('yaw cmd vs pixel centroid')
    try [AX1 Hleft1 Hright1] = plotyy(...
        data.facetoCmd.face.centroid_msg.time, data.facetoCmd.face.centroid_msg.dxdy(:,2),...
        data.optAutopilot.cmd.time, data.optAutopilot.cmd.msg_linear(:,3));
        hold(AX1(1)); % hold the left axis for more plots
        hold(AX1(2)); % hold the right axis for more plots
        set(Hleft1(1), 'Color', 'b', 'Marker', 'x', 'LineStyle', 'none', 'displayname', 'Centroid pixel');
        set(Hright1(1), 'Color', 'r', 'Marker', '.', 'LineStyle', 'none', 'displayname', 'cmd msg z');

        h1 = plot(AX1(2), data.facetoCmd.face.feedback.time, data.facetoCmd.face.feedback.z, 'ko', 'displayname', 'face z cmd');
    catch
    end
    grid on
    legend('toggle')  
    clear current_fig
%% figure(11); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(11); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    title([meta.date meta.run ' ' 'uav heading'])
    hold on
		try plot(data.optAutopilot.face.time, data.optAutopilot.face.angle, '.', 'displayname', 'relative face angle'); catch; end
        try plot(data.sync_vrpn.time, data.sync_vrpn.diff.yaw, 'o', 'displayname', 'relative (mocap) face angle'); catch; end
		try plot(data.optAutopilot.face.time, data.optAutopilot.face.uav_mocap_heading, 'x', 'displayname', 'uav heading mocap'); catch; end
        try plot(data.optAutopilot.face.time, data.optAutopilot.face.desired_global_heading, 'x', 'displayname', 'desired global heading'); catch; end
    hold off
    grid on
    legend('toggle')
    try saveas(gcf, [meta.dataroot meta.date meta.run '/figure' num2str(current_fig.Number) '.png']); catch; end
    clear current_fig
%% figure(16); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(16); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    title('vrpn position, x global imagetime')
    hold on
        try plot(data.sync_camtime.time, data.sync_camtime.uav.pose(:,1), 'k.', 'displayname', 'uav actual'); catch; end
        try plot(data.sync_camtime.time, data.sync_camtime.face.pose(:,1), 'b.', 'displayname', 'face actual'); catch; end
    hold off
    grid on
    legend('toggle')
    clear current_fig
%% figure(17); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(17); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    title('vrpn position, y global imagetime')
    hold on
        try plot(data.sync_camtime.time, data.sync_camtime.uav.pose(:,2), 'k.', 'displayname', 'uav actual'); catch; end
        try plot(data.sync_camtime.time, data.sync_camtime.face.pose(:,2), 'b.', 'displayname', 'face actual'); catch; end
    hold off
    grid on
    legend('toggle')
    clear current_fig
%% figure(18); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(18); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    title('vrpn position, z global imagetime')
    hold on
        try plot(data.sync_camtime.time, data.sync_camtime.uav.pose(:,3), 'k.', 'displayname', 'uav actual'); catch; end
        try plot(data.sync_camtime.time, data.sync_camtime.face.pose(:,3), 'b.', 'displayname', 'face actual'); catch; end
    hold off
    grid on
    legend('toggle')
    clear current_fig
%% figure(19); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(19); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    title('vrpn position, yaw global imagetime')
    hold on
        try plot(data.optAutopilot.mocap_pose.time, data.optAutopilot.mocap_pose.yaw, 'b.', 'displayname', 'mocap pose'); catch; end
        try plot(data.sync_camtime.time, data.sync_camtime.uav.yaw, 'k.', 'displayname', 'uav actual'); catch; end
        try plot(data.vrpn_ardrone.pose.time, data.vrpn_ardrone.pose.yaw, 'r.', 'displayname', 'uav actual'); catch; end
%         try plot(data.sync_camtime.time, data.sync_camtime.face.pose(:,3), 'b.', 'displayname', 'face actual'); catch; end
    hold off
    grid on
    legend('toggle')
    clear current_fig

%% figure(20); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(20); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); 
    title([meta.date meta.run ' ' 'xy top down'])
    hold on
		try plot(data.vrpn_ardrone.pose.linear(:,1), data.vrpn_ardrone.pose.linear(:,2), 'x', 'displayname', 'uav xy'); catch; end
		try plot(data.vrpn_blockhead.pose.linear(:,1), data.vrpn_blockhead.pose.linear(:,2), 'x', 'displayname', 'face xy'); catch; end
        
    hold off
    grid on
    legend('toggle')
    xlabel('x [m]')
    xlabel('y [m]')
    try saveas(gcf, [meta.dataroot meta.date meta.run '/figure' num2str(current_fig.Number) '.png']); catch; end
    clear current_fig
%% figure(21); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(21); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    try 
        [AX1 Hleft1 Hright1] = plotyy(...
            data.facetoCmd.face.centroid_msg.time, data.facetoCmd.face.centroid_msg.dxdy(:,1),...
            data.sync_camtime.time, data.sync_camtime.angle_to_face);
        hold(AX1(1)); % hold the left axis for more plots
        hold(AX1(2)); % hold the right axis for more plots
        set(Hleft1(1), 'Color', 'b', 'Marker', 'x', 'LineStyle', 'none', 'displayname', 'Centroid pixel');
        set(Hright1(1), 'Color', 'r', 'Marker', '.', 'LineStyle', 'none', 'displayname', 'vrpn relative angle');
%         h1 = plot(AX1(2), data.facetoCmd.face.feedback.time, data.facetoCmd.face.feedback.yaw, 'ko', 'displayname', 'face yaw cmd');
    catch
    end
    
    AX1(1).YTick = -400:100:400;
    AX1(1).YLim = [AX1(1).YTick(1) AX1(1).YTick(end)];
    
    AX1(2).YTick = [-8:2:8];
    AX1(2).YLim = [AX1(2).YTick(1) AX1(2).YTick(end)];
    grid on
    legend('toggle')
    title([meta.date meta.run ' ' 'yaw cmd vs pixel centroid'])
    try saveas(gcf, [meta.dataroot meta.date meta.run '/figure' num2str(current_fig.Number) '.png']); catch; end
    clear current_fig
    
%% figure(22); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
figure(22); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']);
    title([meta.date meta.run ' ' 'angle/pixel'])
    
    try 
        [AX1 Hleft1 Hright1] = plotyy(...
            data.sync_camtime.time, data.sync_camtime.angle_to_face ./ data.facetoCmd.face.centroid_msg.dxdy(:,1),...
            data.sync_camtime.time, data.sync_camtime.angle_to_face);
        hold(AX1(1)); % hold the left axis for more plots
        hold(AX1(2)); % hold the right axis for more plots
        set(Hleft1(1), 'Color', 'b', 'Marker', 'x', 'LineStyle', 'none', 'displayname', 'angle ./ pixel');
        set(Hright1(1), 'Color', 'r', 'Marker', '.', 'LineStyle', 'none', 'displayname', 'vrpn relative angle');       
    catch; end
       
    try 
        scale = 0.00175;
        offset = 50;
        adjusted_data = (data.facetoCmd.face.centroid_msg.dxdy(:,1)+offset )*scale;
        h1 = plot(AX1(2), data.facetoCmd.face.centroid_msg.time,adjusted_data , 'ko', 'displayname', 'pixel to angle'); 
    catch;end
    
    mean (adjusted_data - data.sync_camtime.angle_to_face)
    AX1(1).YTick = -0.006:0.002:0.006;
    AX1(1).YLim = [AX1(1).YTick(1) AX1(1).YTick(end)];
    
    AX1(2).YTick = [-0.6:0.2:0.6];
    AX1(2).YLim = [AX1(2).YTick(1) AX1(2).YTick(end)];
    
    grid on

    legend('toggle')
    xlabel('x [m]')
    xlabel('y [m]')
    try saveas(gcf, [meta.dataroot meta.date meta.run '/figure' num2str(current_fig.Number) '.png']); catch; end
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