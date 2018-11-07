%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% author:  Jan Dentler
%          University of Luxembourg
% email:   jan.dentler@uni.lu
% date:    14.06.2017
% licence: For internal use only 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% function script_20170112_plot_ardrone_from_bag()
clc
close all
%% Get BAG DATA
filename='20170717_Ardrone_Identification_2017-07-17-11-33-39_2_withHighVelocities';

%%
loadbagflag=0;
if(loadbagflag)
    bag=rosbag([filename,'.bag']);
    loadbagflag=0;
starttime=bag.StartTime; 
endtime =bag.EndTime;
timeintervall=[starttime endtime];
topic_pose=select(bag,...
    'Time',timeintervall,'Topic','/Ardrone2SimpleLinModel_HASHMARK_0/pose');
topic_ctrl=select(bag,...
    'Time',timeintervall,'Topic','/Ardrone2SimpleLinModel_HASHMARK_0/cmd_vel');
control = timeseries(topic_ctrl,...
    'Linear.X',...
    'Linear.Y',...
    'Linear.Z',...
    'Angular.Z');
position = timeseries(topic_pose,... 
    'Pose.Position.X',...
    'Pose.Position.Y',...
    'Pose.Position.Z');   
quaternion = timeseries(topic_pose,...
    'Pose.Orientation.W',...    
    'Pose.Orientation.X',...
    'Pose.Orientation.Y',...
    'Pose.Orientation.Z');   
    clear bag
    save([filename,'.mat'])
    clear all
end
%%
load([filename,'.mat'])
resolution=150;
height=4.7;
width=16;


%% Forward and sideward channel
tu=control.time-position.time(1);
ux=control.data(:,1);
uy=control.data(:,2);
uz=control.data(:,3);
uyaw=control.data(:,4);

tx=position.time-position.time(1);
x=position.data(:,1);
y=position.data(:,2);
z=position.data(:,3);
%euler=quat2eul(quaternion.data,'ZYX');
%[yaw, pitch, roll]=quat2eul(quaternion.data);
[yaw, pitch, roll] = quat2angle(quaternion.data);
% yaw=(euler(:,1));
% pitch=(euler(:,2));
% roll=(euler(:,3));

tv=position.time(1:end-1)-position.time(1);
vx_unfiltered=     (x(2:end)-x(1:end-1))./(tx(2:end)-tx(1:end-1));
vy_unfiltered=     (y(2:end)-y(1:end-1))./(tx(2:end)-tx(1:end-1));
vz_unfiltered=     (z(2:end)-z(1:end-1))./(tx(2:end)-tx(1:end-1));
vyaw_unfiltered=   (yaw(2:end)-yaw(1:end-1))./(tx(2:end)-tx(1:end-1));
vpitch_unfiltered= (pitch(2:end)-pitch(1:end-1))./(tx(2:end)-tx(1:end-1));
vroll_unfiltered=  (roll(2:end)-roll(1:end-1))./(tx(2:end)-tx(1:end-1));

% filt=designfilt('lowpassfir', 'PassbandFrequency', .01, 'StopbandFrequency', .02, 'PassbandRipple', 1, 'StopbandAttenuation', 60);
% vx=     filter(filt,vx_unfiltered);
% vy=     filter(filt,vy_unfiltered);
% vz=     filter(filt,vz_unfiltered);
% vyaw=   filter(filt,vyaw_unfiltered);
% vpitch= filter(filt,vpitch_unfiltered);
% vroll=  filter(filt,vroll_unfiltered);
meanfiltersamples=10;
vx=     medfilt1(vx_unfiltered,meanfiltersamples);
vy=     medfilt1(vy_unfiltered,meanfiltersamples);
vz=     medfilt1(vz_unfiltered,meanfiltersamples);
vyaw=   medfilt1(vyaw_unfiltered,meanfiltersamples);
vpitch= medfilt1(vpitch_unfiltered,meanfiltersamples);
vroll=  medfilt1(vroll_unfiltered,meanfiltersamples);
yawinterp=interp1(tx,yaw,tv,'previous','extrap');
vf=     vx.*cos(yawinterp)+vy.*sin(yawinterp);
vs=    -vx.*sin(yawinterp)+vy.*cos(yawinterp);

dt=mean(tx(2:end)-tx(1:end-1));

%% Get Timebase
% t_forward_start=  69;
% t_forward_end=    97;
% t_sideward_start= 137;
% t_sideward_end=   156;
% t_upward_start=   106;
% t_upward_end=     109;
% t_yaw_start=      98;
% t_yaw_end=        132;

% t_forward_start=  69;
% t_forward_end=    97;
% t_sideward_start= 137;
% t_sideward_end=   156;
% t_upward_start=   10;
% t_upward_end=     145;
% t_yaw_start=      98;
% t_yaw_end=        132;

t_forward_start=  69;
t_forward_end=    97;
t_sideward_start= 137;
t_sideward_end=   156;
t_upward_start=   105;
t_upward_end=     142;
t_yaw_start=      98;
t_yaw_end=        132;


%% FIGURE VX
timebase=t_forward_start:dt:t_forward_end;
index_x=find(tx>t_forward_start):find(tx>t_forward_end);
index_v=find(tv>t_forward_start):find(tv>t_forward_end);
index_u=find(tu>t_forward_start):find(tu>t_forward_end);
uinterp=interp1(tu(index_u),ux(index_u),timebase,'previous','extrap');
xinterp=interp1(tx(index_x),x(index_x),timebase,'previous','extrap');
vinterp=interp1(tv(index_v),vf(index_v),timebase,'previous','extrap');
uinterp(isnan(uinterp))=0;
xinterp(isnan(xinterp))=0;
vinterp(isnan(vinterp))=0;
ident_forwardchannel=iddata(vinterp',uinterp',dt);
% steep=0.8
% modelx=ss([0,1;0,-steep],[0;3.2*steep],[1,0],0);
% modelvx=ss(-steep,3.2*steep,1,0);
ax=-0.6
bx= 1.2
modelx=ss([0,1;0,ax],[0;bx],[1,0],0);
modelvx=ss(ax,bx,1,0);
mxinterp=lsim(modelx,uinterp,timebase,[0,0])';
mvinterp=lsim(modelvx,uinterp,timebase,0)';

modelxdiscrete=c2d(modelx,dt);
modelvxdiscrete=c2d(modelvx,dt);
% mxinterpdiscrete=lsim(modelxdiscrete,uinterp,timebase,[0,0])';
% mvinterpdiscrete=lsim(modelvxdiscrete,uinterp,timebase,0)';

hd1=figure(1)
% hsp1_1=subplot(2,1,1)
plot(timebase(2:end),xinterp(2:end)-xinterp(2),'-','LineWidth',2)
hold on
plot(timebase,mxinterp,'--','LineWidth',2)
hold on
% plot(timebase,mxinterpdiscrete,':','LineWidth',2)
% hold on
plot(timebase,uinterp,'g-.','LineWidth',2)
hl1_1=legend({'$x[m]$','$\hat{x}[m]$','$u_x[m/s]$'},'Interpreter', 'latex')
axis([t_forward_start,t_forward_end,-3,5])
labels=get(gca, 'XTickLabel');
labels{end}='t[s]';
set(gca, 'XTickLabel',labels)
addpath(genpath('matlab2tikz'))
tikzname=[pwd,'/export/img_',filename,'_','x_ch1','.tikz'];
cleanfigure('handle',hd1,'targetResolution',resolution);
matlab2tikz(tikzname,'width','\fwidth','height','\fheight' ); %(working area — 160 mm by 247 mm)

% title('Dji M100 Simulator X-Channel Identification')
% hsp1_2=subplot(2,1,2)
hd2=figure(2)
plot(timebase,vinterp,'-','LineWidth',2)
hold on
plot(timebase,mvinterp,'--','LineWidth',2)
hold on
plot(timebase,uinterp,'g:','LineWidth',2)

% %Real system Regression Points
% regstart=70.25
% regend=70.8
% timebasereg=regstart:dt:regend;
% index_vreg=find(tv>regstart):find(tv>regend);
% vinterpreg=interp1(tv(index_vreg),vf(index_vreg),timebasereg,'previous','extrap');
% vinterpreg(isnan(vinterpreg))=0;
% [r,m,b] = regression(timebasereg,vinterpreg)
% plot(timebase,m*timebase+b,'b--','Linewidth',1)
% regstart=78.9
% regend=79.62
% timebasereg=regstart:dt:regend;
% index_vreg=find(tv>regstart):find(tv>regend);
% vinterpreg=interp1(tv(index_vreg),vf(index_vreg),timebasereg,'previous','extrap');
% vinterpreg(isnan(vinterpreg))=0;
% [r,m,b] = regression(timebasereg,vinterpreg)
% plot(timebase,m*timebase+b,'b--','Linewidth',1)
% regstart=89
% regend=89.6
% timebasereg=regstart:dt:regend;
% index_vreg=find(tv>regstart):find(tv>regend);
% vinterpreg=interp1(tv(index_vreg),vf(index_vreg),timebasereg,'previous','extrap');
% vinterpreg(isnan(vinterpreg))=0;
% [r,m,b] = regression(timebasereg,vinterpreg)
% plot(timebase,m*timebase+b,'b--','Linewidth',1)
% 
% %Model Regression Points
% regstart=70
% regend=70.8
% timebasereg=regstart:dt:regend;
% index_vreg=find(timebase>regstart):find(timebase>regend);
% vinterpreg=interp1(timebase(index_vreg),mvinterp(index_vreg),timebasereg,'previous','extrap');
% [r,m,b] = regression(timebasereg,vinterpreg)
% plot(timebase,m*timebase+b,'r--','Linewidth',1)
% regstart=78.6
% regend=79
% timebasereg=regstart:dt:regend;
% index_vreg=find(timebase>regstart):find(timebase>regend);
% vinterpreg=interp1(timebase(index_vreg),mvinterp(index_vreg),timebasereg,'previous','extrap');
% [r,m,b] = regression(timebasereg,vinterpreg)
% plot(timebase,m*timebase+b,'r--','Linewidth',1)
% regstart=88.6
% regend=89
% timebasereg=regstart:dt:regend;
% index_vreg=find(timebase>regstart):find(timebase>regend);
% vinterpreg=interp1(timebase(index_vreg),mvinterp(index_vreg),timebasereg,'previous','extrap');
% [r,m,b] = regression(timebasereg,vinterpreg)
% plot(timebase,m*timebase+b,'r--','Linewidth',1)

axis([t_forward_start,t_forward_end,-4,4])
hl1_2=legend({'$v_x[m/s]$','$\hat{v}_x[m/s]$','$u_x[m/s]$'},'Interpreter', 'latex')
labels=get(gca, 'XTickLabel');
labels{end}='t[s]';
set(gca, 'XTickLabel',labels)
addpath(genpath('matlab2tikz'))
tikzname=[pwd,'/export/img_',filename,'_','x_ch2','.tikz'];
cleanfigure('handle',hd2,'targetResolution',resolution);
matlab2tikz(tikzname,'width','\fwidth','height','\fheight' ); %(working area — 160 mm by 247 mm)


%% FIGURE VY
timebase=t_sideward_start:dt:t_sideward_end-dt;
index_x=find(tx>t_sideward_start):find(tx>t_sideward_end)-1;
index_v=find(tv>t_sideward_start):find(tv>t_sideward_end)-1;
index_u=find(tu>t_sideward_start):find(tu>t_sideward_end)-1;
uinterp=interp1(tu(index_u),uy(index_u),timebase,'previous','extrap');
xinterp=interp1(tx(index_x),y(index_x),timebase,'previous','extrap');
vinterp=interp1(tx(index_v),vs(index_v),timebase,'previous','extrap');
uinterp(isnan(uinterp))=0;
xinterp(isnan(xinterp))=0;
vinterp(isnan(vinterp))=0;
ident_sidewardchannel=iddata(vinterp',uinterp',dt);
steep=0.8
modely=ss([0,1;0,-steep],[0;3.2*steep],[1,0],0)
modelvy=ss(-steep,3.2*steep,1,0)
mxinterp=lsim(modely,uinterp,timebase,[0,0])';
mvinterp=lsim(modelvy,uinterp,timebase,0)';


% hsp2_1=subplot(2,1,1)
hd3=figure(3)
plot(timebase(2:end),xinterp(2:end)-xinterp(2),'-','LineWidth',2)
hold on
plot(timebase,mxinterp,'--','LineWidth',2)
hold on
plot(timebase,uinterp,'g:','LineWidth',2)
axis([t_sideward_start,t_sideward_end,-5,5])
hl2_1=legend({'$y[m]$','$\hat{y}[m]$','$u_y[m/s]$'},'Interpreter', 'latex')
labels=get(gca, 'XTickLabel');
labels{end}='t[s]';
set(gca, 'XTickLabel',labels)
addpath(genpath('matlab2tikz'))
tikzname=[pwd,'/export/img_',filename,'_','y_ch1','.tikz'];
cleanfigure('handle',hd3,'targetResolution',resolution);
matlab2tikz(tikzname,'width','\fwidth','height','\fheight' ); %(working area — 160 mm by 247 mm)

% hsp2_2=subplot(2,1,2)
hd4=figure(4)
plot(timebase,vinterp,'-','LineWidth',2)
hold on
plot(timebase,mvinterp,'--','LineWidth',2)
hold on
plot(timebase,uinterp,'g:','LineWidth',2)

% %Real system Regression Points
% regstart=141
% regend=141.3
% timebasereg=regstart:dt:regend;
% index_vreg=find(tv>regstart):find(tv>regend);
% vinterpreg=interp1(tv(index_vreg),vs(index_vreg),timebasereg,'previous','extrap');
% vinterpreg(isnan(vinterpreg))=0;
% [r,m,b] = regression(timebasereg,vinterpreg)
% plot(timebase,m*timebase+b,'b--','Linewidth',1)
% regstart=147.8
% regend=148.1
% timebasereg=regstart:dt:regend;
% index_vreg=find(tv>regstart):find(tv>regend);
% vinterpreg=interp1(tv(index_vreg),vs(index_vreg),timebasereg,'previous','extrap');
% vinterpreg(isnan(vinterpreg))=0;
% [r,m,b] = regression(timebasereg,vinterpreg)
% plot(timebase,m*timebase+b,'b--','Linewidth',1)
% regstart=144
% regend=144.5
% timebasereg=regstart:dt:regend;
% index_vreg=find(tv>regstart):find(tv>regend);
% vinterpreg=interp1(tv(index_vreg),vs(index_vreg),timebasereg,'previous','extrap');
% vinterpreg(isnan(vinterpreg))=0;
% [r,m,b] = regression(timebasereg,vinterpreg)
% plot(timebase,m*timebase+b,'b--','Linewidth',1)
% 
% %Model Regression Points
% regstart=141
% regend=141.3
% timebasereg=regstart:dt:regend;
% index_vreg=find(timebase>regstart):find(timebase>regend);
% vinterpreg=interp1(timebase(index_vreg),mvinterp(index_vreg),timebasereg,'previous','extrap');
% [r,m,b] = regression(timebasereg,vinterpreg)
% plot(timebase,m*timebase+b,'r--','Linewidth',1)
% regstart=147.8
% regend=148.1
% timebasereg=regstart:dt:regend;
% index_vreg=find(timebase>regstart):find(timebase>regend);
% vinterpreg=interp1(timebase(index_vreg),mvinterp(index_vreg),timebasereg,'previous','extrap');
% [r,m,b] = regression(timebasereg,vinterpreg)
% plot(timebase,m*timebase+b,'r--','Linewidth',1)
% regstart=144
% regend=144.5
% timebasereg=regstart:dt:regend;
% index_vreg=find(timebase>regstart):find(timebase>regend);
% vinterpreg=interp1(timebase(index_vreg),mvinterp(index_vreg),timebasereg,'previous','extrap');
% [r,m,b] = regression(timebasereg,vinterpreg)
% plot(timebase,m*timebase+b,'r--','Linewidth',1)

axis([t_sideward_start,t_sideward_end,-3,3])
hl2_2=legend({'$v_y[m/s]$','$\hat{v}_y[m/s]$','$u_y[m/s]$'},'Interpreter', 'latex')
labels=get(gca, 'XTickLabel');
labels{end}='t[s]';
set(gca, 'XTickLabel',labels)
addpath(genpath('matlab2tikz'))
tikzname=[pwd,'/export/img_',filename,'_','y_ch2','.tikz'];
cleanfigure('handle',hd4,'targetResolution',resolution);
matlab2tikz(tikzname,'width','\fwidth','height','\fheight' ); %(working area — 160 mm by 247 mm)


%% FIGURE Z


timebase=t_upward_start:dt:t_upward_end-dt;
index_x=find(tx>t_upward_start):find(tx>t_upward_end)-1;
index_v=find(tv>t_upward_start):find(tv>t_upward_end)-1;
index_u=find(tu>t_upward_start):find(tu>t_upward_end)-1;
uinterp=interp1(tu(index_u),uz(index_u),timebase,'previous','extrap');
xinterp=interp1(tx(index_x),z(index_x),timebase,'previous','extrap');
vinterp=interp1(tx(index_v),vz(index_v),timebase,'previous','extrap');
uinterp(isnan(uinterp))=0;
xinterp(isnan(xinterp))=0;
vinterp(isnan(vinterp))=0;
ident_upwardchannel=iddata(vinterp',uinterp',dt);
% steep=0.3
% modelz=ss([-0.00,1.0;-0.00,-steep],[0;1.0*steep],[1,0],0)
% modelvz=ss(-steep,1.0*steep,1,0)
az=-1.9
bz=2.1
modelz=ss([-0.00,1.0;-0.00,az],[0;bz],[1,0],0)
modelvz=ss(az,bz,1,0)
mxinterp=lsim(modelz,uinterp,timebase,[0;0])';
mvinterp=lsim(modelvz,uinterp,timebase,0)';

hd5=figure(5)
plot(timebase(2:end),xinterp(2:end)-xinterp(2),'-','LineWidth',2)
hold on
plot(timebase,mxinterp,'--','LineWidth',2)
hold on
plot(timebase,uinterp,'g:','LineWidth',2)
% axis([t_upward_start,t_upward_end,-0.5,1.9])
axis([t_upward_start,t_upward_end,-0.3,2.1])
hl3_1=legend({'$z[m]$','$\hat{z}[m]$','$u_z[m/s]$'},'Interpreter', 'latex','Location','North','Orientation','Horizontal')
labels=get(gca, 'XTickLabel');
labels{end}='t[s]';
set(gca, 'XTickLabel',labels)
addpath(genpath('matlab2tikz'))
tikzname=[pwd,'/export/img_',filename,'_','z_ch1','.tikz'];
cleanfigure('handle',hd5,'targetResolution',resolution);
matlab2tikz(tikzname,'width','\fwidth','height','\fheight' ); %(working area — 160 mm by 247 mm)

% hsp3_2=subplot(2,1,2)
hd6=figure(6)
plot(timebase,vinterp,'-','LineWidth',2)
hold on
plot(timebase,mvinterp,'--','LineWidth',2)
hold on
plot(timebase,uinterp,'g:','LineWidth',2)
axis([t_upward_start,t_upward_end,-1.2,1.6])
hl3_2=legend({'$v_z[m/s]$','$\hat{v}_z[m/s]$','$u_z[m/s]$'},'Interpreter', 'latex','Location','South','Orientation','Horizontal')
labels=get(gca, 'XTickLabel');
labels{end}='t[s]';
set(gca, 'XTickLabel',labels)
addpath(genpath('matlab2tikz'))
tikzname=[pwd,'/export/img_',filename,'_','z_ch2','.tikz'];
cleanfigure('handle',hd6,'targetResolution',resolution);
matlab2tikz(tikzname,'width','\fwidth','height','\fheight' ); %(working area — 160 mm by 247 mm)


%% FIGURE YAW
timebase=t_yaw_start:dt:t_yaw_end;
index_x=find(tx>t_yaw_start):find(tx>t_yaw_end);
index_v=find(tv>t_yaw_start):find(tv>t_yaw_end);
index_u=find(tu>t_yaw_start):find(tu>t_yaw_end);
uinterp=interp1(tu(index_u),uyaw(index_u),timebase,'previous','extrap');
xinterp=interp1(tx(index_x),yaw(index_x),timebase,'previous','extrap');
vinterp=interp1(tx(index_v),vyaw(index_v),timebase,'previous','extrap');
uinterp(isnan(uinterp))=0;
xinterp(isnan(xinterp))=0;
vinterp(isnan(vinterp))=0;
ident_yawchannel=iddata(vinterp',uinterp',dt);

% fig4=figure(4)
steep=5
modelyaw=ss([-0.00,1.0;-0.00,-steep],[0;1.8*steep],[1,0],0);
modelvyaw=ss(-steep,1.8*steep,1,0);
yawsim=lsim(modelyaw,uinterp,timebase,[0;0])';
mxinterp=wrapToPi(yawsim);
mvinterp=lsim(modelvyaw,uinterp,timebase,0)';

% hsp4_1=subplot(2,1,1)
hd7=figure(7)
plot(timebase(2:end),xinterp(2:end)-xinterp(2),'-','LineWidth',2)
hold on
plot(timebase,mxinterp,'--','LineWidth',2)
hold on
plot(timebase,uinterp,'g:','LineWidth',2)
axis([t_yaw_start,t_yaw_end,-3.5,3.5])
hl4_1=legend({'${\Psi}[rad]$','$\hat{\Psi}[rad]$','$u_{\Psi}[rad/s]$'},'Interpreter', 'latex')
labels=get(gca, 'XTickLabel');
labels{end}='t[s]';
set(gca, 'XTickLabel',labels)
addpath(genpath('matlab2tikz'))
tikzname=[pwd,'/export/img_',filename,'_','yaw_ch1','.tikz'];
cleanfigure('handle',hd7,'targetResolution',resolution);
matlab2tikz(tikzname,'width','\fwidth','height','\fheight' ); %(working area — 160 mm by 247 mm)

% hsp4_2=subplot(2,1,2)
hd8=figure(8)
% vyaw_  =interp1(tx(index_x),vyaw(index_x),timebase,'previous','extrap');
% uyaw_  =interp1(tu(index_u),uyaw(index_u),timebase,'previous','extrap');
plot(timebase,vinterp,'-','LineWidth',2)
hold on
plot(timebase,mvinterp,'--','LineWidth',2)
hold on
plot(timebase,uinterp,'g:','LineWidth',2)
axis([t_yaw_start,t_yaw_end,-3.2,3.2])
hl4_2=legend({'${v_{\Psi}}[rad/s]$','$\hat{v}_{\Psi}[rad/s]$','$u_{\Psi}[rad/s]$'},'Interpreter', 'latex')
labels=get(gca, 'XTickLabel');
labels{end}='t[s]';
set(gca, 'XTickLabel',labels)
addpath(genpath('matlab2tikz'))
tikzname=[pwd,'/export/img_',filename,'_','yaw_ch2','.tikz'];
cleanfigure('handle',hd8,'targetResolution',resolution);
matlab2tikz(tikzname,'width','\fwidth','height','\fheight' ); %(working area — 160 mm by 247 mm)


% %% Export 
% size=[16.8, 12.8];
% set(fig1,'PaperUnits','centimeters','PaperSize',size,'PaperPosition',[0.0 -0.1 size(1) size(2)])
% print(fig1,'20170330_DJI_M100_simulator_Identification_Xchannel','-dpdf','-r0')
% set(fig2,'PaperUnits','centimeters','PaperSize',size,'PaperPosition',[0.0 -0.1 size(1) size(2)])
% print(fig2,'20170330_DJI_M100_simulator_Identification_Ychannel','-dpdf','-r0')
% set(fig3,'PaperUnits','centimeters','PaperSize',size,'PaperPosition',[0.0 -0.1 size(1) size(2)])
% print(fig3,'20170330_DJI_M100_simulator_Identification_Zchannel','-dpdf','-r0')
% set(fig4,'PaperUnits','centimeters','PaperSize',size,'PaperPosition',[0.0 -0.1 size(1) size(2)])
% print(fig4,'20170330_DJI_M100_simulator_Identification_Pchannel','-dpdf','-r0')
% 





















% % model1=ssest(ident_upwardchannel,2);
% % model2=ssregest(ident_upwardchannel,2);
% % model3=n4sid(ident_upwardchannel,2);
% % model4=pem(ident_upwardchannel,2);
%  Options = n4sidOptions;                                                   
%  Options.Display = 'on';                                                   
%  Options.Focus = [0 100];                                              
%  Options.InitialState = 'zero';                                                                                                                      
% ss1 = n4sid(ident_upwardchannel, 2, 'Form', 'companion', 'Ts', 0, Options);
% ss2 = n4sid(ident_upwardchannel, 2, 'Form', 'modal', 'Ts', 0, Options);
% ss3 = n4sid(ident_upwardchannel, 2, 'Form', 'canonical', 'Ts', 0, Options);
% ss3 = n4sid(ident_upwardchannel, 2, 'Form', 'free', 'Ts', 0, Options);
% [ss1.A,ss2.A,ss3.A,ss4.A]
% [ss1.B,ss2.B,ss3.B,ss4.B]
% [ss1.C,ss2.C,ss3.C,ss4.C]
% %[ss1.D,ss2.D,ss3.D,ss4.D]
% vzsim=lsim(ss4,uinterp',timebase,[0;0]);
% plot(tx(1:end-1),vz)
% hold on;
% plot(timebase,vzsim)


% index_u=find(tu>t_yaw_start):find(tu>t_yaw_end)-1;
% index_x=find(tx>t_yaw_start):find(tx>t_yaw_end)-1;
% timebase=t_yaw_start:dt:t_yaw_end-dt;
% uinterp=interp1(tu(index_u),uyaw(index_u),timebase,'previous','extrap');
% xinterp=interp1(tx(index_x),vyaw(index_x),timebase,'previous','extrap');
% uinterp(isnan(uinterp))=0;
% xinterp(isnan(xinterp))=0;
% ident_headingchannel=iddata(xinterp',uinterp',dt);

% %% PLOT
% figure(1)
% plot(ident_forwardchannel) 
% figure(2)
% plot(ident_sidewardchannel)
% figure(3)
% plot(ident_upwardchannel)
% figure(4)
% plot(ident_headingchannel)
% 
% %% Simulate
% 
% ss_forw  =ss(-0.5092,   1.6 , 1,0);
% ss_sidw  =ss(-0.5092,   1.6,  1,0);
% ss_height=ss(0,  1,  1,0);
% ss_yaw   =ss( 0, 1.6, 1,0);
% 
% % ss_forw  =ss(-0.5092,   1.458,  1,0);
% % ss_sidw  =ss(-0.5092,   1.458,  1,0);
% % ss_height=ss(0,  0.8827,  1,0);
% % ss_yaw   =ss( 0, 1.6, 1,0);
% %ss_forw  =ss(-0.5092,   1.9,  1,0);
% %ss_sidw  =ss(-0.5092,   1.9,  1,0);
% %ss_height=ss(0,  0.8827,  1,0);
% %ss_yaw   =ss( -5, 8, 1,0);
% 
% 
% %Timebase
% sim_timebase=(0:mean(dtv):tx(end))';
% %Create control vectors
% usim_forw  =interp1(tu,ux,sim_timebase,'previous','extrap');
% usim_sidw  =interp1(tu,uy,sim_timebase,'previous','extrap');
% usim_height=interp1(tu,uz,sim_timebase,'previous','extrap');
% usim_yaw   =interp1(tu,uyaw,sim_timebase,'previous','extrap');
% %Delete nan
% ind = find(isnan(usim_forw));
% usim_forw(ind)=0;
% ind = find(isnan(usim_sidw));
% usim_sidw(ind)=0;
% ind = find(isnan(usim_height));
% usim_height(ind)=0;
% ind = find(isnan(usim_yaw));
% usim_yaw(ind)=0;
% %Simulate
% xsim_forw  =lsim(ss_forw.A,ss_forw.B,ss_forw.C,ss_forw.D,usim_forw,sim_timebase,0);
% xsim_sidw  =lsim(ss_sidw.A,ss_sidw.B,ss_sidw.C,ss_sidw.D,usim_sidw,sim_timebase,0);
% xsim_height=lsim(ss_height.A,ss_height.B,ss_height.C,ss_height.D,usim_height,sim_timebase,0);
% sim_timebase_v=sim_timebase(2:end);
% vsim_height=(xsim_height(2:end)-xsim_height(1:end-1))./(sim_timebase(2:end)-sim_timebase(1:end-1));
% xsim_yaw   =lsim(ss_yaw.A,ss_yaw.B,ss_yaw.C,ss_yaw.D,usim_yaw,sim_timebase,0);
% vsim_yaw=(xsim_yaw(2:end)-xsim_yaw(1:end-1))./(sim_timebase(2:end)-sim_timebase(1:end-1));
% 
% 
% %% Plot
% 
% hdl=figure(1);
% hsp1=subplot(4,1,1)
% h1=stairs(tu,ux);
% hold on;
% h3=stairs(tv,vf,'-.');
% hold on;
% h2=stairs(sim_timebase,xsim_forw,'--');
% axis([65,105,-3,3])
% hline=refline([0 1]);
% set(hline,'LineStyle',':','Color',[0 0 0 0.2]);
% hline=refline([0 -1]);
% set(hline,'LineStyle',':','Color',[0 0 0 0.2]);
% killWhiteSpace_x=get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]  -[0.015 0 -0.025 0];
% set(gca, 'Position', get(gca, 'OuterPosition') - killWhiteSpace_x);
% lh1=legend({'$u_{f}[m/s]$','$\dot{x}_{V,m}[m/s]$','$\dot{x}_{V,p}[m/s]$'},...
%     'Interpreter', 'latex','Orientation','horizontal')
% legend boxoff
% 
% 
% hsp2=subplot(4,1,2)
% stairs(tu,uy);
% hold on;
% stairs(tv,vs,'-.');
% hold on
% stairs(sim_timebase,xsim_sidw,'--');
% axis([220,260,-3,3])
% hline=refline([0 1]);
% set(hline,'LineStyle',':','Color',[0 0 0 0.2]);
% hline=refline([0 -1]);
% set(hline,'LineStyle',':','Color',[0 0 0 0.2]);
% killWhiteSpace_x=get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1] -[0.015 0 -0.025 0];
% set(gca, 'Position', get(gca, 'OuterPosition') - killWhiteSpace_x);
% lh2=legend({'$u_{s}[m/s]$','$\dot{y}_{V,m}[m/s]$','$\dot{y}_{V,p}[m/s]$'},...
%     'Interpreter', 'latex','Visible', 'Off');
% legend boxoff
% 
% % subplot(4,1,3)
% % stairs(tu,uz);
% % hold on;
% % stairs(tx,z,'-.');
% % hold on;
% % stairs(sim_timebase,xsim_height,'--');
% % axis([500,550,-2,2])
% % killWhiteSpace_x=get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1] -[0.015 0 -0.025 0];
% % set(gca, 'Position', get(gca, 'OuterPosition') - killWhiteSpace_x);
% % legend({'$u_{z} [m/s]$','$Measurement: \dot{z}_W [m/s]$','$Model: \dot{z}_W [m/s]$'},...
% %     'Interpreter', 'latex','Location',[0.5,0.498,0,0],'Orientation','horizontal')
% 
% hsp3=subplot(4,1,3)
% stairs(tu,uz);
% hold on;
% stairs(tv,vz,'-.');
% hold on;
% stairs(sim_timebase_v,vsim_height,'--');
% axis([500,550,-2,2])
% hline=refline([0 1]);
% set(hline,'LineStyle',':','Color',[0 0 0 0.2]);
% hline=refline([0 -1]);
% set(hline,'LineStyle',':','Color',[0 0 0 0.2]);
% killWhiteSpace_x=get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1] -[0.015 0 -0.025 0];
% set(gca, 'Position', get(gca, 'OuterPosition') - killWhiteSpace_x);
% lh3=legend({'$u_{z}[m/s]$','$\dot{z}_{W,m}[m/s]$','$\dot{z}_{W,p}[m/s]$'},...
%     'Interpreter', 'latex','Orientation','horizontal')
% legend boxoff
% 
% hsp4=subplot(4,1,4)
% stairs(tu,uyaw);
% hold on;
% stairs(tv,vyaw,'-.');
% hold on;
% stairs(sim_timebase_v,vsim_yaw,'--');
% xlabel('$time$ $[s]$','Interpreter', 'latex');
% axis([450,500,-2,2])
% hline=refline([0 1]);
% set(hline,'LineStyle',':','Color',[0 0 0 0.2]);
% hline=refline([0 -1]);
% set(hline,'LineStyle',':','Color',[0 0 0 0.2]);
% killWhiteSpace_x=get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]  -[0.015 0 -0.025 0];
% set(gca, 'Position', get(gca, 'OuterPosition') - killWhiteSpace_x);
% lh4=legend({'$u_{\Psi_W}[\frac{rad}{s}]$','$\dot{\Psi}_{W,m}[\frac{rad}{s}]$','$\dot{\Psi}_{W,p}[\frac{rad}{s}]$'},...
%     'Interpreter', 'latex','Orientation','horizontal')
% legend boxoff
% 
% pos1 = get(hsp1,'position')
% pos2 = get(hsp2,'position')
% pos3 = get(hsp3,'position')
% pos4 = get(hsp4,'position')
% maxleft=0.06 %=max([pos1(1),pos2(1),pos3(1),pos4(1),pos5(1)])
% maxbot=max([pos1(2),pos2(2),pos3(2),pos4(2)])
% maxright  =0.91;%min([pos1(3),pos2(3),pos3(3),pos4(3)])
% maxtop  =max([pos1(4),pos2(4),pos3(4),pos4(4)])
% 
% distancebetweenplotsborder=0.1
% lowerborder=distancebetweenplotsborder/2
% distencebetweensubplots=(1-lowerborder)/4
% innerheightofsubplot=distencebetweensubplots-distancebetweenplotsborder
% % upboarder=(distencebetweensubplots-innerheightofsubplot)/2
% set(hsp1,'position',[maxleft,distancebetweenplotsborder+distencebetweensubplots*3,maxright,innerheightofsubplot])
% set(hsp2,'position',[maxleft,distancebetweenplotsborder+distencebetweensubplots*2,maxright,innerheightofsubplot])
% set(hsp3,'position',[maxleft,distancebetweenplotsborder+distencebetweensubplots*1,maxright,innerheightofsubplot])
% set(hsp4,'position',[maxleft,distancebetweenplotsborder+distencebetweensubplots*0,maxright,innerheightofsubplot])
% 
% set(lh1,'Position',[0.50,1-distencebetweensubplots*0-0.018,0,0])
% set(lh2,'Position',[0.50,1-distencebetweensubplots*1-0.018,0,0])
% set(lh3,'Position',[0.50,1-distencebetweensubplots*2-0.018,0,0])
% set(lh4,'Position',[0.50,1-distencebetweensubplots*3-0.018,0,0])
% 
% set(hdl,'Units','centimeters');
% pos = get(hdl,'Position');
% set(hdl,'PaperUnits','centimeters')
% set(hdl,'PaperSize',[8.4, 12.8])
% set(hdl,'PaperPosition',[0.0 -0.1 8.4 12.8])
% print(hdl,'20151208_Ardrone_Identification_2','-dpdf','-r0')
% set(hdl,'Units','centimeters');
% pos = get(hdl,'Position');
% set(hdl,'PaperUnits','centimeters')
% set(hdl,'PaperSize',[8.4, 12.8])
% set(hdl,'PaperPosition',[0.0 -0.1 8.4 12.8])
% %print(hdl,'20151208_Ardrone_Identification_2','-dpdf','-r0')
% legend boxoff
% 
% 
% 
