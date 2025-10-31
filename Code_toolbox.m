% % Code INTERNET

% Physical parameters
mb = 300;    % kg
mw = 60;     % kg
bs = 1000;   % N/m/s
ks = 16000 ; % N/m
kt = 190000; % N/m

% State matrices
A = [ 0 1 0 0; [-ks -bs ks bs]/mb ; ...
      0 0 0 1; [ks bs -ks-kt -bs]/mw];
B = [ 0 0; 0 1e3/mb ; 0 0 ; [kt -1e3]/mw];
C = [1 0 0 0; 1 0 -1 0; A(2,:)];
%% 
D = [0 0; 0 0; B(2,:)];

qcar = ss(A,B,C,D);
qcar.StateName = {'body travel (m)';'body vel (m/s)';...
          'wheel travel (m)';'wheel vel (m/s)'};
qcar.InputName = {'r';'fs'};
qcar.OutputName = {'xs';'sd';'ab'};

tzero(qcar({'xs','ab'},'fs'))

zero(qcar('sd','fs'))

bodemag(qcar({'ab','sd'},'r'),'b',qcar({'ab','sd'},'fs'),'r',{1 100});
legend('Road disturbance (r)','Actuator force (fs)','location','SouthWest')
title(['Gain from road dist (r) and actuator force (fs)' newline ...
    'to body accel (ab) and suspension travel (sd)'])
%% 

ActNom = tf(1,[1/60 1]);

Wunc = makeweight(0.40,15,3);
unc = ultidyn('unc',[1 1],'SampleStateDim',5);
Act = ActNom*(1 + Wunc*unc);
Act.InputName = 'u';
Act.OutputName = 'fs';

rng('default')
bodemag(Act,'b',Act.NominalValue,'r+',logspace(-1,3,120))
legend('Actionneur générés aléatoirement','Actionneur valeur nominale')
%% 

Wroad = ss(0.07);  Wroad.u = 'd1';   Wroad.y = 'r';
Wact = 0.8*tf([1 50],[1 500]);  Wact.u = 'u';  Wact.y = 'e1';
Wd2 = ss(0.01);  Wd2.u = 'd2';   Wd2.y = 'Wd2';
Wd3 = ss(0.5);   Wd3.u = 'd3';   Wd3.y = 'Wd3';

HandlingTarget = 0.04 * tf([1/8 1],[1/80 1]);
ComfortTarget = 0.1 * tf([1/0.45 1],[1/150 1]);

Targets = [HandlingTarget ; ComfortTarget];
bodemag(qcar({'sd','ab'},'r')*Wroad,'b',Targets,'r--',{1,1000}), grid
legend('déplacement suspension (haut)/ accélaration (bas)','Ght (haut)/Gct (bas)')


%% 

% Three design points
beta = reshape([0.01 0.5 0.99],[1 1 3]);
Wsd = beta / HandlingTarget;
Wsd.u = 'sd';  Wsd.y = 'e3';
Wab = (1-beta) / ComfortTarget;
Wab.u = 'ab';  Wab.y = 'e2';

sdmeas  = sumblk('y1 = sd+Wd2');
abmeas = sumblk('y2 = ab+Wd3');
ICinputs = {'d1';'d2';'d3';'u'};
ICoutputs = {'e1';'e2';'e3';'y1';'y2'};
qcaric = connect(qcar(2:3,:),Act,Wroad,Wact,Wab,Wsd,Wd2,Wd3,...
                 sdmeas,abmeas,ICinputs,ICoutputs)

ncont = 1; % one control signal, u
nmeas = 2; % two measurement signals, sd and ab
K = ss(zeros(ncont,nmeas,3));
gamma = zeros(3,1);
for i=1:3
   [K(:,:,i),~,gamma(i)] = hinfsyn(qcaric(:,:,i),nmeas,ncont);
end

gamma

%% 
% Closed-loop models
K.u = {'sd','ab'};  K.y = 'u';
CL = connect(qcar,Act.Nominal,K,'r',{'xs';'sd';'ab'});

bodemag(qcar(:,'r'),'b', CL(:,:,1),'r-.', ...
   CL(:,:,2),'m-.', CL(:,:,3),'k-.',{1,140}), grid
legend('Boucle ouverte','Comfort','Balanced','Handling','location','SouthEast')
title('Body travel, suspension deflection, and body acceleration')

%% 
% Road disturbance
t = 0:0.0025:1;
roaddist = zeros(size(t));
roaddist(1:21) = 0.02;
roaddist(22:41) = 0;
roaddist(42:61) = 0.02;
roaddist(62:81) =0;
roaddist(82:101) = 0.02;

% Closed-loop model
SIMK = connect(qcar,Act.Nominal,K,'r',{'xs';'sd';'ab';'fs'});

% Simulate
p1 = lsim(qcar(:,1),roaddist,t);
y1 = lsim(SIMK(1:4,1,1),roaddist,t);
y2 = lsim(SIMK(1:4,1,2),roaddist,t);
y3 = lsim(SIMK(1:4,1,3),roaddist,t);

% Plot results
subplot(211)
plot(t,p1(:,1),'b',t,y1(:,1),'r.',t,y2(:,1),'m.',t,y3(:,1),'k.',t,roaddist,'g')
title('Body travel'), ylabel('x_s (m)')
legend('Boucle ouverte','Comfort','Balanced','Handling','Perturbation','location','SouthEast')
subplot(212)
plot(t,p1(:,3),'b',t,y1(:,3),'r.',t,y2(:,3),'m.',t,y3(:,3),'k.',t,roaddist,'g')
title('Body acceleration'), ylabel('a_b (m/s^2)')
legend('Boucle ouverte','Comfort','Balanced','Handling','Perturbation','location','SouthEast')

%% 
subplot(211)
plot(t,p1(:,2),'b',t,y1(:,2),'r.',t,y2(:,2),'m.',t,y3(:,2),'k.',t,roaddist,'g')
title('Suspension deflection'), xlabel('Time (s)'), ylabel('s_d (m)')
subplot(212)
plot(t,zeros(size(t)),'b',t,y1(:,4),'r.',t,y2(:,4),'m.',t,y3(:,4),'k.',t,roaddist,'g')
title('Control force'), xlabel('Time (s)'), ylabel('f_s (kN)')
legend('Open-loop','Comfort','Balanced','Handling','Road Disturbance','location','SouthEast')

%%
[Krob,rpMU] = musyn(qcaric(:,:,2),nmeas,ncont);

% Closed-loop model (nominal)
Krob.u = {'sd','ab'};
Krob.y = 'u';
SIMKrob = connect(qcar,Act.Nominal,Krob,'r',{'xs';'sd';'ab';'fs'});

%sinusoïde
roaddist = zeros(size(t));
roaddist(1:101) = 0.025*(1-cos(32*pi*t(1:101)));

%creneau
% roaddist = zeros(size(t));
% roaddist(1:21) = 0.00;
% roaddist(22:41) = 0;
% roaddist(42:61) = 0.02;
% roaddist(62:81) =0;
% roaddist(82:101) = 0.00;

% Simulate
p1 = lsim(qcar(:,1),roaddist,t);
y1 = lsim(SIMKrob(1:4,1),roaddist,t);

% Plot results
clf, figure(4)
plot(t,p1(:,1),'b',t,y1(:,1),'r',t,roaddist,'g')
title('Body travel'), ylabel('x_b (m)')
legend('Boucle ouverte','Balanced micro synthesis','Perturbation','location','SouthEast')

figure(5)
plot(t,p1(:,3),'b',t,y1(:,3),'r',t,roaddist,'g')
title('Body acceleration'), ylabel('a_b (m/s^2)')
legend('Boucle ouverte','Balanced micro synthesis','Perturbation','location','SouthEast')

%%

%sinusoïde
% roaddist = zeros(size(t));
% roaddist(1:101) = 0.025*(1-cos(32*pi*t(1:101)));

%creneau
roaddist = zeros(size(t));
roaddist(1:21) = 0.02;
roaddist(22:41) = 0;
roaddist(42:61) = 0.02;
roaddist(62:81) =0;
roaddist(82:101) = 0.02;

rng('default'), nsamp = 20;  clf

% Uncertain closed-loop model with balanced H-infinity controller
figure(10)
CLU = connect(qcar,Act,K(:,:,2),'r',{'xs','sd','ab'});
lsim(usample(CLU,nsamp),'y',CLU.Nominal,'r',roaddist,t)
title('Correcteur H infini')
legend('Incertitude','Actionneur nominal','location','SouthEast')

% Uncertain closed-loop model with balanced robust controller
figure(11)
CLU = connect(qcar,Act,Krob,'r',{'xs','sd','ab'});
lsim(usample(CLU,nsamp),'y',CLU.Nominal,'r',roaddist,t)
title('Correcteur mu-synthesis')
legend('Incertitude','Actionneur nominal','location','SouthEast')