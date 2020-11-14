% ===== Trabalho 1 - ENGK38 =====
clear all;clc;

% Dois espaços de estados serao gerados: Com valores de Kfi e Ktheta = 1 e
% outro com Kfi e Ktheta = 2;

% ==========> Parametros com Kfi e Ktheta = 1 <============
Cx = 0.3;
Cy = 0.1;
g = 9.8;
Kz = 1.3;
tz = 0.4;
zmax = 1.0;
wfi = 4.47;
fimax = 0.26;
zetafi = 0.5;
wtheta = 4.47;
thetamax = 0.26;
zetatheta = 0.5;
Kpsi = 1;
tpsi = 0.3;
psimax = 1.74;
Kfi = 1;
Ktheta = 1;

% Modelo Linearizado

FT_1 = tf([Kfi*fimax*(wfi^2)*(-g)],conv([1 Cy 0],[1 2*zetafi*wtheta wfi^2]));


FT_2 = tf([Ktheta*thetamax*(wtheta^2)*g],conv([1 Cx 0],[1 2*zetatheta*wtheta wtheta^2]));


FT_3 = tf([Kz*zmax],[tz 1 0]);


FT_4 = tf([Kpsi*psimax],[tpsi 1 0]);


% Representaçao no espaço de estados

G1 = [FT_1 0 0 0;0 FT_2 0 0;0 0 FT_3 0;0 0 0 FT_4];
ssG1 = ss(G1);
[A1,B2_1,C1,D1] = ssdata(ssG1)

% Controlabilidade do sistema
cont = rank(ctrb(A1,B2_1));
posto_A = rank(A1);


% ==========> Parametros com Kfi e Ktheta = 2 <============
Kfi = 2;
Ktheta = 2;

% Modelo Linearizado

ft_1 = tf([Kfi*fimax*(wfi^2)*(-g)],conv([1 Cy 0],[1 2*zetafi*wtheta wfi^2]));


ft_2 = tf([Ktheta*thetamax*(wtheta^2)*g],conv([1 Cx 0],[1 2*zetatheta*wtheta wtheta^2]));


ft_3 = tf([Kz*zmax],[tz 1 0]);


ft_4 = tf([Kpsi*psimax],[tpsi 1 0]);


% Representaçao no espaço de estados

G2 = [ft_1 0 0 0;0 ft_2 0 0;0 0 ft_3 0;0 0 0 ft_4];
ssG2 = ss(G2);
[A2,B2_2,C2,D2] = ssdata(ssG2)

% Controlabilidade do sistema
cont2 = rank(ctrb(A2,B2_2));
posto_A2 = rank(A2);








% 
% %Funções de Transferencia - Modelo nao linear
% 
% FT1 = tf([K_fi*fimax*(w_fi^2)],[1 2*zeta_fi*w_theta w_fi^2]);
% 
% [num1,den1] = tfdata(FT1,'v');
% 
% FT2 = tf([K_theta*thetamax*(w_theta^2)],[1 2*zeta_theta*w_theta w_theta^2]);
% 
% [num2,den2] = tfdata(FT2,'v');
% 
% FT3 = tf([Kz*zmax],[tz 1]);
% 
% [num3,den3] = tfdata(FT3,'v');
% 
% FT4 = tf([K_psi*psimax],[t_psi 1]);
% 
% [num4,den4] = tfdata(FT4,'v');
% 

