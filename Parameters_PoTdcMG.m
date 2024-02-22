
%% circuit parameters
Tc= 0.0005;
Ns=5;% number of DGs
U_in = [100 80 80 100 120];

% RLC filters
R_t = 0*[0.5 0.2 0.2 0.5 0.5];
L_t = 2e-3*ones(1,Ns);
C_t = 2000*1e-6*ones(1,Ns);

% tie lines
Ns_bus=9; % number of Bus
R_bus_ij=zeros(Ns_bus,Ns_bus);
L_bus_ij=zeros(Ns_bus,Ns_bus);

R_bus_ij(1,4)=0.25;  L_bus_ij(1,4)=50e-6;
R_bus_ij(4,5)=0.15;  L_bus_ij(4,5)=100e-6;
R_bus_ij(4,6)=0.15;  L_bus_ij(4,6)=50e-6;
R_bus_ij(2,7)=0.25;  L_bus_ij(2,7)=50e-6;
R_bus_ij(7,8)=0.25;  L_bus_ij(7,8)=80e-6;
R_bus_ij(8,9)=0.25;  L_bus_ij(8,9)=50e-6;
R_bus_ij(3,9)=0.15;  L_bus_ij(3,9)=80e-6;

for i=1:Ns_bus
    for j=1:Ns_bus
        if R_bus_ij(i,j)~=0
            R_bus_ij(j,i)=R_bus_ij(i,j);
            L_bus_ij(i,j)=L_bus_ij(i,j);
        end
    end
end

% loads
R_L = 1*[10 10 10 10 10];

% 
%% Connections
% Topology of physical circuits
A_com_p =  [0 1 0 1 1;
    1 0 1 1 1;
    0 1 0 1 0;
    1 1 1 0 1;
    1 1 0 1 0];
% Topology of information layer
A_com =  [0 1 0 1 1;
    1 0 1 1 1;
    0 1 0 1 0;
    1 1 1 0 1;
    1 1 0 1 0];
D_comm = zeros(Ns,Ns);
% L_comm  = zeros(Ns,Ns);
for i=1:Ns
    for j=1:Ns
        if A_comm(i,j)~=0
            D_comm(i,i)=D_comm(i,i)+A_comm(i,j);
        end
    end
end
L_comm = D_comm - A_comm;
L_eig = eigs(-L_comm);


%% Inner control loops
kp_inner_I = 1*ones(1,Ns); % Out, voltage
ki_inner_I = 10*ones(1,Ns);
kp_inner_II = 0.01*ones(1,Ns); % In, current
ki_inner_II = 10*ones(1,Ns);

%%
I_max = [1 1 1.8 1.8 1.8];
theta = I_max;
Rd = [2 2 1 1 1];
V_ref=48; 
K_bar=5;  % Cooperation gain for voltage Estimator

%% settings of PoT
N_BC_node = 7;
N_BC_group = Ns;
Watting_time = 100*Tc;

ps=[0.75,0.7,0.65,0.62,0.57,0.55,0.5];
pa=[0.55,0.45,0.35,0.25,0.2,0.15,0.1];
alpha_single=zeros(1,7);  % 7 nodes/channel
beta_single=zeros(1,7);   % 7 nodes/channel
for i=1:N_BC_node
    alpha_single(i)=pa(i)/ps(i);
    beta_single(i)=0.2; 
end

Ay=5;
Au=0.4;
