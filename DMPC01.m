clear 
run('Initialize.m');
tmax=10;  % 仿真的总时间
% 真实系统
x=cell(1,Ns);
x{1,1}=zeros(2,tmax+1);
x{1,2}=zeros(2,tmax+1);
x{1,1}(:,1)=x10;  % 初始状态
x{1,2}(:,1)=x20;  % 初始状态
u=cell(1,Ns);
u{1,1}=zeros(1,tmax);
u{1,2}=zeros(1,tmax);
% nominal model
x_hat=cell(1,Ns);
x_hat{1,1}=zeros(2,tmax+1);
x_hat{1,2}=zeros(2,tmax+1);
u_hat=cell(1,Ns);
u_hat{1,1}=zeros(1,tmax);
u_hat{1,2}=zeros(1,tmax);
% 给邻居的预测
x_til=cell(1,Ns);
x_til{1,1}=zeros(2,tmax+N);
x_til{1,2}=zeros(2,tmax+N);
for i=1:Ns  % 创建初始预测序列，准备发给邻居x[i]k=(Aii+BiKi)kx[i]0
    for k=1:N  % 实际上是k=0~N-1
        x_til{1,i}(:,k)=(A{i,i}+B{1,i}*K{1,i})^(k-1)*x{1,i}(:,1);
    end
end

for t=1:tmax
    for i=1:Ns % 对第i个subsystem
        % MPC optimization problem
        % 创建决策变量
        clear x_pred u_pred
        x_pred=sdpvar(2,N+1);
        u_pred=sdpvar(1,N);
        % 添加约束条件
        C=[Z{1,i}(1,1) <= (x{1,i}(1,t)-x_pred(1,1)) <= Z{1,i}(1,2);
            Z{1,i}(2,1) <= (x{1,i}(2,t)-x_pred(2,1)) <= Z{1,i}(2,2);
            %x_pred(:,N+1)'*P*x_pred(:,N+1) <= 0.1 % 不得不屏蔽掉的非线性约束
            %x_pred(1,N+1)^2+x_pred(2,N+1)^2 <= 0.1
            W{1,i}(1,1) <= x_pred(1,N+1) <= W{1,i}(1,2);
            W{1,i}(2,1) <= x_pred(2,N+1) <= W{1,i}(2,2)
            ];
        sum=zeros(2,N);
        for k=1:N
            C=[C;
                E{1,i}(1,1) <= (x_pred(1,k)-x_til{1,i}(1,t+k-1)) <= E{1,i}(1,2);
                E{1,i}(2,1) <= (x_pred(2,k)-x_til{1,i}(2,t+k-1)) <= E{1,i}(2,2);
                ub{1,i}(1) <= u_pred(i) <= ub{1,i}(2)
                ];
            for j=1:Ns
                if A_comm(i,j)>0
                    sum(:,k)=sum(:,k)+A{i,j}*x_til{1,j}(:,t+k-1);
                end
            end
            C=[C;
                x_pred(:,k+1) == A{i,i}*x_pred(:,k)+B{1,i}*u_pred(k)+sum(:,k)
                ];
        end
        % 配置
        % ops=sdpsettings('verbose',0,'solver','qpoases');
        ops=sdpsettings('verbose',0);
        % 目标函数
        z=0.5*x_pred(:,N+1)'*P{1,i}*x_pred(:,N+1);
        for k=1:N
            z=z+0.5*x_pred(:,k)'*Q{1,i}*x_pred(:,k);
        end
        % 求解
        result=optimize(C,z,ops);
        if result.problem==0  % problem=0代表求解成功
            %value(x_pred)
            %value(z)
        else
            disp('求解出错');
        end
        u{1,i}(t)=u_pred(1)+K{1,i}*(x{1,i}(:,t)-x_pred(:,1));
        sum=zeros(2,1);
        for j=1:Ns
            if A_comm(i,j)>0
                sum=sum+A{i,j}*x{1,j}(:,t);
            end
        end
        x{1,i}(:,t+1) = A{i,i}*x{1,i}(:,t)+B{1,i}*u{1,i}(t)+sum;
        x_til{1,i}(:,t+N)=x_pred(:,N+1);
        x_hat{1,i}(:,t)=x_pred(:,1);
    end
end

% 两个子系统分别的状态演化
t=1:tmax;
figure;
plot(t,x{1,1}(1,t),t,x{1,1}(2,t));
legend('x1','x2')
% axis([0 29 -2.5 2.5]);
figure;
t=1:tmax;
plot(t,x{1,2}(1,t),t,x{1,2}(2,t));
legend('x1','x2')

% 每个状态变量和预测值、优化值的对比
figure;
subplot(2,2,1),plot(t,x{1,1}(1,t),t,x_til{1,1}(1,t),t,x_hat{1,1}(1,t));
subplot(2,2,2),plot(t,x{1,1}(2,t),t,x_til{1,1}(2,t),t,x_hat{1,1}(2,t));
subplot(2,2,3),plot(t,x{1,2}(1,t),t,x_til{1,2}(1,t),t,x_hat{1,2}(1,t));
subplot(2,2,4),plot(t,x{1,2}(2,t),t,x_til{1,2}(2,t),t,x_hat{1,2}(2,t));