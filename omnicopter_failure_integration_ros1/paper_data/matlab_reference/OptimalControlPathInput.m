function [u_new,  X, d, d_r, d_R, v_e, BV, exitflag, v_final, C_,const] = OptimalControlPathInput(u_moment, f_r, ...
    pA_f,  u_max, R_r,  Rr,  weights, A, pA_m_v, max_vr)

% find desired force direction in body frame
d_r = R_r'*f_r;
d_r = d_r/norm(d_r);

d_R = Rr'*f_r;
d_R = d_R/norm(d_R);

RR_2 = cross([d_R(1);d_R(2);d_R(3)],[0;0;1]);

if abs(norm(RR_2)) <  1e-3
    C_ = [1 0;0 1; 0 0];
else
    RR_1 = cross(d_R,RR_2);
    C_ = [RR_2/norm(RR_2), RR_1/norm(RR_1)];
end


%C_ = C__(1:3,1:2);
% desired orientation in tangent space
v_r = C_\(d_r - (d_R'*d_r)*d_R);

% if norm(v_r) > 1
%    v_r(1) = sign(v_r(1))*min(abs(v_r(1)),1/sqrt(2)); 
%    v_r(2) = sign(v_r(1))*min(abs(v_r(2)),1/sqrt(2)); 
% end




if norm(v_r) > max_vr
    v_r = v_r*(max_vr/norm(v_r));
end

% find pA_f st u = pA_f*d_r
pA_f_ = pA_f*norm(f_r);

pA_m_v = -pA_m_v*skew(d_R)*C_;
% set linear constraints
nA_const = [pA_f_(:,1:3), pA_m_v;-pA_f_(:,1:3), -pA_m_v];

% initialize X
X0 = [0*v_r/2];

d_ = weights(4);
c_ = weights(3); % norm of added null  
b_ = weights(2); % angle
a_ = weights(1); % norm of f_e%fun =@(x)  quadobj(x,nAAn,R_r'*f_r,gamma);
fun =@(x)  quadobj(x,v_r);

% % nA_const = [nA;-nA];
  options = optimoptions('fmincon','Display','none','Algorithm','sqp',...
    'StepTolerance',1e-4,'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true);%,...
    %'MaxFunctionEvaluation',6000, 'StepTolerance',1e-12);%,...



nonlcon = @(x) norm_cons(x, u_moment, u_max, C_, d_R, nA_const, v_r);



[X,~, exitflag, ~] = fmincon(fun,X0,[],[],[],[],[],[],nonlcon,options);
%[X,~, exitflag, ~] = fmincon(fun,X0,[],[],[],[],-1/sqrt(2)*ones(2,1),1/sqrt(2)*ones(2,1),nonlcon,options);
%exitflag = 2;
%X = v_r;
const = nonlcon(X)
v_final = X(1:2,1);
v_e = X(1:2,1) - v_r;
BV = C_*X(1:2,1);
if sum(BV.^2) > 1
    nn = 0
else
    nn = sqrt(1-sum(BV.^2));
end
d = d_R*nn + BV;
X_new = [d];
%v = cross(d,d_R);
%v = cross(C_*X, d_R);
%theta = norm(v)
%Re = eye(3) + sin(theta)*skew(v)/theta + (1-cos(theta))*(v*v' - eye(3))/theta^2;

% momentum
%e_Rr_skew = -1/2*(Re' - Re);                     % rotation error skew
%e_Rr = [-e_Rr_skew(2,3) e_Rr_skew(1,3) -e_Rr_skew(1,2)]';   % rotation error

%X = v(1:2);

u_new = pA_f_ * d + u_moment + pA_m_v*X(1:2,1);



wrench = A*u_new;
%w_error = [d_R*norm(f_r);moment] - wrench;
%w_error_2 = (d - d_R)*norm(f_r);
end

%% objective function
function [y, grady] = quadobj(x,v_r)
v_ = x(1:2,1);

y = 1/2*(v_ - v_r)'*(v_ - v_r);

grady = zeros(length(x),1);
if nargout > 1
    grady(:,1) = [(v_ - v_r)];
end

end

%% nonlinear constraints
%  norm limit for each propeller speed
function [c,ceq,gradc,gradceq] = norm_cons(x, u_m, u_max_,C, d_0, nA_const, v_r)
v_ = x(1:2,1);

CC = C'*C;
if v_'* CC *v_ > 1
    v_ = v_/norm(v_);
    nn = 0;
else
    nn = sqrt(1-v_'* CC *v_);

end

c = zeros(13,1);
c(1:12) = nA_const*[nn*d_0+C*v_; v_] + [u_m;-u_m] - [u_max_*ones(6,1);zeros(6,1)];
c(13) = v_'*v_ - v_r'*v_r;

ceq = [];%zeros(1,1);
gradc = zeros(length(x),13);
if nargout > 2
    gradc(1:2,1:12) = (nA_const(:,1:3)*(C - d_0*(CC*v_)'/nn))' + nA_const(:,4:5)';
    gradc(1:2,13) = 2*v_;
    
end
gradceq = [];
end


