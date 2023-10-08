clear all;
clc;

addpath ../thirdparty
addpath ../tools
addpath ../common
addpath ../../matlab_wrappers
addpath Noise_boxplot;
%% The four problem solvers
%% Test scenes on all solvers for different noises
errors = {};
% For px noise
% noise_levels =[10.0];
% planar noise
noise_levels = [0, 1e-3, 1e-2, 0.1, 0.5, 1, 2];

iterations = 1000;
for j = 1:size(noise_levels,2)
    for i = 1:iterations
        if mod(i, 50) == 0
            disp(i)
        end
        all_solver_errors = test(noise_levels(j));
        tn_err(i,:) = all_solver_errors(1,:);
        rot_err(i,:) = all_solver_errors(2,:);
        f_err(i,:) = all_solver_errors(3,:);
    end
    for k = 1:size(all_solver_errors,2)
        errors{k}(j).tn_err = log10(tn_err(:,k));
        errors{k}(j).rot_err = (rot_err(:,k));
        errors{k}(j).f_err = log10(f_err(:,k));
    end
end
%% Boxp plots
solver_names = { 'sH5f_2_c' 'sH5f_2_s' 'sH5f_3_c' 'sH5f_3_{s}' 'sH5f_3_{cf}'}
boxplot_noise_rot(errors(1:5), noise_levels, solver_names, 'Rotation error', 'boxplots\boxplot_noise_rot_sH5f_2_3.pdf');
boxplot_noise_tn(errors(1:5), noise_levels, solver_names, 'Translation error', 'boxplots\boxplot_noise_tn_sH5f_2_3.pdf');
boxplot_noise_f(errors(1:5), noise_levels, solver_names, 'Focal length error', 'boxplots\boxplot_noise_f_sH5f_2_3.pdf');

solver_names = { 'sH5f_2_c' 'sH5f_2_s'}
boxplot_noise_rot(errors(1:2), noise_levels, solver_names, 'Rotation error', 'boxplots\boxplot_noise_rot_sH5f_2.pdf');
boxplot_noise_tn(errors(1:2), noise_levels, solver_names, 'Translation error', 'boxplots\boxplot_noise_tn_sH5f_2.pdf');
boxplot_noise_f(errors(1:2), noise_levels, solver_names, 'Focal length error', 'boxplots\boxplot_noise_f_sH5f_2.pdf');

solver_names = { 'sH5f_3_c' 'sH5f_3_{s}' 'sH5f_3_{cf}'}
boxplot_noise_rot(errors(3:5), noise_levels, solver_names, 'Rotation error', 'boxplots\boxplot_noise_rot_sH5f_3.pdf');
boxplot_noise_tn(errors(3:5), noise_levels, solver_names, 'Translation error', 'boxplots\boxplot_noise_tn_sH5f_3.pdf');
boxplot_noise_f(errors(3:5), noise_levels, solver_names, 'Focal length error', 'boxplots\boxplot_noise_f_sH5f_3.pdf');

solver_names = { 'sH5f_2_s' 'sH5f_3_{cf}'}% 'Ef6+1_d_{mex}' 'Ef5+2' }
boxplot_noise_rot(errors([2,5,9,10]), noise_levels, solver_names, 'Rotation error', 'boxplots\boxplot_noise_rot_unknown_focal.pdf');
boxplot_noise_tn(errors([2,5,9,10]), noise_levels, solver_names, 'Translation error', 'boxplots\boxplot_noise_tn_unknown_focal.pdf');
boxplot_noise_f(errors([2,5,9,10]), noise_levels, solver_names, 'Focal length error', 'boxplots\boxplot_noise_f_unknown_focal.pdf');

%% Test for histogram
errors = {};

n_large_errors = zeros(10,1);

iterations = 5000;
for i = 1:iterations
    if mod(i, 50) == 0
        disp(i)
    end
    all_solver_errors = test(0);
    tn_err(i,:) = all_solver_errors(1,:);
    rot_err(i,:) = all_solver_errors(2,:);
    f_err(i,:) = all_solver_errors(3,:);
end
for k = 1:size(all_solver_errors,2)
    errors{k}.tn_err = (tn_err(:,k));
    % We clamp the minimum "0" values of error in rotation to the next
    % minimum value for better comparison :/
    rot_err(rot_err(:,k)==0,k) = 1e-7;
    errors{k}.rot_err = (rot_err(:,k));
    errors{k}.f_err = (f_err(:,k));
end
%% plots
solver_names = { 'sH5f_2_c' 'sH5f_2_s'}
histogram_rot(errors(1:2), [0], solver_names, "Rotation error frequency in "+int2str(iterations)+" iterations", 'histograms\histogram_rot_sH5f_2.pdf');
histogram_tn(errors(1:2), [0], solver_names, "Translation error frequency in "+int2str(iterations)+" iterations", 'histograms\histogram_tn_sH5f_2.pdf');
histogram_f(errors(1:2), [0], solver_names, "Focal length error frequency in "+int2str(iterations)+" iterations", 'histograms\histogram_f_sH5f_2.pdf');

solver_names = { 'sH5f_3_c' 'sH5f_3_{s}' 'sH5f_3_{cf}'}
histogram_rot(errors(3:5), [0], solver_names, "Rotation error frequency in "+int2str(iterations)+" iterations", 'histograms\histogram_rot_sH5f_3.pdf');
histogram_tn(errors(3:5), [0], solver_names, "Translation error frequency in "+int2str(iterations)+" iterations", 'histograms\histogram_tn_sH5f_3.pdf');
histogram_f(errors(3:5), [0], solver_names, "Focal length error frequency in "+int2str(iterations)+" iterations", 'histograms\histogram_f_sH5f_3.pdf');

solver_names = { 'sH5f_2_s' 'sH5f_3_{cf}' }% 'Ef6+1_d_{mex}' 'Ef5+2' }
histogram_rot(errors([2,5,9,10]), [0], solver_names, "Rotation error frequency in "+int2str(iterations)+" iterations", 'histograms\histogram_rot_unknown_focal.pdf');
histogram_tn(errors([2,5,9,10]), [0], solver_names, "Translation error frequency in "+int2str(iterations)+" iterations", 'histograms\histogram_tn_unknown_focal.pdf');
histogram_f(errors([2,5,9,10]), [0], solver_names, "Focal length error frequency in "+int2str(iterations)+" iterations", 'histograms\histogram_f_unknown_focal.pdf');




%% Test each scene
function errors = test(noise_amp)

f_gt = 5;
K_gt = f_gt*diag([1,1,1/f_gt]);
f_gen = 1;
K_gen = diag([f_gen, f_gen, 1]);

%noise
pixel = 1/1000;
noise = noise_amp *pixel;

% How many points to sample
Npoints = 1000;
Ncams = 7;

%% Various configurations of scene + motion

% sceneType = {'randomplanewithnoise' 'random'};
sceneType = {'randomplane' 'random'};

% sceneType = {'regularplane' 'random'};
% sceneType = {'randomplane' 'forward'};

%sceneType = {'randombox' 'random'};

ptFnd = 0;
while ptFnd == 0
    [Pgt M m m2] = GenerateScene(Npoints, 10, Ncams, 20, 35 , 0, noise,...%%10, 15 , 0, noise, ...
        [K_gt;K_gen;K_gen;K_gen;K_gen;K_gen;K_gen;K_gen], sceneType, [], [], [], true, noise_amp);
    try
        for i = 1:Ncams
            temp = Pgt{i} * [M; ones(1,size(M,2))];
            indices = find(temp(3,:) > 0);
            M = M(:, indices);
            for j = 1:Ncams
                m{j} = m{j}(:, indices);
            end
        end
        indicesOfPts = randperm(size(indices,2), 7);
        ptFnd = 1;
    catch
    end
end
%% Show the cameras
% for index_to_view = 1:6
%     ShowCameras({Pgt{index_to_view}}, {K_gt}, {m{index_to_view}}, M, true, false, true, indicesOfPts,{m2{index_to_view}});
% end

%% Pin hole camera
% For all solvers we need 6 2d points.
center1 = null(Pgt{1}); center1 = center1(1:3)/center1(4,1);
P = Pgt{1};
t = center1;
R = (K_gt\P(:,1:3))';

N = R'*M-R'*t;
q_d = [m{1}(:,[indicesOfPts(1:7)]);ones(1,7)];
M_c = M(:,indicesOfPts);

%% Gen. camera
c1 = null(Pgt{2}); c1 = c1(1:3)/c1(4);
c2 = null(Pgt{3}); c2 = c2(1:3)/c2(4);
c3 = null(Pgt{4}); c3 = c3(1:3)/c3(4);
c4 = null(Pgt{5}); c4 = c4(1:3)/c4(4);
c5 = null(Pgt{6}); c5 = c5(1:3)/c5(4);
c6 = null(Pgt{7}); c6 = c6(1:3)/c6(4);
c7 = null(Pgt{8}); c7 = c7(1:3)/c7(4);

R_gen1 = K_gen\Pgt{2}(:,1:3);
R_gen2 = K_gen\Pgt{3}(:,1:3);
R_gen3 = K_gen\Pgt{4}(:,1:3);
R_gen4 = K_gen\Pgt{5}(:,1:3);
R_gen5 = K_gen\Pgt{6}(:,1:3);
R_gen6 = K_gen\Pgt{7}(:,1:3);
R_gen7 = K_gen\Pgt{8}(:,1:3);

p_d = [(R_gen1'/K_gen)*[m{2}(:,indicesOfPts(1));1], ...
    (R_gen2'/K_gen)*[m{3}(:,indicesOfPts(2));1],...
    (R_gen3'/K_gen)*[m{4}(:,indicesOfPts(3));1],...
    (R_gen4'/K_gen)*[m{5}(:,indicesOfPts(4));1],...
    (R_gen5'/K_gen)*[m{6}(:,indicesOfPts(5));1],...
    (R_gen6'/K_gen)*[m{7}(:,indicesOfPts(6));1]];

p_d = p_d./p_d(3,:);

R_gt = R; t_gt = t;
N_gt = cross(N(:,indicesOfPts(1))-N(:,indicesOfPts(2)), N(:,indicesOfPts(1))-N(:,indicesOfPts(3)));
N_gt = N_gt/(-N_gt'*N(:,indicesOfPts(1)));
H_gt = R_gt-t_gt*N_gt';

%% Tests

%% Test sH5f_2_comp

p_d = [(R_gen1'/K_gen)*[m{2}(:,indicesOfPts(1));1], ...
    (R_gen2'/K_gen)*[m{3}(:,indicesOfPts(2));1],...
    (R_gen3'/K_gen)*[m{4}(:,indicesOfPts(3));1],...
    (R_gen4'/K_gen)*[m{5}(:,indicesOfPts(4));1],...
    (R_gen5'/K_gen)*[m{6}(:,indicesOfPts(5));1]];
p_d = p_d./p_d(3,:);

c_d=[c1,c2,c3,c4,c5];

[Hss, Nss, fss] = mex_sh5f_2_comp([q_d(:,1:5)',p_d(:,1:5)',c_d']);

Rs = []; ts = []; fs = [];
for i=1:size(Nss,2)
    %     [R,t] = extract_original_pose(Nss(:,i), Hss(:, 3*i-2:3*i));
    try
        sols = decomp_homo(Hss(:, 3*i-2:3*i));
        t = [];
        R = [];
        N = [];
        for k = 1:length(sols)
            temp = sols(k).T;
            t = [t, temp(1:3,1:3)'*temp(1:3,4)/norm(Nss(:,i),2)];
            R = [R, temp(1:3,1:3)'];
            N = [N, sols(k).n*norm(Nss(:,i),2)];
        end

        for j = 1:size(t,2)
            Rs = [Rs, R(:,3*j-2:3*j)];
            ts = [ts, t(:,j)];
            fs = [fs, fss(i)];
        end
    catch
    end
end

try
    ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
    error_sh5f_2_comp  = [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2)), abs(fs(ind)-f_gt)/f_gt]';
catch
    error_sh5f_2_comp = [inf;inf;inf];
end


%% Test sH5f_2_sturm

p_d = [(R_gen1'/K_gen)*[m{2}(:,indicesOfPts(1));1], ...
    (R_gen2'/K_gen)*[m{3}(:,indicesOfPts(2));1],...
    (R_gen3'/K_gen)*[m{4}(:,indicesOfPts(3));1],...
    (R_gen4'/K_gen)*[m{5}(:,indicesOfPts(4));1],...
    (R_gen5'/K_gen)*[m{6}(:,indicesOfPts(5));1]];
p_d = p_d./p_d(3,:);

c_d=[c1,c2,c3,c4,c5];

tol = [1e-18;0;0;0;0];

[Hss, Nss, fss] = mex_sH5f_2_sturm([q_d(:,1:5)',p_d(:,1:5)',c_d', tol]);

Rs = []; ts = []; fs = [];
for i=1:size(Nss,2)
    %     [R,t] = extract_original_pose(Nss(:,i), Hss(:, 3*i-2:3*i));
try
    sols = decomp_homo(Hss(:, 3*i-2:3*i));
    t = [];
    R = [];
    N = [];
    for k = 1:length(sols)
        temp = sols(k).T;
        t = [t, temp(1:3,1:3)'*temp(1:3,4)/norm(Nss(:,i),2)];
        R = [R, temp(1:3,1:3)'];
        N = [N, sols(k).n*norm(Nss(:,i),2)];
    end
    
    for j = 1:size(t,2)
        Rs = [Rs, R(:,3*j-2:3*j)];
        ts = [ts, t(:,j)];
        fs = [fs, fss(i)];
    end
catch
end
end
try
    ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
    error_sh5f_2_sturm  = [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2)), abs(fs(ind)-f_gt)/f_gt]';
catch
    error_sh5f_2_sturm = [inf;inf;inf];
end



%% Test sH5f_3_comp

p_d = [(R_gen1'/K_gen)*[m{2}(:,indicesOfPts(1));1], ...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(2));1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(3));1],...
    (R_gen2'/K_gen)*[m{3}(:,indicesOfPts(4));1],...
    (R_gen3'/K_gen)*[m{4}(:,indicesOfPts(5));1]];
p_d = p_d./p_d(3,:);

c_d=[c1,c1,c1,c2,c3];

[Hss, Nss, fss] = mex_sh5f_3_comp([q_d(:,1:5)',p_d(:,1:5)',c_d']);

Rs = []; ts = []; fs = [];
for i=1:size(Nss,2)
    %     [R,t] = extract_original_pose(Nss(:,i), Hss(:, 3*i-2:3*i));
    try
        sols = decomp_homo(Hss(:, 3*i-2:3*i));
        t = [];
        R = [];
        N = [];
        for k = 1:length(sols)
            temp = sols(k).T;
            t = [t, temp(1:3,1:3)'*temp(1:3,4)/norm(Nss(:,i),2)];
            R = [R, temp(1:3,1:3)'];
            N = [N, sols(k).n*norm(Nss(:,i),2)];
        end

        for j = 1:size(t,2)
            Rs = [Rs, R(:,3*j-2:3*j)];
            ts = [ts, t(:,j)];
            fs = [fs, fss(i)];
        end
    catch
    end
end

try
    ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
    error_sh5f_3_comp  = [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2)), abs(fs(ind)-f_gt)/f_gt]';
catch
    error_sh5f_3_comp = [inf;inf;inf];
end


%% Test sH5f_3_sturm

p_d = [(R_gen1'/K_gen)*[m{2}(:,indicesOfPts(1));1], ...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(2));1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(3));1],...
    (R_gen2'/K_gen)*[m{3}(:,indicesOfPts(4));1],...
    (R_gen3'/K_gen)*[m{4}(:,indicesOfPts(5));1]];
p_d = p_d./p_d(3,:);

c_d=[c1,c1,c1,c2,c3];

tol = [1e-18;0;0;0;0];

[Hss, Nss, fss] = mex_sh5f_3_sturm([q_d(:,1:5)',p_d(:,1:5)',c_d',tol]);

Rs = []; ts = []; fs = [];
for i=1:size(Nss,2)
    %     [R,t] = extract_original_pose(Nss(:,i), Hss(:, 3*i-2:3*i));
try
    sols = decomp_homo(Hss(:, 3*i-2:3*i));
    t = [];
    R = [];
    N = [];
    for k = 1:length(sols)
        temp = sols(k).T;
        t = [t, temp(1:3,1:3)'*temp(1:3,4)/norm(Nss(:,i),2)];
        R = [R, temp(1:3,1:3)'];
        N = [N, sols(k).n*norm(Nss(:,i),2)];
    end
    
    for j = 1:size(t,2)
        Rs = [Rs, R(:,3*j-2:3*j)];
        ts = [ts, t(:,j)];
        fs = [fs, fss(i)];
    end
catch
end

end

try
    ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
    error_sh5f_3_sturm  = [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2)), abs(fs(ind)-f_gt)/f_gt]';
catch
    error_sh5f_3_sturm = [inf;inf;inf];
end


%% Test sH5f_3_closed

p_d = [(R_gen1'/K_gen)*[m{2}(:,indicesOfPts(1));1], ...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(2));1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(3));1],...
    (R_gen2'/K_gen)*[m{3}(:,indicesOfPts(4));1],...
    (R_gen3'/K_gen)*[m{4}(:,indicesOfPts(5));1]];
p_d = p_d./p_d(3,:);

c_d=[c1,c1,c1,c2,c3];

[Hss, Nss, fss] = mex_sh5f_3_closed([q_d(:,1:5)',p_d(:,1:5)',c_d']);

Rs = []; ts = []; fs = [];
for i=1:size(Nss,2)
    %     [R,t] = extract_original_pose(Nss(:,i), Hss(:, 3*i-2:3*i));
    try
    sols = decomp_homo(Hss(:, 3*i-2:3*i));
    t = [];
    R = [];
    N = [];
    for k = 1:length(sols)
        temp = sols(k).T;
        t = [t, temp(1:3,1:3)'*temp(1:3,4)/norm(Nss(:,i),2)];
        R = [R, temp(1:3,1:3)'];
        N = [N, sols(k).n*norm(Nss(:,i),2)];
    end
    
    for j = 1:size(t,2)
        Rs = [Rs, R(:,3*j-2:3*j)];
        ts = [ts, t(:,j)];
        fs = [fs, fss(i)];
    end
    catch
    end
end

try
    ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
    error_sh5f_3_closed  = [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2)), abs(fs(ind)-f_gt)/f_gt]';
catch
    error_sh5f_3_closed = [inf;inf;inf];
end




%% 6+1 solver
R_gen1 = Pgt{2}(:,1:3);
R_gen2 = Pgt{3}(:,1:3);
t_gen1 = Pgt{2}(:,4);

% p_d = [m{2}(:,indicesOfPts(1:6)); ones(1,6)];
p_d = [(R_gen1'/K_gen)*[m{2}(:,indicesOfPts(1)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(2)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(3)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(4)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(5)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(6)); 1]];
p_d = [p_d, (R_gen2'/K_gen)*[m{3}(:,indicesOfPts(7)); 1]];
p_d = p_d./p_d(3,:);
c1 = null(Pgt{2}); c1 = c1(1:3)/c1(4);
c2 = null(Pgt{3}); c2 = c2(1:3)/c2(4);
c_d = [c1,c2];

try
    [Rs, ts, fs] = grelpose_6_1(q_d, p_d, [c1,c2]);
    
    ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
    error_grelpose61  = [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2)), abs(fs(ind)-f_gt)/f_gt]';
catch
    error_grelpose61 = [inf;inf;inf];
end

%% 6+1_d solver
R_gen1 = Pgt{2}(:,1:3);
R_gen2 = Pgt{3}(:,1:3);
t_gen1 = Pgt{2}(:,4);

% p_d = [m{2}(:,indicesOfPts(1:6)); ones(1,6)];
p_d = [(R_gen1'/K_gen)*[m{2}(:,indicesOfPts(1)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(2)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(3)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(4)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(5)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(6)); 1]];
p_d = [p_d, (R_gen2'/K_gen)*[m{3}(:,indicesOfPts(7)); 1]];
p_d = p_d./p_d(3,:);
c1 = null(Pgt{2}); c1 = c1(1:3)/c1(4);
c2 = null(Pgt{3}); c2 = c2(1:3)/c2(4);
c_d = [c1,c2];

try
    [Rs, ts, fs] = grelpose_6_1_d(q_d, p_d, [c1,c2]);
    
    ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
    error_grelpose61d  = [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2)), abs(fs(ind)-f_gt)/f_gt]';
catch
    error_grelpose61d = [inf;inf;inf];
end


%% 6+1 mex solver
R_gen1 = Pgt{2}(:,1:3);
R_gen2 = Pgt{3}(:,1:3);
t_gen1 = Pgt{2}(:,4);

% p_d = [m{2}(:,indicesOfPts(1:6)); ones(1,6)];
p_d = [(R_gen1'/K_gen)*[m{2}(:,indicesOfPts(1)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(2)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(3)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(4)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(5)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(6)); 1]];
p_d = [p_d, (R_gen2'/K_gen)*[m{3}(:,indicesOfPts(7)); 1]];
p_d = p_d./p_d(3,:);
c1 = null(Pgt{2}); c1 = c1(1:3)/c1(4);
c2 = null(Pgt{3}); c2 = c2(1:3)/c2(4);
c_d = [c1,c2];

try
    [Rs, ts, fs] = mex_grelpose_6_1([q_d,p_d,c_d]);
    
    ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
    error_grelpose61m  = [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2)), abs(fs(ind)-f_gt)/f_gt]';
catch
    error_grelpose61m = [inf;inf;inf];
end

%% 6+1_d mex solver
R_gen1 = Pgt{2}(:,1:3);
R_gen2 = Pgt{3}(:,1:3);
t_gen1 = Pgt{2}(:,4);

% p_d = [m{2}(:,indicesOfPts(1:6)); ones(1,6)];
p_d = [(R_gen1'/K_gen)*[m{2}(:,indicesOfPts(1)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(2)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(3)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(4)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(5)); 1],...
    (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(6)); 1]];
p_d = [p_d, (R_gen2'/K_gen)*[m{3}(:,indicesOfPts(7)); 1]];
p_d = p_d./p_d(3,:);
c1 = null(Pgt{2}); c1 = c1(1:3)/c1(4);
c2 = null(Pgt{3}); c2 = c2(1:3)/c2(4);
c_d = [c1,c2];

try
    [Rs, ts, fs] = mex_grelpose_6_1_d([q_d,p_d,c_d]);
    
    ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
    error_grelpose61dm  = [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2)), abs(fs(ind)-f_gt)/f_gt]';
catch
    error_grelpose61dm = [inf;inf;inf];
end


%% 5+2 solver
try
    R_gen1 = K_gen\Pgt{2}(:,1:3);
R_gen2 = K_gen\Pgt{3}(:,1:3);
R_gen3 = K_gen\Pgt{4}(:,1:3);

p_d = (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(1:5)); ones(1,5)];
p_d = [p_d, (R_gen2'/K_gen)*[m{3}(:,indicesOfPts(6)); 1], (R_gen3'/K_gen)*[m{4}(:,indicesOfPts(7)); 1]];
p_d = p_d./sqrt(sum(p_d.^2));

c1 = null(Pgt{2}); c1 = c1(1:3)/c1(4);
c2 = null(Pgt{3}); c2 = c2(1:3)/c2(4);
c3 = null(Pgt{4}); c3 = c3(1:3)/c3(4);
c2 = c2-c1; c3=c3-c1;
c_d = [zeros(3,5),c2,c3];


[t,R,f] = point7(p_d, c_d, q_d(1:2,:));
ts = []; Rs = [];fs=[];
for i = 1:size(t,2)
    Rs = [Rs, reshape(R(:,i),3,3)'];
    ts = [ts, -reshape(R(:,i),3,3)'*t(:,i)];
    fs = [fs, f(i)];
end
ts = ts + c1;

    ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
    error_grelpose52  = [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2)), abs(fs(ind)-f_gt)/f_gt]';
catch
    error_grelpose52 = [inf;inf;inf];
end


%% Compiling all the solvers' errors
errors = [error_sh5f_2_comp, error_sh5f_2_sturm, error_sh5f_3_comp, error_sh5f_3_sturm, error_sh5f_3_closed,...
    error_grelpose61, error_grelpose61d, error_grelpose61m, error_grelpose61dm, error_grelpose52];
% disp(errors);
end

%%
% %%
% function new_c = force_h33_0(q,p,c,H_gt,N_gt)
% [~,~,~, ~, Rtransform2, shift, Ktransform] = transform_sh2(q,p,c);
% syms v;
% val = Rtransform2(3,:)* (H_gt(:,3) + N_gt(3)*(shift-[0;0;v]) );
% new_c = [0;0;eval(solve(val,v))];
% end






























% 
% 
% %% Test sH2 2vars
% p_d = [(R_gen1'/K_gen)*[m{2}(:,indicesOfPts(1));1], ...
%     (R_gen2'/K_gen)*[m{3}(:,indicesOfPts(2));1],...
%     (R_gen3'/K_gen)*[m{4}(:,indicesOfPts(3));1],...
%     (R_gen4'/K_gen)*[m{5}(:,indicesOfPts(4));1],...
%     (R_gen5'/K_gen)*[m{6}(:,indicesOfPts(5));1]];
% p_d = p_d./p_d(3,:);
% 
% [Hss, fss, Nss] = minimal_h50f_2vars([q_d(:,1),q_d(:,2),q_d(:,3),q_d(:,4),q_d(:,5)],...
%     [p_d(:,1),p_d(:,2),p_d(:,3),p_d(:,4),p_d(:,5)],...
%     [c1,c2,c3,c4,c5]);
% 
% Rs = []; ts = []; fs = [];
% for i=1:size(Nss,2)
%     %     [R,t] = extract_original_pose(Nss(:,i), Hss(:, 3*i-2:3*i));
%     sols = decomp_homo(Hss(:, 3*i-2:3*i));
%     t = [];
%     R = [];
%     N = [];
%     for k = 1:length(sols)
%         temp = sols(k).T;
%         t = [t, temp(1:3,1:3)'*temp(1:3,4)/norm(Nss(:,i),2)];
%         R = [R, temp(1:3,1:3)'];
%         N = [N, sols(k).n*norm(Nss(:,i),2)];
%     end
%     
%     for j = 1:size(t,2)
%         Rs = [Rs, R(:,3*j-2:3*j)];
%         ts = [ts, t(:,j)];
%         fs = [fs, fss(i)];
%     end
% end
% try
%     ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
%     error_sh2_2vars = [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2)), abs(fs(ind)-f_gt)/f_gt]';
% catch
%     error_sh2_2vars  = [inf;inf;inf];
% end
% 
% %% Test sH2 nondeg
% p_d = [(R_gen1'/K_gen)*[m{2}(:,indicesOfPts(1));1], ...
%     (R_gen2'/K_gen)*[m{3}(:,indicesOfPts(2));1],...
%     (R_gen3'/K_gen)*[m{4}(:,indicesOfPts(3));1],...
%     (R_gen4'/K_gen)*[m{5}(:,indicesOfPts(4));1],...
%     (R_gen5'/K_gen)*[m{6}(:,indicesOfPts(5));1]];
% p_d = p_d./p_d(3,:);
% 
% c_d=[c1,c2,c3,c4,c5];
% % transpose((H_gt/K_gt)*q(:,3))*cross(p(:,3),c(:,3))
% % new_c = force_h33_0(q_d,p_d,c_d,H_gt,N_gt);
% % c_d = c_d - new_c;
% % t_gt = t_gt - new_c;
% [Hss, fss, Nss] = minimal_h50f_sh2([q_d(:,1),q_d(:,2),q_d(:,3),q_d(:,4),q_d(:,5)],...
%     [p_d(:,1),p_d(:,2),p_d(:,3),p_d(:,4),p_d(:,5)],...
%     c_d, R_gt, t_gt, K_gt, N_gt);
% % [Hss, Nss, fss] = mexh5f([q_d(:,1:5)',p_d(:,1:5)',c_d']);
% 
% Rs = []; ts = []; fs = [];
% for i=1:size(Nss,2)
%     %     [R,t] = extract_original_pose(Nss(:,i), Hss(:, 3*i-2:3*i));
%     sols = decomp_homo(Hss(:, 3*i-2:3*i));
%     t = [];
%     R = [];
%     N = [];
%     for k = 1:length(sols)
%         temp = sols(k).T;
%         t = [t, temp(1:3,1:3)'*temp(1:3,4)/norm(Nss(:,i),2)];
%         R = [R, temp(1:3,1:3)'];
%         N = [N, sols(k).n*norm(Nss(:,i),2)];
%     end
%     
%     for j = 1:size(t,2)
%         Rs = [Rs, R(:,3*j-2:3*j)];
%         ts = [ts, t(:,j)];
%         fs = [fs, fss(i)];
%     end
% end
% ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
% error_sh2  = [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2)), abs(fs(ind)-f_gt)/f_gt]';
% 
% %% Test sH3
% 
% p_d = [(R_gen1'/K_gen)*[m{2}(:,indicesOfPts(1));1], ...
%     (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(2));1],...
%     (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(3));1],...
%     (R_gen2'/K_gen)*[m{3}(:,indicesOfPts(4));1],...
%     (R_gen3'/K_gen)*[m{4}(:,indicesOfPts(5));1]];
% p_d = p_d./p_d(3,:);
% 
% [Hss, fss, Nss] = minimal_h50f_sh3([q_d(:,1),q_d(:,2),q_d(:,3),q_d(:,4),q_d(:,5)],...
%     [p_d(:,1),p_d(:,2),p_d(:,3),p_d(:,4),p_d(:,5)],...
%     [c1,c1,c1,c2,c3]);
% 
% Rs = []; ts = []; fs = [];
% for i=1:size(Nss,2)
%     %     [R,t] = extract_original_pose(Nss(:,i), Hss(:, 3*i-2:3*i));
%     sols = decomp_homo(Hss(:, 3*i-2:3*i));
%     t = [];
%     R = [];
%     N = [];
%     for k = 1:length(sols)
%         temp = sols(k).T;
%         t = [t, temp(1:3,1:3)'*temp(1:3,4)/norm(Nss(:,i),2)];
%         R = [R, temp(1:3,1:3)'];
%         N = [N, sols(k).n*norm(Nss(:,i),2)];
%     end
%     
%     for j = 1:size(t,2)
%         Rs = [Rs, R(:,3*j-2:3*j)];
%         ts = [ts, t(:,j)];
%         fs = [fs, fss(i)];
%     end
% end
% ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
% error_sh3 = [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2)), abs(fs(ind)-f_gt)/f_gt]';
% 
% %% Test p5pf+N
% t_gen1 = Pgt{2}(:,4);
% t_gen2 = Pgt{3}(:,4);
% 
% p1 = [m{2}(:,indicesOfPts(1:3));ones(1,3)];
% p2 = [m{3}(:,indicesOfPts(1:3));ones(1,3)];
% 
% R_gen4 = Pgt{2}(:,1:3);
% R_gen5 = Pgt{2}(:,1:3);
% R_gen6 = Pgt{3}(:,1:3);
% R_gen7 = Pgt{3}(:,1:3);
% 
% c4 = null(Pgt{2}); c4 = c4(1:3)/c4(4);
% c5 = null(Pgt{2}); c5 = c5(1:3)/c5(4);
% c6 = null(Pgt{3}); c6 = c6(1:3)/c6(4);
% c7 = null(Pgt{3}); c7 = c7(1:3)/c7(4);
% 
% p_d = [(R_gen4')*[m{2}(:,indicesOfPts(1));1], ...
%     (R_gen5')*[m{2}(:,indicesOfPts(2));1], ...
%     (R_gen6')*[m{3}(:,indicesOfPts(3));1],...
%     (R_gen7')*[m{3}(:,indicesOfPts(4));1],...
%     (R_gen7')*[m{3}(:,indicesOfPts(5));1]];
% p_d = p_d./p_d(3,:);
% c_d = [c4,c5,c6, c7,c7];
% 
% N = compute_N_gen(p1, p2, K_gen, R_gen1, t_gen1, K_gen, R_gen2, t_gen2);
% [Rs, ts, fs] = p5pf_solver(q_d(:,1:5), p_d, c_d, N);
% 
% try
%     ind = find( min(sum(abs(ts-t_gt))) == sum(abs(ts-t_gt)),1 );
%     error_p4pn  = [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'* Rs(:, 3*ind-2:3*ind))-1)/2)), abs(fs(ind)-f_gt)/f_gt]';
% catch
%     error_p4pn  = [inf;inf;inf];
% end


