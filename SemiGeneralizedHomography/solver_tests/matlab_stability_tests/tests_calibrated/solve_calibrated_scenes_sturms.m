clear all;
clc;
addpath ../thirdparty
addpath ../tools
addpath ../common
addpath ../../matlab_wrappers
addpath Noise_boxplot;
% %% The four problem solvers
% % Test scenes on all solvers for different noises
% errors = {};
% % for planar noise 
% % noise_levels = [0.01,0.05,0.1];
% % for noise 
% noise_levels = [0, 0.01,0.1,1.0,2.0];
% for j = 1:size(noise_levels,2)
%     for i = 1:1000
%         i
%         all_solver_errors = test(noise_levels(j));
%         tn_err(i,:) = all_solver_errors(1,:);
%         rot_err(i,:) = all_solver_errors(2,:);
%     end
%     for k = 1:11
%         errors{k}(j).tn_err = log10(tn_err(:,k));
%         errors{k}(j).rot_err = (rot_err(:,k));
%     end
% end
% %% Plots
% solver_names = {'sH5_2_s', 'sH5_2_c', }
% boxplot_noise_rot(errors(1:2), noise_levels, solver_names, '');
% boxplot_noise_tn(errors(1:2), noise_levels, solver_names, '');
% 
% solver_names = {'sH5_3_s', 'sH5_3_c', 'sH5_3_{cf}'}
% boxplot_noise_rot(errors(3:5), noise_levels, solver_names, '');
% boxplot_noise_tn(errors(3:5), noise_levels, solver_names, '');
% 
% solver_names = {'sH45_2_e', 'sH45_2_o', 'sH45_2_d'}
% boxplot_noise_rot(errors(6:8), noise_levels, solver_names, '');
% boxplot_noise_tn(errors(6:8), noise_levels, solver_names, '');
% 
% solver_names = {'sH45_3_e', 'sH45_3_o', 'sH45_3_d'}
% boxplot_noise_rot(errors(9:11), noise_levels, solver_names, '');
% boxplot_noise_tn(errors(9:11), noise_levels, solver_names, '');

%% Test for histogram
errors = {}
tol = 10:4:35;% 10:2:40;

iterations = 5000
for i = 1:iterations
    all_solver_errors = test(0,tol);
    tn_err(i,:) = all_solver_errors(1,:);
    rot_err(i,:) = all_solver_errors(2,:);
end
for k = 1:size(all_solver_errors,2)
    errors{k}.tn_err = (tn_err(:,k));
    % We clamp the minimum "0" values of error in rotation to the next
    % minimum value for better comparison :/
    rot_err(rot_err(:,k)==0,k) = 1e-7;
    errors{k}.rot_err = (rot_err(:,k));
end
%% Plots 
solver_names = {};
tols = size(tol,2);
for t=1:tols
    solver_names{t} = ['sH5_2_{s', num2str(tol(t)), '}'];
end
histogram_rot(errors(1:tols), [0], solver_names, "Rotation error frequency in "+int2str(iterations)+" iterations", 'sturm_histograms\histogram_rot_sH5_2_sturms.pdf');
histogram_tn(errors(1:tols), [0], solver_names, "Translation error frequency in "+int2str(iterations)+" iterations", 'sturm_histograms\histogram_tn_sH5_2_sturms.pdf');

solver_names = {};
for t=1:size(tol,2)
    solver_names{t} = ['sH5_3_{s', num2str(tol(t)), '}'];
end
histogram_rot(errors(tols+1:tols*2), [0], solver_names, "Rotation error frequency in "+int2str(iterations)+" iterations", 'sturm_histograms\histogram_rot_sH5_3_sturms.pdf');
histogram_tn(errors(tols+1:tols*2), [0], solver_names, "Translation error frequency in "+int2str(iterations)+" iterations", 'sturm_histograms\histogram_tn_sH5_3_sturms.pdf');


solver_names = {};
for t=1:size(tol,2)
    solver_names{t} = ['sH45_2_{d', num2str(tol(t)), '}'];
end
histogram_rot(errors(tols*2+1:tols*3), [0], solver_names, "Rotation error frequency in "+int2str(iterations)+" iterations", 'sturm_histograms\histogram_rot_sH45_2_dan.pdf');
histogram_tn(errors(tols*2+1:tols*3), [0], solver_names, "Translation error frequency in "+int2str(iterations)+" iterations", 'sturm_histograms\histogram_tn_sH45_2_dan.pdf');

solver_names = {};
for t=1:size(tol,2)
    solver_names{t} = ['sH45_3_{d', num2str(tol(t)), '}'];
end
histogram_rot(errors(tols*3+1:tols*4), [0], solver_names, "Rotation error frequency in "+int2str(iterations)+" iterations", 'sturm_histograms\histogram_rot_sH45_3_dan.pdf');
histogram_tn(errors(tols*3+1:tols*4), [0], solver_names, "Translation error frequency in "+int2str(iterations)+" iterations", 'sturm_histograms\histogram_tn_sH45_3_dan.pdf');

 
%% Test each scene
function errors = test(noise_amp, tol)

f = 5;
K_pinhole = f*diag([1,1,1/f]);
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

% sceneType = {'randomplanewithnoise' 'sideway'};
sceneType = {'randomplane' 'random'};
% sceneType = {'regularplane' 'random'};
% sceneType = {'regularplane'  'forward'};
% sceneType = {'randomplanewithnoise' 'forward'};

% GenerateScene(npoints, radius, ncams, minz, maxz, noutliers, anoise, ...
%    camcalib, sceneType, pointsseed, camseed, rdcoefs, removeBehind)

ptFnd = 0;
while ptFnd == 0
    [Pgt M m m2] = GenerateScene(Npoints, 10, Ncams, 20, 35 , 0, noise,...
        [K_pinhole;K_gen;K_gen;K_gen;K_gen;K_gen;K_gen], sceneType, [], [], [], true, 0);
    try
        for i = 1:Ncams
            temp = Pgt{i} * [M; ones(1,size(M,2))];
            indices = find(temp(3,:) > 0);
            M = M(:, indices);
        end
        indicesOfPts = randperm(size(indices,2), 6);
        ptFnd = 1;
    catch
    end
end
%% Show the cameras
% for index_to_view = 1:1
%     ShowCameras({Pgt{index_to_view}}, {K_pinhole}, {m{index_to_view}}, M, true, false, false, indicesOfPts, {m2{index_to_view}});
% end

%% Pin hole camera
% For all solvers we need 6 2d points.
center1 = null(Pgt{1}); center1 = center1(1:3)/center1(4,1);
P = Pgt{1};
t = center1;
R = (K_pinhole\P(:,1:3))';

N = R'*M-R'*t;
q_d = K_pinhole\[m{1}(:,[indicesOfPts(1:6)]);ones(1,6)];


%% Gen. camera
c1 = null(Pgt{2}); c1 = c1(1:3)/c1(4);
c2 = null(Pgt{3}); c2 = c2(1:3)/c2(4);
c3 = null(Pgt{4}); c3 = c3(1:3)/c3(4);
c4 = null(Pgt{5}); c4 = c4(1:3)/c4(4);
c5 = null(Pgt{6}); c5 = c5(1:3)/c5(4);
c6 = null(Pgt{7}); c6 = c6(1:3)/c6(4);

R_gen1 = K_gen\Pgt{2}(:,1:3);
R_gen2 = K_gen\Pgt{3}(:,1:3);
R_gen3 = K_gen\Pgt{4}(:,1:3);
R_gen4 = K_gen\Pgt{5}(:,1:3);
R_gen5 = K_gen\Pgt{6}(:,1:3);
R_gen6 = K_gen\Pgt{7}(:,1:3);

p_d = [(R_gen1'/K_gen)*[m{2}(:,indicesOfPts(1));1], ...
    (R_gen2'/K_gen)*[m{3}(:,indicesOfPts(2));1],...
    (R_gen3'/K_gen)*[m{4}(:,indicesOfPts(3));1],...
    (R_gen4'/K_gen)*[m{5}(:,indicesOfPts(4));1],...
    (R_gen5'/K_gen)*[m{6}(:,indicesOfPts(5));1]];
p_d = p_d./p_d(3,:);

R_gt = R; t_gt = t;
N_gt = cross(N(:,indicesOfPts(1))-N(:,indicesOfPts(2)), N(:,indicesOfPts(1))-N(:,indicesOfPts(3)));
N_gt = N_gt/(-N_gt'*N(:,indicesOfPts(1)));
H_gt = R_gt-t_gt*N_gt';


%% Test sH5_2 sturm
error_sh5_2_sturm = [];

for t=1:length(tol)

    [Hss, Nss] = mex_sh5_2_sturm([q_d(:,1:5)',p_d(:,1:5)',[c1,c2,c3,c4,c5]', [eval(['1e-',num2str(tol(t))]);0;0;0;0] ]);

    Rs = []; ts = [];
    for i=1:size(Nss,2)
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
            end
        catch
        end
    end

    try
        ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
        error_sh5_2_sturm  = [error_sh5_2_sturm, [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2))]'];
    catch
        error_sh5_2_sturm = [error_sh5_2_sturm, [inf;inf]];
    end
    
end




%% Test sH45_2 dan
error_sh45_2_dan = [];
for t=1:length(tol)

    [Hss, Nss] = mex_sh45_2_dan([q_d(:,1:5)',p_d(:,1:5)',[c1,c2,c3,c4,c5]', [eval(['1e-',num2str(tol(t))]);0;0;0;0] ]);


    Rs = []; ts = [];
    for i=1:size(Nss,2)
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
            end
        catch
        end
    end

    try
        ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
        error_sh45_2_dan = [error_sh45_2_dan, [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2))]'];
    catch
        error_sh45_2_dan = [error_sh45_2_dan, [inf;inf]];
    end

end



%% Test sH5_3 sturm
error_sh5_3_sturm = [];

for t=1:length(tol)

    p_d = [(R_gen1'/K_gen)*[m{2}(:,indicesOfPts(1));1], ...
        (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(2));1],...
        (R_gen1'/K_gen)*[m{2}(:,indicesOfPts(3));1],...
        (R_gen2'/K_gen)*[m{3}(:,indicesOfPts(4));1],...
        (R_gen3'/K_gen)*[m{4}(:,indicesOfPts(5));1]];
    p_d = p_d./p_d(3,:);

    [Hss, Nss] = mex_sh5_3_sturm([q_d(:,1:5)',p_d(:,1:5)',[c1,c1,c1,c2,c3]', [eval(['1e-',num2str(tol(t))]);0;0;0;0] ]);

    % min(abs(arrayfun(@(i) norm([Hss(:,3*i-2:3*i)]-[H_gt],2), 1:size(Nss,2))))
    % min(abs(arrayfun(@(i) norm([Nss(:,i)]-[N_gt],2), 1:size(Nss,2))))
    Rs = []; ts = [];
    for i=1:size(Nss,2)
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
            end
        catch
        end
    end

    try
        ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
        error_sh5_3_sturm  = [error_sh5_3_sturm, [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2))]'];
    catch
        error_sh5_3_sturm = [error_sh5_3_sturm, [inf;inf]];
    end

end


%% Test sH45_3 dan
error_sh45_3_dan = [];
for t=1:length(tol)

    [Hss, Nss] = mex_sh45_3_dan([q_d(:,1:5)',p_d(:,1:5)',[c1,c1,c1,c2,c3]', [eval(['1e-',num2str(tol(t))]);0;0;0;0] ]);


    Rs = []; ts = [];
    for i=1:size(Nss,2)
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
            end
        catch
        end
    end

    try
        ind = find( abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2))) == min(abs(arrayfun(@(i) norm([Rs(:,3*i-2:3*i), ts(:,i)]-[R_gt,t_gt],2), 1:size(ts,2)))), 1);
        error_sh45_3_dan  = [error_sh45_3_dan, [norm(ts(:,ind)-t_gt,2)/norm(t_gt,2), abs(acosd((trace(R_gt'*Rs(:,3*ind-2:3*ind))-1)/2))]'];
    catch
        error_sh45_3_dan = [error_sh45_3_dan, [inf;inf]];
    end

end




%% Compiling all the solvers' errors
errors = [error_sh5_2_sturm, error_sh5_3_sturm, error_sh45_2_dan, error_sh45_3_dan];
%disp(errors);
end

