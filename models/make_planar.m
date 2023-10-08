% very experimental code
M = readcell('.\KingsCollege\real\model\points3D.txt','delimiter',' ');
disp("done read")
%%  A will contain indices that we want to work with
first = 4;
last = 104441;
M(first:last,2:4);
disp("...")

A = cell2mat(M(first:last,2:4));
disp("...")


%%

pointA = [-5,-1,0];
pointB = [125,75,20];
pointC = [125,75,50];
points=[pointA' pointB' pointC']; % using the data given in the question
normal = cross(pointA-pointB, pointA-pointC);

X_ind = logical(zeros(size(A,1),1));
Y_ind = logical(zeros(size(A,1),1));

for i = 1:last-first+1
    if dot(A(i,:),normal)/(norm(A(i,:))*norm(normal)) < 0
        %X = [X;A(i,:)];
        X_ind(i) = 1;
    else
        %Y = [Y;A(i,:)];
        Y_ind(i) = 1;
    end
end

X = A(X_ind,:);
Y = A(Y_ind,:);


xplane = pcfitplane(pointCloud(X),200000, [1,0,0], 15);
yplane = pcfitplane(pointCloud(Y),200000, [0.2,1,0], 15);

p = xplane.Parameters;
syms x y z
eqn = p(1)*x + p(2)*y + p(3)*z + p(4) == 0;
S = solve(eqn, x, y, z,'Real',true);
P = [S.x,S.y,S.z]

n = xplane.Normal / norm(xplane.Normal);
for i = 1:size(X,1)
    X(i,:) = X(i,:) - dot(X(i,:) - P, n) * n;
end
disp("X done")



p = yplane.Parameters;
syms x y z
eqn = p(1)*x + p(2)*y + p(3)*z + p(4) == 0;
S = solve(eqn, x, y, z,'Real',true);
P = [S.x,S.y,S.z]

n = yplane.Normal / norm(yplane.Normal);
for i = 1:size(Y,1)
    Y(i,:) = Y(i,:) - dot(Y(i,:) - P, n) * n;
end
disp("Y done")

%%     Fit plane to A

[model1,inlierIndices,outlierIndices] = pcfitplane(pointCloud(A), 10, [0,-1,0], 5);
plane1 = select(pointCloud(A),inlierIndices);
remainPtCloud = select(pointCloud(A),outlierIndices);
disp("..,,..")

%%
pointscolor=uint8(zeros(plane1.Count,3));
pointscolor(:,1)=102;
pointscolor(:,2)=255;
pointscolor(:,3)=153;

plane1.Color=pointscolor;

pointscolor=uint8(zeros(remainPtCloud.Count,3));
pointscolor(:,1)=0;
pointscolor(:,2)=102;
pointscolor(:,3)=255;

remainPtCloud.Color=pointscolor;

%%
%X = plane1.Location;     I will project all points this time

X = A;

p = model1.Parameters;
syms x y z
eqn = p(1)*x + p(2)*y + p(3)*z + p(4) == 0;
S = solve(eqn, x, y, z,'Real',true);
P = [S.x,S.y,S.z]

n = model1.Normal / norm(model1.Normal);
for i = 1:size(X,1)
    X(i,:) = X(i,:) - dot(X(i,:) - P, n) * n;
end
disp("X done")


%%
writematrix(X,'all_X2.txt','Delimiter',' ')  
type all_X.txt
disp("......")

%%
pcX = pointCloud(X);

pointscolor=uint8(zeros(pcX.Count,3));
pointscolor(:,1)=255;
pointscolor(:,2)=0;
pointscolor(:,3)=0;

pcX.Color=pointscolor;

%%
cell_ids = [0;1;2; inlierIndices+2];

writematrix(cell_ids,'ids2.txt','Delimiter',' ')  
type ids.txt

writematrix(X,'X2.txt','Delimiter',' ')  
type X.txt
disp("......")


%%
cell_ids = [1;2;3; inlierIndices+3];
planar_cell = M(cell_ids,:);


%%

figure;
hold on;
view(3);
grid on

size(plane1)

pcshow(plane1)
pcshow(remainPtCloud)
pcshow(pcX)

hold off;
disp("..,,..")

%%

figure;
hold on;
view(3);
grid on

plot3(A(30:30000,1), A(30:30000,2), A(30:30000,3), 'o');
plot3(A(30:30000,1), A(30:30000,2), A(30:30000,3), '.r');
hold off;
disp("...")

%%

A(X_ind,:) = X;
A(Y_ind,:) = Y;
disp("...")

%%
M(first:last,2:4) = num2cell(A);
disp("...")


%%
ccccc = M;
disp("...")
%%


mask = cellfun(@fill_missing, ccccc, 'UniformOutput', false);
disp("...")

%%
writecell(mask, '.\KingsCollege\planar_points3D.txt' ,'delimiter',' ');


%%
F = fillmissing(M,'constant',{[]});
disp("...")

%%
writecell(M, '.\KingsCollege\planar_points3D.txt' ,'delimiter',' ');

%%
X_ind = logical([1;1;1;X_ind]);
Y_ind = logical([1;1;1;Y_ind]);

%%
XX = mask(X_ind,:);
YY = mask(Y_ind,:);


writecell(XX, '.\KingsCollege\Lplanar_points3D.txt' ,'delimiter',' ');
writecell(YY, '.\KingsCollege\Rplanar_points3D.txt' ,'delimiter',' ');

%%%%%

function c = fill_missing(x)
    if ismissing(x)
        c = [];
    else
        c = x;
    end
end




