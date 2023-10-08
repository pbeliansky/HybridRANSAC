function boxplot_noise_tn(errors, noise_levels, methodsNames, name, filename)


% colors for methods
cols = {'r' ,[0.9648    0.8086    0.4141],'b' ,[0.3906    0.6992    0.6055], 'k' ,  'g',  'm' };

%number of methods
nmet = size(methodsNames,2);

%noise levels
fnoises = noise_levels;

% methods for displaying
methods = 1:nmet;

%limits of y-axis (can be read from the data)
ymin = 0; ymax = 20;

% name = 'Forward motion';



errors_to_plot = {};
for j = 1:nmet
    for ns=1:length(fnoises)   
        errors_to_plot{j}(ns).err = 100*10.^(errors{j}(ns).tn_err);
    end
end


show_boxplot(errors_to_plot, fnoises, methodsNames, name, methods, ymin, ymax, cols, filename)




function show_boxplot(algs, fnoises, methodsNames, name, methods, ymin, ymax, cols, filename)



ncnt = length(fnoises);

%  allmes{cfg}{focal}{alg}(noise, measurement)
i = 1;
ii = 1;


hnd = figure;
axes_handle=axes('fontsize', 24);
hold on;
set(hnd, 'Name', name);
title(name, 'fontsize', 24);
lims =  [ymin   ymax];
ylim(axes_handle, lims);
xlim(axes_handle, [0.4 length(fnoises)+0.5]);

xt = [];
xn = {};
for ns=1:length(fnoises)
    xt(ns) = ns*1;
    xn{ns} = num2str(fnoises(ns));
    
end

for ns=1:length(methods)
    plot(-1,-1, 'color',cols{ns});
end

set(gca,'XTick', xt);
set(gca,'XTickLabel', xn);

xlabel('Noise \sigma (pixels)', 'fontsize', 24);
%xlabel('Distance of 3D point from plane', 'fontsize', 24);

ylabel('Relative pose error(%)','fontsize', 24);

for ns=2:length(fnoises)
    plot([ns-0.5 ns-0.5], [lims(1) lims(2)], ['-'], 'LineWidth', 2, 'color', [0.9 0.9 0.9]);
end

metshift = 0.9/length(methods);
xsize = metshift/2-0.02;
xxsize = metshift/4-0.02;
ofs = (1 + xsize) - (metshift * length(methods)) / 2;



%plot([0 10], [1 1], 'Color', 'cyan');

scollector = [];
for j=methods
    
    xstart = ofs;
    xpos = xstart;
    
    for ns=1:length(fnoises)
        
        x = [];
        
        % error that will be plot
        %   mx =abs(algs{j}(ns).f(:,1)-(f1gt*pixel))/(f1gt*pixel);
        mx =algs{j}(ns).err;
        x = [x; mx(:)];
        
        
        
        [outlier,loadj,upadj,yy,q1,q3,n2,med,n1] = mb_boxutil(x, 1,1.5,0);
        
        plot([xpos xpos], [q1 n2], ['--' ],'color', cols{ii}, 'LineWidth', 2);
        plot([xpos xpos], [q3 n1], ['--'],'color', cols{ii}, 'LineWidth', 2);
        
        %         plot(xpos * ones(1, length(yy)), yy, ['+' cols{ii}], 'MarkerSize', 3);
        
        plot([xpos-xxsize xpos+xxsize], [q1 q1], ['-' ],'color', cols{ii}, 'LineWidth', 2);
        plot([xpos-xxsize xpos+xxsize], [q3 q3], ['-'],'color', cols{ii}, 'LineWidth', 2);
        
        plot([xpos-xsize xpos+xsize xpos+xsize xpos-xsize xpos-xsize], [n1 n1 n2 n2 n1], ['-'],'color', cols{ii}, 'LineWidth', 2);
        
        plot([xpos-xsize xpos+xsize], [med med], ['-'],'color', cols{ii}, 'LineWidth', 2);
        
        stat = [n2, med, n1];
        
        %scollector(:, i) = stat;
        %collector(:, i) = x;
        i=i+1;
        xpos = xpos + 1;
    end
    
    %X = (1:size(lambdas,2)) + xstart -1;
    
    
    ofs = ofs + metshift;
    ii = ii + 1;
    
    lgn = legend(methodsNames(methods), 'Location','northwest',  'Fontsize',18);
    %     set(lgn,'color','none');
    
    
end
 set(gcf,'Units','points');
    set(gcf,'Position',[0 0 600 400]);

    screenposition = get(gcf,'Position');
    set(gcf,...%'PaperPosition',[0 0 15 10],...
        'Units', 'normalized',...
        'PaperSize',[9 6]);
print(filename, '-dpdf');% -painters tn_error_unknownfocal_Ef6+1.pdf
end

end