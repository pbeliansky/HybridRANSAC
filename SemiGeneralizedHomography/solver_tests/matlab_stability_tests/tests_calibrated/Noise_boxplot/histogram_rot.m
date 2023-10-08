function histogram_rot(errors, noise_levels, methodsNames, name, filename)


% colors for methods
cols = {'r' ,[0.9648    0.8086    0.4141],'b' ,[0.3906    0.6992    0.6055], 'k' ,  'g',  'c', 'm' };


%number of methods
nmet = size(methodsNames,2);

%noise levels
fnoises = noise_levels;

% methods for displaying
methods = 1:nmet;
% methods names
%methodsNames = { 'sH5f_{2}' 'sH5f_{2d}' 'sH5f_{3}'  'P5Pf+N', 'Ef_{6+1}' , 'Ef_{5+2}'};
%name = 'Rotation error';



cfgstyles = {'-' '-.*' '--' '--' ':'};

%limits of y-axis (can be read from the data)
xmin = -10;
xmax = 5;


errors_to_plot = {};
for j = 1:nmet
    
    errors_to_plot{j}.err = (errors{j}.rot_err);
    
end




show_boxplot(errors_to_plot, methodsNames, name, methods, xmin, xmax, cols, cfgstyles, filename)



function show_boxplot(errors_to_plot, methodsNames, name, methods, xmin, xmax, cols, cfgstyles, filename)


hnd = figure;
axes('fontsize', 24);
hold on;
set(hnd, 'Name', name);
title(name, 'fontsize', 24);

%title(['focal length 1 ' int2str(fc*18) 'mm']);
% title(name, 'fontsize', 12);
% noisefree

% xlabel('$Log_{10} (|| t - t_{gt} ||_{2}/|| t_{gt} ||_{2})$','Interpreter','Latex');
xlabel('Log_{10} rotation error ', 'fontsize', 24);
ylabel('Frequency', 'fontsize', 24);

log_range = xmin:0.3:xmax;
cfgs = [1];
% log10 bez 0
for cfg=cfgs
    
    ii = 1;
    for j=methods
        
        data = [];
        
        data = [data  errors_to_plot{j}.err];
        
        
        if 1
            gr = data;
            
            % filter exact data
            used = find(gr ~= 0);
            totalmin = 1e-20;
            used = find(gr == 0);
            gr(used) = totalmin;
            
            h = hist(log10(abs(gr)), log_range);
            
            % smoothing
            if 1
                ks = 3;
                smask = [0.2 0.5 1 2 1 0.5 0.2];
                %s = 2;
                %smask = [fliplr(exp(-(1:ks).^s)) 1 exp(-(1:ks).^s)];
                v = conv(h, smask)*(1/sum(smask));
                h = v(ks:end-(ks+1));
            end
            
            plot( log_range , h, [cfgstyles{cfg}], 'LineWidth', 2);%, 'color', cols{ii});
            ii = ii + 1;
        end
    end
end

% add legend
% legend(methodsNames(methods), 'Location','northwest');%, 'fontsize', 20, 'Orientation','horizontal', 'NumColumns', 2, 'NumColumnsMode', 'manual');
% 
% set(gcf,'Position',[0 0 1500 1000]);
% 
% set(gcf,'Units','inches');
% screenposition = get(gcf,'Position');
% set(gcf,...
%     'PaperPosition',[0 0 screenposition(3:4)],...
%     'PaperSize',[15 10]);

legend(methodsNames(methods),'Location', 'Best');%, 'Location','northwest');	
set(gcf,'Units','points');	
set(gcf,'Position',[0 0 600 400]);	
screenposition = get(gcf,'Position');	
set(gcf,...%'PaperPosition',[0 0 15 10],...	
    'Units', 'normalized',...	
    'PaperSize',[9 6]);
print(filename, '-dpdf');% -painters histogram_rot_sH5_3_sH45_3.pdf

end

end

