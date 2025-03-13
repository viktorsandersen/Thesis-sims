function [Ymat] = get_big_regressor(reg_mat, q, dq, ddq, use_friction)
%% Build regression matrix
if use_friction
    Ymat = % insert 
else
    Ymat = % insert
end

f = waitbar(0,'1','Name','Building regressor...',...
    'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');
setappdata(f,'canceling',0);

for i = 1:size(q, 1)
    % Check for clicked Cancel button
    if getappdata(f,'canceling')
        break
    end

    waitbar(i / size(q, 1), f, sprintf("%.1f %%", 100 * i / size(q, 1)));

    if use_friction
        Y = % insert
    else
        Y = % insert
    end
    
    % Insert Y into Ymat
end

delete(f)
end

