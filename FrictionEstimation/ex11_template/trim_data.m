function [t, q, dq, ddq, tau] = trim_data(t, q, dq, ddq, tau, max_time_index)
index_cut = [1 size(t,1)];
index_min = 1;
index_cut(1) = index_min + find(any(abs([dq ddq])'>5e-4),1,'first');
index_cut(2) = index_cut(2) - find(any(abs(flip([dq ddq]))'>5e-4),1,'first');
index_cut(2) = min(index_cut(2), index_cut(1) + max_time_index);

q = q(index_cut(1) : index_cut(2), :);
dq = dq(index_cut(1) : index_cut(2), :);
ddq = ddq(index_cut(1) : index_cut(2), :);
tau = tau(index_cut(1) : index_cut(2), :);
t = t(index_cut(1) : index_cut(2));
end

