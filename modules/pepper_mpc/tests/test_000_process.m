clear
source ('test_000.m')

num_iter = numel(humoto);

cstate_profile = [];
cop_profile = [];

for i = 1:num_iter
    cstate_profile = [cstate_profile, humoto{i}.mpcmg.cstate_profile];
    cop_profile = [cop_profile, humoto{i}.mpcmg.cop_profile];
end

body_state_offset = size(cstate_profile,1)/2;

figure
hold on
plot(cstate_profile(1,:), 'b--');
plot(cstate_profile(body_state_offset + 1,:), 'k--');
plot(cstate_profile(1,:) + cop_profile(1,:), 'g:');
title('CoP, base, body position X')
hold off

figure
hold on
plot(cstate_profile(4,:), 'b--');
plot(cstate_profile(body_state_offset + 4,:), 'k--');
plot(cstate_profile(4,:) + cop_profile(2,:), 'g:');
hold off
title('CoP, base, body position Y')


figure
hold on
plot(cstate_profile(1,:), cstate_profile(4,:), 'b--');
plot(cstate_profile(body_state_offset + 1,:), cstate_profile(body_state_offset + 4,:), 'k--');
plot(cstate_profile(1,:) + cop_profile(1,:), cstate_profile(4,:) + cop_profile(2,:), 'g:');
hold off
axis equal
title('CoP, base, body position X/Y')
