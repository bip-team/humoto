if (exist('time_perf_lexls.m', 'file'))
    source time_perf_lexls.m
    figure
    title('lexls')
    hold on
    plot(lexls_simulation.no_hotstart.average_time_to_run_full_loop, 'b')
    plot(lexls_simulation.as_guess_prev_iter.average_time_to_run_full_loop, 'r--')
    plot(lexls_simulation.sol_guess_as_guess_prev_iter.average_time_to_run_full_loop, 'k--')
    plot(lexls_simulation.sol_guess_no_as_guess.average_time_to_run_full_loop, 'c--')
    hold off
end

if (exist('time_perf_qpoases.m', 'file'))
    source time_perf_qpoases.m
    figure
    title('qpoases')
    hold on
    plot(qpoases_simulation.no_hotstart.average_time_to_run_full_loop, 'b')
    plot(qpoases_simulation.as_guess_prev_iter.average_time_to_run_full_loop, 'r--')
    plot(qpoases_simulation.sol_guess_as_guess_prev_iter.average_time_to_run_full_loop, 'k--')
    plot(qpoases_simulation.sol_guess_no_as_guess.average_time_to_run_full_loop, 'c--')
    hold off
end
