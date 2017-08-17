function [p2, pcom_f, pcom_intersect, interation_steps] = NR_searching_method_multi_test(p1, p2_initial, pcom_i, pcom_f, ...
    vcomx_i, vcomx_f, acomx_i, acomx_f, vcomy_i, acomy_i, pcom_intersect, inc_x, inc_t)


pp2(1) = p2_initial(2);
pp3(1) = p2_initial(3);
d_vcom_y_end(1) = -0.001;
%this item is changed.
vcom_y_end(1) = -0.2;
i =1;

while (i<=20 && abs(vcom_y_end(end))>=5e-3)
    
%     pp2_initial(:,i) = [p2_initial(1); pp2(i); p2_initial(3)];
    pp2_initial(:,i) = [p2_initial(1); pp2(i); pp3(i)];

    [tcom_tmp, tcom_intersect_tmp, pcom_tmp, vcom_tmp, acom_tmp] = one_step_integration_multi(p1, pp2_initial(:,i), pcom_i, pcom_f,...
    vcomx_i, vcomx_f, acomx_i, acomx_f, vcomy_i, acomy_i, pcom_intersect, inc_x, inc_t);

     
    vcom_y_end(i) = vcom_tmp(2,end);
    pp2(i+1) = pp2(i) - vcom_y_end(i)/d_vcom_y_end(i)
    
    if pp2(i+1) > 0.4
        pp2(i+1) = 0.4;
    end
    
    if pp2(i+1) < -0.4
        pp2(i+1) = -0.4;
    end
    
    pp2(i+1)
    pp2(i)
    disp('time');
    if pp2(i+1) >=0
%         pp3(i+1) = pp3(i) + 0.1 * (pp2(i+1) - pp2(i))
%         pp3(i+1) = pp3(i) - 0.15 * (pp2(i+1) - pp2(i))
        pp3(i+1) =  p2_initial(3) - 0.15 * pp2(i+1)
%         p2_initial(3)
% 0.2 is not okay
%         pp3(i)
    else
%         pp3(i+1) = pp3(i) - 0.1 * (pp2(i+1) - pp2(i))

%         pp3(i+1) = pp3(i) - 0.15 * (pp2(i+1) - pp2(i))
        pp3(i+1) =   p2_initial(3) + 0.15 * pp2(i+1) 
%         p2_initial(3)
%         pp2(i+1)
    end
    pcom_f(3) = pcom_f(3) + 0.3 * (pp3(i+1) - p2_initial(3));
    pcom_intersect(3) = pcom_intersect(3) + 0.5 * (pp3(i+1) - p2_initial(3));
%     pp2(i+1)
%     pp3(i+1) = 0;
    
    pp2_initial(:,i+1) = [p2_initial(1); pp2(i+1); pp3(i+1)];
%     p2_initial(3)
    [tcom_tmp, tcom_intersect_tmp, pcom_tmp, vcom_tmp, acom_tmp] = one_step_integration_multi(p1, pp2_initial(:,i+1), pcom_i, pcom_f,...
    vcomx_i, vcomx_f, acomx_i, acomx_f, vcomy_i, acomy_i, pcom_intersect, inc_x, inc_t);

    vcom_y_end(i+1) = vcom_tmp(2,end)
    d_vcom_y_end(i+1) = (vcom_y_end(i+1) - vcom_y_end(i))/(pp2(i+1) - pp2(i));
    i = i + 1
end

% p2 = [p2_initial(1); pp2(i); p2_initial(3)]
p2 = [p2_initial(1); pp2(i); pp3(i)]

interation_steps = i-1;




