function [Aeq, beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = zeros(4, 1);
    % expression of Aeq_start and beq_start
    %
    for k = 0:3
        for i = k:7
            Aeq_start(k+1,i+1) = factorial(i)/factorial(i-k) * 0^(i-k);
        end
    end
    
    beq_start(1,1) = start_cond(1);
    beq_start(2,1) = start_cond(2);
    beq_start(3,1) = start_cond(3);
    beq_start(4,1) = start_cond(4);
    %
    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    % expression of Aeq_end and beq_end
    %
    for k = 0:3
        for i = k:7
            Aeq_end(k+1,(n_seg-1)*(n_order+1)+i+1) = factorial(i)/factorial(i-k) * ts(1)^(i-k);
        end
    end
    
    beq_end(1,1) = end_cond(1);
    beq_end(2,1) = end_cond(2);
    beq_end(3,1) = end_cond(3);
    beq_end(4,1) = end_cond(4);
    %
    
    %#####################################################
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % expression of Aeq_wp and beq_wp
    %
    for i = 1:n_seg-1   
        Aeq_wp(i,i*(n_order+1)+1) = 1;
    end
    
    for i = 1:n_seg-1
        beq_wp(i,1) = waypoints(i+1);
        
    end
    %
    
    %#####################################################
    % position continuity constrain between each 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    % expression of Aeq_con_p and beq_con_p
    %
    for i = 1:n_seg-1
        for j = 0:n_order
            Aeq_con_p(i,(i-1)*(n_order+1)+j+1) = ts(1)^(j);
        end
    end
    
    for i = 1:n_seg-1
        beq_con_p(i,1) = waypoints(i+1);
    end
    %
    
    %#####################################################
    % velocity continuity constrain between each 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    % expression of Aeq_con_v and beq_con_v
    %
    k = 1;
    for i = 1:n_seg-1
        for j = k:n_order
            Aeq_con_v(i,(i-1)*(n_order+1)+j+1) = factorial(j)/factorial(j-k) * ts(1)^(j-k);
        end
    end
    for i = 1:n_seg-1
        for j = k:n_order
            Aeq_con_v(i,i*(n_order+1)+j+1) = -factorial(j)/factorial(j-k) * 0^(j-k);
        end
    end
    %

    %#####################################################
    % acceleration continuity constrain between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    % expression of Aeq_con_a and beq_con_a
    %
    k = 2;
    for i = 1:n_seg-1
        for j = k:n_order
            Aeq_con_a(i,(i-1)*(n_order+1)+j+1) = factorial(j)/factorial(j-k) * ts(1)^(j-k);
        end
    end
    for i = 1:n_seg-1
        for j = k:n_order
            Aeq_con_a(i,i*(n_order+1)+j+1) = -factorial(j)/factorial(j-k) * 0^(j-k);
        end
    end
    %
    
    %#####################################################
    % jerk continuity constrain between each 2 segments
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    % expression of Aeq_con_j and beq_con_j
    %
    k = 3;
    for i = 1:n_seg-1
        for j = k:n_order
            Aeq_con_j(i,(i-1)*(n_order+1)+j+1) = factorial(j)/factorial(j-k) * ts(1)^(j-k);
        end
    end
    for i = 1:n_seg-1
        for j = k:n_order
            Aeq_con_j(i,i*(n_order+1)+j+1) = -factorial(j)/factorial(j-k) * 0^(j-k);
        end
    end
    %
    
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end