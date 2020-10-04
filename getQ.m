function Q = getQ(n_seg, n_order, ts)
    Q = [];
    for k = 1:n_seg
        Q_k = [];
        %#####################################################
        % calculate Q_k of the k-th segment 
        for i = 4:7
            for l = 4:7
                Q_k(i+1,l+1) = (i*(i-1)*(i-2)*(i-3)*l*(l-1)*(l-2)*(l-3)) / (i+l-7) * ts(k)^(i+l-7);  
            end
        end
        %#####################################################
        Q = blkdiag(Q, Q_k);
    end
end