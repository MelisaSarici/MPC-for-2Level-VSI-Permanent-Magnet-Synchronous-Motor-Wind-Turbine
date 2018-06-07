function P = ARE_diag(A, B, Q, R)

    n = size(A,1);
    
    P = zeros(n,n);
    
    H = [A -(B/R)*B'; -Q -A'];
    
    [D, H_b] = matrix_balance(H);
    
    [T,L] = eig(H_b);
    
    T = D*T;
    
    lambda = diag(L);
    ord_lambda = zeros(1,2*n);
    for k = 1 : 2*n;
        if real(lambda(k)) < 0
            ord_lambda(k) = -1;
        elseif real(lambda(k)) > 0
            ord_lambda(k) = 1;
        else
            %disp('!!!');
        end
    end
    
    [~, ord_index] = sort(ord_lambda, 'descend');

    T_ord = T(:,ord_index);

    T22 = T_ord(n+1:2*n,n+1:2*n);
    T12 = T_ord(1:n,n+1:2*n);
    
    P = real(T22/T12);
    
end

