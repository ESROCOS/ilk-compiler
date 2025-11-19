function [q_ik] = ik1(mc, q, v)
    [~, J] = fk1(mc, q)
    aux = J(4:6,:);
    q_ik = ilk.leastSquaresSolve(aux, v);
