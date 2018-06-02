%% dq-Frame PCC Algorithm for Grid-Connected 2L-VSI

% P11 = 1-(ri*Ts/Li); P12 = (wg*Ts); P21 = -(wg*Ts); P22 = 1-(ri*Ts/Li);
% Gi11 = Gi22 = Ts/Li; Gg11 = Gg22 = -Ts/Li;
 j_op = 1000;
 g_op = 1000000000;
 
 for j = 1:1:9
     
    
         vi_ab(1) = vc(1)*vinv(j).s_i_ab(1);
         vi_ab(2) = vc(1)*vinv(j).s_i_ab(2);
         vi_dq(1) = vi_ab(1)*cstheta_g(1) + vi_ab(2)*cstheta_g(2);
         vi_dq(2) = -vi_ab(1)*cstheta_g(2) + vi_ab(2)*cstheta_g(1);
         ig_dq_k1(1) = P11*ig_dq(1)+P12*ig_dq(2)+Gi11*vi_dq(1)+Gg11*vg_dq(1);
         ig_dq_k1(2) = P21*ig_dq(1)+P22*ig_dq(2)+Gi22*vi_dq(2)+Gg22*vg_dq(2);
         g_id = (ig_ref_k1(1)-ig_dq_k1(1))*(ig_ref_k1(1)-ig_dq_k1(1));
         g_iq = (ig_ref_k1(2)-ig_dq_k1(2))*(ig_ref_k1(2)-ig_dq_k1(2));
         g_swi = (vinv(j).s_i_abc(1)-s_i_km1(1))*(vinv(j).s_i_abc(1)-s_i_km1(1)) + (vinv(j).s_i_abc(2)-s_i_km1(2))*(vinv(j).s_i_abc(2)-s_i_km1(2)) + (vinv(j).s_i_abc(3)-s_i_km1(3))*(vinv(j).s_i_abc(3)-s_i_km1(3));
         g_i = g_id + g_iq + alpha(1)*g_swi;
 
         if (g_i < g_op) 
                 j_op = j;
                 g_op = g_i;    
         end
 end
 
 s_ai1(1) = vinv(j_op).s_i(1);
 s_bi1(1) = vinv(j_op).s_i(2);
 s_ci1(1) = vinv(j_op).s_i(3);
 s_i_km1(1) = s_ai1(1);
 s_i_km1(2) = s_bi1(1);
 s_i_km1(3) = s_ci1(1);
